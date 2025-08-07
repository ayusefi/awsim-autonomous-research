#include "awsim_trajectory_planner/obstacle_detector.hpp"
#include "awsim_trajectory_planner/trajectory_generator.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace awsim_trajectory_planner
{

ObstacleDetector::ObstacleDetector(const ObstacleDetectionParams& params, rclcpp::Node* node)
: params_(params),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock(), tf2::durationFromSec(10.0))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
}

std::vector<Obstacle> ObstacleDetector::detectObstacles(
    const sensor_msgs::msg::PointCloud2& cloud_msg,
    const geometry_msgs::msg::Point& vehicle_position) const
{
  std::vector<Obstacle> obstacles;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  RCLCPP_INFO(rclcpp::get_logger("ObstacleDetector"), 
             "Processing point cloud: %d points, width=%d, height=%d, fields=%zu",
             cloud_msg.width * cloud_msg.height, cloud_msg.width, 
             cloud_msg.height, cloud_msg.fields.size());

  // Log field information
  for (const auto& field : cloud_msg.fields) {
    RCLCPP_DEBUG(rclcpp::get_logger("ObstacleDetector"), 
                "Field: %s, offset=%d, datatype=%d, count=%d",
                field.name.c_str(), field.offset, field.datatype, field.count);
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ObstacleDetector"), 
             "Vehicle position for filtering: x=%.2f, y=%.2f, z=%.2f",
             vehicle_position.x, vehicle_position.y, vehicle_position.z);
  
  try {
    // Check if we can transform to map frame
    if (!tf_buffer_->canTransform("map", cloud_msg.header.frame_id, 
                                  cloud_msg.header.stamp, rclcpp::Duration::from_seconds(0.1))) {
      RCLCPP_WARN(rclcpp::get_logger("ObstacleDetector"), 
                 "Cannot transform from %s to map frame", cloud_msg.header.frame_id.c_str());
      return obstacles;
    }

    // Transform to map frame
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        "map", cloud_msg.header.frame_id, cloud_msg.header.stamp);
    
    tf2::doTransform(cloud_msg, transformed_cloud, transform);
    RCLCPP_DEBUG(rclcpp::get_logger("ObstacleDetector"), "Point cloud transformed to map frame");

    // Extract points with vehicle-relative filtering
    std::vector<geometry_msgs::msg::Point> valid_points = extractPointsFromCloud(
        transformed_cloud, vehicle_position);
    
    RCLCPP_INFO(rclcpp::get_logger("ObstacleDetector"), 
               "Extracted %zu valid points from %d total points", 
               valid_points.size(), cloud_msg.width * cloud_msg.height);

    if (valid_points.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("ObstacleDetector"), "No valid points after filtering");
      return obstacles;
    }

    // Cluster points with improved parameters
    auto clusters = clusterPoints(valid_points);
    RCLCPP_INFO(rclcpp::get_logger("ObstacleDetector"), 
               "Found %zu clusters from %zu points", clusters.size(), valid_points.size());

    // Convert clusters to obstacles
    for (const auto& cluster : clusters) {
      auto obstacle = createObstacleFromCluster(cluster);
      obstacles.push_back(obstacle);
      
      RCLCPP_DEBUG(rclcpp::get_logger("ObstacleDetector"), 
                  "Created obstacle: bbox[%.2f,%.2f]x[%.2f,%.2f], size=%.2fx%.2f",
                  obstacle.x_min, obstacle.x_max, obstacle.y_min, obstacle.y_max,
                  obstacle.getWidth(), obstacle.getHeight());
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    RCLCPP_INFO(rclcpp::get_logger("ObstacleDetector"), 
               "Obstacle detection completed: %zu obstacles in %.2f ms", 
               obstacles.size(), duration.count() / 1000.0);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetector"), 
                "Exception in obstacle detection: %s", e.what());
    return obstacles;
  }

  return obstacles;
}

std::vector<geometry_msgs::msg::Point> ObstacleDetector::extractPointsFromCloud(
  const sensor_msgs::msg::PointCloud2& pointcloud,
  const geometry_msgs::msg::Point& reference_point) const  // Added reference point parameter
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(pointcloud.width * pointcloud.height / 4);

  // Check if pointcloud is empty
  if (pointcloud.data.empty() || pointcloud.width == 0 || pointcloud.height == 0) {
    RCLCPP_WARN(rclcpp::get_logger("ObstacleDetector"), 
               "Empty or invalid pointcloud: width=%d, height=%d, data_size=%zu", 
               pointcloud.width, pointcloud.height, pointcloud.data.size());
    return points;
  }

  // Check if we have x, y, z fields
  bool has_x = false, has_y = false, has_z = false;
  for (const auto& field : pointcloud.fields) {
    if (field.name == "x") has_x = true;
    if (field.name == "y") has_y = true;
    if (field.name == "z") has_z = true;
  }

  if (!has_x || !has_y || !has_z) {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetector"), 
                "Pointcloud missing required fields: x=%d, y=%d, z=%d", has_x, has_y, has_z);
    return points;
  }

  // Debug: Get min/max values for each axis
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

    // Spatial filtering relative to reference point (vehicle position)
    const double max_range_sq = params_.max_detection_range * params_.max_detection_range;
    size_t total_points = 0;
    size_t valid_points = 0;
    size_t finite_filtered = 0;
    size_t height_filtered = 0;
    size_t range_filtered = 0;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      total_points++;
      
      // Update min/max for debugging
      min_x = std::min(min_x, *iter_x);
      max_x = std::max(max_x, *iter_x);
      min_y = std::min(min_y, *iter_y);
      max_y = std::max(max_y, *iter_y);
      min_z = std::min(min_z, *iter_z);
      max_z = std::max(max_z, *iter_z);
      
      // Early validity check
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        finite_filtered++;
        continue;
      }

      // Height filtering - made more permissive
      if (*iter_z < params_.min_obstacle_height || *iter_z > params_.max_obstacle_height) {
        height_filtered++;
        continue;
      }

      // Range filtering: now relative to reference point (vehicle position)
      double dx = *iter_x - reference_point.x;
      double dy = *iter_y - reference_point.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq > max_range_sq) {
        range_filtered++;
        continue;
      }

      geometry_msgs::msg::Point point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      points.push_back(point);
      valid_points++;
    }

    // Log detailed statistics
    RCLCPP_INFO(rclcpp::get_logger("ObstacleDetector"), 
                "Point cloud statistics: x[%.2f,%.2f], y[%.2f,%.2f], z[%.2f,%.2f]",
                min_x, max_x, min_y, max_y, min_z, max_z);
                
    RCLCPP_INFO(rclcpp::get_logger("ObstacleDetector"), 
                "Filtering results: total=%zu, valid=%zu, finite_filtered=%zu, height_filtered=%zu, range_filtered=%zu", 
                total_points, valid_points, finite_filtered, height_filtered, range_filtered);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ObstacleDetector"), 
                "Exception during point extraction: %s", e.what());
  }

  return points;
}

std::vector<std::vector<geometry_msgs::msg::Point>> ObstacleDetector::clusterPoints(
  const std::vector<geometry_msgs::msg::Point>& points) const
{
  std::vector<std::vector<geometry_msgs::msg::Point>> clusters;
  
  if (points.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("ObstacleDetector"), "No points to cluster");
    return clusters;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("ObstacleDetector"), 
              "Clustering %zu points with tolerance %.2f, min_points=%d", 
              points.size(), params_.clustering_distance, params_.min_points_per_cluster);

  std::vector<bool> visited(points.size(), false);
  const double cluster_dist_sq = params_.clustering_distance * params_.clustering_distance;

  for (size_t i = 0; i < points.size(); ++i) {
    if (visited[i]) {
      continue;
    }

    std::vector<geometry_msgs::msg::Point> cluster;
    std::vector<size_t> stack;
    stack.reserve(100); // Pre-allocate space
    stack.push_back(i);

    while (!stack.empty()) {
      size_t current_idx = stack.back();
      stack.pop_back();

      if (visited[current_idx]) {
        continue;
      }

      visited[current_idx] = true;
      cluster.push_back(points[current_idx]);

      const auto& current_point = points[current_idx];
      
      // Use spatial proximity to limit search (optimized neighborhood search)
      for (size_t j = 0; j < points.size(); ++j) {
        if (!visited[j]) {
          double dx = points[j].x - current_point.x;
          double dy = points[j].y - current_point.y;
          double dz = points[j].z - current_point.z;
          double dist_sq = dx * dx + dy * dy + dz * dz;
          
          if (dist_sq <= cluster_dist_sq) {
            stack.push_back(j);
          }
        }
      }
    }

    if (cluster.size() >= static_cast<size_t>(params_.min_points_per_cluster)) {
      clusters.push_back(std::move(cluster));
      RCLCPP_DEBUG(rclcpp::get_logger("ObstacleDetector"), 
                  "Created cluster with %zu points", cluster.size());
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("ObstacleDetector"), 
                  "Discarded cluster with %zu points (min required: %d)", 
                  cluster.size(), params_.min_points_per_cluster);
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("ObstacleDetector"), 
              "Clustering complete: %zu clusters from %zu points", clusters.size(), points.size());

  return clusters;
}

Obstacle ObstacleDetector::createObstacleFromCluster(
  const std::vector<geometry_msgs::msg::Point>& cluster) const
{
  Obstacle obstacle;

  if (cluster.empty()) {
    return obstacle;
  }

  // Calculate centroid
  double sum_x = 0.0, sum_y = 0.0;
  for (const auto& point : cluster) {
    sum_x += point.x;
    sum_y += point.y;
  }
  obstacle.center_x = sum_x / cluster.size();
  obstacle.center_y = sum_y / cluster.size();

  // Calculate covariance matrix for PCA
  double cov_xx = 0.0, cov_xy = 0.0, cov_yy = 0.0;
  for (const auto& point : cluster) {
    double dx = point.x - obstacle.center_x;
    double dy = point.y - obstacle.center_y;
    cov_xx += dx * dx;
    cov_xy += dx * dy;
    cov_yy += dy * dy;
  }
  cov_xx /= cluster.size();
  cov_xy /= cluster.size();
  cov_yy /= cluster.size();

  // Find principal components (eigenvalues and eigenvectors)
  double trace = cov_xx + cov_yy;
  double det = cov_xx * cov_yy - cov_xy * cov_xy;
  double lambda1 = trace / 2.0 + std::sqrt(trace * trace / 4.0 - det);
  // Note: lambda2 = trace / 2.0 - std::sqrt(trace * trace / 4.0 - det); (not used)

  // Calculate orientation from the largest eigenvector
  if (std::abs(cov_xy) > 1e-6) {
    obstacle.orientation = std::atan2(lambda1 - cov_xx, cov_xy);
  } else {
    obstacle.orientation = (cov_xx > cov_yy) ? 0.0 : M_PI / 2.0;
  }

  // Project points onto principal axes to find dimensions
  double min_u = std::numeric_limits<double>::max();
  double max_u = std::numeric_limits<double>::lowest();
  double min_v = std::numeric_limits<double>::max();
  double max_v = std::numeric_limits<double>::lowest();

  double cos_theta = std::cos(obstacle.orientation);
  double sin_theta = std::sin(obstacle.orientation);

  for (const auto& point : cluster) {
    double dx = point.x - obstacle.center_x;
    double dy = point.y - obstacle.center_y;
    
    // Project onto principal axes
    double u = dx * cos_theta + dy * sin_theta;
    double v = -dx * sin_theta + dy * cos_theta;
    
    min_u = std::min(min_u, u);
    max_u = std::max(max_u, u);
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }

  // Set dimensions with some padding
  const double padding = 0.2;  // 20cm padding
  obstacle.width = (max_u - min_u) + 2 * padding;
  obstacle.height = (max_v - min_v) + 2 * padding;

  // Ensure minimum size to avoid degenerate cases
  obstacle.width = std::max(obstacle.width, 0.5);
  obstacle.height = std::max(obstacle.height, 0.5);

  // For very elongated objects, swap dimensions if needed to have width >= height
  if (obstacle.height > obstacle.width) {
    std::swap(obstacle.width, obstacle.height);
    obstacle.orientation += M_PI / 2.0;
    
    // Normalize orientation to [-π, π]
    while (obstacle.orientation > M_PI) obstacle.orientation -= 2 * M_PI;
    while (obstacle.orientation < -M_PI) obstacle.orientation += 2 * M_PI;
  }

  // Update axis-aligned bounding box for quick checks
  obstacle.updateAABB();

  // For simplicity, assume static obstacles
  obstacle.velocity_x = 0.0;
  obstacle.velocity_y = 0.0;

  RCLCPP_DEBUG(rclcpp::get_logger("ObstacleDetector"), 
              "Created OBB: center(%.2f,%.2f), size(%.2fx%.2f), orientation=%.2f°, AABB[%.2f,%.2f]x[%.2f,%.2f]",
              obstacle.center_x, obstacle.center_y, obstacle.width, obstacle.height,
              obstacle.orientation * 180.0 / M_PI,
              obstacle.x_min, obstacle.x_max, obstacle.y_min, obstacle.y_max);

  return obstacle;
}

bool ObstacleDetector::isPointCollisionFree(
  double x, double y,
  const std::vector<Obstacle>& obstacles,
  double safety_margin) const
{
  for (const auto& obstacle : obstacles) {
    // Check if point is inside the expanded bounding box
    if (x >= obstacle.x_min - safety_margin && 
        x <= obstacle.x_max + safety_margin &&
        y >= obstacle.y_min - safety_margin && 
        y <= obstacle.y_max + safety_margin) {
      return false;
    }
  }
  return true;
}

bool ObstacleDetector::isCollisionFree(
  const std::vector<TrajectoryPoint>& trajectory,
  const std::vector<Obstacle>& obstacles,
  double safety_margin) const
{
  for (const auto& point : trajectory) {
    if (!isPointCollisionFree(point.x, point.y, obstacles, safety_margin)) {
      return false;
    }
  }
  return true;
}


nav_msgs::msg::OccupancyGrid ObstacleDetector::generateOccupancyGrid(
  const std::vector<Obstacle>& obstacles,
  double center_x, double center_y,
  double width, double height) const
{
  nav_msgs::msg::OccupancyGrid grid;
  
  grid.header.frame_id = "map";
  grid.header.stamp = rclcpp::Clock().now();
  
  grid.info.resolution = params_.grid_resolution;
  grid.info.width = static_cast<uint32_t>(width / params_.grid_resolution);
  grid.info.height = static_cast<uint32_t>(height / params_.grid_resolution);
  
  grid.info.origin.position.x = center_x - width / 2.0;
  grid.info.origin.position.y = center_y - height / 2.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(grid.info.width * grid.info.height, 0);

  // Fill grid with obstacle information
  for (size_t y = 0; y < grid.info.height; ++y) {
    for (size_t x = 0; x < grid.info.width; ++x) {
      double world_x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
      double world_y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;

      bool is_occupied = !isPointCollisionFree(world_x, world_y, obstacles, 0.0);
      
      size_t index = y * grid.info.width + x;
      grid.data[index] = is_occupied ? 100 : 0;
    }
  }

  return grid;
}

double ObstacleDetector::calculateDistance(
  const geometry_msgs::msg::Point& p1,
  const geometry_msgs::msg::Point& p2) const
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace awsim_trajectory_planner
