#include "awsim_trajectory_planner/obstacle_detector.hpp"
#include "awsim_trajectory_planner/trajectory_generator.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

namespace awsim_trajectory_planner
{

ObstacleDetector::ObstacleDetector(const ObstacleDetectionParams& params)
: params_(params)
{
}

std::vector<Obstacle> ObstacleDetector::detectObstacles(
  const sensor_msgs::msg::PointCloud2& pointcloud)
{
  std::vector<Obstacle> obstacles;

  try {
    // Extract points from point cloud
    auto points = extractPointsFromCloud(pointcloud);

    // Filter points by height and range
    auto filtered_points = filterPointsByHeight(points);
    filtered_points = filterPointsByRange(filtered_points);

    // Cluster points to form obstacles
    auto clusters = clusterPoints(filtered_points);

    // Create obstacles from clusters
    for (const auto& cluster : clusters) {
      if (cluster.size() >= static_cast<size_t>(params_.min_points_per_cluster)) {
        // obstacles.push_back(createObstacleFromCluster(cluster));
        RCLCPP_DEBUG(rclcpp::get_logger("ObstacleDetector"), "Detected obstacle: ");
      }
    }

  } catch (const std::exception& e) {
    // Handle point cloud processing errors gracefully
    // In a real implementation, you might want to log this
  }

  return obstacles;
}

std::vector<geometry_msgs::msg::Point> ObstacleDetector::extractPointsFromCloud(
  const sensor_msgs::msg::PointCloud2& pointcloud) const
{
  std::vector<geometry_msgs::msg::Point> points;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    geometry_msgs::msg::Point point;
    point.x = *iter_x;
    point.y = *iter_y;
    point.z = *iter_z;

    // Basic validity check
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      points.push_back(point);
    }
  }

  return points;
}

std::vector<geometry_msgs::msg::Point> ObstacleDetector::filterPointsByHeight(
  const std::vector<geometry_msgs::msg::Point>& points) const
{
  std::vector<geometry_msgs::msg::Point> filtered_points;

  for (const auto& point : points) {
    if (point.z >= params_.min_obstacle_height && point.z <= params_.max_obstacle_height) {
      filtered_points.push_back(point);
    }
  }

  return filtered_points;
}

std::vector<geometry_msgs::msg::Point> ObstacleDetector::filterPointsByRange(
  const std::vector<geometry_msgs::msg::Point>& points) const
{
  std::vector<geometry_msgs::msg::Point> filtered_points;

  for (const auto& point : points) {
    double distance = std::sqrt(point.x * point.x + point.y * point.y);
    if (distance <= params_.max_detection_range) {
      filtered_points.push_back(point);
    }
  }

  return filtered_points;
}

std::vector<std::vector<geometry_msgs::msg::Point>> ObstacleDetector::clusterPoints(
  const std::vector<geometry_msgs::msg::Point>& points) const
{
  std::vector<std::vector<geometry_msgs::msg::Point>> clusters;
  std::vector<bool> visited(points.size(), false);

  for (size_t i = 0; i < points.size(); ++i) {
    if (visited[i]) {
      continue;
    }

    std::vector<geometry_msgs::msg::Point> cluster;
    std::vector<size_t> stack;
    stack.push_back(i);

    while (!stack.empty()) {
      size_t current_idx = stack.back();
      stack.pop_back();

      if (visited[current_idx]) {
        continue;
      }

      visited[current_idx] = true;
      cluster.push_back(points[current_idx]);

      // Find neighbors
      for (size_t j = 0; j < points.size(); ++j) {
        if (!visited[j]) {
          double distance = calculateDistance(points[current_idx], points[j]);
          if (distance <= params_.clustering_distance) {
            stack.push_back(j);
          }
        }
      }
    }

    if (!cluster.empty()) {
      clusters.push_back(cluster);
    }
  }

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

  obstacle.x = sum_x / cluster.size();
  obstacle.y = sum_y / cluster.size();

  // Calculate bounding radius
  double max_distance = 0.0;
  for (const auto& point : cluster) {
    double dx = point.x - obstacle.x;
    double dy = point.y - obstacle.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    max_distance = std::max(max_distance, distance);
  }

  obstacle.radius = std::max(0.3, max_distance + 0.2);  // Add some padding

  // For simplicity, assume static obstacles
  obstacle.velocity_x = 0.0;
  obstacle.velocity_y = 0.0;

  return obstacle;
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

bool ObstacleDetector::isPointCollisionFree(
  double x, double y,
  const std::vector<Obstacle>& obstacles,
  double safety_margin) const
{
  for (const auto& obstacle : obstacles) {
    double dx = x - obstacle.x;
    double dy = y - obstacle.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < obstacle.radius + safety_margin) {
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
