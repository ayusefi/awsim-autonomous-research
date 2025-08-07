
#include <multi_object_tracker_msgs/msg/tracked_object.hpp>
#ifndef AWSIM_TRAJECTORY_PLANNER__OBSTACLE_DETECTOR_HPP_
#define AWSIM_TRAJECTORY_PLANNER__OBSTACLE_DETECTOR_HPP_
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "awsim_trajectory_planner/trajectory_generator.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

namespace awsim_trajectory_planner
{

struct Obstacle
{
  // Oriented Bounding Box representation
  double center_x = 0.0;    // Center X coordinate
  double center_y = 0.0;    // Center Y coordinate
  double width = 0.0;       // Width along local X axis
  double height = 0.0;      // Height along local Y axis
  double orientation = 0.0; // Rotation angle in radians (yaw)
  
  // Store original quaternion components for visualization consistency with tracker
  double original_quat_z = 0.0;
  double original_quat_w = 1.0;
  
  // Axis-aligned bounding box (for backward compatibility and quick checks)
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  
  // For dynamic obstacles (future use)
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  
  // Helper functions for backward compatibility
  inline double getWidth() const { return width; }
  inline double getHeight() const { return height; }
  inline double getCenterX() const { return center_x; }
  inline double getCenterY() const { return center_y; }
  
  // Get the four corners of the oriented bounding box
  std::vector<std::pair<double, double>> getCorners() const {
    double cos_theta = std::cos(orientation);
    double sin_theta = std::sin(orientation);
    double half_width = width / 2.0;
    double half_height = height / 2.0;
    
    std::vector<std::pair<double, double>> corners(4);
    
    // Local corners (before rotation)
    std::vector<std::pair<double, double>> local_corners = {
      {-half_width, -half_height}, // Bottom-left
      { half_width, -half_height}, // Bottom-right
      { half_width,  half_height}, // Top-right
      {-half_width,  half_height}  // Top-left
    };
    
    // Transform to global coordinates
    for (size_t i = 0; i < 4; ++i) {
      double local_x = local_corners[i].first;
      double local_y = local_corners[i].second;
      
      corners[i].first = center_x + local_x * cos_theta - local_y * sin_theta;
      corners[i].second = center_y + local_x * sin_theta + local_y * cos_theta;
    }
    
    return corners;
  }
  
  // Check if a point is inside the oriented bounding box
  bool containsPoint(double px, double py) const {
    // Transform point to local coordinate system
    double dx = px - center_x;
    double dy = py - center_y;
    double cos_theta = std::cos(-orientation);  // Negative for inverse rotation
    double sin_theta = std::sin(-orientation);
    
    double local_x = dx * cos_theta - dy * sin_theta;
    double local_y = dx * sin_theta + dy * cos_theta;
    
    // Check if point is within local bounding box
    return (std::abs(local_x) <= width / 2.0) && (std::abs(local_y) <= height / 2.0);
  }
  
  // Calculate minimum distance from point to oriented bounding box
  double distanceToPoint(double px, double py) const {
    // If point is inside, distance is 0
    if (containsPoint(px, py)) {
      return 0.0;
    }
    
    // Get all corners
    auto corners = getCorners();
    
    double min_distance = std::numeric_limits<double>::max();
    
    // Check distance to each edge of the oriented box
    for (size_t i = 0; i < 4; ++i) {
      size_t next_i = (i + 1) % 4;
      double edge_dist = distanceToLineSegment(
        px, py,
        corners[i].first, corners[i].second,
        corners[next_i].first, corners[next_i].second
      );
      min_distance = std::min(min_distance, edge_dist);
    }
    
    return min_distance;
  }
  
  // Update axis-aligned bounding box from oriented box (for quick culling)
  void updateAABB() {
    auto corners = getCorners();
    
    x_min = x_max = corners[0].first;
    y_min = y_max = corners[0].second;
    
    for (const auto& corner : corners) {
      x_min = std::min(x_min, corner.first);
      x_max = std::max(x_max, corner.first);
      y_min = std::min(y_min, corner.second);
      y_max = std::max(y_max, corner.second);
    }
  }

private:
  // Helper function to calculate distance from point to line segment
  double distanceToLineSegment(double px, double py, double x1, double y1, double x2, double y2) const {
    double dx = x2 - x1;
    double dy = y2 - y1;
    double length_sq = dx * dx + dy * dy;
    
    if (length_sq == 0.0) {
      // Degenerate case: line segment is a point
      double dpx = px - x1;
      double dpy = py - y1;
      return std::sqrt(dpx * dpx + dpy * dpy);
    }
    
    // Project point onto line segment
    double t = ((px - x1) * dx + (py - y1) * dy) / length_sq;
    t = std::max(0.0, std::min(1.0, t));  // Clamp to [0, 1]
    
    double closest_x = x1 + t * dx;
    double closest_y = y1 + t * dy;
    
    double dpx = px - closest_x;
    double dpy = py - closest_y;
    return std::sqrt(dpx * dpx + dpy * dpy);
  }
};

struct ObstacleDetectionParams
{
  double max_detection_range = 100.0;  // meters - increased for global coordinates
  double min_obstacle_height = -2.0;   // meters - allow negative heights
  double max_obstacle_height = 10.0;   // meters - increased for higher obstacles
  double grid_resolution = 0.2;       // meters
  double clustering_distance = 5.0;   // meters - increased for global coordinates
  int min_points_per_cluster = 2;      // reduced minimum threshold
};

class ObstacleDetector
{
public:
  // Create obstacles from tracked objects
  std::vector<Obstacle> obstaclesFromTrackedObjects(const std::vector<multi_object_tracker_msgs::msg::TrackedObject>& tracked_objects) const;
  explicit ObstacleDetector(const ObstacleDetectionParams& params);
  ObstacleDetector(const ObstacleDetectionParams& params, rclcpp::Node* node);
  ~ObstacleDetector() = default;
  
  // Main obstacle detection function - updated to accept vehicle position
  std::vector<Obstacle> detectObstacles(
    const sensor_msgs::msg::PointCloud2& pointcloud,
    const geometry_msgs::msg::Point& vehicle_position) const;
  
  // Collision checking
  bool isCollisionFree(
    const std::vector<TrajectoryPoint>& trajectory,
    const std::vector<Obstacle>& obstacles,
    double safety_margin) const;
    
  bool isPointCollisionFree(
    double x, double y,
    const std::vector<Obstacle>& obstacles,
    double safety_margin) const;
    
  // Generate occupancy grid for visualization
  nav_msgs::msg::OccupancyGrid generateOccupancyGrid(
    const std::vector<Obstacle>& obstacles,
    double center_x, double center_y,
    double width, double height) const;

private:
  // Point cloud processing - updated to accept reference point
  std::vector<geometry_msgs::msg::Point> extractPointsFromCloud(
    const sensor_msgs::msg::PointCloud2& pointcloud,
    const geometry_msgs::msg::Point& reference_point) const;
    
  // Clustering
  std::vector<std::vector<geometry_msgs::msg::Point>> clusterPoints(
    const std::vector<geometry_msgs::msg::Point>& points) const;
    
  Obstacle createObstacleFromCluster(
    const std::vector<geometry_msgs::msg::Point>& cluster) const;
    
  // Utility functions
  double calculateDistance(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2) const;
    
  // Debug function for point cloud analysis
  void debugPointCloud(const sensor_msgs::msg::PointCloud2& cloud_msg) const;

  ObstacleDetectionParams params_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace awsim_trajectory_planner

#endif  // AWSIM_TRAJECTORY_PLANNER__OBSTACLE_DETECTOR_HPP_