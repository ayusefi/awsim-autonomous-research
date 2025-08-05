#ifndef AWSIM_TRAJECTORY_PLANNER__OBSTACLE_DETECTOR_HPP_
#define AWSIM_TRAJECTORY_PLANNER__OBSTACLE_DETECTOR_HPP_

#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "awsim_trajectory_planner/trajectory_generator.hpp"

namespace awsim_trajectory_planner
{

struct Obstacle
{
  double x = 0.0;
  double y = 0.0;
  double radius = 0.5;  // Simple circular representation
  double velocity_x = 0.0;  // For dynamic obstacles
  double velocity_y = 0.0;
};

struct ObstacleDetectionParams
{
  double max_detection_range = 50.0;  // meters
  double min_obstacle_height = 0.2;   // meters
  double max_obstacle_height = 3.0;   // meters
  double grid_resolution = 0.2;       // meters
  double clustering_distance = 0.5;   // meters
  int min_points_per_cluster = 5;
};

class ObstacleDetector
{
public:
  explicit ObstacleDetector(const ObstacleDetectionParams& params);
  ~ObstacleDetector() = default;

  // Main obstacle detection function
  std::vector<Obstacle> detectObstacles(const sensor_msgs::msg::PointCloud2& pointcloud);

  // Collision checking
  bool isCollisionFree(const std::vector<TrajectoryPoint>& trajectory,
                       const std::vector<Obstacle>& obstacles,
                       double safety_margin) const;

  bool isPointCollisionFree(double x, double y,
                           const std::vector<Obstacle>& obstacles,
                           double safety_margin) const;

  // Generate occupancy grid for visualization
  nav_msgs::msg::OccupancyGrid generateOccupancyGrid(
    const std::vector<Obstacle>& obstacles,
    double center_x, double center_y,
    double width, double height) const;

private:
  // Point cloud processing
  std::vector<geometry_msgs::msg::Point> extractPointsFromCloud(
    const sensor_msgs::msg::PointCloud2& pointcloud) const;

  std::vector<geometry_msgs::msg::Point> filterPointsByHeight(
    const std::vector<geometry_msgs::msg::Point>& points) const;

  std::vector<geometry_msgs::msg::Point> filterPointsByRange(
    const std::vector<geometry_msgs::msg::Point>& points) const;

  // Clustering
  std::vector<std::vector<geometry_msgs::msg::Point>> clusterPoints(
    const std::vector<geometry_msgs::msg::Point>& points) const;

  Obstacle createObstacleFromCluster(
    const std::vector<geometry_msgs::msg::Point>& cluster) const;

  // Utility functions
  double calculateDistance(const geometry_msgs::msg::Point& p1,
                          const geometry_msgs::msg::Point& p2) const;

  ObstacleDetectionParams params_;
};

}  // namespace awsim_trajectory_planner

#endif  // AWSIM_TRAJECTORY_PLANNER__OBSTACLE_DETECTOR_HPP_
