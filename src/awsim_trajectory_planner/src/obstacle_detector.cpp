#include "awsim_trajectory_planner/obstacle_detector.hpp"
#include <multi_object_tracker_msgs/msg/tracked_object.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>

namespace awsim_trajectory_planner {

std::vector<Obstacle> ObstacleDetector::obstaclesFromTrackedObjects(
    const std::vector<multi_object_tracker_msgs::msg::TrackedObject>& tracked_objects) const {
  std::vector<Obstacle> obstacles;

  std::cout << "tracked objects size: " << tracked_objects.size() << std::endl;

  // Direct conversion without any filtering or processing
  for (const auto& obj : tracked_objects) {
    Obstacle obs;
    obs.center_x = obj.position.x;
    obs.center_y = obj.position.y;
    obs.width = obj.dimensions.x;
    obs.height = obj.dimensions.y;
    
    // Calculate orientation from quaternion (yaw angle)
    // Standard formula: yaw = atan2(2(w*z + x*y), 1 - 2(y^2 + z^2))
    double siny_cosp = 2.0 * (obj.orientation.w * obj.orientation.z + obj.orientation.x * obj.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (obj.orientation.y * obj.orientation.y + obj.orientation.z * obj.orientation.z);
    obs.orientation = std::atan2(siny_cosp, cosy_cosp);
    
    // Store original quaternion components for visualization (to match tracker)
    obs.original_quat_z = obj.orientation.z;
    obs.original_quat_w = obj.orientation.w;
    
    std::cout << "Object ID " << obj.id << " quaternion: w=" << obj.orientation.w 
              << " x=" << obj.orientation.x << " y=" << obj.orientation.y << " z=" << obj.orientation.z
              << " -> yaw=" << (obs.orientation * 180.0 / M_PI) << " degrees" << std::endl;
    
    obs.velocity_x = obj.velocity.x;
    obs.velocity_y = obj.velocity.y;
    obs.updateAABB();
    
    obstacles.push_back(obs);
  }
  
  return obstacles;
}

} // namespace awsim_trajectory_planner
#include "awsim_trajectory_planner/obstacle_detector.hpp"
#include "awsim_trajectory_planner/trajectory_generator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

namespace awsim_trajectory_planner
{

ObstacleDetector::ObstacleDetector(const ObstacleDetectionParams& params, rclcpp::Node* node)
: params_(params)
{
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

}  // namespace awsim_trajectory_planner
