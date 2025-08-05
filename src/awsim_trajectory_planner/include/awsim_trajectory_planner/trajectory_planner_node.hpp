#ifndef AWSIM_TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_NODE_HPP_
#define AWSIM_TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "awsim_trajectory_planner/trajectory_generator.hpp"
#include "awsim_trajectory_planner/obstacle_detector.hpp"
#include "awsim_trajectory_planner/cost_evaluator.hpp"

namespace awsim_trajectory_planner
{

struct TrajectoryPlannerParams
{
  // Trajectory generation parameters
  double prediction_horizon = 5.0;  // seconds
  double time_step = 0.1;           // seconds
  int num_rollouts = 7;             // number of candidate trajectories
  double max_lateral_offset = 3.0;  // meters
  
  // Vehicle constraints
  double max_velocity = 15.0;       // m/s
  double max_acceleration = 2.0;    // m/s^2
  double max_deceleration = -4.0;   // m/s^2
  double max_curvature = 0.2;       // 1/m
  
  // Cost function weights
  double obstacle_cost_weight = 1000.0;
  double path_deviation_weight = 10.0;
  double smoothness_weight = 1.0;
  double comfort_weight = 5.0;
  
  // Safety parameters
  double safety_margin = 1.5;       // meters
  double collision_check_resolution = 0.2;  // meters
};

class TrajectoryPlannerNode : public rclcpp::Node
{
public:
  explicit TrajectoryPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TrajectoryPlannerNode() = default;

private:
  void initializeParameters();
  void initializeSubscribersAndPublishers();
  
  // Callback functions
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void poseWithCovarianceCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  // Main planning function
  void planTrajectory();
  
  // Utility functions
  void publishTrajectories(const std::vector<nav_msgs::msg::Path>& candidate_trajectories,
                          const nav_msgs::msg::Path& selected_trajectory);
  void publishVisualization(const std::vector<nav_msgs::msg::Path>& candidate_trajectories,
                           const nav_msgs::msg::Path& selected_trajectory);

  // ROS 2 communication
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_trajectories_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  
  rclcpp::TimerBase::SharedPtr planning_timer_;
  
  // TF
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  
  // Core components
  std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<ObstacleDetector> obstacle_detector_;
  std::unique_ptr<CostEvaluator> cost_evaluator_;
  
  // State variables
  nav_msgs::msg::Path::SharedPtr global_path_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr previous_pose_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
  
  // Velocity estimation
  double estimateVelocity();
  
  // Parameters
  TrajectoryPlannerParams params_;
  
  // Frame IDs
  std::string base_frame_id_;
  std::string map_frame_id_;
};

}  // namespace awsim_trajectory_planner

#endif  // AWSIM_TRAJECTORY_PLANNER__TRAJECTORY_PLANNER_NODE_HPP_
