#pragma once

#include "awsim_trajectory_planner/trajectory_generator.hpp"
#include "awsim_trajectory_planner/obstacle_detector.hpp"
#include "awsim_trajectory_planner/cost_evaluator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <chrono>

namespace awsim_trajectory_planner
{

struct TrajectoryPlannerParams
{
  double prediction_horizon = 10.0;
  double time_step = 0.1;
  int num_rollouts = 15;
  double max_lateral_offset = 2.0;
  
  double max_velocity = 5.0;
  double max_acceleration = 1.0;
  double max_deceleration = 2.0;
  double max_curvature = 0.5;
  
  double obstacle_cost_weight = 1.0;
  double path_deviation_weight = 1.0;
  double smoothness_weight = 0.5;
  double comfort_weight = 0.3;
  
  double safety_margin = 1.0;
  double collision_check_resolution = 0.1;
};

class TrajectoryPlannerNode : public rclcpp::Node
{
public:
  explicit TrajectoryPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void initializeParameters();
  void initializeSubscribersAndPublishers();
  void initializeComponents();
  
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void poseWithCovarianceCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  void planTrajectory();
  void publishTrajectories(const nav_msgs::msg::Path& selected_trajectory);
  void publishVisualization(
    const std::vector<nav_msgs::msg::Path>& candidate_trajectories,
    const std::vector<TrajectoryCost>& costs,
    int selected_idx,
    const std::vector<Obstacle>& obstacles);
  

  geometry_msgs::msg::Point createPoint(double x, double y, double z) const;

  nav_msgs::msg::Path createCurrentPosePath();
  double estimateVelocity();

  // Parameters
  TrajectoryPlannerParams params_;
  std::string base_frame_id_;
  std::string map_frame_id_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;

  // Core components
  std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
  std::unique_ptr<ObstacleDetector> obstacle_detector_;
  std::unique_ptr<CostEvaluator> cost_evaluator_;

  // Timer
  rclcpp::TimerBase::SharedPtr planning_timer_;

  // Data storage
  nav_msgs::msg::Path::SharedPtr global_path_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr previous_pose_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
};

}  // namespace awsim_trajectory_planner