#include "awsim_trajectory_planner/trajectory_planner_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rmw/qos_profiles.h>

namespace awsim_trajectory_planner
{

TrajectoryPlannerNode::TrajectoryPlannerNode(const rclcpp::NodeOptions & options)
: Node("trajectory_planner_node", options),
  base_frame_id_("base_link"),
  map_frame_id_("map")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Trajectory Planner Node");

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize parameters
  initializeParameters();

  // Initialize subscribers and publishers
  initializeSubscribersAndPublishers();

  // Initialize core components
  TrajectoryGenerationParams traj_params;
  traj_params.prediction_horizon = params_.prediction_horizon;
  traj_params.time_step = params_.time_step;
  traj_params.num_rollouts = params_.num_rollouts;
  traj_params.max_lateral_offset = params_.max_lateral_offset;
  traj_params.max_velocity = params_.max_velocity;
  traj_params.max_acceleration = params_.max_acceleration;
  traj_params.max_deceleration = params_.max_deceleration;
  traj_params.max_curvature = params_.max_curvature;
  
  trajectory_generator_ = std::make_unique<TrajectoryGenerator>(traj_params);

  // Configure obstacle detection parameters
  ObstacleDetectionParams obs_params;
  obs_params.max_detection_range = this->get_parameter("max_detection_range").as_double();
  obs_params.min_obstacle_height = this->get_parameter("min_obstacle_height").as_double();
  obs_params.max_obstacle_height = this->get_parameter("max_obstacle_height").as_double();
  obs_params.clustering_distance = this->get_parameter("cluster_tolerance").as_double();
  obs_params.min_points_per_cluster = this->get_parameter("min_points_per_cluster").as_int();
  
  obstacle_detector_ = std::make_unique<ObstacleDetector>(obs_params);

  CostEvaluationParams cost_params;
  cost_params.obstacle_cost_weight = params_.obstacle_cost_weight;
  cost_params.path_deviation_weight = params_.path_deviation_weight;
  cost_params.smoothness_weight = params_.smoothness_weight;
  cost_params.comfort_weight = params_.comfort_weight;
  cost_params.safety_margin = params_.safety_margin;
  cost_params.collision_check_resolution = params_.collision_check_resolution;
  
  cost_evaluator_ = std::make_unique<CostEvaluator>(cost_params);

  // Create planning timer (10 Hz)
  planning_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TrajectoryPlannerNode::planTrajectory, this));

  RCLCPP_INFO(this->get_logger(), "Trajectory Planner Node initialized successfully");
}

void TrajectoryPlannerNode::initializeParameters()
{
  // Declare and get parameters
  this->declare_parameter("prediction_horizon", params_.prediction_horizon);
  this->declare_parameter("time_step", params_.time_step);
  this->declare_parameter("num_rollouts", params_.num_rollouts);
  this->declare_parameter("max_lateral_offset", params_.max_lateral_offset);
  
  this->declare_parameter("max_velocity", params_.max_velocity);
  this->declare_parameter("max_acceleration", params_.max_acceleration);
  this->declare_parameter("max_deceleration", params_.max_deceleration);
  this->declare_parameter("max_curvature", params_.max_curvature);
  
  this->declare_parameter("obstacle_cost_weight", params_.obstacle_cost_weight);
  this->declare_parameter("path_deviation_weight", params_.path_deviation_weight);
  this->declare_parameter("smoothness_weight", params_.smoothness_weight);
  this->declare_parameter("comfort_weight", params_.comfort_weight);
  
  this->declare_parameter("safety_margin", params_.safety_margin);
  this->declare_parameter("collision_check_resolution", params_.collision_check_resolution);
  
  // Obstacle detection parameters
  this->declare_parameter("max_detection_range", 30.0);
  this->declare_parameter("min_obstacle_height", 0.3);
  this->declare_parameter("max_obstacle_height", 3.0);
  this->declare_parameter("cluster_tolerance", 0.5);
  this->declare_parameter("min_points_per_cluster", 10);
  
  this->declare_parameter("base_frame_id", base_frame_id_);
  this->declare_parameter("map_frame_id", map_frame_id_);

  // Get parameter values
  params_.prediction_horizon = this->get_parameter("prediction_horizon").as_double();
  params_.time_step = this->get_parameter("time_step").as_double();
  params_.num_rollouts = this->get_parameter("num_rollouts").as_int();
  params_.max_lateral_offset = this->get_parameter("max_lateral_offset").as_double();
  
  params_.max_velocity = this->get_parameter("max_velocity").as_double();
  params_.max_acceleration = this->get_parameter("max_acceleration").as_double();
  params_.max_deceleration = this->get_parameter("max_deceleration").as_double();
  params_.max_curvature = this->get_parameter("max_curvature").as_double();
  
  params_.obstacle_cost_weight = this->get_parameter("obstacle_cost_weight").as_double();
  params_.path_deviation_weight = this->get_parameter("path_deviation_weight").as_double();
  params_.smoothness_weight = this->get_parameter("smoothness_weight").as_double();
  params_.comfort_weight = this->get_parameter("comfort_weight").as_double();
  
  params_.safety_margin = this->get_parameter("safety_margin").as_double();
  params_.collision_check_resolution = this->get_parameter("collision_check_resolution").as_double();
  
  base_frame_id_ = this->get_parameter("base_frame_id").as_string();
  map_frame_id_ = this->get_parameter("map_frame_id").as_string();
}

void TrajectoryPlannerNode::initializeSubscribersAndPublishers()
{
  // Define QoS profiles to match publishers
  
  // For global path: RELIABLE, KEEP_LAST(10)
  auto reliable_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  
  // For LiDAR pointcloud: BEST_EFFORT, KEEP_LAST(5)
  auto best_effort_qos = rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  
  // Subscribers with appropriate QoS profiles
  global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/planning/global_path", reliable_qos,
    std::bind(&TrajectoryPlannerNode::globalPathCallback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/odometry", reliable_qos,
    std::bind(&TrajectoryPlannerNode::poseWithCovarianceCallback, this, std::placeholders::_1));

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar/concatenated/pointcloud", best_effort_qos,
    std::bind(&TrajectoryPlannerNode::pointCloudCallback, this, std::placeholders::_1));

  // Publishers - using reliable QoS for trajectory planning outputs
  local_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/planning/local_trajectory", reliable_qos);

  candidate_trajectories_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/planning/candidate_trajectories", reliable_qos);

  visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/trajectory_visualization", reliable_qos);
}

void TrajectoryPlannerNode::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  global_path_ = msg;
  RCLCPP_DEBUG(this->get_logger(), "Received global path with %zu poses", msg->poses.size());
}

void TrajectoryPlannerNode::poseWithCovarianceCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  previous_pose_ = current_pose_;
  current_pose_ = msg;
  RCLCPP_DEBUG(this->get_logger(), "Received pose with covariance update");
}

void TrajectoryPlannerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_pointcloud_ = msg;
  RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);
}

void TrajectoryPlannerNode::planTrajectory()
{
  // Check if we have all required data
  if (!global_path_ || !current_pose_ || !latest_pointcloud_) {
    RCLCPP_DEBUG(this->get_logger(), "Missing required data for trajectory planning");
    return;
  }

  try {
    // Convert current state from PoseWithCovarianceStamped
    geometry_msgs::msg::PoseStamped current_pose_stamped;
    current_pose_stamped.header = current_pose_->header;
    current_pose_stamped.pose = current_pose_->pose.pose;
    
    // Create a twist with estimated velocity
    geometry_msgs::msg::Twist current_twist;
    double estimated_velocity = estimateVelocity();
    current_twist.linear.x = estimated_velocity;
    current_twist.linear.y = 0.0;
    current_twist.angular.z = 0.0;
    
    VehicleState current_state = trajectory_generator_->poseAndTwistToVehicleState(
      current_pose_stamped, current_twist);

    RCLCPP_DEBUG(this->get_logger(), "Current state: x=%.2f, y=%.2f, yaw=%.2f, v=%.2f", 
                 current_state.x, current_state.y, current_state.yaw, current_state.velocity);

    // Generate candidate trajectories
    auto candidate_trajectories = trajectory_generator_->generateCandidateTrajectories(
      *global_path_, current_state);

    RCLCPP_INFO(this->get_logger(), "Generated %zu candidate trajectories", candidate_trajectories.size());

    if (candidate_trajectories.empty()) {
      RCLCPP_WARN(this->get_logger(), "No candidate trajectories generated");
      RCLCPP_WARN(this->get_logger(), "Global path size: %zu poses", global_path_->poses.size());
      RCLCPP_WARN(this->get_logger(), "Current state: x=%.2f, y=%.2f, yaw=%.2f, v=%.2f", 
                   current_state.x, current_state.y, current_state.yaw, current_state.velocity);
      return;
    }

    // Detect obstacles
    auto obstacles = obstacle_detector_->detectObstacles(*latest_pointcloud_);
    
    RCLCPP_INFO(this->get_logger(), "Detected %zu obstacles", obstacles.size());

    // Evaluate trajectories
    auto costs = cost_evaluator_->evaluateTrajectories(
      candidate_trajectories, *global_path_, obstacles);

    // Select best trajectory
    int best_idx = cost_evaluator_->selectBestTrajectory(costs);
    
    if (best_idx < 0 || best_idx >= static_cast<int>(candidate_trajectories.size())) {
      RCLCPP_WARN(this->get_logger(), "No valid trajectory found");
      return;
    }

    // Convert to ROS messages
    std::vector<nav_msgs::msg::Path> candidate_paths;
    for (const auto& traj : candidate_trajectories) {
      candidate_paths.push_back(
        trajectory_generator_->trajectoryToPath(traj, map_frame_id_));
    }

    nav_msgs::msg::Path selected_path = 
      trajectory_generator_->trajectoryToPath(candidate_trajectories[best_idx], map_frame_id_);

    // Publish results
    publishTrajectories(candidate_paths, selected_path);
    publishVisualization(candidate_paths, selected_path);

    RCLCPP_DEBUG(this->get_logger(), 
      "Published trajectory with cost: %.2f (obstacles: %.2f, deviation: %.2f, smoothness: %.2f, comfort: %.2f)",
      costs[best_idx].total_cost, costs[best_idx].obstacle_cost, 
      costs[best_idx].path_deviation_cost, costs[best_idx].smoothness_cost, 
      costs[best_idx].comfort_cost);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error in trajectory planning: %s", e.what());
  }
}

void TrajectoryPlannerNode::publishTrajectories(
  const std::vector<nav_msgs::msg::Path>& candidate_trajectories,
  const nav_msgs::msg::Path& selected_trajectory)
{
  local_trajectory_pub_->publish(selected_trajectory);
  if (!candidate_trajectories.empty()) {
    for (size_t i = 0; i < candidate_trajectories.size(); ++i) {
      candidate_trajectories_pub_->publish(candidate_trajectories[i]);  // Requires QoS adjustment or topic change
    }
  }
}

void TrajectoryPlannerNode::publishVisualization(
  const std::vector<nav_msgs::msg::Path>& candidate_trajectories,
  const nav_msgs::msg::Path& selected_trajectory)
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = map_frame_id_;
  clear_marker.header.stamp = this->now();
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  // Visualize candidate trajectories
  for (size_t i = 0; i < candidate_trajectories.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = this->now();
    marker.ns = "candidate_trajectories";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.scale.x = 0.05;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 0.5;

    for (const auto& pose : candidate_trajectories[i].poses) {
      marker.points.push_back(pose.pose.position);
    }
    
    marker_array.markers.push_back(marker);
  }

  // Visualize selected trajectory
  visualization_msgs::msg::Marker selected_marker;
  selected_marker.header.frame_id = map_frame_id_;
  selected_marker.header.stamp = this->now();
  selected_marker.ns = "selected_trajectory";
  selected_marker.id = 0;
  selected_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  selected_marker.action = visualization_msgs::msg::Marker::ADD;
  
  selected_marker.scale.x = 0.1;
  selected_marker.color.r = 0.0;
  selected_marker.color.g = 1.0;
  selected_marker.color.b = 0.0;
  selected_marker.color.a = 1.0;

  for (const auto& pose : selected_trajectory.poses) {
    selected_marker.points.push_back(pose.pose.position);
  }
  
  marker_array.markers.push_back(selected_marker);

  visualization_pub_->publish(marker_array);
}

double TrajectoryPlannerNode::estimateVelocity()
{
  if (!current_pose_ || !previous_pose_) {
    return 0.0;
  }

  // Calculate time difference
  double dt = (current_pose_->header.stamp.sec - previous_pose_->header.stamp.sec) +
              (current_pose_->header.stamp.nanosec - previous_pose_->header.stamp.nanosec) * 1e-9;

  if (dt <= 0.0) {
    return 0.0;
  }

  // Calculate distance difference
  double dx = current_pose_->pose.pose.position.x - previous_pose_->pose.pose.position.x;
  double dy = current_pose_->pose.pose.position.y - previous_pose_->pose.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // Calculate velocity
  double velocity = distance / dt;

  // Apply some basic filtering/limiting
  velocity = std::min(velocity, params_.max_velocity);

  return velocity;
}

}  // namespace awsim_trajectory_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(awsim_trajectory_planner::TrajectoryPlannerNode)
