#include "awsim_path_planner/path_planner_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rmw/qos_profiles.h>
#include <cmath>

namespace awsim_path_planner
{

PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions & options)
: Node("path_planner_node", options),
  has_current_pose_(false),
  has_goal_pose_(false),
  has_map_pointcloud_(false),
  has_raw_pointcloud_(false)
{
  RCLCPP_INFO(this->get_logger(), "Initializing AWSIM Path Planner Node");
  
  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize parameters
  initialize_parameters();
  
  // Setup ROS communication
  setup_subscribers_and_publishers();
  
  // Initialize planning components
  astar_planner_ = std::make_unique<AStarPlanner>(this);
  rrt_star_planner_ = std::make_unique<RRTStarPlanner>(this);
  hd_map_manager_ = std::make_unique<HDMapManager>(this);
  
  // Configure planners
  astar_planner_->set_grid_resolution(grid_resolution_);
  
  // Load HD map if specified
  if (!hd_map_path_.empty()) {
    if (hd_map_manager_->load_map(hd_map_path_)) {
      RCLCPP_INFO(this->get_logger(), "Successfully loaded HD map from: %s", hd_map_path_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to load HD map from: %s", hd_map_path_.c_str());
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized successfully");
  RCLCPP_INFO(this->get_logger(), "Planning algorithm: %s", planning_algorithm_.c_str());
  RCLCPP_INFO(this->get_logger(), "Grid resolution: %.2f m", grid_resolution_);
  RCLCPP_INFO(this->get_logger(), "Max planning range: %.2f m", max_planning_range_);
}

void PathPlannerNode::initialize_parameters()
{
  // Declare and get parameters with defaults
  this->declare_parameter("planning_algorithm", "astar");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_link_frame", "base_link");
  this->declare_parameter("hd_map_path", "");
  this->declare_parameter("grid_resolution", 0.5);
  this->declare_parameter("planning_timeout", 5.0);
  this->declare_parameter("max_planning_range", 500.0);
  this->declare_parameter("use_hd_map_constraints", true);
  this->declare_parameter("visualize_search_space", true);
  
  // A* specific parameters
  this->declare_parameter("astar.heuristic_weight", 1.0);
  this->declare_parameter("astar.search_radius", 100.0);
  this->declare_parameter("astar.obstacle_inflation_radius", 1.0);
  
  // RRT* specific parameters
  this->declare_parameter("rrt_star.max_iterations", 5000);
  this->declare_parameter("rrt_star.step_size", 2.0);
  this->declare_parameter("rrt_star.goal_tolerance", 2.0);
  this->declare_parameter("rrt_star.rewiring_radius", 10.0);
  
  // Ground filtering parameters
  this->declare_parameter("ground_filter.height_threshold", 0.3);
  this->declare_parameter("ground_filter.angle_threshold", 15.0);
  this->declare_parameter("dynamic_obstacle_max_range", 50.0);
  
  // Get parameter values
  planning_algorithm_ = this->get_parameter("planning_algorithm").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_link_frame_ = this->get_parameter("base_link_frame").as_string();
  hd_map_path_ = this->get_parameter("hd_map_path").as_string();
  grid_resolution_ = this->get_parameter("grid_resolution").as_double();
  planning_timeout_ = this->get_parameter("planning_timeout").as_double();
  max_planning_range_ = this->get_parameter("max_planning_range").as_double();
  use_hd_map_constraints_ = this->get_parameter("use_hd_map_constraints").as_bool();
  visualize_search_space_ = this->get_parameter("visualize_search_space").as_bool();
  
  // Get ground filtering parameters
  ground_filter_height_threshold_ = this->get_parameter("ground_filter.height_threshold").as_double();
  ground_filter_angle_threshold_ = this->get_parameter("ground_filter.angle_threshold").as_double();
  dynamic_obstacle_max_range_ = this->get_parameter("dynamic_obstacle_max_range").as_double();
}

void PathPlannerNode::setup_subscribers_and_publishers()
{
  // Create QoS profiles for different data types
  auto reliable_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  auto sensor_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  
  // Subscribers
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/pose_with_covariance", reliable_qos,
    std::bind(&PathPlannerNode::current_pose_callback, this, std::placeholders::_1));
  
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/planning/goal_pose", reliable_qos,
    std::bind(&PathPlannerNode::goal_pose_callback, this, std::placeholders::_1));
  
  // Subscribe to map point cloud (static obstacles) - RELIABLE QoS
  map_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/localization/map", reliable_qos,
    std::bind(&PathPlannerNode::map_pointcloud_callback, this, std::placeholders::_1));
  
  // Subscribe to raw point cloud (dynamic obstacles) - BEST_EFFORT QoS to match AWSIM LiDAR
  raw_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar/top/pointcloud_raw", sensor_qos,
    std::bind(&PathPlannerNode::raw_pointcloud_callback, this, std::placeholders::_1));
  
  // Publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/path", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/visualization_markers", 10);
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/planning/occupancy_grid", 10);
  
  RCLCPP_INFO(this->get_logger(), "Publishers and subscribers initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribed to map: /localization/map");
  RCLCPP_INFO(this->get_logger(), "Subscribed to raw point cloud: /sensing/lidar/top/pointcloud_raw");
}

void PathPlannerNode::current_pose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Convert PoseWithCovarianceStamped to PoseStamped
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;
  
  // Transform to map frame if necessary
  if (current_pose_.header.frame_id != map_frame_) {
    try {
      current_pose_ = transform_pose(current_pose_, map_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform pose to map frame: %s", ex.what());
      return;
    }
  }
  
  has_current_pose_ = true;
  
  // Attempt planning if we have both current pose and goal
  if (has_current_pose_ && has_goal_pose_) {
    plan_path(current_pose_, goal_pose_);
  }
}

void PathPlannerNode::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = *msg;
  
  // Transform to map frame if necessary
  if (goal_pose_.header.frame_id != map_frame_) {
    try {
      goal_pose_ = transform_pose(goal_pose_, map_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform goal to map frame: %s", ex.what());
      return;
    }
  }
  
  has_goal_pose_ = true;
  
  RCLCPP_INFO(this->get_logger(), "Received goal pose at (%.2f, %.2f)",
              goal_pose_.pose.position.x, goal_pose_.pose.position.y);
  
  // Attempt planning if we have both current pose and goal
  if (has_current_pose_ && has_goal_pose_) {
    plan_path(current_pose_, goal_pose_);
  }
}

void PathPlannerNode::map_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received map point cloud with %d points", msg->width * msg->height);
  
  // Filter ground points from map point cloud too - we only want obstacles!
  auto filtered_msg = filter_ground_points(msg, true);  // true = map data, no range limit
  map_pointcloud_ = filtered_msg;
  has_map_pointcloud_ = true;
  
  // Update combined point cloud when we get new map data
  if (has_raw_pointcloud_) {
    combined_pointcloud_ = combine_pointclouds(map_pointcloud_, raw_pointcloud_);
  }
  
  RCLCPP_INFO(this->get_logger(), "Map point cloud filtered and stored");
}

void PathPlannerNode::raw_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received raw point cloud with %d points", msg->width * msg->height);
  
  // Filter ground points from raw point cloud
  auto filtered_msg = filter_ground_points(msg, false);  // false = sensor data, apply range limit
  raw_pointcloud_ = filtered_msg;
  has_raw_pointcloud_ = true;
  
  // Update combined point cloud when we get new raw data
  if (has_map_pointcloud_) {
    combined_pointcloud_ = combine_pointclouds(map_pointcloud_, raw_pointcloud_);
  }
  
  RCLCPP_INFO(this->get_logger(), "Raw point cloud filtered and stored");
}

bool PathPlannerNode::plan_path(const geometry_msgs::msg::PoseStamped & start, 
                                const geometry_msgs::msg::PoseStamped & goal)
{
  auto start_time = this->get_clock()->now();
  
  RCLCPP_INFO(this->get_logger(), "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
              start.pose.position.x, start.pose.position.y,
              goal.pose.position.x, goal.pose.position.y);
  
  // Check if goal is within planning range
  double distance = std::sqrt(
    std::pow(goal.pose.position.x - start.pose.position.x, 2) +
    std::pow(goal.pose.position.y - start.pose.position.y, 2));
  
  if (distance > max_planning_range_) {
    RCLCPP_WARN(this->get_logger(), "Goal is too far (%.2f m > %.2f m)", distance, max_planning_range_);
    return false;
  }
  
  std::vector<geometry_msgs::msg::PoseStamped> planned_path;
  
  // Check if we have required data
  if (!has_map_pointcloud_ && !has_raw_pointcloud_) {
    RCLCPP_WARN(this->get_logger(), "No point cloud data available for planning");
    return false;
  }
  
  // Use combined point cloud if available, otherwise use what we have
  sensor_msgs::msg::PointCloud2::SharedPtr planning_cloud;
  if (combined_pointcloud_) {
    planning_cloud = combined_pointcloud_;
    RCLCPP_DEBUG(this->get_logger(), "Using combined point cloud for planning");
  } else if (has_map_pointcloud_) {
    planning_cloud = map_pointcloud_;
    RCLCPP_DEBUG(this->get_logger(), "Using map point cloud for planning");
  } else {
    planning_cloud = raw_pointcloud_;
    RCLCPP_DEBUG(this->get_logger(), "Using raw point cloud for planning");
  }
  
  try {
    // Plan with selected algorithm
    if (planning_algorithm_ == "astar") {
      planned_path = astar_planner_->plan_path(start, goal, planning_cloud);
    } else if (planning_algorithm_ == "rrt_star") {
      planned_path = rrt_star_planner_->plan_path(start, goal, planning_cloud);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown planning algorithm: %s", planning_algorithm_.c_str());
      return false;
    }
    
    // Apply HD map constraints if enabled and available
    if (use_hd_map_constraints_ && hd_map_manager_->is_map_loaded()) {
      if (hd_map_manager_->is_path_valid(planned_path)) {
        planned_path = hd_map_manager_->optimize_path_to_lanes(planned_path);
        RCLCPP_INFO(this->get_logger(), "Path optimized using HD map constraints");
      } else {
        RCLCPP_WARN(this->get_logger(), "Planned path violates HD map constraints");
      }
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Path planning failed: %s", e.what());
    return false;
  }
  
  auto end_time = this->get_clock()->now();
  auto planning_duration = (end_time - start_time).seconds();
  
  if (planned_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found after %.3f seconds", planning_duration);
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "Path planned successfully in %.3f seconds (%zu waypoints)",
              planning_duration, planned_path.size());
  
  // Publish results
  publish_path(planned_path);
  
  if (visualize_search_space_) {
    publish_visualization_markers();
  }
  
  return true;
}

void PathPlannerNode::publish_path(const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = map_frame_;
  path_msg.poses = path;
  
  path_pub_->publish(path_msg);
}

void PathPlannerNode::publish_visualization_markers()
{
  visualization_msgs::msg::MarkerArray markers;
  
  // Get algorithm-specific visualizations
  if (planning_algorithm_ == "astar") {
    auto astar_markers = astar_planner_->get_search_visualization();
    markers.markers.insert(markers.markers.end(), 
                          astar_markers.markers.begin(), 
                          astar_markers.markers.end());
    
    // Publish occupancy grid
    auto grid = astar_planner_->get_occupancy_grid();
    grid_pub_->publish(grid);
  } else if (planning_algorithm_ == "rrt_star") {
    auto rrt_markers = rrt_star_planner_->get_tree_visualization();
    markers.markers.insert(markers.markers.end(), 
                          rrt_markers.markers.begin(), 
                          rrt_markers.markers.end());
  }
  
  // Add HD map visualization if available
  if (hd_map_manager_->is_map_loaded()) {
    auto map_markers = hd_map_manager_->get_map_visualization();
    markers.markers.insert(markers.markers.end(), 
                          map_markers.markers.begin(), 
                          map_markers.markers.end());
  }
  
  marker_pub_->publish(markers);
}

geometry_msgs::msg::PoseStamped PathPlannerNode::transform_pose(
  const geometry_msgs::msg::PoseStamped & pose_in,
  const std::string & target_frame)
{
  geometry_msgs::msg::PoseStamped pose_out;
  
  try {
    tf_buffer_->transform(pose_in, pose_out, target_frame, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    throw;
  }
  
  return pose_out;
}

sensor_msgs::msg::PointCloud2::SharedPtr PathPlannerNode::filter_ground_points(
  const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud, bool is_map_data)
{
  if (!pointcloud) {
    return nullptr;
  }
  
  // Create output point cloud
  auto filtered_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  *filtered_cloud = *pointcloud;
  
  // Convert to PCL for processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::fromROSMsg(*pointcloud, *pcl_cloud);
  
  // Simple ground filtering based on height and angle
  for (const auto& point : pcl_cloud->points) {
    // Skip invalid points
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
      continue;
    }
    
    // Calculate distance from origin
    double distance = std::sqrt(point.x * point.x + point.y * point.y);
    
    // Skip points too far away ONLY for raw sensor data (not for map data)
    if (!is_map_data && distance > dynamic_obstacle_max_range_) {
      continue;
    }
    
    // Filter based on height (assuming ground is around z=0 with some tolerance)
    if (point.z > ground_filter_height_threshold_) {
      // Calculate angle from horizontal plane
      double angle = std::atan2(point.z, distance) * 180.0 / M_PI;
      
      // Keep points that are not part of the ground plane
      if (std::abs(angle) > ground_filter_angle_threshold_) {
        filtered_pcl_cloud->points.push_back(point);
      }
    }
  }
  
  // Convert back to ROS message
  filtered_pcl_cloud->width = filtered_pcl_cloud->points.size();
  filtered_pcl_cloud->height = 1;
  filtered_pcl_cloud->is_dense = false;
  
  pcl::toROSMsg(*filtered_pcl_cloud, *filtered_cloud);
  filtered_cloud->header = pointcloud->header;
  
  RCLCPP_INFO(this->get_logger(), "Ground filtering: %zu -> %zu points (filtered %.1f%%)",
               pcl_cloud->points.size(), filtered_pcl_cloud->points.size(),
               100.0 * (1.0 - double(filtered_pcl_cloud->points.size()) / double(pcl_cloud->points.size())));
  
  return filtered_cloud;
}

sensor_msgs::msg::PointCloud2::SharedPtr PathPlannerNode::combine_pointclouds(
  const sensor_msgs::msg::PointCloud2::SharedPtr & map_cloud,
  const sensor_msgs::msg::PointCloud2::SharedPtr & dynamic_cloud)
{
  if (!map_cloud && !dynamic_cloud) {
    return nullptr;
  }
  
  if (!map_cloud) {
    return dynamic_cloud;
  }
  
  if (!dynamic_cloud) {
    return map_cloud;
  }
  
  // Convert both clouds to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr combined_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::fromROSMsg(*map_cloud, *map_pcl);
  pcl::fromROSMsg(*dynamic_cloud, *dynamic_pcl);
  
  // Combine point clouds
  *combined_pcl = *map_pcl;
  *combined_pcl += *dynamic_pcl;
  
  // Convert back to ROS message
  auto combined_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*combined_pcl, *combined_cloud);
  combined_cloud->header = map_cloud->header;
  
  RCLCPP_DEBUG(this->get_logger(), "Combined point clouds: %zu + %zu = %zu points",
               map_pcl->points.size(), dynamic_pcl->points.size(), combined_pcl->points.size());
  
  return combined_cloud;
}

}  // namespace awsim_path_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(awsim_path_planner::PathPlannerNode)
