#include "awsim_path_planner/path_planner_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <rmw/qos_profiles.h>
#include <cmath>
#include <set>

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
  
  // Create timer for periodic occupancy grid publishing
  grid_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),  // Publish every 1 second
    std::bind(&PathPlannerNode::publish_occupancy_grid, this));
  
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
  
  // Ground filtering parameters - enhanced for advanced filtering
  this->declare_parameter("ground_filter.height_threshold", 0.3);
  this->declare_parameter("ground_filter.angle_threshold", 15.0);
  this->declare_parameter("ground_filter.ransac_distance_threshold", 0.1);
  this->declare_parameter("ground_filter.ransac_max_iterations", 1000);
  this->declare_parameter("ground_filter.min_ground_points", 100);
  this->declare_parameter("ground_filter.voxel_leaf_size", 0.05);
  this->declare_parameter("ground_filter.use_progressive_morphological", true);
  this->declare_parameter("ground_filter.pmf_max_window_size", 33);
  this->declare_parameter("ground_filter.pmf_slope", 1.0);
  this->declare_parameter("ground_filter.pmf_initial_distance", 0.5);
  this->declare_parameter("ground_filter.pmf_max_distance", 3.0);
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
  
  // Get ground filtering parameters - enhanced for advanced filtering
  ground_filter_height_threshold_ = this->get_parameter("ground_filter.height_threshold").as_double();
  ground_filter_angle_threshold_ = this->get_parameter("ground_filter.angle_threshold").as_double();
  ransac_distance_threshold_ = this->get_parameter("ground_filter.ransac_distance_threshold").as_double();
  ransac_max_iterations_ = this->get_parameter("ground_filter.ransac_max_iterations").as_int();
  min_ground_points_ = this->get_parameter("ground_filter.min_ground_points").as_int();
  voxel_leaf_size_ = this->get_parameter("ground_filter.voxel_leaf_size").as_double();
  use_progressive_morphological_ = this->get_parameter("ground_filter.use_progressive_morphological").as_bool();
  pmf_max_window_size_ = this->get_parameter("ground_filter.pmf_max_window_size").as_int();
  pmf_slope_ = this->get_parameter("ground_filter.pmf_slope").as_double();
  pmf_initial_distance_ = this->get_parameter("ground_filter.pmf_initial_distance").as_double();
  pmf_max_distance_ = this->get_parameter("ground_filter.pmf_max_distance").as_double();
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
  
  // Update occupancy grid when map data changes
  update_occupancy_grid();
}

void PathPlannerNode::raw_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received raw point cloud with %d points", msg->width * msg->height);
  
  // Filter ground points from raw point cloud
  auto filtered_msg = filter_ground_points(msg, false);  // false = sensor data, apply range limit
  raw_pointcloud_ = filtered_msg;
  has_raw_pointcloud_ = true;
  
  // Update combined point cloud when we get new sensor data
  if (has_map_pointcloud_) {
    combined_pointcloud_ = combine_pointclouds(map_pointcloud_, raw_pointcloud_);
  } else {
    // If no map data available, use just sensor data
    combined_pointcloud_ = filtered_msg;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Raw point cloud filtered and stored");
  
  // Update occupancy grid when sensor data changes
  update_occupancy_grid();
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
  
  // Convert to PCL for processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pointcloud, *pcl_cloud);
  
  if (pcl_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    return pointcloud;
  }
  
  size_t original_size = pcl_cloud->points.size();
  RCLCPP_DEBUG(this->get_logger(), "Processing point cloud with %zu points", original_size);
  
  // Step 1: Preprocess the cloud (denoising, downsampling)
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessed_cloud = preprocess_cloud(pcl_cloud, is_map_data);
  
  // Step 2: Apply ground detection - we want to KEEP ground points and REMOVE obstacles
  pcl::PointIndices::Ptr obstacle_indices;
  
  if (is_map_data) {
    // For static map data, we need to be very careful about filtering
    // Static maps usually contain road/building data where:
    // - Roads should be filtered out as "ground" (not sent as obstacles)
    // - Buildings/walls should be kept as obstacles
    obstacle_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
    
    // Find Z statistics to understand the terrain
    std::vector<float> z_values;
    for (const auto& point : preprocessed_cloud->points) {
      z_values.push_back(point.z);
    }
    std::sort(z_values.begin(), z_values.end());
    
    // Use statistical approach for ground level detection
    float z_10th = z_values[std::min((size_t)(z_values.size() * 0.1), z_values.size() - 1)];
    float z_90th = z_values[std::min((size_t)(z_values.size() * 0.9), z_values.size() - 1)];
    
    // For static map, be more aggressive in filtering road points
    // Road points are typically at lower Z values, buildings at higher Z values
    float road_threshold = z_10th + 1.0;  // Points below this are likely road/ground
    float building_threshold = z_90th - 1.0; // Points above this are likely buildings
    
    // Only mark points as obstacles if they are clearly elevated (buildings, walls, etc.)
    for (size_t i = 0; i < preprocessed_cloud->points.size(); ++i) {
      float point_z = preprocessed_cloud->points[i].z;
      // Keep only high points as obstacles (buildings, walls)
      if (point_z > road_threshold && point_z >= building_threshold) {
        obstacle_indices->indices.push_back(i);
      }
      // Points between road_threshold and building_threshold might be curbs, small objects - filter them out for now
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Static map filtering: road_thresh=%.2f, building_thresh=%.2f, obstacles=%zu/%zu", 
                 road_threshold, building_threshold, obstacle_indices->indices.size(), preprocessed_cloud->points.size());
  } else {
    // For sensor data, use more sophisticated ground detection
    pcl::PointIndices::Ptr ground_indices;
    
    if (use_progressive_morphological_) {
      // Use Progressive Morphological Filter for sensor data (best for varied terrain)
      ground_indices = progressive_morphological_filter(preprocessed_cloud);
      RCLCPP_DEBUG(this->get_logger(), "Used Progressive Morphological Filter");
    } else {
      // Use multi-hypothesis approach combining RANSAC and height-based filtering
      ground_indices = multi_hypothesis_ground_detection(preprocessed_cloud);
      RCLCPP_DEBUG(this->get_logger(), "Used Multi-hypothesis Ground Detection");
    }
    
    // Convert ground indices to obstacle indices (everything NOT ground)
    obstacle_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
    std::set<int> ground_set(ground_indices->indices.begin(), ground_indices->indices.end());
    
    for (size_t i = 0; i < preprocessed_cloud->points.size(); ++i) {
      if (ground_set.find(i) == ground_set.end()) {
        obstacle_indices->indices.push_back(i);
      }
    }
  }
  
  // Step 3: Extract obstacle points only
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(preprocessed_cloud);
  extract.setIndices(obstacle_indices);
  extract.setNegative(false);  // Extract obstacle points only
  extract.filter(*filtered_cloud);
  
  // Step 4: Range filtering for sensor data
  if (!is_map_data) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr range_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : filtered_cloud->points) {
      double distance = std::sqrt(point.x * point.x + point.y * point.y);
      if (distance <= dynamic_obstacle_max_range_) {
        range_filtered_cloud->points.push_back(point);
      }
    }
    range_filtered_cloud->width = range_filtered_cloud->points.size();
    range_filtered_cloud->height = 1;
    range_filtered_cloud->is_dense = false;
    filtered_cloud = range_filtered_cloud;
  }
  
  // Convert back to ROS message
  auto filtered_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*filtered_cloud, *filtered_msg);
  filtered_msg->header = pointcloud->header;
  
  double filter_ratio = (double)filtered_cloud->points.size() / (double)original_size;
  RCLCPP_INFO(this->get_logger(), 
    "Ground filtering (%s): %zu -> %zu obstacle points (%.1f%% are obstacles, %.1f%% filtered as ground)",
    is_map_data ? "map" : "sensor",
    original_size, filtered_cloud->points.size(), 
    filter_ratio * 100.0, (1.0 - filter_ratio) * 100.0);
  
  return filtered_msg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PathPlannerNode::preprocess_cloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, bool is_map_data)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Step 1: Remove invalid points
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *processed_cloud, indices);
  
  // Step 2: Statistical outlier removal (for sensor data to reduce noise)
  if (!is_map_data) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(processed_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoised_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*denoised_cloud);
    processed_cloud = denoised_cloud;
  }
  
  // Step 3: Voxel grid downsampling (for computational efficiency)
  if (processed_cloud->points.size() > 10000) {  // Only downsample large clouds
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(processed_cloud);
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*downsampled_cloud);
    processed_cloud = downsampled_cloud;
  }
  
  return processed_cloud;
}

pcl::PointIndices::Ptr PathPlannerNode::ransac_ground_detection(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
  
  // RANSAC plane segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_max_iterations_);
  seg.setDistanceThreshold(ransac_distance_threshold_);
  
  seg.setInputCloud(cloud);
  seg.segment(*ground_indices, *coefficients);
  
  if (ground_indices->indices.size() < min_ground_points_) {
    RCLCPP_WARN(this->get_logger(), 
      "RANSAC found only %zu ground points (min: %d), using fallback method",
      ground_indices->indices.size(), min_ground_points_);
    ground_indices->indices.clear();
  } else {
    // Validate that the detected plane is roughly horizontal
    double nx = coefficients->values[0];
    double ny = coefficients->values[1];
    double nz = coefficients->values[2];
    double angle = std::acos(std::abs(nz) / std::sqrt(nx*nx + ny*ny + nz*nz)) * 180.0 / M_PI;
    
    if (angle > ground_filter_angle_threshold_) {
      RCLCPP_WARN(this->get_logger(), 
        "RANSAC plane too steep (%.1f° > %.1f°), using fallback method",
        angle, ground_filter_angle_threshold_);
      ground_indices->indices.clear();
    }
  }
  
  return ground_indices;
}

pcl::PointIndices::Ptr PathPlannerNode::progressive_morphological_filter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
  
  // Simple implementation of Progressive Morphological Filter concept
  // This is a height-based progressive filtering approach
  
  std::vector<bool> is_ground(cloud->points.size(), false);
  
  // Find minimum Z value as initial ground estimate
  float min_z = std::numeric_limits<float>::max();
  for (const auto& point : cloud->points) {
    if (point.z < min_z) {
      min_z = point.z;
    }
  }
  
  // Progressive filtering with increasing window sizes
  for (int window = 3; window <= pmf_max_window_size_; window += 2) {
    double current_threshold = pmf_initial_distance_ + 
      (pmf_max_distance_ - pmf_initial_distance_) * 
      (double(window) / double(pmf_max_window_size_));
    
    // Grid-based morphological operations simulation
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      const auto& point = cloud->points[i];
      
      // Height-based filtering with progressive threshold
      if (point.z <= (min_z + current_threshold)) {
        is_ground[i] = true;
      }
      
      // Check local neighborhood consistency
      int ground_neighbors = 0;
      int total_neighbors = 0;
      
      for (size_t j = 0; j < cloud->points.size(); ++j) {
        if (i == j) continue;
        
        const auto& neighbor = cloud->points[j];
        double dist = std::sqrt(
          (point.x - neighbor.x) * (point.x - neighbor.x) +
          (point.y - neighbor.y) * (point.y - neighbor.y)
        );
        
        if (dist < (window * 0.1)) { // Local neighborhood
          total_neighbors++;
          if (is_ground[j]) {
            ground_neighbors++;
          }
        }
      }
      
      // Morphological consistency check
      if (total_neighbors > 0 && ground_neighbors >= (total_neighbors / 2)) {
        is_ground[i] = true;
      }
    }
  }
  
  // Collect ground indices
  for (size_t i = 0; i < is_ground.size(); ++i) {
    if (is_ground[i]) {
      ground_indices->indices.push_back(i);
    }
  }
  
  return ground_indices;
}

pcl::PointIndices::Ptr PathPlannerNode::multi_hypothesis_ground_detection(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  // Try multiple ground detection methods and combine results
  
  // Method 1: RANSAC plane detection
  pcl::PointIndices::Ptr ransac_ground = ransac_ground_detection(cloud);
  
  // Method 2: Height-based filtering
  pcl::PointIndices::Ptr height_ground(new pcl::PointIndices);
  
  // Find approximate ground level using robust statistics
  std::vector<float> z_values;
  for (const auto& point : cloud->points) {
    z_values.push_back(point.z);
  }
  std::sort(z_values.begin(), z_values.end());
  
  // Use 5th percentile as ground level estimate (was 10th - now more conservative)
  float ground_level = z_values[std::min((size_t)(z_values.size() * 0.05), z_values.size() - 1)];
  
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].z <= (ground_level + ground_filter_height_threshold_)) {
      height_ground->indices.push_back(i);
    }
  }
  
  // For map data, be very conservative with ground filtering
  // Combine methods: use height-based if RANSAC fails, but be more conservative
  if (!ransac_ground->indices.empty() && ransac_ground->indices.size() >= min_ground_points_) {
    // Use the smaller of the two sets to be more conservative
    if (height_ground->indices.size() < ransac_ground->indices.size()) {
      RCLCPP_DEBUG(this->get_logger(), "Using conservative height-based ground detection (%zu points)", 
                   height_ground->indices.size());
      return height_ground;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Using RANSAC ground detection (%zu points)", 
                   ransac_ground->indices.size());
      return ransac_ground;
    }
  } else {
    // Use even more conservative height-based filtering
    pcl::PointIndices::Ptr conservative_ground(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      if (cloud->points[i].z <= (ground_level + ground_filter_height_threshold_ * 0.5)) {  // Even more conservative
        conservative_ground->indices.push_back(i);
      }
    }
    RCLCPP_DEBUG(this->get_logger(), "Using very conservative height-based ground detection (%zu points)", 
                 conservative_ground->indices.size());
    return conservative_ground;
  }
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

void PathPlannerNode::publish_occupancy_grid()
{
  if (planning_algorithm_ == "astar" && (has_raw_pointcloud_ || combined_pointcloud_)) {
    auto grid = astar_planner_->get_occupancy_grid();
    grid_pub_->publish(grid);
    RCLCPP_DEBUG(this->get_logger(), "Published occupancy grid");
  }
}

void PathPlannerNode::update_occupancy_grid()
{
  // Update occupancy grid using available point cloud data
  sensor_msgs::msg::PointCloud2::SharedPtr planning_cloud;
  
  if (combined_pointcloud_) {
    planning_cloud = combined_pointcloud_;
  } else if (has_raw_pointcloud_) {
    planning_cloud = raw_pointcloud_;
  } else {
    return; // No data available
  }
  
  if (planning_algorithm_ == "astar" && planning_cloud) {
    // Use current vehicle position if available, otherwise use origin
    geometry_msgs::msg::PoseStamped vehicle_pose;
    vehicle_pose.header.frame_id = map_frame_;
    vehicle_pose.header.stamp = this->get_clock()->now();
    
    if (has_current_pose_) {
      vehicle_pose = current_pose_;
    } else {
      // Default to origin if no pose available
      vehicle_pose.pose.position.x = 0.0;
      vehicle_pose.pose.position.y = 0.0;
      vehicle_pose.pose.position.z = 0.0;
      vehicle_pose.pose.orientation.w = 1.0;
    }
    
    // Update A* planner's grid for visualization (vehicle-centered)
    try {
      astar_planner_->update_grid_for_visualization(vehicle_pose, planning_cloud);
      RCLCPP_DEBUG(this->get_logger(), "Updated vehicle-centered occupancy grid");
    } catch (const std::exception& e) {
      RCLCPP_DEBUG(this->get_logger(), "Grid update completed: %s", e.what());
    }
  }
}

}  // namespace awsim_path_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(awsim_path_planner::PathPlannerNode)
