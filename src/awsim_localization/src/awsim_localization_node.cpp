#include "awsim_localization_node.hpp"

namespace awsim_localization
{

AwsimLocalizationNode::AwsimLocalizationNode(const rclcpp::NodeOptions & options)
: Node("awsim_localization_node", options),
  map_cloud_(new pcl::PointCloud<pcl::PointXYZI>),
  pose_initialized_(false),
  map_loaded_(false),
  first_scan_(true),
  fitness_score_(0.0)
{
  // Initialize map origin offset
  map_origin_offset_.x = 0.0;
  map_origin_offset_.y = 0.0;
  map_origin_offset_.z = 0.0;
  
  // Declare parameters
  declareParameters();
  loadParameters();
  
  // Initialize transform handling
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  // Initialize NDT
  ndt_ = std::make_unique<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
  ndt_->setResolution(ndt_resolution_);
  ndt_->setMaximumIterations(ndt_max_iterations_);
  ndt_->setTransformationEpsilon(ndt_transformation_epsilon_);
  ndt_->setStepSize(ndt_step_size_);
  
  if (ndt_num_threads_ > 0) {
    ndt_->setNumThreads(ndt_num_threads_);
  } else {
    ndt_->setNumThreads(omp_get_max_threads());
  }
  
  // Initialize voxel grid filter
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  
  // Create subscribers with AWSIM-compatible QoS profiles
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar/top/pointcloud_raw", createSensorQoS(),
    std::bind(&AwsimLocalizationNode::pointCloudCallback, this, std::placeholders::_1));
  
  ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/awsim/ground_truth/localization/kinematic_state", createGroundTruthQoS(),
    std::bind(&AwsimLocalizationNode::groundTruthCallback, this, std::placeholders::_1));
  
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", createLocalizationQoS(),
    std::bind(&AwsimLocalizationNode::initialPoseCallback, this, std::placeholders::_1));
  
  // Create publishers with appropriate QoS profiles
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/ground_truth_pose", createLocalizationQoS());
  
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", createLocalizationQoS());
  
  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/localization/map", createLocalizationQoS());
  
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/localization/markers", createLocalizationQoS());
  
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/localization/status", createLocalizationQoS());
  
  ground_truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/pose_with_covariance", createLocalizationQoS());
  
  ground_truth_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/localization/ground_truth_odom", createLocalizationQoS());
  
  // Load map
  if (!loadMap()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map! Check map path: %s", map_path_.c_str());
    return;
  }
  
  // Create timer to periodically republish map for RViz visibility
  map_publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), 
    std::bind(&AwsimLocalizationNode::publishMap, this));
  
  // Initialize current pose
  current_pose_.header.frame_id = map_frame_;
  current_pose_.pose.pose.orientation.w = 1.0;  // Identity quaternion
  
  RCLCPP_INFO(this->get_logger(), "AWSIM localization node initialized successfully");
}

void AwsimLocalizationNode::declareParameters()
{
  // Algorithm selection parameter
  this->declare_parameter("algorithm_type", "ndt");  // "ndt" or "icp"
  
  // Map and frame parameters
  this->declare_parameter("map_path", "/home/abdullah/workspaces/career_sprint/awsim-autonomous-research/shinjuku_map/map/pointcloud_map.pcd");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_link_frame", "base_link");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("lidar_frame", "velodyne_top");
  
  // Map centering parameters
  this->declare_parameter("use_manual_map_center", false);  // Enable manual center override
  this->declare_parameter("manual_map_center_x", 85231.62); // Default Shinjuku map center X
  this->declare_parameter("manual_map_center_y", 50677.12); // Default Shinjuku map center Y
  this->declare_parameter("manual_map_center_z", 45.12);    // Default Shinjuku map center Z
  
  // NDT parameters
  this->declare_parameter("ndt_resolution", 2.0);
  this->declare_parameter("ndt_max_iterations", 35);
  this->declare_parameter("ndt_transformation_epsilon", 0.01);
  this->declare_parameter("ndt_step_size", 0.1);
  this->declare_parameter("ndt_num_threads", 4);
  
  // ICP parameters
  this->declare_parameter("icp_max_iterations", 50);
  this->declare_parameter("icp_transformation_epsilon", 1e-6);
  this->declare_parameter("icp_max_correspondence_distance", 1.0);
  this->declare_parameter("icp_euclidean_fitness_epsilon", 0.01);
  
  // Filtering parameters
  this->declare_parameter("voxel_leaf_size", 0.5);
  this->declare_parameter("lidar_max_range", 100.0);
  this->declare_parameter("lidar_min_range", 1.0);
  
  // Localization parameters
  this->declare_parameter("scan_matching_score_threshold", 20.0);
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("use_ground_truth_init", true);
}

void AwsimLocalizationNode::loadParameters()
{
  map_path_ = this->get_parameter("map_path").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_link_frame_ = this->get_parameter("base_link_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  lidar_frame_ = this->get_parameter("lidar_frame").as_string();
  
  // Map centering parameters
  use_manual_map_center_ = this->get_parameter("use_manual_map_center").as_bool();
  manual_map_center_x_ = this->get_parameter("manual_map_center_x").as_double();
  manual_map_center_y_ = this->get_parameter("manual_map_center_y").as_double();
  manual_map_center_z_ = this->get_parameter("manual_map_center_z").as_double();
  
  ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
  ndt_max_iterations_ = this->get_parameter("ndt_max_iterations").as_int();
  ndt_transformation_epsilon_ = this->get_parameter("ndt_transformation_epsilon").as_double();
  ndt_step_size_ = this->get_parameter("ndt_step_size").as_double();
  ndt_num_threads_ = this->get_parameter("ndt_num_threads").as_int();
  
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  lidar_max_range_ = this->get_parameter("lidar_max_range").as_double();
  lidar_min_range_ = this->get_parameter("lidar_min_range").as_double();
  
  scan_matching_score_threshold_ = this->get_parameter("scan_matching_score_threshold").as_double();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  use_ground_truth_init_ = this->get_parameter("use_ground_truth_init").as_bool();
}

rclcpp::QoS AwsimLocalizationNode::createSensorQoS()
{
  // AWSIM sensor data uses BEST_EFFORT for performance
  return rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .history(rclcpp::HistoryPolicy::KeepLast);
}

rclcpp::QoS AwsimLocalizationNode::createGroundTruthQoS()
{
  // AWSIM ground truth uses BEST_EFFORT for simulation performance
  return rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .history(rclcpp::HistoryPolicy::KeepLast);
}

rclcpp::QoS AwsimLocalizationNode::createLocalizationQoS()
{
  // Localization output uses RELIABLE for critical data
  return rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .history(rclcpp::HistoryPolicy::KeepLast);
}

bool AwsimLocalizationNode::loadMap()
{
  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_path_.c_str());
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr original_map(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path_, *original_map) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Could not read file %s", map_path_.c_str());
    return false;
  }
  
  if (original_map->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Loaded map is empty!");
    return false;
  }
  
  // Calculate or use manual map center for translation to origin
  if (use_manual_map_center_) {
    // Use manually specified center coordinates
    map_origin_offset_.x = manual_map_center_x_;
    map_origin_offset_.y = manual_map_center_y_;
    map_origin_offset_.z = manual_map_center_z_;
    
    RCLCPP_INFO(this->get_logger(), 
               "Using manual map center: (%.2f, %.2f, %.2f)", 
               map_origin_offset_.x, map_origin_offset_.y, map_origin_offset_.z);
  } else {
    // Calculate map center automatically from centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*original_map, centroid);
    map_origin_offset_.x = centroid[0];
    map_origin_offset_.y = centroid[1];
    map_origin_offset_.z = centroid[2];
    
    RCLCPP_INFO(this->get_logger(), 
               "Calculated map center from centroid: (%.2f, %.2f, %.2f)", 
               map_origin_offset_.x, map_origin_offset_.y, map_origin_offset_.z);
  }
  
  // Translate map to center it at origin for better RViz precision
  map_cloud_->clear();
  map_cloud_->reserve(original_map->size());
  for (const auto& point : original_map->points) {
    pcl::PointXYZI translated_point = point;
    translated_point.x -= map_origin_offset_.x;
    translated_point.y -= map_origin_offset_.y;
    translated_point.z -= map_origin_offset_.z;
    map_cloud_->push_back(translated_point);
  }
  
  map_cloud_->width = original_map->width;
  map_cloud_->height = original_map->height;
  map_cloud_->is_dense = original_map->is_dense;
  
  RCLCPP_INFO(this->get_logger(), 
             "Map loaded and centered at origin with %zu points (offset: %.2f, %.2f, %.2f)", 
             map_cloud_->size(), map_origin_offset_.x, map_origin_offset_.y, map_origin_offset_.z);
  
  // Set target cloud for NDT
  ndt_->setInputTarget(map_cloud_);
  
  // Publish map for visualization
  publishMap();
  
  map_loaded_ = true;
  return true;
}

void AwsimLocalizationNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!map_loaded_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Map not loaded, skipping scan matching");
    return;
  }
  
  // Transform point cloud to map frame if necessary
  sensor_msgs::msg::PointCloud2::SharedPtr transformed_msg = msg;
  if (msg->header.frame_id != base_link_frame_) {
    try {
      // First try to get transform at the exact timestamp
      geometry_msgs::msg::TransformStamped transform_stamped;
      try {
        transform_stamped = tf_buffer_->lookupTransform(
          base_link_frame_, msg->header.frame_id, msg->header.stamp, 
          rclcpp::Duration::from_nanoseconds(50000000)); // 50ms timeout
      } catch (const tf2::TransformException & ex) {
        // If exact timestamp fails, try latest available transform
        RCLCPP_DEBUG(this->get_logger(), 
                    "Exact timestamp transform failed, trying latest: %s", ex.what());
        transform_stamped = tf_buffer_->lookupTransform(
          base_link_frame_, msg->header.frame_id, rclcpp::Time(0), 
          rclcpp::Duration::from_nanoseconds(50000000)); // 50ms timeout
      }
                                   
      // Create transformed message
      transformed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2::doTransform(*msg, *transformed_msg, transform_stamped);
      transformed_msg->header.stamp = msg->header.stamp; // Preserve original timestamp
      
      RCLCPP_DEBUG(this->get_logger(), "Transformed point cloud from %s to %s", 
                   msg->header.frame_id.c_str(), base_link_frame_.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Could not transform point cloud from %s to %s: %s", 
                          msg->header.frame_id.c_str(), base_link_frame_.c_str(), ex.what());
      return;
    }
  }
  
  // Convert ROS PointCloud2 to PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*transformed_msg, *scan_cloud);
  
  if (scan_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    return;
  }
  
  // Initialize pose from ground truth if this is the first scan and option is enabled
  if (first_scan_ && use_ground_truth_init_ && !pose_initialized_) {
    if (!last_ground_truth_.header.frame_id.empty()) {
      // Transform ground truth pose to centered coordinate system for initialization
      geometry_msgs::msg::Pose centered_pose = last_ground_truth_.pose.pose;
      centered_pose.position.x -= map_origin_offset_.x;
      centered_pose.position.y -= map_origin_offset_.y;
      centered_pose.position.z -= map_origin_offset_.z;
      
      initializePose(centered_pose);
      RCLCPP_INFO(this->get_logger(), "Initialized pose from ground truth (centered coordinates)");
    }
    first_scan_ = false;
  }
  
  if (!pose_initialized_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Pose not initialized. Use /initialpose topic or enable ground truth initialization");
    return;
  }
  
  // Filter point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = filterPointCloud(scan_cloud);
  
  // Perform scan matching
  performScanMatching(filtered_cloud);
  
  // Publish results
  current_pose_.header.stamp = this->get_clock()->now();
  pose_pub_->publish(current_pose_);
  
  // Publish as odometry for navigation stack compatibility
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header = current_pose_.header;
  odom_msg.child_frame_id = base_link_frame_;
  odom_msg.pose = current_pose_.pose;
  odom_pub_->publish(odom_msg);
  
  if (publish_tf_) {
    publishTransform(current_pose_);
  }
  
  publishVisualization();
}

void AwsimLocalizationNode::groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_ground_truth_ = *msg;
  
  // Transform ground truth coordinates to the centered coordinate system
  geometry_msgs::msg::PoseWithCovarianceStamped ground_truth_pose;
  ground_truth_pose.header = msg->header;
  ground_truth_pose.header.frame_id = map_frame_;  // Use map frame for visualization
  ground_truth_pose.pose = msg->pose;
  
  // Apply coordinate transformation to center the pose
  ground_truth_pose.pose.pose.position.x -= map_origin_offset_.x;
  ground_truth_pose.pose.pose.position.y -= map_origin_offset_.y;
  ground_truth_pose.pose.pose.position.z -= map_origin_offset_.z;
  
  ground_truth_pub_->publish(ground_truth_pose);
  
  // Republish ground truth as Odometry in map frame with centered coordinates
  nav_msgs::msg::Odometry ground_truth_odom;
  ground_truth_odom.header = msg->header;
  ground_truth_odom.header.frame_id = map_frame_;  // Use map frame
  ground_truth_odom.child_frame_id = base_link_frame_;
  ground_truth_odom.pose = msg->pose;
  ground_truth_odom.twist = msg->twist;  // Keep velocity information
  
  // Apply coordinate transformation to center the odometry
  ground_truth_odom.pose.pose.position.x -= map_origin_offset_.x;
  ground_truth_odom.pose.pose.position.y -= map_origin_offset_.y;
  ground_truth_odom.pose.pose.position.z -= map_origin_offset_.z;
  
  ground_truth_odom_pub_->publish(ground_truth_odom);
}

void AwsimLocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received initial pose estimate from RViz");
  
  // Since RViz is working with the centered coordinate system, we can use the pose directly
  // No coordinate transformation needed here as both RViz and our centered map use origin-based coordinates
  initializePose(msg->pose.pose);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr AwsimLocalizationNode::filterPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  // Apply voxel grid filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_grid_.setInputCloud(cloud);
  voxel_grid_.filter(*filtered_cloud);
  
  // Apply range filter
  pcl::PointCloud<pcl::PointXYZI>::Ptr range_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto & point : filtered_cloud->points) {
    double range = sqrt(point.x * point.x + point.y * point.y);
    if (range >= lidar_min_range_ && range <= lidar_max_range_) {
      range_filtered_cloud->push_back(point);
    }
  }
  
  return range_filtered_cloud;
}

void AwsimLocalizationNode::performScanMatching(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan)
{
  // Prepare initial guess from current pose
  Eigen::Affine3d affine;
  tf2::fromMsg(current_pose_.pose.pose, affine);
  Eigen::Matrix4f initial_guess = affine.matrix().cast<float>();
  
  // Perform NDT alignment
  ndt_->setInputSource(scan);
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  
  auto start_time = std::chrono::steady_clock::now();
  ndt_->align(*aligned_cloud, initial_guess);
  auto end_time = std::chrono::steady_clock::now();
  
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  
  // Check if alignment converged
  bool has_converged = ndt_->hasConverged();
  fitness_score_ = ndt_->getFitnessScore();
  
  if (!has_converged) {
    RCLCPP_WARN(this->get_logger(), "NDT alignment did not converge");
    
    // Publish failure status
    std_msgs::msg::String status_msg;
    status_msg.data = "SCAN_MATCHING_FAILED_CONVERGENCE";
    status_pub_->publish(status_msg);
    return;
  }
  
  if (fitness_score_ > scan_matching_score_threshold_) {
    RCLCPP_WARN(this->get_logger(), 
               "Scan matching failed: fitness score %.4f exceeds threshold %.4f",
               fitness_score_, scan_matching_score_threshold_);
    
    // Publish failure status
    std_msgs::msg::String status_msg;
    status_msg.data = "SCAN_MATCHING_FAILED_SCORE";
    status_pub_->publish(status_msg);
    return;
  }
  
  // Update pose from transformation result
  Eigen::Matrix4f final_transformation = ndt_->getFinalTransformation();
  updatePoseFromTransformation(final_transformation, fitness_score_);
  
  // Publish success status
  std_msgs::msg::String status_msg;
  status_msg.data = "SCAN_MATCHING_OK";
  status_pub_->publish(status_msg);
  
  RCLCPP_DEBUG(this->get_logger(), 
              "NDT alignment successful: fitness=%.4f, time=%ldms",
              fitness_score_, duration.count());
}

void AwsimLocalizationNode::initializePose(const geometry_msgs::msg::Pose& pose)
{
  current_pose_.pose.pose = pose;
  current_pose_.header.frame_id = map_frame_;
  current_pose_.header.stamp = this->get_clock()->now();
  pose_initialized_ = true;
  
  // Immediately publish the transform to establish map->base_link connection
  if (publish_tf_) {
    publishTransform(current_pose_);
  }
  
  // Also publish the initial pose
  pose_pub_->publish(current_pose_);
  
  RCLCPP_INFO(this->get_logger(), "Pose initialized at [%.2f, %.2f, %.2f]",
             current_pose_.pose.pose.position.x, current_pose_.pose.pose.position.y, current_pose_.pose.pose.position.z);
}

void AwsimLocalizationNode::updatePoseFromTransformation(const Eigen::Matrix4f& transformation, double fitness_score)
{
  // Update current pose with result
  current_pose_.pose.pose.position.x = transformation(0, 3);
  current_pose_.pose.pose.position.y = transformation(1, 3);
  current_pose_.pose.pose.position.z = transformation(2, 3);
  
  // Convert rotation matrix to quaternion
  Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
  Eigen::Quaternionf q_eigen(rotation);
  current_pose_.pose.pose.orientation.x = q_eigen.x();
  current_pose_.pose.pose.orientation.y = q_eigen.y();
  current_pose_.pose.pose.orientation.z = q_eigen.z();
  current_pose_.pose.pose.orientation.w = q_eigen.w();
  
  // Set covariance based on fitness score
  double covariance_scale = std::min(fitness_score * 0.1, 1.0);
  for (int i = 0; i < 36; ++i) {
    current_pose_.pose.covariance[i] = 0.0;
  }
  current_pose_.pose.covariance[0] = covariance_scale;         // x variance
  current_pose_.pose.covariance[7] = covariance_scale;         // y variance
  current_pose_.pose.covariance[14] = covariance_scale * 0.1;  // z variance
  current_pose_.pose.covariance[21] = covariance_scale * 0.1;  // roll variance
  current_pose_.pose.covariance[28] = covariance_scale * 0.1;  // pitch variance
  current_pose_.pose.covariance[35] = covariance_scale;        // yaw variance
  
  RCLCPP_DEBUG(this->get_logger(), 
              "Pose updated: [%.2f, %.2f, %.2f], fitness=%.4f",
              current_pose_.pose.pose.position.x,
              current_pose_.pose.pose.position.y,
              current_pose_.pose.pose.position.z,
              fitness_score);
}

void AwsimLocalizationNode::publishTransform(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
{
  if (!publish_tf_) {
    return;
  }

  // Publish only map->base_link transform for localization
  // This is the core responsibility of a localization system
  geometry_msgs::msg::TransformStamped map_to_base_link;
  map_to_base_link.header.frame_id = map_frame_;
  map_to_base_link.header.stamp = this->get_clock()->now();
  map_to_base_link.child_frame_id = base_link_frame_;
  
  // Use the localization result directly
  map_to_base_link.transform.translation.x = pose_msg.pose.pose.position.x;
  map_to_base_link.transform.translation.y = pose_msg.pose.pose.position.y;
  map_to_base_link.transform.translation.z = pose_msg.pose.pose.position.z;
  map_to_base_link.transform.rotation = pose_msg.pose.pose.orientation;
  
  tf_broadcaster_->sendTransform(map_to_base_link);
}

void AwsimLocalizationNode::publishVisualization()
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Current pose marker
  visualization_msgs::msg::Marker pose_marker;
  pose_marker.header = current_pose_.header;
  pose_marker.ns = "awsim_localization";
  pose_marker.id = 0;
  pose_marker.type = visualization_msgs::msg::Marker::ARROW;
  pose_marker.action = visualization_msgs::msg::Marker::ADD;
  pose_marker.pose = current_pose_.pose.pose;
  pose_marker.scale.x = 2.0;
  pose_marker.scale.y = 0.5;
  pose_marker.scale.z = 0.5;
  pose_marker.color.r = 0.0;
  pose_marker.color.g = 1.0;
  pose_marker.color.b = 0.0;
  pose_marker.color.a = 0.8;
  
  marker_array.markers.push_back(pose_marker);
  
  // Fitness score text marker
  visualization_msgs::msg::Marker text_marker;
  text_marker.header = current_pose_.header;
  text_marker.ns = "awsim_localization";
  text_marker.id = 1;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose = current_pose_.pose.pose;
  text_marker.pose.position.z += 2.0;  // Display above the vehicle
  text_marker.scale.z = 1.0;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.text = "NDT Score: " + std::to_string(fitness_score_);
  
  marker_array.markers.push_back(text_marker);
  
  marker_pub_->publish(marker_array);
}

void AwsimLocalizationNode::publishMap()
{
  if (!map_cloud_ || map_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot publish empty map");
    return;
  }
  
  // Create a less aggressive sparse version of the map for visualization
  pcl::PointCloud<pcl::PointXYZI>::Ptr sparse_map(new pcl::PointCloud<pcl::PointXYZI>);
  
  // Apply voxel grid filter with moderate leaf size for better visualization
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(map_cloud_);
  voxel_filter.setLeafSize(1.0f, 1.0f, 1.0f);  // 1m voxels instead of 2m for better visibility
  voxel_filter.filter(*sparse_map);
  
  // Convert to ROS message
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*sparse_map, map_msg);
  map_msg.header.frame_id = map_frame_;
  map_msg.header.stamp = this->get_clock()->now();
  
  // Publish the sparse map
  map_pub_->publish(map_msg);
  
  RCLCPP_INFO(this->get_logger(), 
             "Published sparse map with %zu points (filtered from %zu points)", 
             sparse_map->size(), map_cloud_->size());
}

}  // namespace awsim_localization

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(awsim_localization::AwsimLocalizationNode)
