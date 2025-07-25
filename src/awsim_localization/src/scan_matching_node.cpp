#include "awsim_localization/scan_matching_node.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace awsim_localization
{

ScanMatchingNode::ScanMatchingNode(const rclcpp::NodeOptions & options)
  : Node("scan_matching_node", options),
    map_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
    map_loaded_(false),
    pose_initialized_(false),
    first_scan_(true)
{
  // Declare and load parameters
  declareParameters();
  loadParameters();
  
  // Initialize transform handling
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  // Initialize scan matching algorithms
  ndt_matcher_ = std::make_unique<NDTMatcher>();
  icp_matcher_ = std::make_unique<ICPMatcher>();
  
  // Configure algorithms with parameters
  ndt_matcher_->setResolution(ndt_resolution_);
  ndt_matcher_->setMaxIterations(ndt_max_iterations_);
  ndt_matcher_->setTransformationEpsilon(ndt_transformation_epsilon_);
  ndt_matcher_->setStepSize(ndt_step_size_);
  
  icp_matcher_->setMaxIterations(icp_max_iterations_);
  icp_matcher_->setTransformationEpsilon(icp_transformation_epsilon_);
  icp_matcher_->setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
  
  // Create subscribers with proper QoS profiles for AWSIM compatibility
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar/top/pointcloud_raw", createSensorQoS(),
    std::bind(&ScanMatchingNode::pointCloudCallback, this, std::placeholders::_1));
  
  ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/awsim/ground_truth/localization/kinematic_state", createGroundTruthQoS(),
    std::bind(&ScanMatchingNode::groundTruthCallback, this, std::placeholders::_1));
  
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", createLocalizationQoS(),
    std::bind(&ScanMatchingNode::initialPoseCallback, this, std::placeholders::_1));
  
  // Create publishers with appropriate QoS profiles
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/pose_with_covariance", createLocalizationQoS());
  
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/localization/markers", createLocalizationQoS());
  
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/localization/status", createLocalizationQoS());
  
  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/localization/map", createLocalizationQoS());
  
  ground_truth_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/ground_truth_pose", createLocalizationQoS());
  
  // Load map
  if (!loadMap()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map! Check map path: %s", map_path_.c_str());
    return;
  }
  
  // Initialize current pose
  current_pose_.header.frame_id = map_frame_;
  current_pose_.pose.pose.orientation.w = 1.0;  // Identity quaternion
  
  RCLCPP_INFO(this->get_logger(), "Scan matching node initialized with %s algorithm", 
    algorithm_type_.c_str());
}

void ScanMatchingNode::declareParameters()
{
  // Map and frame parameters
  this->declare_parameter("map_path", "/home/abdullah/workspaces/career_sprint/awsim-autonomous-research/shinjuku_map/map/pointcloud_map.pcd");
  this->declare_parameter("algorithm_type", "ndt");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_link_frame", "base_link");
  this->declare_parameter("odom_frame", "odom");
  
  // NDT parameters
  this->declare_parameter("ndt_resolution", 2.0);
  this->declare_parameter("ndt_max_iterations", 35);
  this->declare_parameter("ndt_transformation_epsilon", 0.01);
  this->declare_parameter("ndt_step_size", 0.1);
  
  // ICP parameters
  this->declare_parameter("icp_max_iterations", 50);
  this->declare_parameter("icp_transformation_epsilon", 1e-6);
  this->declare_parameter("icp_max_correspondence_distance", 1.0);
  
  // General parameters
  this->declare_parameter("scan_matching_score_threshold", 2000.0);
  this->declare_parameter("publish_tf", true);
  this->declare_parameter("use_ground_truth_init", true);
}

void ScanMatchingNode::loadParameters()
{
  map_path_ = this->get_parameter("map_path").as_string();
  algorithm_type_ = this->get_parameter("algorithm_type").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_link_frame_ = this->get_parameter("base_link_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  
  ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
  ndt_max_iterations_ = this->get_parameter("ndt_max_iterations").as_int();
  ndt_transformation_epsilon_ = this->get_parameter("ndt_transformation_epsilon").as_double();
  ndt_step_size_ = this->get_parameter("ndt_step_size").as_double();
  
  icp_max_iterations_ = this->get_parameter("icp_max_iterations").as_int();
  icp_transformation_epsilon_ = this->get_parameter("icp_transformation_epsilon").as_double();
  icp_max_correspondence_distance_ = this->get_parameter("icp_max_correspondence_distance").as_double();
  
  scan_matching_score_threshold_ = this->get_parameter("scan_matching_score_threshold").as_double();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  use_ground_truth_init_ = this->get_parameter("use_ground_truth_init").as_bool();
}

rclcpp::QoS ScanMatchingNode::createSensorQoS()
{
  // AWSIM sensor data (LiDAR) typically uses BEST_EFFORT reliability for performance
  return rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .history(rclcpp::HistoryPolicy::KeepLast);
}

rclcpp::QoS ScanMatchingNode::createGroundTruthQoS()
{
  // AWSIM ground truth data uses BEST_EFFORT for simulation performance
  return rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .history(rclcpp::HistoryPolicy::KeepLast);
}

rclcpp::QoS ScanMatchingNode::createLocalizationQoS()
{
  // Localization output should use RELIABLE for critical data
  return rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .history(rclcpp::HistoryPolicy::KeepLast);
}

bool ScanMatchingNode::loadMap()
{
  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_path_.c_str());
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *map_cloud_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Could not read file %s", map_path_.c_str());
    return false;
  }
  
  if (map_cloud_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Loaded map is empty!");
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "Map loaded successfully with %zu points", map_cloud_->size());
  
  // Set target cloud for both algorithms
  ndt_matcher_->setTargetCloud(map_cloud_);
  icp_matcher_->setTargetCloud(map_cloud_);
  
  // Publish sparse map for visualization
  publishMap();
  
  map_loaded_ = true;
  return true;
}

void ScanMatchingNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!map_loaded_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
      "Map not loaded, skipping scan matching");
    return;
  }
  
  // Transform point cloud to map frame if necessary
  sensor_msgs::msg::PointCloud2::SharedPtr transformed_msg = msg;
  if (msg->header.frame_id != map_frame_) {
    try {
      // Use latest available transform to avoid timestamp synchronization issues with AWSIM
      geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, rclcpp::Time(0), 
                                   rclcpp::Duration::from_nanoseconds(100000000)); // 100ms timeout
                                   
      // Create transformed message with updated timestamp
      transformed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2::doTransform(*msg, *transformed_msg, transform_stamped);
      transformed_msg->header.stamp = msg->header.stamp; // Preserve original timestamp
      
      RCLCPP_DEBUG(this->get_logger(), "Transformed point cloud from %s to %s", 
                   msg->header.frame_id.c_str(), map_frame_.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Could not transform point cloud from %s to %s: %s", 
        msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
      return;
    }
  }
  
  // Convert ROS PointCloud2 to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*transformed_msg, *scan_cloud);
  
  if (scan_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    return;
  }
  
  // Initialize pose from ground truth if this is the first scan and option is enabled
  if (first_scan_ && use_ground_truth_init_ && !pose_initialized_) {
    if (!last_ground_truth_.header.frame_id.empty()) {
      initializePose(last_ground_truth_.pose.pose);
      RCLCPP_INFO(this->get_logger(), "Initialized pose from ground truth");
    }
    first_scan_ = false;
  }
  
  if (!pose_initialized_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
      "Pose not initialized. Use /initialpose topic or enable ground truth initialization");
    return;
  }
  
  // Perform scan matching
  performScanMatching(scan_cloud);
  
  // Publish results
  current_pose_.header.stamp = msg->header.stamp;
  pose_pub_->publish(current_pose_);
  
  if (publish_tf_) {
    publishTransform(current_pose_);
  }
  
  publishVisualization();
}

void ScanMatchingNode::groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_ground_truth_ = *msg;
  
  // Republish ground truth as PoseWithCovarianceStamped for RViz visualization
  geometry_msgs::msg::PoseWithCovarianceStamped ground_truth_pose;
  ground_truth_pose.header = msg->header;
  ground_truth_pose.header.frame_id = map_frame_;  // Use map frame for visualization
  ground_truth_pose.pose = msg->pose;
  
  ground_truth_pub_->publish(ground_truth_pose);
}

void ScanMatchingNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received initial pose estimate");
  initializePose(msg->pose.pose);
}

void ScanMatchingNode::performScanMatching(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan)
{
  // Prepare initial guess from current pose
  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  
  // Convert current pose to transformation matrix
  auto& pose = current_pose_.pose.pose;
  initial_guess(0, 3) = pose.position.x;
  initial_guess(1, 3) = pose.position.y;
  initial_guess(2, 3) = pose.position.z;
  
  // Convert quaternion to rotation matrix
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3 rot_matrix(q);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      initial_guess(i, j) = rot_matrix[i][j];
    }
  }
  
  // Perform scan matching based on selected algorithm
  Eigen::Matrix4f result_transformation;
  double fitness_score = 0.0;
  bool converged = false;
  
  if (algorithm_type_ == "ndt") {
    auto ndt_result = ndt_matcher_->matchScan(scan, initial_guess);
    result_transformation = ndt_result.transformation;
    fitness_score = ndt_result.fitness_score;
    converged = ndt_result.has_converged;
  } else if (algorithm_type_ == "icp") {
    auto icp_result = icp_matcher_->matchScan(scan, initial_guess);
    result_transformation = icp_result.transformation;
    fitness_score = icp_result.fitness_score;
    converged = icp_result.has_converged;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown algorithm type: %s", algorithm_type_.c_str());
    return;
  }
  
  // Check if result is valid
  if (!converged || fitness_score > scan_matching_score_threshold_) {
    RCLCPP_WARN(this->get_logger(), 
      "Scan matching failed: converged=%s, score=%.4f (threshold=%.4f)",
      converged ? "true" : "false", fitness_score, scan_matching_score_threshold_);
    
    // Publish failure status
    std_msgs::msg::String status_msg;
    status_msg.data = "SCAN_MATCHING_FAILED";
    status_pub_->publish(status_msg);
    return;
  }
  
  // Update pose and publish
  updatePoseFromTransformation(result_transformation, fitness_score);
}

void ScanMatchingNode::initializePose(const geometry_msgs::msg::Pose& pose)
{
  current_pose_.pose.pose = pose;
  current_pose_.header.frame_id = map_frame_;
  current_pose_.header.stamp = this->get_clock()->now();
  pose_initialized_ = true;
  
  // Immediately publish the transform to establish map->base_link connection
  // This is crucial so that point cloud transformation can work
  if (publish_tf_) {
    publishTransform(current_pose_);
  }
  
  // Also publish the initial pose
  pose_pub_->publish(current_pose_);
  
  RCLCPP_INFO(this->get_logger(), "Pose initialized at [%.2f, %.2f, %.2f]",
    pose.position.x, pose.position.y, pose.position.z);
}


void ScanMatchingNode::updatePoseFromTransformation(const Eigen::Matrix4f& transformation, double fitness_score)
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
    
    // Update first scan flag
    if (first_scan_) {
        first_scan_ = false;
        RCLCPP_INFO(this->get_logger(), "First scan processed successfully");
    }
    
    // Publish results
    std_msgs::msg::String status_msg;
    status_msg.data = "SCAN_MATCHING_OK";
    status_pub_->publish(status_msg);
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Scan matching successful: score=%.4f, pose=[%.2f, %.2f, %.2f]",
        fitness_score, 
        current_pose_.pose.pose.position.x,
        current_pose_.pose.pose.position.y,
        current_pose_.pose.pose.position.z);
}

void ScanMatchingNode::publishTransform(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = pose_msg.header.frame_id;
  transform.header.stamp = pose_msg.header.stamp; // Use message timestamp for AWSIM compatibility
  transform.child_frame_id = base_link_frame_;
  
  transform.transform.translation.x = pose_msg.pose.pose.position.x;
  transform.transform.translation.y = pose_msg.pose.pose.position.y;
  transform.transform.translation.z = pose_msg.pose.pose.position.z;
  
  transform.transform.rotation = pose_msg.pose.pose.orientation;
  
  tf_broadcaster_->sendTransform(transform);
}

void ScanMatchingNode::publishVisualization()
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Current pose marker
  visualization_msgs::msg::Marker pose_marker;
  pose_marker.header = current_pose_.header;
  pose_marker.ns = "localization";
  pose_marker.id = 0;
  pose_marker.type = visualization_msgs::msg::Marker::ARROW;
  pose_marker.action = visualization_msgs::msg::Marker::ADD;
  pose_marker.pose = current_pose_.pose.pose;
  pose_marker.scale.x = 2.0;
  pose_marker.scale.y = 0.5;
  pose_marker.scale.z = 0.5;
  pose_marker.color.r = 1.0;
  pose_marker.color.g = 0.0;
  pose_marker.color.b = 0.0;
  pose_marker.color.a = 0.8;
  
  marker_array.markers.push_back(pose_marker);
  
  // Algorithm type text marker
  visualization_msgs::msg::Marker text_marker;
  text_marker.header = current_pose_.header;
  text_marker.ns = "localization";
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
  text_marker.text = "Algorithm: " + algorithm_type_;
  
  marker_array.markers.push_back(text_marker);
  
  marker_pub_->publish(marker_array);
}

void ScanMatchingNode::publishMap()
{
  if (!map_cloud_ || map_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot publish empty map");
    return;
  }
  
  // Create a sparse version of the map for visualization
  // Use aggressive downsampling to prevent performance issues
  pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_map(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Apply voxel grid filter with large leaf size for sparse visualization
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(map_cloud_);
  voxel_filter.setLeafSize(1.0f, 1.0f, 1.0f);  // 1m voxels for sparse map
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<awsim_localization::ScanMatchingNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(awsim_localization::ScanMatchingNode)
