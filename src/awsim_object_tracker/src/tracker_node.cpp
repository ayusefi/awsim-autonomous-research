#include "awsim_object_tracker/tracker_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <chrono>

namespace awsim_object_tracker {


TrackerNode::TrackerNode() : TrackerNode(rclcpp::NodeOptions()) {}

TrackerNode::TrackerNode(const rclcpp::NodeOptions & options)
  : Node("awsim_object_tracker_node", options), marker_id_(0) {
  // Initialize TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Declare parameters
  declareParameters();
  // Get parameters
  getParameters();
  // Initialize tracker and perception pipeline
  initialize();
  // Create subscribers and publishers
  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, 10, std::bind(&TrackerNode::pointCloudCallback, this, std::placeholders::_1));
  tracked_objects_pub_ = create_publisher<multi_object_tracker_msgs::msg::TrackedObject>(
    output_topic_, 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_, 10);
  RCLCPP_INFO(get_logger(), "Multi-Object Tracker Node initialized");
}

void TrackerNode::declareParameters() {
  declare_parameter("input_topic", "/input_point_cloud");
  declare_parameter("output_topic", "/tracked_objects");
  declare_parameter("marker_topic", "/tracked_objects_markers");
  declare_parameter("target_frame", "map");
  declare_parameter("max_distance", 3.0);
  declare_parameter("dt", 0.1);
  declare_parameter("dbscan_eps", 0.7);
  declare_parameter("dbscan_min_points", 10);
  declare_parameter("filter_min_volume", 1.0);
  declare_parameter("filter_max_volume", 100.0);
  declare_parameter("filter_min_points", 20);
  declare_parameter("filter_max_points", 5000);
}

void TrackerNode::getParameters() {
  input_topic_ = get_parameter("input_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();
  marker_topic_ = get_parameter("marker_topic").as_string();
  target_frame_ = get_parameter("target_frame").as_string();
  max_distance_ = get_parameter("max_distance").as_double();
  dt_ = get_parameter("dt").as_double();
  dbscan_eps_ = get_parameter("dbscan_eps").as_double();
  dbscan_min_points_ = get_parameter("dbscan_min_points").as_int();
  filter_min_volume_ = get_parameter("filter_min_volume").as_double();
  filter_max_volume_ = get_parameter("filter_max_volume").as_double();
  filter_min_points_ = get_parameter("filter_min_points").as_int();
  filter_max_points_ = get_parameter("filter_max_points").as_int();
}

void TrackerNode::initialize() {
  // Configure perception pipeline
  PerceptionPipeline::Config pipeline_config;
  pipeline_config.dbscan_eps = dbscan_eps_;
  pipeline_config.dbscan_min_points = dbscan_min_points_;
  pipeline_config.filter_min_volume = filter_min_volume_;
  pipeline_config.filter_max_volume = filter_max_volume_;
  pipeline_config.filter_min_points = filter_min_points_;
  pipeline_config.filter_max_points = filter_max_points_;
  
  pipeline_ = std::make_shared<PerceptionPipeline>(pipeline_config);
  
  // Initialize tracker
  tracker_ = std::make_shared<MultiObjectTracker>(max_distance_, dt_);
}

void TrackerNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  
  // Check if we have a valid TF transform
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      target_frame_, msg->header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Could not transform %s to %s: %s", 
                         msg->header.frame_id.c_str(), 
                         target_frame_.c_str(), ex.what());
    return;
  }
  
  // Transform point cloud to target frame
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, transform_stamped);
  
  // Update header to target frame
  transformed_cloud.header.frame_id = target_frame_;
  
  // Convert to PCL point cloud for processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_cloud, *cloud);
  
  // Process point cloud and get detections
  auto raw_detections = pipeline_->processPointCloud(cloud);
  
  // Convert raw detections to shared_ptr
  std::vector<std::shared_ptr<Detection>> detections;
  for (const auto& detection : raw_detections) {
    detections.push_back(std::make_shared<Detection>(detection));
  }
  
  // Update tracker with detections
  tracker_->update(detections);
  
  // Publish tracked objects with reduced frequency
  static int counter = 0;
  if (++counter % 2 == 0) {  // Publish every other frame to reduce load
    publishTrackedObjects(transformed_cloud.header);
  }
  
  // Publish markers with minimal data
  publishMarkers(transformed_cloud.header);
}

void TrackerNode::publishTrackedObjects(const std_msgs::msg::Header& header) {
  auto tracks = tracker_->getActiveTracks();  // Only publish active tracks for performance
  
  for (const auto& track : tracks) {
    // Create tracked object message
    multi_object_tracker_msgs::msg::TrackedObject tracked_obj;
    tracked_obj.header = header;
    tracked_obj.id = track->id;
    
    // Position
    auto pos = track->getPosition();
    tracked_obj.position.x = pos.x();
    tracked_obj.position.y = pos.y();
    tracked_obj.position.z = pos.z();
    
    // Orientation
    auto orientation = track->getOrientation();
    tracked_obj.orientation.x = orientation.x();
    tracked_obj.orientation.y = orientation.y();
    tracked_obj.orientation.z = orientation.z();
    tracked_obj.orientation.w = orientation.w();
    
    // Velocity
    auto vel = track->getVelocity();
    tracked_obj.velocity.x = vel.x();
    tracked_obj.velocity.y = vel.y();
    tracked_obj.velocity.z = vel.z();
    
    // Angular velocity
    auto ang_vel = track->getAngularVelocity();
    tracked_obj.angular_velocity.x = ang_vel.x();
    tracked_obj.angular_velocity.y = ang_vel.y();
    tracked_obj.angular_velocity.z = ang_vel.z();
    
    // Dimensions (from last detection if available)
    if (track->last_detection) {
      auto dims = track->last_detection->dimensions;
      tracked_obj.dimensions.x = dims.x();
      tracked_obj.dimensions.y = dims.y();
      tracked_obj.dimensions.z = dims.z();
    } else {
      tracked_obj.dimensions.x = 0.0;
      tracked_obj.dimensions.y = 0.0;
      tracked_obj.dimensions.z = 0.0;
    }
    
    // Track state and info
    tracked_obj.state = trackStateToString(track->state);
    tracked_obj.age = track->age;
    tracked_obj.time_since_update = track->time_since_update;
    
    // Publish
    tracked_objects_pub_->publish(tracked_obj);
  }
}

void TrackerNode::publishMarkers(const std_msgs::msg::Header& header) {
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Colors for different track states
  std::map<TrackState, std::array<double, 3>> colors = {
    {TrackState::TENTATIVE, {1.0, 0.5, 0.0}},  // Orange
    {TrackState::CONFIRMED, {0.0, 1.0, 0.0}},  // Green
  };
  
  auto tracks = tracker_->getActiveTracks();  // Only get active tracks
  std::set<int> current_marker_ids;
  
  for (const auto& track : tracks) {
    // Skip tracks that haven't been updated recently (avoid trail effect)
    if (track->time_since_update > 1) {  // More aggressive filtering
      continue;
    }
    
    current_marker_ids.insert(track->id);
    
    // Get position and dimensions
    auto pos = track->getPosition();
    
    Eigen::Vector3d dims;
    if (track->last_detection) {
      dims = track->last_detection->dimensions;
    } else {
      dims = Eigen::Vector3d(1.0, 1.0, 1.0);  // Default size
    }
    
    // Create cube marker for bounding box
    visualization_msgs::msg::Marker cube_marker;
    cube_marker.header = header;
    cube_marker.ns = "tracked_objects";
    cube_marker.id = track->id;  // Use track ID instead of incrementing counter
    cube_marker.type = visualization_msgs::msg::Marker::CUBE;
    cube_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position and orientation
    cube_marker.pose.position.x = pos.x();
    cube_marker.pose.position.y = pos.y();
    cube_marker.pose.position.z = pos.z();
    
    // Use the track's orientation
    auto orientation = track->getOrientation();
    cube_marker.pose.orientation.x = 0.0;
    cube_marker.pose.orientation.y = 0.0;
    cube_marker.pose.orientation.z = orientation.z();
    cube_marker.pose.orientation.w = orientation.w();
    
    // Scale
    cube_marker.scale.x = dims.x();
    cube_marker.scale.y = dims.y();
    cube_marker.scale.z = dims.z();
    
    // Set color to yellow for all bounding boxes
    cube_marker.color.r = 1.0;   // Red component
    cube_marker.color.g = 1.0;   // Green component  
    cube_marker.color.b = 0.0;   // Blue component (yellow = red + green)
    cube_marker.color.a = 0.8;   // More opaque for better visibility
    
    // Very short lifetime to stay synchronized
    cube_marker.lifetime.sec = 0.1;

    marker_array.markers.push_back(cube_marker);
  }
  
  // Delete markers for tracks that no longer exist
  for (int old_id : previous_marker_ids_) {
    if (current_marker_ids.find(old_id) == current_marker_ids.end()) {
      // Delete cube marker
      visualization_msgs::msg::Marker delete_cube;
      delete_cube.header = header;
      delete_cube.ns = "tracked_objects";
      delete_cube.id = old_id;
      delete_cube.action = visualization_msgs::msg::Marker::DELETE;
      marker_array.markers.push_back(delete_cube);
    }
  }
  
  // Update previous marker IDs
  previous_marker_ids_ = current_marker_ids;
  
  // Publish all markers
  markers_pub_->publish(marker_array);
}

}  // namespace awsim_object_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(awsim_object_tracker::TrackerNode)