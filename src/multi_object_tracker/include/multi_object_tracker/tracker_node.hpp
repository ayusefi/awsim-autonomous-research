#ifndef MULTI_OBJECT_TRACKER_TRACKER_NODE_HPP
#define MULTI_OBJECT_TRACKER_TRACKER_NODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "multi_object_tracker_msgs/msg/tracked_object.hpp"
#include "multi_object_tracker/perception_pipeline.hpp"
#include "multi_object_tracker/object_tracker.hpp"

namespace multi_object_tracker {

/**
 * @brief ROS2 Node for multi-object tracking
 */
class TrackerNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Tracker Node object
   */
  TrackerNode();
  TrackerNode(const rclcpp::NodeOptions & options);

private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  // Publishers
  rclcpp::Publisher<multi_object_tracker_msgs::msg::TrackedObject>::SharedPtr tracked_objects_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string target_frame_;
  double max_distance_;
  double dt_;
  double dbscan_eps_;
  int dbscan_min_points_;
  double filter_min_volume_;
  int filter_min_points_;

  // TF2 for frame transformations
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Tracker and perception pipeline
  std::shared_ptr<MultiObjectTracker> tracker_;
  std::shared_ptr<PerceptionPipeline> pipeline_;

  // Marker ID counter
  int marker_id_;
  
  // Track previous marker IDs for cleanup
  std::set<int> previous_marker_ids_;

  /**
   * @brief Declare parameters
   */
  void declareParameters();

  /**
   * @brief Get parameters
   */
  void getParameters();

  /**
   * @brief Initialize tracker and perception pipeline
   */
  void initialize();

  /**
   * @brief Callback for point cloud messages
   * 
   * @param msg PointCloud2 message
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Publish tracked objects as custom messages
   * 
   * @param header Header from the input point cloud
   */
  void publishTrackedObjects(const std_msgs::msg::Header& header);

  /**
   * @brief Publish visualization markers for RViz
   * 
   * @param header Header from the input point cloud
   */
  void publishMarkers(const std_msgs::msg::Header& header);
};

}  // namespace multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER_TRACKER_NODE_HPP