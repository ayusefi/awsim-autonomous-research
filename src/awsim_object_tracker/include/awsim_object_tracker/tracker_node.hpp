#ifndef AWSIM_OBJECT_TRACKER_TRACKER_NODE_HPP
#define AWSIM_OBJECT_TRACKER_TRACKER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <multi_object_tracker_msgs/msg/tracked_object.hpp>
#include <multi_object_tracker_msgs/msg/tracked_object_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <string>
#include <set>
#include <map>
#include <array>

#include "awsim_object_tracker/object_tracker.hpp"
#include "awsim_object_tracker/perception_pipeline.hpp"

namespace awsim_object_tracker {

class TrackerNode : public rclcpp::Node {
public:
  TrackerNode();
  explicit TrackerNode(const rclcpp::NodeOptions & options);

private:
  void declareParameters();
  void getParameters();
  void initialize();
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishTrackedObjects(const std_msgs::msg::Header& header);
  void publishMarkers(const std_msgs::msg::Header& header);

  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string target_frame_;
  double max_distance_;
  double dt_;
  double dbscan_eps_;
  int dbscan_min_points_;
  double filter_min_volume_;
  double filter_max_volume_;
  int filter_min_points_;
  int filter_max_points_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<multi_object_tracker_msgs::msg::TrackedObjectArray>::SharedPtr tracked_objects_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<PerceptionPipeline> pipeline_;
  std::shared_ptr<MultiObjectTracker> tracker_;

  std::set<int> previous_marker_ids_;
  int marker_id_;
};

std::string trackStateToString(TrackState state);

} // namespace awsim_object_tracker

#endif // AWSIM_OBJECT_TRACKER_TRACKER_NODE_HPP
