#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>

namespace awsim_path_planner {
  class HDMapManager;
  struct LaneInfo;
}

class LaneletVisualizerNode : public rclcpp::Node
{
public:
  LaneletVisualizerNode();
  
private:
  void publish_visualization();
  void publish_comprehensive_visualization();
  std::vector<const awsim_path_planner::LaneInfo*> select_lanes_for_visualization() const;
  
  std::unique_ptr<awsim_path_planner::HDMapManager> hd_map_manager_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::string map_frame_;
  bool show_boundaries_;
  bool show_centerlines_;
  bool show_lane_ids_;
  bool show_speed_limits_;
  double max_distance_;
  bool use_combined_markers_;
  int max_lanes_;
  
  // Additional feature visualization flags
  bool show_traffic_lights_;
  bool show_traffic_signs_;
  bool show_crosswalks_;
  bool show_road_markings_;
  bool show_regulatory_elements_;
};
