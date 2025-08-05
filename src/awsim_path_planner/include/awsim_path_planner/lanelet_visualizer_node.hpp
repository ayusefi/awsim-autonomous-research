#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <memory>
#include <unordered_map>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_projection/LocalCartesian.h>

class LaneletVisualizerNode : public rclcpp::Node
{
public:
  LaneletVisualizerNode();
  
private:
  void publish_visualization();
  void publish_comprehensive_visualization();
  std::vector<lanelet::ConstLanelet> select_lanelets_for_visualization() const;
  
  lanelet::LaneletMapPtr lanelet_map_;
  std::unique_ptr<lanelet::projection::UtmProjector> projector_;
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
  
  // Coordinate transformation variables
  double map_origin_offset_x_;
  double map_origin_offset_y_;
  
  // Optimization: local coordinate lookup map
  std::unordered_map<int64_t, std::pair<double, double>> local_coord_cache_;
  
  // Coordinate transformation offset (calculated from local_x/local_y vs projected coordinates)
  double local_to_global_offset_x_;
  double local_to_global_offset_y_;
};
