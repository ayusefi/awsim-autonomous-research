#ifndef AWSIM_PATH_PLANNER__ROUTE_PLANNER_HPP_
#define AWSIM_PATH_PLANNER__ROUTE_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>

#include <memory>
#include <vector>
#include <string>
#include <optional>
#include <unordered_map>

namespace awsim_path_planner
{


// Define the projection result structure
struct ProjectionResult {
    size_t segment_index;           // Index of the segment where projection occurs
    double t;                       // Parameter [0,1] within the segment
    geometry_msgs::msg::Point projection_point;  // Projected point
    double distance;                // Distance from the point to the projection
};


struct RoutePlannerParam
{
  std::string map_file_path;
  std::string traffic_rules_name;
  std::string participant_name;
  double goal_search_radius;
  double centerline_resolution;
  bool enable_lane_change;
  std::string map_frame;
};

class RoutePlanner
{
public:
  explicit RoutePlanner(rclcpp::Node * node);
  ~RoutePlanner() = default;

  // Initialize with lanelet2 map
  bool initialize(const RoutePlannerParam & param);
  
  // Plan route from start to goal pose, finding centerline that matches goal
  std::optional<autoware_planning_msgs::msg::Path> plan_route(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose);
  
  // Convert route to centerline path
  autoware_planning_msgs::msg::Path convert_route_to_path(
    const lanelet::routing::Route & route,
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose);
  
  // Find nearest lanelet to a given pose
  std::optional<lanelet::ConstLanelet> find_nearest_lanelet(
    const geometry_msgs::msg::PoseStamped & pose,
    double search_radius = 10.0);
  
  // Check if map is loaded
  bool is_map_loaded() const { return map_loaded_; }
  
  // Get visualization markers for lanelet map
  visualization_msgs::msg::MarkerArray get_lanelet_visualization() const;
  
  // Get the lanelet map
  lanelet::LaneletMapPtr get_map() const { return lanelet_map_; }
  
  // Transform coordinates from lat/lon to local frame
  geometry_msgs::msg::Point convert_to_local_coordinates(double lat, double lon) const;
  
  // Transform coordinates from local frame to lat/lon
  std::pair<double, double> convert_to_global_coordinates(
    const geometry_msgs::msg::Point & local_point) const;

private:
  // Convert lanelet centerline to path points
  std::vector<autoware_planning_msgs::msg::PathPoint> create_centerline_path(
    const lanelet::ConstLanelet & lanelet,
    double resolution = 0.5);

  ProjectionResult project_point_to_path(
    const std::vector<autoware_planning_msgs::msg::PathPoint>& path_points,
    const geometry_msgs::msg::Point& point);
  
  // Find goal lanelet that best matches the goal pose orientation
  std::optional<lanelet::ConstLanelet> find_goal_lanelet(
    const geometry_msgs::msg::PoseStamped & goal_pose,
    double search_radius);
  
  // Calculate distance between pose and lanelet centerline
  double calculate_distance_to_centerline(
    const geometry_msgs::msg::PoseStamped & pose,
    const lanelet::ConstLanelet & lanelet);
  
  // Calculate distance using raw coordinates (for coordinate system matching)
  double calculate_distance_to_centerline_raw(
    double pose_x, double pose_y,
    const lanelet::ConstLanelet & lanelet);
    
  // Calculate distance using global coordinates (standard approach)
  double calculate_distance_to_centerline_global(
    double pose_x, double pose_y,
    const lanelet::ConstLanelet & lanelet);
  
  // Calculate orientation similarity between pose and lanelet direction
  double calculate_orientation_similarity(
    const geometry_msgs::msg::PoseStamped & pose,
    const lanelet::ConstLanelet & lanelet);
  
  // Create path point from geometry
  autoware_planning_msgs::msg::PathPoint create_path_point(
    const lanelet::BasicPoint3d & point,
    double velocity = 0.0);

  // Fallback path creation when lanelet2 map is not available
  std::optional<autoware_planning_msgs::msg::Path> create_fallback_path(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose);

  // Node pointer for logging
  rclcpp::Node * node_;
  
  // Lanelet2 components
  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  std::unique_ptr<lanelet::projection::UtmProjector> projector_;
  
  // Parameters
  RoutePlannerParam param_;
  bool map_loaded_;
  
  // Map bounds for coordinate transformation
  double map_origin_lat_;
  double map_origin_lon_;
  double map_origin_x_;
  double map_origin_y_;
  
  // Coordinate transformation offsets (same as HDMapManager)
  double map_origin_offset_x_;
  double map_origin_offset_y_;
  
  // Standard approach: local coordinate cache and automatic offset
  std::unordered_map<int64_t, std::pair<double, double>> local_coord_cache_;
  double local_to_global_offset_x_;
  double local_to_global_offset_y_;
};

}  // namespace awsim_path_planner

#endif  // AWSIM_PATH_PLANNER__ROUTE_PLANNER_HPP_
