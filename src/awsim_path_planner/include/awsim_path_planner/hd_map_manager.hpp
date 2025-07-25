#ifndef AWSIM_PATH_PLANNER__HD_MAP_MANAGER_HPP_
#define AWSIM_PATH_PLANNER__HD_MAP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <string>
#include <memory>

namespace awsim_path_planner
{

struct LaneInfo
{
  int id;
  std::vector<std::pair<double, double>> centerline;
  std::vector<std::pair<double, double>> left_boundary;
  std::vector<std::pair<double, double>> right_boundary;
  double speed_limit;
  std::vector<int> predecessor_ids;
  std::vector<int> successor_ids;
  bool is_driveable;
};

struct MapBounds
{
  double min_x, max_x;
  double min_y, max_y;
};

class HDMapManager
{
public:
  explicit HDMapManager(rclcpp::Node* node);
  ~HDMapManager() = default;

  // Map loading and initialization
  bool load_map(const std::string& map_file_path);
  bool is_map_loaded() const { return map_loaded_; }
  
  // Query functions
  bool is_point_driveable(double x, double y) const;
  bool is_path_valid(const std::vector<geometry_msgs::msg::PoseStamped>& path) const;
  LaneInfo get_nearest_lane(double x, double y) const;
  std::vector<LaneInfo> get_lanes_in_region(double min_x, double min_y, 
                                            double max_x, double max_y) const;
  
  // Cost functions for path planning
  double get_lane_cost(double x, double y) const;
  double get_boundary_distance_cost(double x, double y) const;
  double get_speed_limit_cost(double x, double y, double desired_speed) const;
  
  // Map bounds
  MapBounds get_map_bounds() const { return map_bounds_; }
  
  // Path optimization
  std::vector<geometry_msgs::msg::PoseStamped> optimize_path_to_lanes(
    const std::vector<geometry_msgs::msg::PoseStamped>& input_path) const;
  
  // Visualization
  visualization_msgs::msg::MarkerArray get_map_visualization() const;

private:
  // Map parsing (simplified - would use actual Lanelet2 in production)
  bool parse_osm_file(const std::string& file_path);
  void extract_lanes_from_osm();
  void calculate_map_bounds();
  
  // Geometric utilities
  double point_to_line_distance(double px, double py,
                               double x1, double y1, double x2, double y2) const;
  bool is_point_in_polygon(double x, double y, 
                          const std::vector<std::pair<double, double>>& polygon) const;
  std::pair<double, double> get_closest_point_on_line(double px, double py,
                                                     double x1, double y1, 
                                                     double x2, double y2) const;
  
  // Lane processing
  void process_lane_connectivity();
  void generate_lane_boundaries();
  
  // ROS components
  rclcpp::Node* node_;
  
  // Map data
  std::vector<LaneInfo> lanes_;
  MapBounds map_bounds_;
  bool map_loaded_;
  
  // Configuration parameters
  double lane_width_;
  double boundary_buffer_;
  double driveable_area_buffer_;
  
  // Cost parameters
  double off_lane_penalty_;
  double boundary_proximity_penalty_;
  double speed_deviation_penalty_;
};

}  // namespace awsim_path_planner

#endif  // AWSIM_PATH_PLANNER__HD_MAP_MANAGER_HPP_
