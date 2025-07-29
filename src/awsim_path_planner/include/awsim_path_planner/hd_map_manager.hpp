#ifndef AWSIM_PATH_PLANNER__HD_MAP_MANAGER_HPP_
#define AWSIM_PATH_PLANNER__HD_MAP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>

namespace awsim_path_planner
{

enum class LaneType {
  DRIVING,
  PARKING,
  SHOULDER,
  SIDEWALK,
  BICYCLE,
  RESTRICTED
};

enum class TurnDirection {
  STRAIGHT,
  LEFT,
  RIGHT,
  U_TURN
};

struct TrafficRule {
  double speed_limit;  // m/s
  bool one_way;
  std::vector<TurnDirection> allowed_turns;
  bool traffic_light;  // Has traffic light
  bool stop_sign;      // Has stop sign
  bool yield_sign;     // Has yield sign
};

struct LaneInfo
{
  int id;
  LaneType type;
  std::vector<std::pair<double, double>> centerline;
  std::vector<std::pair<double, double>> left_boundary;
  std::vector<std::pair<double, double>> right_boundary;
  TrafficRule traffic_rule;
  std::vector<int> predecessor_ids;
  std::vector<int> successor_ids;
  std::vector<int> left_neighbor_ids;
  std::vector<int> right_neighbor_ids;
  bool is_driveable;
  double width;
  double length;
  std::string road_name;
  
  // Lanelet2 specific
  long left_boundary_way_id;
  long right_boundary_way_id;
  LaneType lane_type;
};

struct OSMNode {
  long id;
  double lat, lon;
  double x, y;  // Converted to local coordinates
  double local_x, local_y;  // Lanelet2 local coordinates
  std::unordered_map<std::string, std::string> tags;
};

struct OSMWay {
  long id;
  std::vector<long> node_refs;
  std::unordered_map<std::string, std::string> tags;
};

struct OSMRelationMember {
  std::string type;  // "way", "node", "relation"
  std::string role;  // "left", "right", "regulatory_element", etc.
  long ref;          // ID of the referenced element
};

struct OSMRelation {
  long id;
  std::vector<OSMRelationMember> members;
  std::unordered_map<std::string, std::string> tags;
};

struct MapBounds
{
  double min_x, max_x;
  double min_y, max_y;
};

// Additional HD Map Features
struct TrafficLight {
  long id;
  double x, y, z;
  std::string state;  // red, yellow, green, red_yellow_green
  std::vector<long> controlled_lane_ids;
  double height;
};

struct TrafficSign {
  long id;
  double x, y, z;
  std::string sign_type;  // stop_sign, speed_limit, yield, etc.
  std::string value;      // speed limit value, etc.
  std::vector<long> affected_lane_ids;
  double height;
};

struct Crosswalk {
  long id;
  std::vector<std::pair<double, double>> boundary;
  std::string crossing_type;
  bool has_traffic_light;
  long traffic_light_id;
};

struct RoadMarking {
  long id;
  std::vector<std::pair<double, double>> geometry;
  std::string marking_type;  // solid, dashed, etc.
  std::string color;
  double width;
};

struct RegulatoryElement {
  long id;
  std::string subtype;  // right_of_way, stop_sign, traffic_light
  std::vector<long> yield_lane_ids;
  std::vector<long> right_of_way_lane_ids;
  std::vector<long> stop_line_ids;
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
  
  // Enhanced OSM-aware queries
  bool is_turn_allowed(const geometry_msgs::msg::PoseStamped& from,
                      const geometry_msgs::msg::PoseStamped& to) const;
  double get_speed_limit_at_point(double x, double y) const;
  std::vector<geometry_msgs::msg::PoseStamped> get_lane_following_path(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) const;
  bool has_traffic_control_at_point(double x, double y, std::string& control_type) const;
  
  // Lane-aware path planning integration
  std::vector<int> get_lane_sequence(const geometry_msgs::msg::PoseStamped& start,
                                    const geometry_msgs::msg::PoseStamped& goal) const;
  bool is_lane_change_safe(int from_lane_id, int to_lane_id, 
                          const geometry_msgs::msg::PoseStamped& position) const;
  
  // Cost functions for path planning
  double get_lane_cost(double x, double y) const;
  double get_boundary_distance_cost(double x, double y) const;
  double get_speed_limit_cost(double x, double y, double desired_speed) const;
  
  // Map bounds
  MapBounds get_map_bounds() const { return map_bounds_; }
  
  // Path optimization
  std::vector<geometry_msgs::msg::PoseStamped> optimize_path_to_lanes(
    const std::vector<geometry_msgs::msg::PoseStamped>& input_path) const;
  
  // Visualization access methods
  const std::vector<LaneInfo>& get_lanes() const { return lanes_; }
  
  // Additional feature access methods
  const std::vector<TrafficLight>& get_traffic_lights() const { return traffic_lights_; }
  const std::vector<TrafficSign>& get_traffic_signs() const { return traffic_signs_; }
  const std::vector<Crosswalk>& get_crosswalks() const { return crosswalks_; }
  const std::vector<RoadMarking>& get_road_markings() const { return road_markings_; }
  const std::vector<RegulatoryElement>& get_regulatory_elements() const { return regulatory_elements_; }
  
  // Visualization
  visualization_msgs::msg::MarkerArray get_map_visualization() const;

private:
  // Enhanced OSM parsing
  bool parse_osm_file(const std::string& file_path);
  void extract_lanes_from_osm();
  void calculate_map_bounds();
  
  // OSM data processing
  bool parse_osm_nodes(const std::string& xml_content);
  bool parse_osm_ways(const std::string& xml_content);
  bool parse_osm_relations(const std::string& xml_content);
  void convert_coordinates();  // Convert lat/lon to local coordinates
  
  // Coordinate transformation (same as localization system)
  std::pair<double, double> transform_coordinates(double local_x, double local_y) const;
  
  LaneType determine_lane_type(const std::unordered_map<std::string, std::string>& tags) const;
  TrafficRule extract_traffic_rules(const std::unordered_map<std::string, std::string>& tags) const;
  double parse_speed_limit(const std::string& speed_str) const;
  
  // Lane processing
  void process_lane_connectivity();
  void generate_lane_boundaries();
  void build_lane_graph();
  std::vector<std::pair<double, double>> interpolate_centerline(
    const std::vector<long>& node_refs) const;
  
  // Path planning utilities
  std::vector<int> find_shortest_lane_path(int start_lane_id, int goal_lane_id) const;
  double calculate_lane_transition_cost(int from_lane_id, int to_lane_id) const;
  
  // Geometric utilities
  double point_to_line_distance(double px, double py,
                               double x1, double y1, double x2, double y2) const;
  bool is_point_in_polygon(double x, double y, 
                          const std::vector<std::pair<double, double>>& polygon) const;
  std::pair<double, double> get_closest_point_on_line(double px, double py,
                                                     double x1, double y1, 
                                                     double x2, double y2) const;
  double calculate_heading(const std::pair<double, double>& p1,
                          const std::pair<double, double>& p2) const;
  double calculate_lane_length(const std::vector<std::pair<double, double>>& centerline) const;
  
  // Lane utility methods
  bool is_position_in_lane(double x, double y, const LaneInfo& lane) const;
  double calculate_distance_to_lane_centerline(double x, double y, const LaneInfo& lane) const;
  
  // Additional feature parsing methods
  void parse_traffic_lights();
  void parse_traffic_signs();  
  void parse_crosswalks();
  void parse_road_markings();
  void parse_regulatory_elements();
  
  // Lane path planning helper functions
  std::vector<int> find_lane_sequence(int start_lane_id, int goal_lane_id) const;
  std::vector<geometry_msgs::msg::PoseStamped> create_path_along_lane(
    const LaneInfo& lane, 
    const geometry_msgs::msg::PoseStamped& start_pose,
    const geometry_msgs::msg::PoseStamped& goal_pose) const;
  
  // ROS components
  rclcpp::Node* node_;
  
  // OSM data storage
  std::unordered_map<long, OSMNode> osm_nodes_;
  std::unordered_map<long, OSMWay> osm_ways_;
  std::unordered_map<long, OSMRelation> osm_relations_;
  
  // Processed map data
  std::vector<LaneInfo> lanes_;
  std::unordered_map<int, std::vector<int>> lane_graph_;  // Adjacency list
  MapBounds map_bounds_;
  bool map_loaded_;
  std::string map_file_path_;  // Store file path for re-reading relations
  
  // Additional HD Map features
  std::vector<TrafficLight> traffic_lights_;
  std::vector<TrafficSign> traffic_signs_;
  std::vector<Crosswalk> crosswalks_;
  std::vector<RoadMarking> road_markings_;
  std::vector<RegulatoryElement> regulatory_elements_;
  
  // Coordinate conversion parameters
  double origin_lat_, origin_lon_;  // Reference point for coordinate conversion
  
  // Map coordinate transformation (same as localization system)
  double map_origin_offset_x_;  // X offset for coordinate transformation
  double map_origin_offset_y_;  // Y offset for coordinate transformation
  double map_origin_offset_z_;  // Z offset for coordinate transformation
  
  // Configuration parameters
  double lane_width_;
  double boundary_buffer_;
  double driveable_area_buffer_;
  
  // Cost parameters
  double off_lane_penalty_;
  double boundary_proximity_penalty_;
  double speed_deviation_penalty_;
  double wrong_way_penalty_;
  double traffic_rule_violation_penalty_;
};

}  // namespace awsim_path_planner

#endif  // AWSIM_PATH_PLANNER__HD_MAP_MANAGER_HPP_
