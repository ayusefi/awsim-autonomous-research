#include "awsim_path_planner/hd_map_manager.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

namespace awsim_path_planner
{

HDMapManager::HDMapManager(rclcpp::Node* node)
: node_(node),
  map_loaded_(false),
  lane_width_(3.5),  // Standard lane width in meters
  boundary_buffer_(0.5),
  driveable_area_buffer_(2.0),
  off_lane_penalty_(100.0),
  boundary_proximity_penalty_(50.0),
  speed_deviation_penalty_(10.0)
{
  RCLCPP_INFO(node_->get_logger(), "HD Map Manager initialized");
}

bool HDMapManager::load_map(const std::string& map_file_path)
{
  RCLCPP_INFO(node_->get_logger(), "Loading HD map from: %s", map_file_path.c_str());
  
  // Clear existing map data
  lanes_.clear();
  map_loaded_ = false;
  
  // Check if file exists
  std::ifstream file(map_file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot open map file: %s", map_file_path.c_str());
    return false;
  }
  
  try {
    // Parse OSM file (simplified parsing for demonstration)
    if (!parse_osm_file(map_file_path)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse OSM file");
      return false;
    }
    
    // Extract lane information
    extract_lanes_from_osm();
    
    // Process lane connectivity
    process_lane_connectivity();
    
    // Generate lane boundaries
    generate_lane_boundaries();
    
    // Calculate map bounds
    calculate_map_bounds();
    
    map_loaded_ = true;
    RCLCPP_INFO(node_->get_logger(), "Successfully loaded HD map with %zu lanes", lanes_.size());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error loading map: %s", e.what());
    return false;
  }
  
  return true;
}

bool HDMapManager::parse_osm_file(const std::string& file_path)
{
  // Simplified OSM parsing - in a real implementation, you would use
  // a proper XML parser and Lanelet2 library
  
  std::ifstream file(file_path);
  std::string line;
  
  // For demonstration, create some sample lanes
  // In reality, this would parse the actual OSM XML structure
  
  // Sample lane 1: Straight road segment
  LaneInfo lane1;
  lane1.id = 1;
  lane1.speed_limit = 50.0;  // km/h
  lane1.is_driveable = true;
  
  // Create a simple straight centerline
  for (int i = 0; i <= 20; ++i) {
    double x = 3750.0 + i * 5.0;  // 100m long segment
    double y = 73700.0;
    lane1.centerline.emplace_back(x, y);
  }
  
  lanes_.push_back(lane1);
  
  // Sample lane 2: Curved road segment
  LaneInfo lane2;
  lane2.id = 2;
  lane2.speed_limit = 40.0;
  lane2.is_driveable = true;
  lane2.predecessor_ids = {1};
  
  // Create a curved segment
  for (int i = 0; i <= 15; ++i) {
    double t = static_cast<double>(i) / 15.0;
    double angle = t * M_PI / 4;  // 45 degree curve
    double x = 3850.0 + 30.0 * std::cos(angle);
    double y = 73700.0 + 30.0 * std::sin(angle);
    lane2.centerline.emplace_back(x, y);
  }
  
  lanes_.push_back(lane2);
  
  // Sample lane 3: Intersection approach
  LaneInfo lane3;
  lane3.id = 3;
  lane3.speed_limit = 30.0;
  lane3.is_driveable = true;
  lane3.predecessor_ids = {2};
  
  for (int i = 0; i <= 10; ++i) {
    double x = 3880.0 + i * 3.0;
    double y = 73730.0 + i * 2.0;
    lane3.centerline.emplace_back(x, y);
  }
  
  lanes_.push_back(lane3);
  
  RCLCPP_INFO(node_->get_logger(), "Parsed %zu sample lanes from map", lanes_.size());
  return true;
}

void HDMapManager::extract_lanes_from_osm()
{
  // In a real implementation, this would extract lane information
  // from the parsed OSM data structures
  RCLCPP_INFO(node_->get_logger(), "Extracted lane information from OSM data");
}

void HDMapManager::process_lane_connectivity()
{
  // Set up successor relationships based on predecessor relationships
  for (auto& lane : lanes_) {
    for (int pred_id : lane.predecessor_ids) {
      // Find predecessor lane and add this lane as successor
      auto pred_it = std::find_if(lanes_.begin(), lanes_.end(),
        [pred_id](const LaneInfo& l) { return l.id == pred_id; });
      
      if (pred_it != lanes_.end()) {
        pred_it->successor_ids.push_back(lane.id);
      }
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Processed lane connectivity");
}

void HDMapManager::generate_lane_boundaries()
{
  for (auto& lane : lanes_) {
    if (lane.centerline.size() < 2) continue;
    
    // Generate left and right boundaries parallel to centerline
    for (size_t i = 0; i < lane.centerline.size(); ++i) {
      double x = lane.centerline[i].first;
      double y = lane.centerline[i].second;
      
      // Calculate perpendicular direction
      double dx, dy;
      if (i == 0) {
        // Use direction to next point
        dx = lane.centerline[i + 1].first - x;
        dy = lane.centerline[i + 1].second - y;
      } else if (i == lane.centerline.size() - 1) {
        // Use direction from previous point
        dx = x - lane.centerline[i - 1].first;
        dy = y - lane.centerline[i - 1].second;
      } else {
        // Use average direction
        double dx1 = lane.centerline[i + 1].first - x;
        double dy1 = lane.centerline[i + 1].second - y;
        double dx2 = x - lane.centerline[i - 1].first;
        double dy2 = y - lane.centerline[i - 1].second;
        dx = (dx1 + dx2) / 2.0;
        dy = (dy1 + dy2) / 2.0;
      }
      
      // Normalize and rotate 90 degrees for perpendicular
      double length = std::sqrt(dx * dx + dy * dy);
      if (length > 0) {
        dx /= length;
        dy /= length;
        
        // Left boundary (90 degrees counterclockwise)
        double left_x = x - dy * lane_width_ / 2.0;
        double left_y = y + dx * lane_width_ / 2.0;
        lane.left_boundary.emplace_back(left_x, left_y);
        
        // Right boundary (90 degrees clockwise)
        double right_x = x + dy * lane_width_ / 2.0;
        double right_y = y - dx * lane_width_ / 2.0;
        lane.right_boundary.emplace_back(right_x, right_y);
      }
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Generated lane boundaries");
}

void HDMapManager::calculate_map_bounds()
{
  if (lanes_.empty()) {
    map_bounds_ = {0, 0, 0, 0};
    return;
  }
  
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  
  for (const auto& lane : lanes_) {
    for (const auto& point : lane.centerline) {
      min_x = std::min(min_x, point.first);
      max_x = std::max(max_x, point.first);
      min_y = std::min(min_y, point.second);
      max_y = std::max(max_y, point.second);
    }
  }
  
  // Add buffer
  double buffer = 50.0;  // 50m buffer
  map_bounds_ = {min_x - buffer, max_x + buffer, min_y - buffer, max_y + buffer};
  
  RCLCPP_INFO(node_->get_logger(), "Map bounds: [%.2f, %.2f] x [%.2f, %.2f]",
              map_bounds_.min_x, map_bounds_.max_x, 
              map_bounds_.min_y, map_bounds_.max_y);
}

bool HDMapManager::is_point_driveable(double x, double y) const
{
  if (!map_loaded_) return true;  // If no map, assume driveable
  
  // Check if point is within any driveable lane
  for (const auto& lane : lanes_) {
    if (!lane.is_driveable) continue;
    
    // Simple check: is point close to lane centerline?
    for (const auto& center_point : lane.centerline) {
      double distance = std::sqrt(
        (x - center_point.first) * (x - center_point.first) +
        (y - center_point.second) * (y - center_point.second));
      
      if (distance <= lane_width_ / 2.0 + driveable_area_buffer_) {
        return true;
      }
    }
  }
  
  return false;
}

bool HDMapManager::is_path_valid(const std::vector<geometry_msgs::msg::PoseStamped>& path) const
{
  if (!map_loaded_) return true;
  
  for (const auto& pose : path) {
    if (!is_point_driveable(pose.pose.position.x, pose.pose.position.y)) {
      return false;
    }
  }
  
  return true;
}

LaneInfo HDMapManager::get_nearest_lane(double x, double y) const
{
  if (lanes_.empty()) {
    return LaneInfo{};  // Return empty lane info
  }
  
  const LaneInfo* nearest_lane = &lanes_[0];
  double min_distance = std::numeric_limits<double>::max();
  
  for (const auto& lane : lanes_) {
    if (!lane.is_driveable) continue;
    
    for (const auto& center_point : lane.centerline) {
      double distance = std::sqrt(
        (x - center_point.first) * (x - center_point.first) +
        (y - center_point.second) * (y - center_point.second));
      
      if (distance < min_distance) {
        min_distance = distance;
        nearest_lane = &lane;
      }
    }
  }
  
  return *nearest_lane;
}

std::vector<LaneInfo> HDMapManager::get_lanes_in_region(double min_x, double min_y, 
                                                        double max_x, double max_y) const
{
  std::vector<LaneInfo> region_lanes;
  
  for (const auto& lane : lanes_) {
    bool lane_in_region = false;
    
    for (const auto& point : lane.centerline) {
      if (point.first >= min_x && point.first <= max_x &&
          point.second >= min_y && point.second <= max_y) {
        lane_in_region = true;
        break;
      }
    }
    
    if (lane_in_region) {
      region_lanes.push_back(lane);
    }
  }
  
  return region_lanes;
}

double HDMapManager::get_lane_cost(double x, double y) const
{
  if (!map_loaded_) return 0.0;
  
  // Return penalty if not on a driveable lane
  if (!is_point_driveable(x, y)) {
    return off_lane_penalty_;
  }
  
  return 0.0;  // No cost if on lane
}

double HDMapManager::get_boundary_distance_cost(double x, double y) const
{
  if (!map_loaded_) return 0.0;
  
  LaneInfo nearest_lane = get_nearest_lane(x, y);
  if (nearest_lane.centerline.empty()) return 0.0;
  
  // Find distance to nearest lane boundary
  double min_boundary_distance = std::numeric_limits<double>::max();
  
  for (const auto& left_point : nearest_lane.left_boundary) {
    double distance = std::sqrt(
      (x - left_point.first) * (x - left_point.first) +
      (y - left_point.second) * (y - left_point.second));
    min_boundary_distance = std::min(min_boundary_distance, distance);
  }
  
  for (const auto& right_point : nearest_lane.right_boundary) {
    double distance = std::sqrt(
      (x - right_point.first) * (x - right_point.first) +
      (y - right_point.second) * (y - right_point.second));
    min_boundary_distance = std::min(min_boundary_distance, distance);
  }
  
  // Apply penalty based on proximity to boundary
  if (min_boundary_distance < boundary_buffer_) {
    return boundary_proximity_penalty_ * (boundary_buffer_ - min_boundary_distance);
  }
  
  return 0.0;
}

double HDMapManager::get_speed_limit_cost(double x, double y, double desired_speed) const
{
  if (!map_loaded_) return 0.0;
  
  LaneInfo nearest_lane = get_nearest_lane(x, y);
  if (nearest_lane.centerline.empty()) return 0.0;
  
  double speed_difference = std::abs(desired_speed - nearest_lane.speed_limit);
  return speed_deviation_penalty_ * speed_difference / nearest_lane.speed_limit;
}

std::vector<geometry_msgs::msg::PoseStamped> HDMapManager::optimize_path_to_lanes(
  const std::vector<geometry_msgs::msg::PoseStamped>& input_path) const
{
  if (!map_loaded_ || input_path.empty()) {
    return input_path;
  }
  
  std::vector<geometry_msgs::msg::PoseStamped> optimized_path;
  
  for (const auto& pose : input_path) {
    geometry_msgs::msg::PoseStamped optimized_pose = pose;
    
    // Find nearest lane and project point onto centerline
    LaneInfo nearest_lane = get_nearest_lane(pose.pose.position.x, pose.pose.position.y);
    
    if (!nearest_lane.centerline.empty()) {
      // Find closest point on centerline
      double min_distance = std::numeric_limits<double>::max();
      std::pair<double, double> closest_point;
      
      for (const auto& center_point : nearest_lane.centerline) {
        double distance = std::sqrt(
          (pose.pose.position.x - center_point.first) * (pose.pose.position.x - center_point.first) +
          (pose.pose.position.y - center_point.second) * (pose.pose.position.y - center_point.second));
        
        if (distance < min_distance) {
          min_distance = distance;
          closest_point = center_point;
        }
      }
      
      // Move pose slightly towards centerline
      double blend_factor = 0.3;  // 30% towards centerline
      optimized_pose.pose.position.x = 
        pose.pose.position.x + blend_factor * (closest_point.first - pose.pose.position.x);
      optimized_pose.pose.position.y = 
        pose.pose.position.y + blend_factor * (closest_point.second - pose.pose.position.y);
    }
    
    optimized_path.push_back(optimized_pose);
  }
  
  RCLCPP_INFO(node_->get_logger(), "Optimized path using HD map lane constraints");
  return optimized_path;
}

double HDMapManager::point_to_line_distance(double px, double py,
                                           double x1, double y1, double x2, double y2) const
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  double length_sq = dx * dx + dy * dy;
  
  if (length_sq == 0) {
    // Point to point distance
    return std::sqrt((px - x1) * (px - x1) + (py - y1) * (py - y1));
  }
  
  // Calculate projection parameter
  double t = ((px - x1) * dx + (py - y1) * dy) / length_sq;
  t = std::max(0.0, std::min(1.0, t));  // Clamp to line segment
  
  // Calculate closest point on line
  double closest_x = x1 + t * dx;
  double closest_y = y1 + t * dy;
  
  // Return distance to closest point
  return std::sqrt((px - closest_x) * (px - closest_x) + (py - closest_y) * (py - closest_y));
}

bool HDMapManager::is_point_in_polygon(double x, double y, 
                                      const std::vector<std::pair<double, double>>& polygon) const
{
  if (polygon.size() < 3) return false;
  
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    double xi = polygon[i].first, yi = polygon[i].second;
    double xj = polygon[j].first, yj = polygon[j].second;
    
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  
  return inside;
}

std::pair<double, double> HDMapManager::get_closest_point_on_line(double px, double py,
                                                                 double x1, double y1, 
                                                                 double x2, double y2) const
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  double length_sq = dx * dx + dy * dy;
  
  if (length_sq == 0) {
    return {x1, y1};  // Line is actually a point
  }
  
  double t = ((px - x1) * dx + (py - y1) * dy) / length_sq;
  t = std::max(0.0, std::min(1.0, t));  // Clamp to line segment
  
  return {x1 + t * dx, y1 + t * dy};
}

visualization_msgs::msg::MarkerArray HDMapManager::get_map_visualization() const
{
  visualization_msgs::msg::MarkerArray markers;
  
  if (!map_loaded_) return markers;
  
  int marker_id = 1000;  // Start from high ID to avoid conflicts
  
  for (const auto& lane : lanes_) {
    // Lane centerline
    visualization_msgs::msg::Marker centerline_marker;
    centerline_marker.header.frame_id = "map";
    centerline_marker.header.stamp = node_->get_clock()->now();
    centerline_marker.ns = "hd_map_centerlines";
    centerline_marker.id = marker_id++;
    centerline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    centerline_marker.action = visualization_msgs::msg::Marker::ADD;
    centerline_marker.scale.x = 0.2;
    centerline_marker.color.r = 1.0;
    centerline_marker.color.g = 1.0;
    centerline_marker.color.b = 0.0;
    centerline_marker.color.a = 0.8;
    
    for (const auto& point : lane.centerline) {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = point.first;
      marker_point.y = point.second;
      marker_point.z = 0.05;
      centerline_marker.points.push_back(marker_point);
    }
    
    markers.markers.push_back(centerline_marker);
    
    // Lane boundaries
    if (!lane.left_boundary.empty()) {
      visualization_msgs::msg::Marker left_boundary_marker;
      left_boundary_marker.header.frame_id = "map";
      left_boundary_marker.header.stamp = node_->get_clock()->now();
      left_boundary_marker.ns = "hd_map_boundaries";
      left_boundary_marker.id = marker_id++;
      left_boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      left_boundary_marker.action = visualization_msgs::msg::Marker::ADD;
      left_boundary_marker.scale.x = 0.1;
      left_boundary_marker.color.r = 0.8;
      left_boundary_marker.color.g = 0.8;
      left_boundary_marker.color.b = 0.8;
      left_boundary_marker.color.a = 0.6;
      
      for (const auto& point : lane.left_boundary) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.first;
        marker_point.y = point.second;
        marker_point.z = 0.02;
        left_boundary_marker.points.push_back(marker_point);
      }
      
      markers.markers.push_back(left_boundary_marker);
    }
    
    if (!lane.right_boundary.empty()) {
      visualization_msgs::msg::Marker right_boundary_marker;
      right_boundary_marker.header.frame_id = "map";
      right_boundary_marker.header.stamp = node_->get_clock()->now();
      right_boundary_marker.ns = "hd_map_boundaries";
      right_boundary_marker.id = marker_id++;
      right_boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      right_boundary_marker.action = visualization_msgs::msg::Marker::ADD;
      right_boundary_marker.scale.x = 0.1;
      right_boundary_marker.color.r = 0.8;
      right_boundary_marker.color.g = 0.8;
      right_boundary_marker.color.b = 0.8;
      right_boundary_marker.color.a = 0.6;
      
      for (const auto& point : lane.right_boundary) {
        geometry_msgs::msg::Point marker_point;
        marker_point.x = point.first;
        marker_point.y = point.second;
        marker_point.z = 0.02;
        right_boundary_marker.points.push_back(marker_point);
      }
      
      markers.markers.push_back(right_boundary_marker);
    }
  }
  
  return markers;
}

}  // namespace awsim_path_planner
