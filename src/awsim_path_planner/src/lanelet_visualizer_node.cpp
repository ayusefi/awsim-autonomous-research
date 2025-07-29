#include "awsim_path_planner/lanelet_visualizer_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "awsim_path_planner/hd_map_manager.hpp"

LaneletVisualizerNode::LaneletVisualizerNode() : Node("lanelet_visualizer_node")
{
  // Declare parameters
  this->declare_parameter<std::string>("hd_map_path", 
    "/home/abdullah/workspaces/career_sprint/awsim-autonomous-research/shinjuku_map/map/lanelet2_map.osm");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<double>("visualization_update_rate", 0.5); // Slower update rate
  this->declare_parameter<bool>("show_lane_boundaries", true);
  this->declare_parameter<bool>("show_centerlines", true);
  this->declare_parameter<bool>("show_lane_ids", false); // Disabled by default
  this->declare_parameter<bool>("show_speed_limits", false); // Disabled by default
  this->declare_parameter<double>("max_visualization_distance", 200.0); // Only show nearby lanes
  this->declare_parameter<bool>("use_combined_markers", true); // Combine multiple lanes into single markers
  this->declare_parameter<int>("max_lanes_to_visualize", 50); // Hard limit on number of lanes
  
  // Declare additional feature parameters
  this->declare_parameter<bool>("show_traffic_lights", false);
  this->declare_parameter<bool>("show_traffic_signs", false);
  this->declare_parameter<bool>("show_crosswalks", false);
  this->declare_parameter<bool>("show_road_markings", false);
  this->declare_parameter<bool>("show_regulatory_elements", false);
  
  // Get parameters
  std::string hd_map_path = this->get_parameter("hd_map_path").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  double update_rate = this->get_parameter("visualization_update_rate").as_double();
  show_boundaries_ = this->get_parameter("show_lane_boundaries").as_bool();
  show_centerlines_ = this->get_parameter("show_centerlines").as_bool();
  show_lane_ids_ = this->get_parameter("show_lane_ids").as_bool();
  show_speed_limits_ = this->get_parameter("show_speed_limits").as_bool();
  max_distance_ = this->get_parameter("max_visualization_distance").as_double();
  use_combined_markers_ = this->get_parameter("use_combined_markers").as_bool();
  max_lanes_ = this->get_parameter("max_lanes_to_visualize").as_int();
  
  // Get additional feature parameters
  show_traffic_lights_ = this->get_parameter("show_traffic_lights").as_bool();
  show_traffic_signs_ = this->get_parameter("show_traffic_signs").as_bool();
  show_crosswalks_ = this->get_parameter("show_crosswalks").as_bool();
  show_road_markings_ = this->get_parameter("show_road_markings").as_bool();
  show_regulatory_elements_ = this->get_parameter("show_regulatory_elements").as_bool();
  
  // Initialize HD map manager
  hd_map_manager_ = std::make_unique<awsim_path_planner::HDMapManager>(this);
  
  // Load the map
  if (!hd_map_manager_->load_map(hd_map_path)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load HD map from: %s", hd_map_path.c_str());
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Successfully loaded HD map with %zu lanes", 
              hd_map_manager_->get_lanes().size());
  
  // Create publishers
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/lanelet_visualization", 10);
  
  // Create timer for periodic visualization updates
  auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(timer_period, 
    std::bind(&LaneletVisualizerNode::publish_visualization, this));
  
  RCLCPP_INFO(this->get_logger(), "Lanelet visualizer node initialized successfully");
  RCLCPP_INFO(this->get_logger(), "Publishing visualization markers to /lanelet_visualization");
  RCLCPP_INFO(this->get_logger(), "Map frame: %s", map_frame_.c_str());
}

void LaneletVisualizerNode::publish_visualization()
{
  if (!hd_map_manager_->is_map_loaded()) {
    return;
  }

  // Always use comprehensive visualization that handles all features
  publish_comprehensive_visualization();
}

void LaneletVisualizerNode::publish_comprehensive_visualization()
{
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = map_frame_;
  clear_marker.header.stamp = this->now();
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  clear_marker.id = 0;
  marker_array.markers.push_back(clear_marker);
  
  // Select a subset of lanes for visualization
  auto selected_lanes = select_lanes_for_visualization();
  
  RCLCPP_DEBUG(this->get_logger(), "Visualizing %zu lanes out of %zu total", 
               selected_lanes.size(), hd_map_manager_->get_lanes().size());
  
  // Create combined markers for better performance
  
  // 1. Combined centerlines marker
  if (show_centerlines_ && !selected_lanes.empty()) {
    visualization_msgs::msg::Marker centerlines_marker;
    centerlines_marker.header.frame_id = map_frame_;
    centerlines_marker.header.stamp = this->now();
    centerlines_marker.ns = "combined_centerlines";
    centerlines_marker.id = 1;
    centerlines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    centerlines_marker.action = visualization_msgs::msg::Marker::ADD;
    
    centerlines_marker.scale.x = 0.5; // Line width
    centerlines_marker.color.r = 0.0;
    centerlines_marker.color.g = 1.0;
    centerlines_marker.color.b = 0.0;
    centerlines_marker.color.a = 0.8;
    
    // Add all centerlines as line segments
    for (const auto* lane : selected_lanes) {
      if (lane->centerline.size() < 2) continue;
      
      for (size_t i = 0; i < lane->centerline.size() - 1; ++i) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = lane->centerline[i].first;
        p1.y = lane->centerline[i].second;
        p1.z = 0.1;
        
        p2.x = lane->centerline[i + 1].first;
        p2.y = lane->centerline[i + 1].second;
        p2.z = 0.1;
        
        centerlines_marker.points.push_back(p1);
        centerlines_marker.points.push_back(p2);
      }
    }
    
    if (!centerlines_marker.points.empty()) {
      marker_array.markers.push_back(centerlines_marker);
      RCLCPP_INFO_ONCE(this->get_logger(), "Added centerlines marker with %zu points", 
                       centerlines_marker.points.size());
    }
  }
  
  // 2. Combined boundaries marker (simplified)
  if (show_boundaries_ && !selected_lanes.empty()) {
    visualization_msgs::msg::Marker boundaries_marker;
    boundaries_marker.header.frame_id = map_frame_;
    boundaries_marker.header.stamp = this->now();
    boundaries_marker.ns = "combined_boundaries";
    boundaries_marker.id = 2;
    boundaries_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    boundaries_marker.action = visualization_msgs::msg::Marker::ADD;
    
    boundaries_marker.scale.x = 0.2;
    boundaries_marker.color.r = 1.0;
    boundaries_marker.color.g = 1.0;
    boundaries_marker.color.b = 0.0;
    boundaries_marker.color.a = 0.5;
    
    const double lane_width = 3.5;
    const double half_width = lane_width / 2.0;
    
    // Only add boundaries for a subset of lanes to reduce load
    int boundary_count = 0;
    const int max_boundary_lanes = std::min(20, static_cast<int>(selected_lanes.size()));
    
    for (const auto* lane : selected_lanes) {
      if (boundary_count >= max_boundary_lanes) break;
      if (lane->centerline.size() < 2) continue;
      
      // Simplify - only add boundary points every few centerline points
      for (size_t i = 0; i < lane->centerline.size() - 1; i += 3) { // Skip points for performance
        double x = lane->centerline[i].first;
        double y = lane->centerline[i].second;
        double x2 = lane->centerline[i + 1].first;
        double y2 = lane->centerline[i + 1].second;
        
        double dx = x2 - x;
        double dy = y2 - y;
        double length = std::sqrt(dx * dx + dy * dy);
        
        if (length > 0.001) {
          dx /= length;
          dy /= length;
          
          double perp_x = -dy;
          double perp_y = dx;
          
          // Left boundary segment
          geometry_msgs::msg::Point left1, left2;
          left1.x = x + perp_x * half_width;
          left1.y = y + perp_y * half_width;
          left1.z = 0.05;
          left2.x = x2 + perp_x * half_width;
          left2.y = y2 + perp_y * half_width;
          left2.z = 0.05;
          
          boundaries_marker.points.push_back(left1);
          boundaries_marker.points.push_back(left2);
          
          // Right boundary segment  
          geometry_msgs::msg::Point right1, right2;
          right1.x = x - perp_x * half_width;
          right1.y = y - perp_y * half_width;
          right1.z = 0.05;
          right2.x = x2 - perp_x * half_width;
          right2.y = y2 - perp_y * half_width;
          right2.z = 0.05;
          
          boundaries_marker.points.push_back(right1);
          boundaries_marker.points.push_back(right2);
        }
      }
      boundary_count++;
    }
    
    if (!boundaries_marker.points.empty()) {
      marker_array.markers.push_back(boundaries_marker);
      RCLCPP_INFO_ONCE(this->get_logger(), "Added boundaries marker with %zu points for %d lanes", 
                       boundaries_marker.points.size(), boundary_count);
    }
  }
  
  // 3. Lane IDs - show more when combined markers are disabled
  if (show_lane_ids_ && !selected_lanes.empty()) {
    int text_count = 0;
    const int max_text_markers = use_combined_markers_ ? 
        std::min(20, static_cast<int>(selected_lanes.size())) : 
        std::min(50, static_cast<int>(selected_lanes.size()));
    
    for (const auto* lane : selected_lanes) {
      if (text_count >= max_text_markers) break;
      if (lane->centerline.empty()) continue;
      
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = map_frame_;
      text_marker.header.stamp = this->now();
      text_marker.ns = "lane_ids";
      text_marker.id = 10 + text_count;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      
      size_t mid_idx = lane->centerline.size() / 2;
      text_marker.pose.position.x = lane->centerline[mid_idx].first;
      text_marker.pose.position.y = lane->centerline[mid_idx].second;
      text_marker.pose.position.z = 2.0;
      text_marker.pose.orientation.w = 1.0;
      
      text_marker.scale.z = 1.5; // Text size
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 0.0; // Yellow text
      text_marker.color.a = 1.0;
      
      text_marker.text = "Lane " + std::to_string(lane->id);
      
      marker_array.markers.push_back(text_marker);
      text_count++;
    }
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Added %d lane ID markers", text_count);
  }
  
  // 3b. Speed limits - show at different positions than lane IDs
  if (show_speed_limits_ && !selected_lanes.empty()) {
    int text_count = 0;
    const int max_speed_markers = use_combined_markers_ ? 
        std::min(15, static_cast<int>(selected_lanes.size())) : 
        std::min(30, static_cast<int>(selected_lanes.size()));
    
    for (const auto* lane : selected_lanes) {
      if (text_count >= max_speed_markers) break;
      if (lane->centerline.empty()) continue;
      
      visualization_msgs::msg::Marker speed_marker;
      speed_marker.header.frame_id = map_frame_;
      speed_marker.header.stamp = this->now();
      speed_marker.ns = "speed_limits";
      speed_marker.id = 500 + text_count;
      speed_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      speed_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Position at 3/4 along the lane to avoid collision with lane IDs
      size_t pos_idx = std::min(lane->centerline.size() - 1, 
                               static_cast<size_t>(lane->centerline.size() * 0.75));
      speed_marker.pose.position.x = lane->centerline[pos_idx].first;
      speed_marker.pose.position.y = lane->centerline[pos_idx].second;
      speed_marker.pose.position.z = 1.0; // Lower than lane IDs
      speed_marker.pose.orientation.w = 1.0;
      
      speed_marker.scale.z = 1.2; // Slightly smaller text
      speed_marker.color.r = 0.0;
      speed_marker.color.g = 1.0;
      speed_marker.color.b = 1.0; // Cyan text
      speed_marker.color.a = 0.9;
      
      speed_marker.text = std::to_string(static_cast<int>(lane->traffic_rule.speed_limit * 3.6)) + " km/h";
      
      marker_array.markers.push_back(speed_marker);
      text_count++;
    }
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Added %d speed limit markers", text_count);
  }
  
  // 4. Add traffic lights
  if (show_traffic_lights_) {
    int marker_id = 1000; // Start with high ID to avoid conflicts
    const auto& traffic_lights = hd_map_manager_->get_traffic_lights();
    int valid_lights = 0;
    
    for (const auto& traffic_light : traffic_lights) {
      // Skip lights with invalid coordinates
      if (std::abs(traffic_light.x) > 2000 || std::abs(traffic_light.y) > 2000) {
        continue;
      }
      
      visualization_msgs::msg::Marker light_marker;
      light_marker.header.frame_id = map_frame_;
      light_marker.header.stamp = this->now();
      light_marker.ns = "traffic_lights";
      light_marker.id = marker_id++;
      light_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      light_marker.action = visualization_msgs::msg::Marker::ADD;
      
      light_marker.pose.position.x = traffic_light.x;
      light_marker.pose.position.y = traffic_light.y;
      light_marker.pose.position.z = 5.0; // Fixed height above ground
      light_marker.pose.orientation.w = 1.0;
      
      light_marker.scale.x = 1.0; // Larger diameter for visibility
      light_marker.scale.y = 1.0;
      light_marker.scale.z = 2.0; // Taller height
      
      // Color based on state
      if (traffic_light.state == "red_yellow_green") {
        light_marker.color.r = 1.0; light_marker.color.g = 1.0; light_marker.color.b = 0.0; // Yellow
      } else {
        light_marker.color.r = 0.0; light_marker.color.g = 1.0; light_marker.color.b = 0.0; // Green
      }
      light_marker.color.a = 0.9; // More opaque
      
      marker_array.markers.push_back(light_marker);
      valid_lights++;
      
      // Debug first few lights
      if (valid_lights <= 3) {
        RCLCPP_INFO(this->get_logger(), "Traffic light %d: pos(%.2f, %.2f, %.2f)", 
                    traffic_light.id, traffic_light.x, traffic_light.y, light_marker.pose.position.z);
      }
    }
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Added %d traffic light markers (from %zu total)", 
                     valid_lights, traffic_lights.size());
  }
  
  // 5. Add traffic signs
  if (show_traffic_signs_) {
    int marker_id = 2000;
    const auto& traffic_signs = hd_map_manager_->get_traffic_signs();
    int valid_signs = 0;
    
    for (const auto& traffic_sign : traffic_signs) {
      // Skip signs with invalid coordinates
      if (std::abs(traffic_sign.x) > 2000 || std::abs(traffic_sign.y) > 2000) {
        continue;
      }
      
      visualization_msgs::msg::Marker sign_marker;
      sign_marker.header.frame_id = map_frame_;
      sign_marker.header.stamp = this->now();
      sign_marker.ns = "traffic_signs";
      sign_marker.id = marker_id++;
      sign_marker.type = visualization_msgs::msg::Marker::CUBE;
      sign_marker.action = visualization_msgs::msg::Marker::ADD;
      
      sign_marker.pose.position.x = traffic_sign.x;
      sign_marker.pose.position.y = traffic_sign.y;
      sign_marker.pose.position.z = 3.0; // Fixed height
      sign_marker.pose.orientation.w = 1.0;
      
      sign_marker.scale.x = 0.6; // Larger and more visible
      sign_marker.scale.y = 0.1; // Thicker
      sign_marker.scale.z = 0.8;
      
      // Color based on sign type
      if (traffic_sign.sign_type == "stop_sign") {
        sign_marker.color.r = 1.0; sign_marker.color.g = 0.0; sign_marker.color.b = 0.0; // Red
      } else {
        sign_marker.color.r = 0.0; sign_marker.color.g = 0.0; sign_marker.color.b = 1.0; // Blue
      }
      sign_marker.color.a = 0.9;
      
      marker_array.markers.push_back(sign_marker);
      valid_signs++;
      
      // Debug first few signs
      if (valid_signs <= 3) {
        RCLCPP_INFO(this->get_logger(), "Traffic sign %d (%s): pos(%.2f, %.2f, %.2f)", 
                    traffic_sign.id, traffic_sign.sign_type.c_str(), 
                    traffic_sign.x, traffic_sign.y, sign_marker.pose.position.z);
      }
    }
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Added %d traffic sign markers (from %zu total)", 
                     valid_signs, traffic_signs.size());
  }
  
  // 6. Add crosswalks  
  if (show_crosswalks_) {
    int marker_id = 3000;
    const auto& crosswalks = hd_map_manager_->get_crosswalks();
    int valid_crosswalks = 0;
    
    for (const auto& crosswalk : crosswalks) {
      if (crosswalk.boundary.size() < 2) continue;
      
      // Check if crosswalk coordinates are reasonable
      bool valid_coords = true;
      for (const auto& point : crosswalk.boundary) {
        if (std::abs(point.first) > 2000 || std::abs(point.second) > 2000) {
          valid_coords = false;
          break;
        }
      }
      if (!valid_coords) continue;
      
      visualization_msgs::msg::Marker crosswalk_marker;
      crosswalk_marker.header.frame_id = map_frame_;
      crosswalk_marker.header.stamp = this->now();
      crosswalk_marker.ns = "crosswalks";
      crosswalk_marker.id = marker_id++;
      crosswalk_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      crosswalk_marker.action = visualization_msgs::msg::Marker::ADD;
      
      crosswalk_marker.scale.x = 0.5; // Thicker line width
      crosswalk_marker.color.r = 1.0; 
      crosswalk_marker.color.g = 0.5; 
      crosswalk_marker.color.b = 0.0; // Orange for visibility
      crosswalk_marker.color.a = 0.9;
      
      for (const auto& point : crosswalk.boundary) {
        geometry_msgs::msg::Point p;
        p.x = point.first;
        p.y = point.second;
        p.z = 0.1; // Slightly higher above ground
        crosswalk_marker.points.push_back(p);
      }
      
      marker_array.markers.push_back(crosswalk_marker);
      valid_crosswalks++;
      
      // Debug first few crosswalks
      if (valid_crosswalks <= 2) {
        RCLCPP_INFO(this->get_logger(), "Crosswalk %d: %zu points, first point(%.2f, %.2f)", 
                    crosswalk.id, crosswalk.boundary.size(), 
                    crosswalk.boundary[0].first, crosswalk.boundary[0].second);
      }
    }
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Added %d crosswalk markers (from %zu total)", 
                     valid_crosswalks, crosswalks.size());
  }
  
  // 7. Add road markings
  if (show_road_markings_) {
    int marker_id = 4000;
    const auto& road_markings = hd_map_manager_->get_road_markings();
    
    // Limit road markings to avoid overwhelming RViz
    size_t max_markings = use_combined_markers_ ? 100 : 50;
    size_t marking_count = 0;
    
    for (const auto& road_marking : road_markings) {
      if (marking_count >= max_markings) break;
      if (road_marking.geometry.size() < 2) continue;
      
      // Check if road marking coordinates are reasonable
      bool valid_coords = true;
      for (const auto& point : road_marking.geometry) {
        if (std::abs(point.first) > 2000 || std::abs(point.second) > 2000) {
          valid_coords = false;
          break;
        }
      }
      if (!valid_coords) continue;
      
      visualization_msgs::msg::Marker marking_marker;
      marking_marker.header.frame_id = map_frame_;
      marking_marker.header.stamp = this->now();
      marking_marker.ns = "road_markings";
      marking_marker.id = marker_id++;
      marking_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marking_marker.action = visualization_msgs::msg::Marker::ADD;
      
      marking_marker.scale.x = 0.15; // Line width
      
      // Color based on marking type
      if (road_marking.marking_type == "dashed") {
        marking_marker.color.r = 1.0; marking_marker.color.g = 1.0; marking_marker.color.b = 0.0; // Yellow
      } else {
        marking_marker.color.r = 1.0; marking_marker.color.g = 1.0; marking_marker.color.b = 1.0; // White
      }
      marking_marker.color.a = 0.7;
      
      for (const auto& point : road_marking.geometry) {
        geometry_msgs::msg::Point p;
        p.x = point.first;
        p.y = point.second;
        p.z = 0.01; // Just above ground
        marking_marker.points.push_back(p);
      }
      
      marker_array.markers.push_back(marking_marker);
      marking_count++;
    }
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Added %zu road marking markers (limited from %zu total)", 
                     marking_count, road_markings.size());
  }
  
  // Publish the marker array
  marker_publisher_->publish(marker_array);
  
  RCLCPP_DEBUG(this->get_logger(), "Published %zu markers for %zu selected lanes", 
               marker_array.markers.size(), selected_lanes.size());
}

std::vector<const awsim_path_planner::LaneInfo*> LaneletVisualizerNode::select_lanes_for_visualization() const
{
  const auto& all_lanes = hd_map_manager_->get_lanes();
  std::vector<const awsim_path_planner::LaneInfo*> selected_lanes;
  
  // Simple selection: take first max_lanes_ lanes with non-empty centerlines
  // In a real implementation, you'd select based on distance to vehicle
  
  for (const auto& lane : all_lanes) {
    if (selected_lanes.size() >= static_cast<size_t>(max_lanes_)) {
      break;
    }
    
    if (!lane.centerline.empty()) {
      selected_lanes.push_back(&lane);
    }
  }
  
  RCLCPP_INFO_ONCE(this->get_logger(), 
    "Selected %zu lanes for visualization (max: %d, total available: %zu)", 
    selected_lanes.size(), max_lanes_, all_lanes.size());
  
  // Debug visualization parameters
  RCLCPP_INFO_ONCE(this->get_logger(), 
    "Visualization settings: centerlines=%s, boundaries=%s, lane_ids=%s, speed_limits=%s",
    show_centerlines_ ? "true" : "false",
    show_boundaries_ ? "true" : "false", 
    show_lane_ids_ ? "true" : "false",
    show_speed_limits_ ? "true" : "false");
  
  return selected_lanes;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<LaneletVisualizerNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting Lanelet Visualizer Node...");
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in lanelet visualizer: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
