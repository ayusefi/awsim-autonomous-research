#include "awsim_path_planner/lanelet_visualizer_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/utility/Utilities.h>

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
  this->declare_parameter<int>("max_lanes_to_visualize", 100000); // Hard limit on number of lanes
  
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
  
  // Use UTM projector with map origin for standard approach
  // Based on the OSM file coordinates: lat=35.688, lon=139.691 (Tokyo area)
  // This matches the coordinate system used by the localization system
  lanelet::Origin origin({35.688552, 139.691427}); // Map origin from OSM file
  projector_ = std::make_unique<lanelet::projection::UtmProjector>(origin);
  
  // Create publishers first (always create them, even if map loading fails)
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/lanelet_visualization", 10);
  
  // Load the lanelet2 map using robust method
  bool map_loaded = false;
  try {
    RCLCPP_INFO(this->get_logger(), "Loading lanelet2 map from: %s", hd_map_path.c_str());
    
    // Initialize lanelet map
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
    
    // Use error-tolerant loading directly
    lanelet::ErrorMessages errors;
    lanelet_map_ = lanelet::load(hd_map_path, *projector_, &errors);
    
    if (!errors.empty()) {
      RCLCPP_INFO(this->get_logger(), "Map loaded with %zu parsing warnings (regulatory elements with issues gracefully handled)", errors.size());
      for (size_t i = 0; i < std::min(errors.size(), size_t(5)); ++i) {
        RCLCPP_DEBUG(this->get_logger(), "Warning %zu: %s", i+1, errors[i].c_str());
      }
      if (errors.size() > 5) {
        RCLCPP_DEBUG(this->get_logger(), "... and %zu more warnings", errors.size() - 5);
      }
    }
    
    // Validate the map loaded correctly
    if (!lanelet_map_ || lanelet_map_->laneletLayer.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load any lanelets from map file");
      map_loaded = false;
    } else {
      // Validate with standard approach: check lanelet types
      
      lanelet::ConstLanelets all_lanelets;
      for (const auto& lanelet : lanelet_map_->laneletLayer) {
        all_lanelets.push_back(lanelet);
      }

      lanelet::ConstLanelets road_lanelets;
      for (const auto& lanelet : all_lanelets) {
        if (lanelet.attributeOr("subtype", std::string()) == "road") {
          road_lanelets.push_back(lanelet);
        }
      }

      RCLCPP_INFO(this->get_logger(), "Successfully loaded lanelet2 map with %zu total lanelets (%zu road lanelets)", 
                  all_lanelets.size(), road_lanelets.size());
      
      // Use the standard approach: read local_x/local_y attributes from OSM
      // The localization system uses manual_map_center = (0.0, 0.0) which means
      // it publishes poses in global coordinates (same as local_x/local_y in OSM)
      // Therefore, we use the local_x/local_y coordinates directly from OSM attributes
      map_origin_offset_x_ = 0.0;
      map_origin_offset_y_ = 0.0;
      
      RCLCPP_INFO(this->get_logger(), 
                  "Using standard approach: reading local_x/local_y attributes from OSM");
      
      // Debug: Check if some nodes have local_x/local_y attributes AND check if lanelets have them
      int nodes_with_local_coords = 0;
      int lanelets_with_local_coords = 0;
      double first_local_x = 0.0, first_local_y = 0.0;
      double first_projected_x = 0.0, first_projected_y = 0.0;
      bool offset_calculated = false;
      
      for (const auto& point : lanelet_map_->pointLayer) {
        if (point.hasAttribute("local_x") && point.hasAttribute("local_y")) {
          nodes_with_local_coords++;
          // Build the local coordinate cache for efficient lookup
          double local_x = std::stod(point.attribute("local_x").value());
          double local_y = std::stod(point.attribute("local_y").value());
          local_coord_cache_[point.id()] = std::make_pair(local_x, local_y);
          
          // Calculate offset from first point
          if (!offset_calculated) {
            first_local_x = local_x;
            first_local_y = local_y;
            first_projected_x = point.x();
            first_projected_y = point.y();
            local_to_global_offset_x_ = first_local_x - first_projected_x;
            local_to_global_offset_y_ = first_local_y - first_projected_y;
            offset_calculated = true;
            
            RCLCPP_INFO(this->get_logger(), 
                       "Calculated global offset: X=%.2f, Y=%.2f (local: %.2f,%.2f - projected: %.2f,%.2f)", 
                       local_to_global_offset_x_, local_to_global_offset_y_,
                       first_local_x, first_local_y, first_projected_x, first_projected_y);
          }
          
          if (nodes_with_local_coords <= 3) { // Log first few for debugging
            RCLCPP_INFO(this->get_logger(), 
                       "Point ID %ld: local_x=%.2f, local_y=%.2f", 
                       point.id(), local_x, local_y);
          }
        }
      }
      
      // Also check if lanelets have local_x/local_y attributes (alternative approach)
      for (const auto& lanelet : all_lanelets) {
        if (lanelet.hasAttribute("local_x") && lanelet.hasAttribute("local_y")) {
          lanelets_with_local_coords++;
          if (lanelets_with_local_coords <= 3) {
            RCLCPP_INFO(this->get_logger(), 
                       "Lanelet ID %ld: local_x=%s, local_y=%s", 
                       lanelet.id(), 
                       lanelet.attribute("local_x").value().c_str(),
                       lanelet.attribute("local_y").value().c_str());
          }
        }
      }
      
      RCLCPP_INFO(this->get_logger(), 
                  "Found %d points and %d lanelets with local_x/local_y attributes, cached for efficient lookup", 
                  nodes_with_local_coords, lanelets_with_local_coords);
      
      map_loaded = true;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load lanelet2 map from: %s. Error: %s", 
                 hd_map_path.c_str(), e.what());
    RCLCPP_WARN(this->get_logger(), "Map loading failed, but node will continue running. No visualization will be published.");
    map_loaded = false;
  }
  
  // Create timer for periodic visualization updates (always create it)
  auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
  timer_ = this->create_wall_timer(timer_period, 
    std::bind(&LaneletVisualizerNode::publish_visualization, this));
  
  RCLCPP_INFO(this->get_logger(), "Lanelet visualizer node initialized successfully (map loaded: %s)", 
              map_loaded ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Publishing visualization markers to /lanelet_visualization");
  RCLCPP_INFO(this->get_logger(), "Map frame: %s", map_frame_.c_str());
}

void LaneletVisualizerNode::publish_visualization()
{
  if (!lanelet_map_ || lanelet_map_->laneletLayer.empty()) {
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
  auto selected_lanelets = select_lanelets_for_visualization();
  
  RCLCPP_DEBUG(this->get_logger(), "Visualizing %zu lanelets out of %zu total", 
               selected_lanelets.size(), lanelet_map_->laneletLayer.size());
  
  // Create combined centerlines marker
  if (show_centerlines_ && !selected_lanelets.empty()) {
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
    for (const auto& lanelet : selected_lanelets) {
      auto centerline = lanelet.centerline();
      if (centerline.size() < 2) continue;
      
      for (size_t i = 0; i < centerline.size() - 1; ++i) {
        geometry_msgs::msg::Point p1, p2;
        
        // Standard approach: Try cached coordinates first, then apply global offset
        const auto& point1 = centerline[i];
        const auto& point2 = centerline[i + 1];
        
        // Use cached local coordinates if available
        auto it1 = local_coord_cache_.find(point1.id());
        if (it1 != local_coord_cache_.end()) {
          p1.x = it1->second.first;  // local_x from OSM
          p1.y = it1->second.second; // local_y from OSM
        } else {
          // Transform projected coordinates to global coordinates using calculated offset
          p1.x = point1.x() + local_to_global_offset_x_;
          p1.y = point1.y() + local_to_global_offset_y_;
        }
        
        auto it2 = local_coord_cache_.find(point2.id());
        if (it2 != local_coord_cache_.end()) {
          p2.x = it2->second.first;  // local_x from OSM
          p2.y = it2->second.second; // local_y from OSM
        } else {
          // Transform projected coordinates to global coordinates using calculated offset
          p2.x = point2.x() + local_to_global_offset_x_;
          p2.y = point2.y() + local_to_global_offset_y_;
        }
        
        p1.z = 0.1;
        p2.z = 0.1;
        
        centerlines_marker.points.push_back(p1);
        centerlines_marker.points.push_back(p2);
      }
    }
    
    if (!centerlines_marker.points.empty()) {
      marker_array.markers.push_back(centerlines_marker);
      RCLCPP_INFO_ONCE(this->get_logger(), "Added centerlines marker with %zu points", 
                       centerlines_marker.points.size());
      
      // Debug: log first few coordinate values to verify they're in the right range
      if (centerlines_marker.points.size() >= 2) {
        RCLCPP_INFO_ONCE(this->get_logger(), 
                         "Sample coordinates: P1(%.2f, %.2f), P2(%.2f, %.2f)", 
                         centerlines_marker.points[0].x, centerlines_marker.points[0].y,
                         centerlines_marker.points[1].x, centerlines_marker.points[1].y);
      }
      
      // Debug: Check if local coordinates are being used correctly
      if (!selected_lanelets.empty()) {
        auto first_centerline = selected_lanelets[0].centerline();
        if (first_centerline.size() > 0) {
          const auto& first_point = first_centerline[0];
          auto cache_lookup = local_coord_cache_.find(first_point.id());
          if (cache_lookup != local_coord_cache_.end()) {
            RCLCPP_INFO_ONCE(this->get_logger(), 
                           "Cache HIT: Point ID %ld has local coords (%.2f, %.2f)", 
                           first_point.id(), cache_lookup->second.first, cache_lookup->second.second);
          } else {
            double transformed_x = first_point.x() + local_to_global_offset_x_;
            double transformed_y = first_point.y() + local_to_global_offset_y_;
            RCLCPP_INFO_ONCE(this->get_logger(), 
                           "Cache MISS: Point ID %ld transformed (%.2f, %.2f) from projected (%.2f, %.2f) + offset (%.2f, %.2f)", 
                           first_point.id(), transformed_x, transformed_y,
                           first_point.x(), first_point.y(),
                           local_to_global_offset_x_, local_to_global_offset_y_);
          }
        }
      }
    }
  }
  
  // Publish the marker array
  marker_publisher_->publish(marker_array);
}

std::vector<lanelet::ConstLanelet> LaneletVisualizerNode::select_lanelets_for_visualization() const
{
  std::vector<lanelet::ConstLanelet> selected_lanelets;
  
  // Simple selection: take first max_lanes_ lanelets
  // In a real implementation, you'd select based on distance to vehicle
  
  const auto& lanelets = lanelet_map_->laneletLayer;
  for (const auto& lanelet : lanelets) {
    if (selected_lanelets.size() >= static_cast<size_t>(max_lanes_)) {
      break;
    }
    
    // Basic validity check
    if (lanelet.centerline().size() > 1) {
      selected_lanelets.push_back(lanelet);
    }
  }
  
  RCLCPP_INFO_ONCE(this->get_logger(), 
    "Selected %zu lanelets for visualization (max: %d, total available: %zu)", 
    selected_lanelets.size(), max_lanes_, lanelet_map_->laneletLayer.size());
  
  // Debug visualization parameters
  RCLCPP_INFO_ONCE(this->get_logger(), 
    "Visualization settings: centerlines=%s, boundaries=%s, lane_ids=%s, speed_limits=%s",
    show_centerlines_ ? "true" : "false",
    show_boundaries_ ? "true" : "false", 
    show_lane_ids_ ? "true" : "false",
    show_speed_limits_ ? "true" : "false");
  
  return selected_lanelets;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<LaneletVisualizerNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting Lanelet Visualizer Node...");
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in visualizer node: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
