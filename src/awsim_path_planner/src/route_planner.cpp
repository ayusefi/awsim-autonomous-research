#include "awsim_path_planner/route_planner.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/utility/Utilities.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <algorithm>
#include <cmath>

namespace awsim_path_planner
{

RoutePlanner::RoutePlanner(rclcpp::Node * node)
: node_(node), map_loaded_(false)
{
  RCLCPP_INFO(node_->get_logger(), "Initializing Route Planner with lanelet2");
}

bool RoutePlanner::initialize(const RoutePlannerParam & param)
{
  param_ = param;
  
  try {
    // Load lanelet2 map using robust error handling
    RCLCPP_INFO(node_->get_logger(), "Loading lanelet2 map from: %s", param_.map_file_path.c_str());
    
    // Create UTM projector using the same coordinate system as HDMapManager and localization
    // The OSM map already contains local_x/local_y coordinates that match the localization system
    // We need to use the same map origin offsets to maintain coordinate consistency
    lanelet::Origin origin({35.6762, 139.6503});  // Shinjuku coordinates for reference
    projector_ = std::make_unique<lanelet::projection::UtmProjector>(origin, false, false);
    
    // Use standard approach: read local_x/local_y attributes from OSM
    // Same approach as the lanelet visualizer for coordinate consistency
    map_origin_offset_x_ = 0.0;  // Will be replaced by automatic offset calculation
    map_origin_offset_y_ = 0.0;  // Will be replaced by automatic offset calculation
    
    // Initialize the lanelet map pointer
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
    
    // Create traffic rules FIRST (Autoware pattern)
    std::string location = "de";  // Default to German rules
    std::string participant = "vehicle";
    
    if (param_.traffic_rules_name == "japan") {
      // Japan uses left-hand traffic, but we'll use German rules as fallback
      location = "de";
      RCLCPP_INFO(node_->get_logger(), "Using German traffic rules as fallback for Japan (note: actual Japan rules not available)");
    } else if (param_.traffic_rules_name == "german" || param_.traffic_rules_name == "de") {
      location = "de";
      RCLCPP_INFO(node_->get_logger(), "Using German traffic rules");
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unknown traffic rules '%s', defaulting to German", param_.traffic_rules_name.c_str());
      location = "de";
    }
    
    traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(location, participant);
    
    // Load the map with robust error handling
    try {
      // Use error-tolerant loading method directly
      // This gracefully handles unsupported regulatory elements
      lanelet::ErrorMessages errors;
      lanelet_map_ = lanelet::load(param_.map_file_path, *projector_, &errors);
      
      if (!errors.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Map loaded with %zu parsing warnings (regulatory elements with issues gracefully handled)", errors.size());
        for (size_t i = 0; i < std::min(errors.size(), size_t(5)); ++i) {
          RCLCPP_DEBUG(node_->get_logger(), "Warning %zu: %s", i+1, errors[i].c_str());
        }
        if (errors.size() > 5) {
          RCLCPP_DEBUG(node_->get_logger(), "... and %zu more warnings", errors.size() - 5);
        }
      }
      
      // Final validation
      if (!lanelet_map_ || lanelet_map_->laneletLayer.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load any lanelets from map file");
        map_loaded_ = false;
        return true;  // Continue without map
      }
      
      RCLCPP_INFO(node_->get_logger(), "Successfully loaded lanelet2 map with %zu lanelets", 
                  lanelet_map_->laneletLayer.size());
      
      // Create routing graph using standard pattern with traffic rules integration
      routing_graph_ = lanelet::routing::RoutingGraph::build(*lanelet_map_, *traffic_rules_);
      
      if (!routing_graph_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create routing graph with traffic rules");
        map_loaded_ = false;
        return true;  // Continue without routing capabilities
      }
      
      // Validate routing graph by checking connectivity
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
      
      RCLCPP_INFO(node_->get_logger(), "Created routing graph successfully with %zu total lanelets (%zu road lanelets)", 
                  all_lanelets.size(), road_lanelets.size());
      
      // Build coordinate cache and calculate automatic offset
      // Same approach as lanelet visualizer for consistency
      RCLCPP_INFO(node_->get_logger(), "Using standard approach: reading local_x/local_y attributes from OSM");
      
      int nodes_with_local_coords = 0;
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
            double projected_x = point.x();
            double projected_y = point.y();
            local_to_global_offset_x_ = local_x - projected_x;
            local_to_global_offset_y_ = local_y - projected_y;
            offset_calculated = true;
            
            RCLCPP_INFO(node_->get_logger(), 
                       "Calculated global offset: X=%.2f, Y=%.2f (local: %.2f,%.2f - projected: %.2f,%.2f)", 
                       local_to_global_offset_x_, local_to_global_offset_y_,
                       local_x, local_y, projected_x, projected_y);
          }
        }
      }
      
      RCLCPP_INFO(node_->get_logger(), 
                  "Found %d points with local_x/local_y attributes, cached for efficient lookup", 
                  nodes_with_local_coords);
      
      // Set origin coordinates for compatibility  
      map_origin_lat_ = 35.6762;  // Tokyo/Shinjuku reference
      map_origin_lon_ = 139.6503;

      map_loaded_ = true;
      RCLCPP_INFO(node_->get_logger(), "Route planner initialized successfully with lanelet2 map");
      return true;
                  
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load Lanelet2 map: %s", e.what());
      RCLCPP_WARN(node_->get_logger(), "Route planner will operate in fallback mode");
      
      // Set map_loaded to false but continue with initialization
      map_loaded_ = false;
      RCLCPP_INFO(node_->get_logger(), "Route planner initialized in fallback mode");
      return true;  // Allow initialization to succeed even without map
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "General error in route planner initialization: %s", e.what());
    map_loaded_ = false;
    return true;  // Allow initialization to succeed even without map
  }
}

std::optional<autoware_planning_msgs::msg::Path> RoutePlanner::plan_route(
  const geometry_msgs::msg::PoseStamped & start_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  if (!map_loaded_) {
    RCLCPP_WARN(node_->get_logger(), "Route planner operating in fallback mode (no lanelet2 map available)");
    RCLCPP_INFO(node_->get_logger(), "Creating basic straight-line path from start to goal");
    
    // Create a simple straight-line path as fallback
    return create_fallback_path(start_pose, goal_pose);
  }
  
  RCLCPP_INFO(node_->get_logger(), "Planning route from (%.2f, %.2f) to (%.2f, %.2f)",
              start_pose.pose.position.x, start_pose.pose.position.y,
              goal_pose.pose.position.x, goal_pose.pose.position.y);
  
  // Find start lanelet
  auto start_lanelet = find_nearest_lanelet(start_pose, param_.goal_search_radius);
  if (!start_lanelet) {
    RCLCPP_WARN(node_->get_logger(), "Could not find start lanelet");
    return std::nullopt;
  }
  
  // Find goal lanelet that best matches the goal pose orientation
  auto goal_lanelet = find_goal_lanelet(goal_pose, param_.goal_search_radius);
  if (!goal_lanelet) {
    RCLCPP_WARN(node_->get_logger(), "Could not find goal lanelet matching pose orientation");
    return std::nullopt;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Found start lanelet ID: %ld, goal lanelet ID: %ld",
              start_lanelet->id(), goal_lanelet->id());
  
  try {
    // Plan route using lanelet2 routing
    auto route = routing_graph_->getRoute(*start_lanelet, *goal_lanelet, 0);
    
    if (!route) {
      RCLCPP_WARN(node_->get_logger(), "No route found between lanelets");
      return std::nullopt;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Found route with %zu segments", route->size());
    
    // Convert route to path
    auto path = convert_route_to_path(*route, start_pose, goal_pose);
    
    RCLCPP_INFO(node_->get_logger(), "Generated route path with %zu points", path.points.size());
    return path;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Route planning failed: %s", e.what());
    return std::nullopt;
  }
}

autoware_planning_msgs::msg::Path RoutePlanner::convert_route_to_path(
  const lanelet::routing::Route & route,
  const geometry_msgs::msg::PoseStamped & start_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  autoware_planning_msgs::msg::Path path;
  path.header.frame_id = param_.map_frame;
  path.header.stamp = node_->get_clock()->now();
  
  // Get all lanelets in the route
  auto route_lanelets = route.shortestPath();
  
  // Process each lanelet with position-aware trimming to avoid zigzag in lane changes
  geometry_msgs::msg::Point last_position = start_pose.pose.position;
  
  for (size_t i = 0; i < route_lanelets.size(); ++i) {
    auto centerline_points = create_centerline_path(route_lanelets[i], param_.centerline_resolution);
    
    if (centerline_points.empty()) {
      continue;
    }
    
    // For the first lanelet, find the closest point to start position and trim before it
    if (i == 0) {
      ProjectionResult start_proj = project_point_to_path(centerline_points, start_pose.pose.position);
      
      // Start from the projection point forward to avoid going backwards
      size_t start_index = start_proj.segment_index;
      
      // Add the projected start point first if we have interpolation
      if (start_proj.t > 0.0 && start_proj.segment_index < centerline_points.size() - 1) {
        const auto& p1 = centerline_points[start_proj.segment_index];
        const auto& p2 = centerline_points[start_proj.segment_index + 1];
        autoware_planning_msgs::msg::PathPoint start_point;
        start_point.pose.position.x = p1.pose.position.x + start_proj.t * (p2.pose.position.x - p1.pose.position.x);
        start_point.pose.position.y = p1.pose.position.y + start_proj.t * (p2.pose.position.y - p1.pose.position.y);
        start_point.pose.position.z = p1.pose.position.z + start_proj.t * (p2.pose.position.z - p1.pose.position.z);
        start_point.pose.orientation = start_pose.pose.orientation;
        start_point.longitudinal_velocity_mps = 5.0;
        path.points.push_back(start_point);
        last_position = start_point.pose.position;
        start_index = start_proj.segment_index + 1;
      } else {
        // Use the exact start pose
        autoware_planning_msgs::msg::PathPoint start_point;
        start_point.pose = start_pose.pose;
        start_point.longitudinal_velocity_mps = 5.0;
        path.points.push_back(start_point);
        last_position = start_pose.pose.position;
      }
      
      // Add remaining points from this lanelet
      for (size_t j = start_index; j < centerline_points.size(); ++j) {
        path.points.push_back(centerline_points[j]);
      }
      
      if (!centerline_points.empty()) {
        last_position = centerline_points.back().pose.position;
      }
    }
    // For subsequent lanelets, find the closest point to avoid going backwards
    else {
      ProjectionResult connection_proj = project_point_to_path(centerline_points, last_position);
      
      // Start from the closest point or slightly ahead to ensure forward progress
      size_t start_index = std::min(connection_proj.segment_index + 1, centerline_points.size() - 1);
      
      // Add transition point if there's a significant gap
      if (connection_proj.distance > 1.0) {
        autoware_planning_msgs::msg::PathPoint transition_point;
        transition_point.pose.position = connection_proj.projection_point;
        
        // Calculate orientation pointing towards the next segment
        if (start_index < centerline_points.size()) {
          double dx = centerline_points[start_index].pose.position.x - last_position.x;
          double dy = centerline_points[start_index].pose.position.y - last_position.y;
          double yaw = std::atan2(dy, dx);
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          transition_point.pose.orientation = tf2::toMsg(q);
        } else {
          transition_point.pose.orientation = centerline_points[0].pose.orientation;
        }
        
        transition_point.longitudinal_velocity_mps = 3.0;  // Slower for lane change
        path.points.push_back(transition_point);
      }
      
      // Add points from the connection point forward
      for (size_t j = start_index; j < centerline_points.size(); ++j) {
        path.points.push_back(centerline_points[j]);
      }
      
      if (!centerline_points.empty()) {
        last_position = centerline_points.back().pose.position;
      }
    }
  }
  
  // Ensure we have a reasonable path
  if (path.points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Generated empty path");
    return path;
  }
  
  // Handle the goal pose - project and trim the path appropriately
  ProjectionResult goal_proj = project_point_to_path(path.points, goal_pose.pose.position);
  
  // Trim the path to end at or near the goal
  if (goal_proj.segment_index < path.points.size()) {
    // Keep points up to the goal projection
    path.points.erase(path.points.begin() + goal_proj.segment_index + 1, path.points.end());
  }
  
  // Add the exact goal pose as final point
  autoware_planning_msgs::msg::PathPoint goal_point;
  goal_point.pose = goal_pose.pose;
  goal_point.longitudinal_velocity_mps = 0.0;  // Stop at goal
  path.points.push_back(goal_point);
  
  return path;
}

ProjectionResult RoutePlanner::project_point_to_path(
    const std::vector<autoware_planning_msgs::msg::PathPoint>& path_points,
    const geometry_msgs::msg::Point& point)
{
    ProjectionResult best_proj;
    best_proj.distance = std::numeric_limits<double>::max();

    // Iterate through each segment of the path
    for (size_t i = 0; i < path_points.size() - 1; ++i) {
        const auto& p1 = path_points[i].pose.position;
        const auto& p2 = path_points[i + 1].pose.position;

        // Vector from p1 to p2 (segment direction)
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;

        // Vector from p1 to the point
        double ax = point.x - p1.x;
        double ay = point.y - p1.y;
        double az = point.z - p1.z;

        // Compute projection parameter t
        double len_sq = dx * dx + dy * dy + dz * dz;
        double t = (len_sq > 0) ? (ax * dx + ay * dy + az * dz) / len_sq : 0.0;
        t = std::max(0.0, std::min(1.0, t));  // Clamp t to [0,1]

        // Calculate the projection point
        geometry_msgs::msg::Point proj_point;
        proj_point.x = p1.x + t * dx;
        proj_point.y = p1.y + t * dy;
        proj_point.z = p1.z + t * dz;

        // Compute distance from the point to the projection
        double dist = std::sqrt(
            (point.x - proj_point.x) * (point.x - proj_point.x) +
            (point.y - proj_point.y) * (point.y - proj_point.y) +
            (point.z - proj_point.z) * (point.z - proj_point.z)
        );

        // Update if this is the closest projection so far
        if (dist < best_proj.distance) {
            best_proj.distance = dist;
            best_proj.segment_index = i;
            best_proj.t = t;
            best_proj.projection_point = proj_point;
        }
    }

    return best_proj;
}

std::optional<lanelet::ConstLanelet> RoutePlanner::find_nearest_lanelet(
  const geometry_msgs::msg::PoseStamped & pose, double search_radius)
{
  if (!map_loaded_) {
    return std::nullopt;
  }
  
  // Transform pose coordinates using standard approach
  // For the inside check, we need to use coordinates in the same system as the lanelet
  // The pose comes from localization in global coordinates (~81k, ~49k range)
  // But for the inside check, we need to transform back to the lanelet coordinate system
  double search_x = pose.pose.position.x;  // Use pose coordinates directly for distance calc
  double search_y = pose.pose.position.y;
  
  // For inside check, transform pose coordinates back to lanelet coordinate system
  double lanelet_search_x = pose.pose.position.x - local_to_global_offset_x_;
  double lanelet_search_y = pose.pose.position.y - local_to_global_offset_y_;
  lanelet::BasicPoint2d search_point(lanelet_search_x, lanelet_search_y);
  
  RCLCPP_DEBUG(node_->get_logger(), "Searching for lanelet near pose (%.2f, %.2f) -> lanelet coords (%.2f, %.2f)", 
               search_x, search_y, lanelet_search_x, lanelet_search_y);
  
  double min_distance = std::numeric_limits<double>::max();
  std::optional<lanelet::ConstLanelet> nearest_lanelet;
  
  for (const auto & lanelet : lanelet_map_->laneletLayer) {
    // Check if point is within the lanelet using the original function
    if (lanelet::geometry::inside(lanelet, search_point)) {
      nearest_lanelet = lanelet;
      RCLCPP_DEBUG(node_->get_logger(), "Found exact match lanelet ID: %ld", nearest_lanelet->id());
      break;  // Found exact match
    }
    
    // Calculate distance to lanelet centerline using global coordinates
    double distance = calculate_distance_to_centerline_global(search_x, search_y, lanelet);
    
    if (distance < min_distance && distance <= search_radius) {
      min_distance = distance;
      nearest_lanelet = lanelet;
    }
  }
  
  if (nearest_lanelet) {
    RCLCPP_DEBUG(node_->get_logger(), "Found nearest lanelet ID: %ld, distance: %.2f",
                 nearest_lanelet->id(), min_distance);
  }
  
  return nearest_lanelet;
}

std::optional<lanelet::ConstLanelet> RoutePlanner::find_goal_lanelet(
  const geometry_msgs::msg::PoseStamped & goal_pose, double search_radius)
{
  if (!map_loaded_) {
    return std::nullopt;
  }
  
  // Transform goal pose coordinates using standard approach
  // Use global coordinates for distance calculations
  double goal_x = goal_pose.pose.position.x;  // Already in global range (~81k, ~49k)
  double goal_y = goal_pose.pose.position.y;
  
  double best_score = std::numeric_limits<double>::max();
  std::optional<lanelet::ConstLanelet> best_lanelet;
  
  for (const auto & lanelet : lanelet_map_->laneletLayer) {
    double distance = calculate_distance_to_centerline_global(goal_x, goal_y, lanelet);
    
    if (distance > search_radius) {
      continue;
    }
    
    // Calculate orientation similarity (lower is better)
    double orientation_diff = 1.0 - calculate_orientation_similarity(goal_pose, lanelet);
    
    // Combined score: distance + orientation difference
    double score = distance + orientation_diff * 10.0;  // Weight orientation heavily
    
    if (score < best_score) {
      best_score = score;
      best_lanelet = lanelet;
    }
  }
  
  if (best_lanelet) {
    RCLCPP_DEBUG(node_->get_logger(), 
                 "Found goal lanelet ID: %ld, distance: %.2f, score: %.2f",
                 best_lanelet->id(), 
                 calculate_distance_to_centerline_global(goal_x, goal_y, *best_lanelet),
                 best_score);
  }
  
  return best_lanelet;
}

double RoutePlanner::calculate_distance_to_centerline(
  const geometry_msgs::msg::PoseStamped & pose,
  const lanelet::ConstLanelet & lanelet)
{
  lanelet::BasicPoint2d pose_point(pose.pose.position.x, pose.pose.position.y);
  
  double min_distance = std::numeric_limits<double>::max();
  
  auto centerline = lanelet.centerline();
  for (size_t i = 0; i < centerline.size() - 1; ++i) {
    auto p1 = centerline[i];
    auto p2 = centerline[i + 1];
    
    // Calculate distance from point to line segment
    double A = pose_point.x() - p1.x();
    double B = pose_point.y() - p1.y();
    double C = p2.x() - p1.x();
    double D = p2.y() - p1.y();
    
    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    
    double param = (len_sq != 0) ? dot / len_sq : -1;
    
    double xx, yy;
    if (param < 0) {
      xx = p1.x();
      yy = p1.y();
    } else if (param > 1) {
      xx = p2.x();
      yy = p2.y();
    } else {
      xx = p1.x() + param * C;
      yy = p1.y() + param * D;
    }
    
    double distance = std::sqrt((pose_point.x() - xx) * (pose_point.x() - xx) + 
                               (pose_point.y() - yy) * (pose_point.y() - yy));
    
    min_distance = std::min(min_distance, distance);
  }
  
  return min_distance;
}

double RoutePlanner::calculate_distance_to_centerline_raw(
  double pose_x, double pose_y, const lanelet::ConstLanelet & lanelet)
{
  double min_distance = std::numeric_limits<double>::max();
  
  auto centerline = lanelet.centerline();
  for (size_t i = 0; i < centerline.size() - 1; ++i) {
    auto p1 = centerline[i];
    auto p2 = centerline[i + 1];
    
    // Calculate distance from point to line segment
    double A = pose_x - p1.x();
    double B = pose_y - p1.y();
    double C = p2.x() - p1.x();
    double D = p2.y() - p1.y();
    
    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    
    double param = (len_sq != 0) ? dot / len_sq : -1;
    
    double xx, yy;
    if (param < 0) {
      xx = p1.x();
      yy = p1.y();
    } else if (param > 1) {
      xx = p2.x();
      yy = p2.y();
    } else {
      xx = p1.x() + param * C;
      yy = p1.y() + param * D;
    }
    
    double distance = std::sqrt((pose_x - xx) * (pose_x - xx) + 
                               (pose_y - yy) * (pose_y - yy));
    
    min_distance = std::min(min_distance, distance);
  }
  
  return min_distance;
}

double RoutePlanner::calculate_distance_to_centerline_global(
  double pose_x, double pose_y, const lanelet::ConstLanelet & lanelet)
{
  double min_distance = std::numeric_limits<double>::max();
  
  auto centerline = lanelet.centerline();
  for (size_t i = 0; i < centerline.size() - 1; ++i) {
    auto p1 = centerline[i];
    auto p2 = centerline[i + 1];
    
    // Convert points to global coordinates using cache or offset
    double p1_x, p1_y, p2_x, p2_y;
    
    // Try to get cached coordinates first for p1
    auto cache_it1 = local_coord_cache_.find(p1.id());
    if (cache_it1 != local_coord_cache_.end()) {
      p1_x = cache_it1->second.first;
      p1_y = cache_it1->second.second;
    } else {
      p1_x = p1.x() + local_to_global_offset_x_;
      p1_y = p1.y() + local_to_global_offset_y_;
    }
    
    // Try to get cached coordinates first for p2
    auto cache_it2 = local_coord_cache_.find(p2.id());
    if (cache_it2 != local_coord_cache_.end()) {
      p2_x = cache_it2->second.first;
      p2_y = cache_it2->second.second;
    } else {
      p2_x = p2.x() + local_to_global_offset_x_;
      p2_y = p2.y() + local_to_global_offset_y_;
    }
    
    // Calculate distance from point to line segment in global coordinates
    double A = pose_x - p1_x;
    double B = pose_y - p1_y;
    double C = p2_x - p1_x;
    double D = p2_y - p1_y;
    
    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    
    double param = (len_sq != 0) ? dot / len_sq : -1;
    
    double xx, yy;
    if (param < 0) {
      xx = p1_x;
      yy = p1_y;
    } else if (param > 1) {
      xx = p2_x;
      yy = p2_y;
    } else {
      xx = p1_x + param * C;
      yy = p1_y + param * D;
    }
    
    double global_distance = std::sqrt((pose_x - xx) * (pose_x - xx) + 
                                      (pose_y - yy) * (pose_y - yy));
    
    min_distance = std::min(min_distance, global_distance);
  }
  
  return min_distance;
}

double RoutePlanner::calculate_orientation_similarity(
  const geometry_msgs::msg::PoseStamped & pose,
  const lanelet::ConstLanelet & lanelet)
{
  // Get pose orientation
  tf2::Quaternion pose_quat;
  tf2::fromMsg(pose.pose.orientation, pose_quat);
  tf2::Matrix3x3 pose_matrix(pose_quat);
  double pose_roll, pose_pitch, pose_yaw;
  pose_matrix.getRPY(pose_roll, pose_pitch, pose_yaw);
  
  // Calculate lanelet direction at the point closest to the pose
  auto centerline = lanelet.centerline();
  if (centerline.size() < 2) {
    return 0.0;  // Cannot determine direction
  }
  
  // Find closest centerline segment
  lanelet::BasicPoint2d pose_point(pose.pose.position.x, pose.pose.position.y);
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_segment = 0;
  
  for (size_t i = 0; i < centerline.size() - 1; ++i) {
    auto p1 = centerline[i];
    auto p2 = centerline[i + 1];
    
    // Calculate distance to segment midpoint
    double mid_x = (p1.x() + p2.x()) / 2.0;
    double mid_y = (p1.y() + p2.y()) / 2.0;
    double distance = std::sqrt((pose_point.x() - mid_x) * (pose_point.x() - mid_x) + 
                               (pose_point.y() - mid_y) * (pose_point.y() - mid_y));
    
    if (distance < min_distance) {
      min_distance = distance;
      closest_segment = i;
    }
  }
  
  // Calculate lanelet direction
  auto p1 = centerline[closest_segment];
  auto p2 = centerline[closest_segment + 1];
  double lanelet_yaw = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
  
  // Calculate similarity (cosine of angle difference)
  double angle_diff = std::abs(pose_yaw - lanelet_yaw);
  if (angle_diff > M_PI) {
    angle_diff = 2.0 * M_PI - angle_diff;
  }
  
  return std::cos(angle_diff);  // 1.0 = perfect alignment, 0.0 = perpendicular
}

std::vector<autoware_planning_msgs::msg::PathPoint> RoutePlanner::create_centerline_path(
  const lanelet::ConstLanelet & lanelet, double resolution)
{
  std::vector<autoware_planning_msgs::msg::PathPoint> path_points;
  
  auto centerline = lanelet.centerline();
  if (centerline.size() < 2) {
    return path_points;
  }
  
  // Sample points along the centerline
  for (size_t i = 0; i < centerline.size() - 1; ++i) {
    auto p1 = centerline[i];
    auto p2 = centerline[i + 1];
    
    double segment_length = std::sqrt(
      (p2.x() - p1.x()) * (p2.x() - p1.x()) + 
      (p2.y() - p1.y()) * (p2.y() - p1.y()));
    
    int num_points = std::max(1, static_cast<int>(segment_length / resolution));
    
    for (int j = 0; j < num_points; ++j) {
      double t = static_cast<double>(j) / static_cast<double>(num_points);
      
      lanelet::BasicPoint3d interpolated_point(
        p1.x() + t * (p2.x() - p1.x()),
        p1.y() + t * (p2.y() - p1.y()),
        p1.z() + t * (p2.z() - p1.z())
      );
      
      auto path_point = create_path_point(interpolated_point);
      
      // Calculate orientation based on segment direction
      double yaw = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
      tf2::Quaternion quat;
      quat.setRPY(0, 0, yaw);
      path_point.pose.orientation = tf2::toMsg(quat);
      
      path_points.push_back(path_point);
    }
  }
  
  // Add the last point
  auto last_point = centerline.back();
  auto last_path_point = create_path_point(last_point);
  
  // Use the same orientation as the last segment
  if (centerline.size() >= 2) {
    auto p1 = centerline[centerline.size() - 2];
    auto p2 = centerline[centerline.size() - 1];
    double yaw = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    last_path_point.pose.orientation = tf2::toMsg(quat);
  }
  
  path_points.push_back(last_path_point);
  
  return path_points;
}

autoware_planning_msgs::msg::PathPoint RoutePlanner::create_path_point(
  const lanelet::BasicPoint3d & point, double velocity)
{
  autoware_planning_msgs::msg::PathPoint path_point;
  
  // For BasicPoint3d, we can't access the ID directly
  // We'll use the projected coordinates with automatic offset transformation
  // This ensures consistency with the coordinate system used elsewhere
  path_point.pose.position.x = point.x() + local_to_global_offset_x_;
  path_point.pose.position.y = point.y() + local_to_global_offset_y_;
  path_point.pose.position.z = point.z();
  
  // Set default orientation (will be updated by calling function)
  path_point.pose.orientation.w = 1.0;
  
  path_point.longitudinal_velocity_mps = velocity;
  
  return path_point;
}

visualization_msgs::msg::MarkerArray RoutePlanner::get_lanelet_visualization() const
{
  visualization_msgs::msg::MarkerArray markers;
  
  if (!map_loaded_) {
    return markers;
  }
  
  int marker_id = 0;
  
  for (const auto & lanelet : lanelet_map_->laneletLayer) {
    // Create centerline marker
    visualization_msgs::msg::Marker centerline_marker;
    centerline_marker.header.frame_id = param_.map_frame;
    centerline_marker.header.stamp = node_->get_clock()->now();
    centerline_marker.ns = "lanelet_centerlines";
    centerline_marker.id = marker_id++;
    centerline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    centerline_marker.action = visualization_msgs::msg::Marker::ADD;
    centerline_marker.scale.x = 0.2;
    centerline_marker.color.a = 1.0;
    centerline_marker.color.r = 0.0;
    centerline_marker.color.g = 1.0;
    centerline_marker.color.b = 0.0;
    
    auto centerline = lanelet.centerline();
    for (const auto & point : centerline) {
      geometry_msgs::msg::Point p;
      // Convert lanelet coordinates to global coordinates using cache or offset
      auto cache_it = local_coord_cache_.find(point.id());
      if (cache_it != local_coord_cache_.end()) {
        p.x = cache_it->second.first;   // local_x from OSM
        p.y = cache_it->second.second;  // local_y from OSM
      } else {
        p.x = point.x() + local_to_global_offset_x_;
        p.y = point.y() + local_to_global_offset_y_;
      }
      p.z = point.z();
      centerline_marker.points.push_back(p);
    }
    
    if (!centerline_marker.points.empty()) {
      markers.markers.push_back(centerline_marker);
    }
    
    // Create boundary markers
    visualization_msgs::msg::Marker left_boundary_marker = centerline_marker;
    left_boundary_marker.ns = "lanelet_left_boundaries";
    left_boundary_marker.id = marker_id++;
    left_boundary_marker.color.r = 1.0;
    left_boundary_marker.color.g = 0.0;
    left_boundary_marker.color.b = 0.0;
    left_boundary_marker.points.clear();
    
    auto left_boundary = lanelet.leftBound();
    for (const auto & point : left_boundary) {
      geometry_msgs::msg::Point p;
      // Convert lanelet coordinates to global coordinates using cache or offset
      auto cache_it = local_coord_cache_.find(point.id());
      if (cache_it != local_coord_cache_.end()) {
        p.x = cache_it->second.first;   // local_x from OSM
        p.y = cache_it->second.second;  // local_y from OSM
      } else {
        p.x = point.x() + local_to_global_offset_x_;
        p.y = point.y() + local_to_global_offset_y_;
      }
      p.z = point.z();
      left_boundary_marker.points.push_back(p);
    }
    
    if (!left_boundary_marker.points.empty()) {
      markers.markers.push_back(left_boundary_marker);
    }
    
    visualization_msgs::msg::Marker right_boundary_marker = centerline_marker;
    right_boundary_marker.ns = "lanelet_right_boundaries";
    right_boundary_marker.id = marker_id++;
    right_boundary_marker.color.r = 0.0;
    right_boundary_marker.color.g = 0.0;
    right_boundary_marker.color.b = 1.0;
    right_boundary_marker.points.clear();
    
    auto right_boundary = lanelet.rightBound();
    for (const auto & point : right_boundary) {
      geometry_msgs::msg::Point p;
      // Convert lanelet coordinates to global coordinates using cache or offset
      auto cache_it = local_coord_cache_.find(point.id());
      if (cache_it != local_coord_cache_.end()) {
        p.x = cache_it->second.first;   // local_x from OSM
        p.y = cache_it->second.second;  // local_y from OSM
      } else {
        p.x = point.x() + local_to_global_offset_x_;
        p.y = point.y() + local_to_global_offset_y_;
      }
      p.z = point.z();
      right_boundary_marker.points.push_back(p);
    }
    
    if (!right_boundary_marker.points.empty()) {
      markers.markers.push_back(right_boundary_marker);
    }
  }
  
  return markers;
}

geometry_msgs::msg::Point RoutePlanner::convert_to_local_coordinates(double lat, double lon) const
{
  geometry_msgs::msg::Point local_point;
  
  if (projector_) {
    auto projected = projector_->forward({lat, lon});
    local_point.x = projected.x();
    local_point.y = projected.y();
    local_point.z = 0.0;
  }
  
  return local_point;
}

std::pair<double, double> RoutePlanner::convert_to_global_coordinates(
  const geometry_msgs::msg::Point & local_point) const
{
  // Simplified coordinate conversion for now
  // In a real implementation, you'd use proper coordinate transformation
  return {0.0, 0.0};
}

std::optional<autoware_planning_msgs::msg::Path> RoutePlanner::create_fallback_path(
  const geometry_msgs::msg::PoseStamped & start_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose)
{
  autoware_planning_msgs::msg::Path path;
  path.header.frame_id = param_.map_frame;
  path.header.stamp = node_->get_clock()->now();
  
  // Create a simple straight-line path with intermediate points
  const int num_points = 10;  // Number of waypoints between start and goal
  
  for (int i = 0; i <= num_points; ++i) {
    double t = static_cast<double>(i) / num_points;  // Interpolation parameter [0, 1]
    
    autoware_planning_msgs::msg::PathPoint point;
    
    // Linear interpolation between start and goal positions
    point.pose.position.x = start_pose.pose.position.x + t * (goal_pose.pose.position.x - start_pose.pose.position.x);
    point.pose.position.y = start_pose.pose.position.y + t * (goal_pose.pose.position.y - start_pose.pose.position.y);
    point.pose.position.z = start_pose.pose.position.z + t * (goal_pose.pose.position.z - start_pose.pose.position.z);
    
    // Linear interpolation for orientation (simplified)
    if (i == 0) {
      point.pose.orientation = start_pose.pose.orientation;
    } else if (i == num_points) {
      point.pose.orientation = goal_pose.pose.orientation;
    } else {
      // Calculate orientation pointing towards goal
      double dx = goal_pose.pose.position.x - start_pose.pose.position.x;
      double dy = goal_pose.pose.position.y - start_pose.pose.position.y;
      double yaw = std::atan2(dy, dx);
      
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      point.pose.orientation = tf2::toMsg(q);
    }
    
    // Set basic motion parameters
    point.longitudinal_velocity_mps = 5.0;  // 5 m/s default speed
    point.lateral_velocity_mps = 0.0;
    point.heading_rate_rps = 0.0;
    
    path.points.push_back(point);
  }
  
  RCLCPP_INFO(node_->get_logger(), "Created fallback straight-line path with %zu points", path.points.size());
  return path;
}

}  // namespace awsim_path_planner
