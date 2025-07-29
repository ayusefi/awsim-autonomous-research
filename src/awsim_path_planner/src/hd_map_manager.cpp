#include "awsim_path_planner/hd_map_manager.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <queue>

namespace awsim_path_planner
{

HDMapManager::HDMapManager(rclcpp::Node* node)
: node_(node),
  map_loaded_(false),
  origin_lat_(35.6762),  // Tokyo area default
  origin_lon_(139.6503),
  // Use the SAME coordinate transformation as the localization system
  // These values MUST match the localization launch file parameters exactly
  // The localization system transforms: pose = gnss_pose - map_center_offset
  // The HD map must use the same transformation for consistency
  map_origin_offset_x_(81237.62),  // Manual map center X (from localization launch file)
  map_origin_offset_y_(49905.12),  // Manual map center Y (from localization launch file)
  map_origin_offset_z_(42.12),     // Manual map center Z (from localization launch file)
  lane_width_(3.5),  // Standard lane width in meters
  boundary_buffer_(0.5),
  driveable_area_buffer_(2.0),
  off_lane_penalty_(100.0),
  boundary_proximity_penalty_(50.0),
  speed_deviation_penalty_(10.0),
  wrong_way_penalty_(200.0),
  traffic_rule_violation_penalty_(150.0)
{
  RCLCPP_INFO(node_->get_logger(), "HD Map Manager initialized");
}

bool HDMapManager::load_map(const std::string& map_file_path)
{
  map_file_path_ = map_file_path;
  
  RCLCPP_INFO(node_->get_logger(), "Loading HD map from: %s", map_file_path.c_str());
  
  if (!parse_osm_file(map_file_path)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse OSM file");
    return false;
  }
  
  extract_lanes_from_osm();
  calculate_map_bounds();
  
  map_loaded_ = true;
  RCLCPP_INFO(node_->get_logger(), "HD map loaded successfully with %zu lanes", lanes_.size());
  
  return true;
}

bool HDMapManager::parse_osm_file(const std::string& file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot open OSM file: %s", file_path.c_str());
    return false;
  }
  
  std::string xml_content((std::istreambuf_iterator<char>(file)),
                          std::istreambuf_iterator<char>());
  file.close();
  
  // Parse nodes and ways from XML content
  if (!parse_osm_nodes(xml_content)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse OSM nodes");
    return false;
  }
  
  if (!parse_osm_ways(xml_content)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse OSM ways");
    return false;
  }
  
  if (!parse_osm_relations(xml_content)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse OSM relations");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Found %zu nodes, %zu ways, and %zu relations in OSM file", 
              osm_nodes_.size(), osm_ways_.size(), osm_relations_.size());
  
  return true;
}

bool HDMapManager::parse_osm_nodes(const std::string& xml_content)
{
  osm_nodes_.clear();
  
  size_t pos = 0;
  while ((pos = xml_content.find("<node", pos)) != std::string::npos) {
    size_t end_pos = xml_content.find(">", pos);
    if (end_pos == std::string::npos) break;
    
    std::string node_line = xml_content.substr(pos, end_pos - pos + 1);
    
    // Extract node attributes
    long id = 0;
    double lat = 0.0, lon = 0.0;
    double local_x = 0.0, local_y = 0.0;
    
    // Parse id
    size_t id_pos = node_line.find("id=\"");
    if (id_pos != std::string::npos) {
      id_pos += 4;
      size_t id_end = node_line.find("\"", id_pos);
      if (id_end != std::string::npos) {
        id = std::stol(node_line.substr(id_pos, id_end - id_pos));
      }
    }
    
    // Parse lat (keep for reference, but prefer local_x/local_y)
    size_t lat_pos = node_line.find("lat=\"");
    if (lat_pos != std::string::npos) {
      lat_pos += 5;
      size_t lat_end = node_line.find("\"", lat_pos);
      if (lat_end != std::string::npos) {
        lat = std::stod(node_line.substr(lat_pos, lat_end - lat_pos));
      }
    }
    
    // Parse lon (keep for reference, but prefer local_x/local_y)
    size_t lon_pos = node_line.find("lon=\"");
    if (lon_pos != std::string::npos) {
      lon_pos += 5;
      size_t lon_end = node_line.find("\"", lon_pos);
      if (lon_end != std::string::npos) {
        lon = std::stod(node_line.substr(lon_pos, lon_end - lon_pos));
      }
    }

    // Find the node's tag section to extract local_x and local_y
    size_t tag_start = pos;
    size_t node_close = xml_content.find(">", pos);
    size_t node_end = xml_content.find("</node>", pos);
    if (node_end == std::string::npos) {
      node_end = xml_content.find("/>", pos);
    }
    
    if (node_close != std::string::npos && node_end != std::string::npos) {
      std::string node_content = xml_content.substr(tag_start, node_end - tag_start);
      
      // Extract local_x tag
      size_t local_x_pos = node_content.find("local_x");
      if (local_x_pos != std::string::npos) {
        size_t value_start = node_content.find("v=\"", local_x_pos);
        if (value_start != std::string::npos) {
          value_start += 3;
          size_t value_end = node_content.find("\"", value_start);
          if (value_end != std::string::npos) {
            local_x = std::stod(node_content.substr(value_start, value_end - value_start));
          }
        }
      }
      
      // Extract local_y tag
      size_t local_y_pos = node_content.find("local_y");
      if (local_y_pos != std::string::npos) {
        size_t value_start = node_content.find("v=\"", local_y_pos);
        if (value_start != std::string::npos) {
          value_start += 3;
          size_t value_end = node_content.find("\"", value_start);
          if (value_end != std::string::npos) {
            local_y = std::stod(node_content.substr(value_start, value_end - value_start));
          }
        }
      }
    }
    
    if (id != 0) {
      OSMNode node;
      node.id = id;
      node.lat = lat;
      node.lon = lon;
      
      // Use local_x and local_y from OSM tags if available, otherwise convert lat/lon
      if (local_x != 0.0 && local_y != 0.0) {
        // Use the local coordinates directly from OSM tags
        node.x = local_x;
        node.y = local_y;
      } else {
        // Fallback: Convert lat/lon to UTM/local coordinates
        // For Japan (approximate conversion), assuming UTM zone 54N
        double x_utm = (lon - 139.0) * 111320.0 * std::cos(lat * M_PI / 180.0);
        double y_utm = (lat - 35.0) * 110540.0;
        node.x = x_utm;
        node.y = y_utm;
      }
      
      osm_nodes_[id] = node;
    }
    
    pos = end_pos;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Parsed %zu nodes from OSM", osm_nodes_.size());
  return true;
}

bool HDMapManager::parse_osm_ways(const std::string& xml_content)
{
  osm_ways_.clear();
  
  size_t pos = 0;
  while ((pos = xml_content.find("<way", pos)) != std::string::npos) {
    size_t way_end = xml_content.find("</way>", pos);
    if (way_end == std::string::npos) break;
    
    std::string way_section = xml_content.substr(pos, way_end - pos);
    
    // Extract way id
    long way_id = 0;
    size_t id_pos = way_section.find("id=\"");
    if (id_pos != std::string::npos) {
      id_pos += 4;
      size_t id_end = way_section.find("\"", id_pos);
      if (id_end != std::string::npos) {
        way_id = std::stol(way_section.substr(id_pos, id_end - id_pos));
      }
    }
    
    if (way_id != 0) {
      OSMWay way;
      way.id = way_id;
      
      // Parse node references
      size_t nd_pos = 0;
      while ((nd_pos = way_section.find("<nd ref=\"", nd_pos)) != std::string::npos) {
        nd_pos += 9;
        size_t nd_end = way_section.find("\"", nd_pos);
        if (nd_end != std::string::npos) {
          long node_ref = std::stol(way_section.substr(nd_pos, nd_end - nd_pos));
          way.node_refs.push_back(node_ref);
        }
        nd_pos = nd_end;
      }
      
      // Parse tags
      size_t tag_pos = 0;
      while ((tag_pos = way_section.find("<tag k=\"", tag_pos)) != std::string::npos) {
        tag_pos += 8;
        size_t key_end = way_section.find("\"", tag_pos);
        if (key_end == std::string::npos) break;
        
        std::string key = way_section.substr(tag_pos, key_end - tag_pos);
        
        size_t val_pos = way_section.find("v=\"", key_end);
        if (val_pos == std::string::npos) break;
        val_pos += 3;
        
        size_t val_end = way_section.find("\"", val_pos);
        if (val_end == std::string::npos) break;
        
        std::string value = way_section.substr(val_pos, val_end - val_pos);
        way.tags[key] = value;
        
        tag_pos = val_end;
      }
      
      osm_ways_[way_id] = way;
    }
    
    pos = way_end;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Parsed %zu ways from OSM", osm_ways_.size());
  return true;
}

bool HDMapManager::parse_osm_relations(const std::string& xml_content)
{
  osm_relations_.clear();
  
  size_t pos = 0;
  while ((pos = xml_content.find("<relation", pos)) != std::string::npos) {
    size_t relation_end = xml_content.find("</relation>", pos);
    if (relation_end == std::string::npos) break;
    
    std::string relation_section = xml_content.substr(pos, relation_end - pos);
    
    // Extract relation id
    long relation_id = 0;
    size_t id_pos = relation_section.find("id=\"");
    if (id_pos != std::string::npos) {
      id_pos += 4;
      size_t id_end = relation_section.find("\"", id_pos);
      if (id_end != std::string::npos) {
        relation_id = std::stol(relation_section.substr(id_pos, id_end - id_pos));
      }
    }
    
    if (relation_id != 0) {
      OSMRelation relation;
      relation.id = relation_id;
      
      // Parse member elements
      size_t member_pos = 0;
      while ((member_pos = relation_section.find("<member", member_pos)) != std::string::npos) {
        size_t member_end = relation_section.find("/>", member_pos);
        if (member_end == std::string::npos) break;
        
        std::string member_section = relation_section.substr(member_pos, member_end - member_pos);
        
        OSMRelationMember member;
        
        // Extract type
        size_t type_pos = member_section.find("type=\"");
        if (type_pos != std::string::npos) {
          type_pos += 6;
          size_t type_end = member_section.find("\"", type_pos);
          if (type_end != std::string::npos) {
            member.type = member_section.substr(type_pos, type_end - type_pos);
          }
        }
        
        // Extract ref
        size_t ref_pos = member_section.find("ref=\"");
        if (ref_pos != std::string::npos) {
          ref_pos += 5;
          size_t ref_end = member_section.find("\"", ref_pos);
          if (ref_end != std::string::npos) {
            member.ref = std::stol(member_section.substr(ref_pos, ref_end - ref_pos));
          }
        }
        
        // Extract role
        size_t role_pos = member_section.find("role=\"");
        if (role_pos != std::string::npos) {
          role_pos += 6;
          size_t role_end = member_section.find("\"", role_pos);
          if (role_end != std::string::npos) {
            member.role = member_section.substr(role_pos, role_end - role_pos);
          }
        }
        
        relation.members.push_back(member);
        member_pos = member_end;
      }
      
      // Parse tags
      size_t tag_pos = 0;
      while ((tag_pos = relation_section.find("<tag k=\"", tag_pos)) != std::string::npos) {
        tag_pos += 8;
        size_t key_end = relation_section.find("\"", tag_pos);
        if (key_end == std::string::npos) break;
        
        std::string key = relation_section.substr(tag_pos, key_end - tag_pos);
        
        size_t val_pos = relation_section.find("v=\"", key_end);
        if (val_pos == std::string::npos) break;
        val_pos += 3;
        
        size_t val_end = relation_section.find("\"", val_pos);
        if (val_end == std::string::npos) break;
        
        std::string value = relation_section.substr(val_pos, val_end - val_pos);
        relation.tags[key] = value;
        
        tag_pos = val_end;
      }
      
      osm_relations_[relation_id] = relation;
    }
    
    pos = relation_end;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Parsed %zu relations from OSM", osm_relations_.size());
  return true;
}

void HDMapManager::convert_coordinates()
{
  RCLCPP_INFO(node_->get_logger(), "Coordinate conversion completed");
}

void HDMapManager::extract_lanes_from_osm()
{
  lanes_.clear();
  
  RCLCPP_INFO(node_->get_logger(), "Extracting actual lanelets from Lanelet2 OSM relations...");
  
  int lanelet_count = 0;
  int relations_processed = 0;
  
  // Process OSM relations to find actual lanelets (not ways!)
  for (const auto& relation : osm_relations_) {
    relations_processed++;
    
    // Debug first few relations to see what tags are available
    if (relations_processed <= 3) {
      RCLCPP_INFO(node_->get_logger(), "Relation %ld tags:", relation.first);
      for (const auto& tag : relation.second.tags) {
        RCLCPP_INFO(node_->get_logger(), "  %s = %s", tag.first.c_str(), tag.second.c_str());
      }
      RCLCPP_INFO(node_->get_logger(), "  Members: %zu", relation.second.members.size());
      for (const auto& member : relation.second.members) {
        RCLCPP_INFO(node_->get_logger(), "    %s %ld role=%s", member.type.c_str(), member.ref, member.role.c_str());
      }
    }
    
    // Check if this relation is a lanelet
    auto type_tag = relation.second.tags.find("type");
    auto subtype_tag = relation.second.tags.find("subtype");
    
    bool is_lanelet = false;
    if (type_tag != relation.second.tags.end() && type_tag->second == "lanelet") {
      if (subtype_tag != relation.second.tags.end() && subtype_tag->second == "road") {
        is_lanelet = true;
      }
    }
    
    if (is_lanelet) {
      lanelet_count++;
      
      RCLCPP_DEBUG(node_->get_logger(), "Processing lanelet relation %ld", relation.first);
      
      LaneInfo lane;
      lane.id = relation.first;  // Use relation ID as lane ID
      lane.traffic_rule.speed_limit = 40.0;  // Default speed limit (common in OSM data)
      lane.is_driveable = true;
      
      // Extract speed limit if available
      auto speed_tag = relation.second.tags.find("maxspeed");
      if (speed_tag != relation.second.tags.end()) {
        try {
          std::string speed_str = speed_tag->second;
          // Handle different speed formats like "40", "40 km/h", "40 kmh"
          if (speed_str.find("km") != std::string::npos) {
            speed_str = speed_str.substr(0, speed_str.find(" "));
          }
          lane.traffic_rule.speed_limit = std::stod(speed_str);
        } catch (const std::exception& e) {
          RCLCPP_WARN(node_->get_logger(), "Failed to parse speed limit for lanelet %ld: %s", 
                     relation.first, speed_tag->second.c_str());
        }
      }
      
      // Extract turn direction if available
      auto turn_tag = relation.second.tags.find("turn_direction");
      if (turn_tag != relation.second.tags.end()) {
        // Use turn direction for path planning hints
        RCLCPP_DEBUG(node_->get_logger(), "Lanelet %ld turn direction: %s", 
                    relation.first, turn_tag->second.c_str());
      }
      
      // Find left and right boundary ways to generate centerline
      long left_way_id = -1, right_way_id = -1;
      
      for (const auto& member : relation.second.members) {
        if (member.type == "way") {
          if (member.role == "left") {
            left_way_id = member.ref;
          } else if (member.role == "right") {
            right_way_id = member.ref;
          }
        }
      }
      
      // Generate centerline from left and right boundaries
      if (left_way_id != -1 && right_way_id != -1) {
        auto left_way_it = osm_ways_.find(left_way_id);
        auto right_way_it = osm_ways_.find(right_way_id);
        
        if (left_way_it != osm_ways_.end() && right_way_it != osm_ways_.end()) {
          const OSMWay& left_way = left_way_it->second;
          const OSMWay& right_way = right_way_it->second;
          
          // Build left and right boundary points
          std::vector<std::pair<double, double>> left_points, right_points;
          
          // Get left boundary points
          for (long node_id : left_way.node_refs) {
            auto node_it = osm_nodes_.find(node_id);
            if (node_it != osm_nodes_.end()) {
              const OSMNode& node = node_it->second;
              // Apply same coordinate transformation as localization system:
              // Subtract map center to center coordinates (same as localization)
              double local_x = node.x - map_origin_offset_x_;
              double local_y = node.y - map_origin_offset_y_;
              left_points.emplace_back(local_x, local_y);
              
              // Debug: print first few coordinate transformations
              if (left_points.size() <= 3) {
                RCLCPP_DEBUG(node_->get_logger(), 
                  "Node %ld: Raw UTM(%.2f, %.2f) -> Local(%.2f, %.2f)", 
                  node_id, node.x, node.y, local_x, local_y);
              }
            }
          }
          
          // Get right boundary points
          for (long node_id : right_way.node_refs) {
            auto node_it = osm_nodes_.find(node_id);
            if (node_it != osm_nodes_.end()) {
              const OSMNode& node = node_it->second;
              // Apply same coordinate transformation as localization system:
              // Subtract map center to center coordinates (same as localization)
              double local_x = node.x - map_origin_offset_x_;
              double local_y = node.y - map_origin_offset_y_;
              right_points.emplace_back(local_x, local_y);
            }
          }
          
          // Generate centerline by interpolating between left and right boundaries
          size_t min_points = std::min(left_points.size(), right_points.size());
          if (min_points >= 2) {
            for (size_t i = 0; i < min_points; ++i) {
              double center_x = (left_points[i].first + right_points[i].first) / 2.0;
              double center_y = (left_points[i].second + right_points[i].second) / 2.0;
              lane.centerline.emplace_back(center_x, center_y);
            }
            
            // Add lane to the list
            lanes_.push_back(lane);
            
            RCLCPP_DEBUG(node_->get_logger(), 
                        "Added lanelet %d with %zu centerline points, speed limit: %.1f km/h", 
                        lane.id, lane.centerline.size(), lane.traffic_rule.speed_limit);
          } else {
            RCLCPP_WARN(node_->get_logger(), "Insufficient boundary points for lanelet %ld", relation.first);
          }
        } else {
          RCLCPP_WARN(node_->get_logger(), "Could not find boundary ways for lanelet %ld (left=%ld, right=%ld)", 
                     relation.first, left_way_id, right_way_id);
        }
      } else {
        RCLCPP_WARN(node_->get_logger(), "Lanelet %ld missing left/right boundary ways", relation.first);
      }
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Extracted %d lanelets from %zu relations in OSM data", 
              lanelet_count, osm_relations_.size());
              
  // If we didn't find enough lanes, create some fallback lanes around current position
  if (lanes_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No lanelets found in OSM relations, creating fallback lane");
    
    LaneInfo fallback_lane;
    fallback_lane.id = 999;
    fallback_lane.traffic_rule.speed_limit = 50.0;
    fallback_lane.is_driveable = true;
    
    // Create a straight lane around expected vehicle position in local coordinates
    for (int i = 0; i <= 20; ++i) {
      double x = 130.0 + i * 2.0;  // Around expected vehicle position
      double y = 10.0;
      fallback_lane.centerline.emplace_back(x, y);
    }
    
    lanes_.push_back(fallback_lane);
  }
  
  RCLCPP_INFO(node_->get_logger(), "Lanelet extraction completed with %zu total lanes", lanes_.size());
  
  // Parse additional HD map features
  parse_traffic_lights();
  parse_traffic_signs();
  parse_crosswalks();
  parse_road_markings();
  parse_regulatory_elements();
}

void HDMapManager::calculate_map_bounds()
{
  if (lanes_.empty()) {
    map_bounds_ = {0.0, 0.0, 0.0, 0.0};
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
  
  map_bounds_ = {min_x, max_x, min_y, max_y};
  
  RCLCPP_INFO(node_->get_logger(), 
    "Map bounds after coordinate transformation: X[%.2f, %.2f], Y[%.2f, %.2f]", 
    min_x, max_x, min_y, max_y);
  RCLCPP_INFO(node_->get_logger(), 
    "Map origin offsets being used: X=%.2f, Y=%.2f, Z=%.2f",
    map_origin_offset_x_, map_origin_offset_y_, map_origin_offset_z_);
}

bool HDMapManager::is_point_driveable(double x, double y) const
{
  if (!map_loaded_) return false;
  
  for (const auto& lane : lanes_) {
    if (lane.is_driveable && is_position_in_lane(x, y, lane)) {
      return true;
    }
  }
  
  return false;
}

bool HDMapManager::is_position_in_lane(double x, double y, const LaneInfo& lane) const
{
  // Simple distance-based check for now
  double distance = calculate_distance_to_lane_centerline(x, y, lane);
  return distance <= lane_width_ / 2.0;
}

double HDMapManager::calculate_distance_to_lane_centerline(double x, double y, const LaneInfo& lane) const
{
  if (lane.centerline.empty()) {
    return std::numeric_limits<double>::max();
  }
  
  double min_distance = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < lane.centerline.size() - 1; ++i) {
    double dist = point_to_line_distance(x, y,
                                        lane.centerline[i].first, lane.centerline[i].second,
                                        lane.centerline[i+1].first, lane.centerline[i+1].second);
    min_distance = std::min(min_distance, dist);
  }
  
  return min_distance;
}

bool HDMapManager::is_path_valid(const std::vector<geometry_msgs::msg::PoseStamped>& path) const
{
  if (!map_loaded_ || path.empty()) {
    return false;
  }
  
  // For lane-following paths, be more permissive
  // Allow paths that stay within reasonable distance of lanes
  const double max_lane_deviation = lane_width_ * 1.5; // Allow 1.5x lane width deviation
  
  for (const auto& pose : path) {
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    
    // Check if point is within reasonable distance of any lane
    bool near_lane = false;
    for (const auto& lane : lanes_) {
      if (lane.is_driveable) {
        double distance = calculate_distance_to_lane_centerline(x, y, lane);
        if (distance <= max_lane_deviation) {
          near_lane = true;
          break;
        }
      }
    }
    
    if (!near_lane) {
      RCLCPP_WARN(node_->get_logger(), "Path point (%.2f, %.2f) too far from any driveable lane", x, y);
      return false;
    }
  }
  
  return true;
}

LaneInfo HDMapManager::get_nearest_lane(double x, double y) const
{
  if (lanes_.empty()) {
    LaneInfo empty_lane;
    empty_lane.id = -1;
    return empty_lane;
  }
  
  const LaneInfo* nearest_lane = nullptr;
  double min_distance = std::numeric_limits<double>::max();
  
  for (const auto& lane : lanes_) {
    double distance = calculate_distance_to_lane_centerline(x, y, lane);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_lane = &lane;
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
  
  // For simplicity, use distance to centerline as boundary cost
  double centerline_distance = calculate_distance_to_lane_centerline(x, y, nearest_lane);
  
  if (centerline_distance > lane_width_ / 2.0) {
    return boundary_proximity_penalty_;
  }
  
  return 0.0;
}

double HDMapManager::get_speed_limit_cost(double x, double y, double desired_speed) const
{
  if (!map_loaded_) return 0.0;
  
  LaneInfo nearest_lane = get_nearest_lane(x, y);
  if (nearest_lane.id == -1) return 0.0;
  
  double speed_limit = nearest_lane.traffic_rule.speed_limit;
  double speed_diff = std::abs(desired_speed - speed_limit);
  
  return speed_diff * speed_deviation_penalty_;
}

double HDMapManager::get_speed_limit_at_point(double x, double y) const
{
  LaneInfo nearest_lane = get_nearest_lane(x, y);
  if (nearest_lane.id != -1) {
    return nearest_lane.traffic_rule.speed_limit;
  }
  return 50.0 / 3.6;  // Default 50 km/h in m/s
}

// Geometric utilities
double HDMapManager::point_to_line_distance(double px, double py,
                                            double x1, double y1, double x2, double y2) const
{
  double A = px - x1;
  double B = py - y1;
  double C = x2 - x1;
  double D = y2 - y1;
  
  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  
  if (len_sq < 1e-9) return std::sqrt(A * A + B * B);
  
  double param = dot / len_sq;
  param = std::max(0.0, std::min(1.0, param));
  
  double xx = x1 + param * C;
  double yy = y1 + param * D;
  
  double dx = px - xx;
  double dy = py - yy;
  
  return std::sqrt(dx * dx + dy * dy);
}

bool HDMapManager::is_point_in_polygon(double x, double y, 
                                       const std::vector<std::pair<double, double>>& polygon) const
{
  if (polygon.size() < 3) return false;
  
  bool inside = false;
  size_t j = polygon.size() - 1;
  
  for (size_t i = 0; i < polygon.size(); ++i) {
    if (((polygon[i].second > y) != (polygon[j].second > y)) &&
        (x < (polygon[j].first - polygon[i].first) * (y - polygon[i].second) / 
         (polygon[j].second - polygon[i].second) + polygon[i].first)) {
      inside = !inside;
    }
    j = i;
  }
  
  return inside;
}

double HDMapManager::calculate_lane_length(const std::vector<std::pair<double, double>>& centerline) const
{
  if (centerline.size() < 2) return 0.0;
  
  double length = 0.0;
  for (size_t i = 1; i < centerline.size(); ++i) {
    double dx = centerline[i].first - centerline[i-1].first;
    double dy = centerline[i].second - centerline[i-1].second;
    length += std::sqrt(dx * dx + dy * dy);
  }
  
  return length;
}

std::vector<std::pair<double, double>> HDMapManager::interpolate_centerline(
  const std::vector<long>& node_refs) const
{
  std::vector<std::pair<double, double>> centerline;
  
  for (long node_ref : node_refs) {
    auto it = osm_nodes_.find(node_ref);
    if (it != osm_nodes_.end()) {
      // Apply same coordinate transformation as localization system:
      // Subtract map center to center coordinates (same as localization)
      double local_x = it->second.x - map_origin_offset_x_;
      double local_y = it->second.y - map_origin_offset_y_;
      centerline.emplace_back(local_x, local_y);
    }
  }
  
  return centerline;
}

LaneType HDMapManager::determine_lane_type(const std::unordered_map<std::string, std::string>& tags) const
{
  (void)tags;  // Suppress unused parameter warning
  return LaneType::DRIVING;
}

TrafficRule HDMapManager::extract_traffic_rules(const std::unordered_map<std::string, std::string>& tags) const
{
  (void)tags;  // Suppress unused parameter warning
  TrafficRule rule;
  rule.speed_limit = 50.0;  // Default
  rule.one_way = false;
  return rule;
}

double HDMapManager::parse_speed_limit(const std::string& speed_str) const
{
  try {
    return std::stod(speed_str);
  } catch (const std::exception&) {
    return 50.0;  // Default speed limit
  }
}

// Placeholder implementations for remaining functions
bool HDMapManager::is_turn_allowed(const geometry_msgs::msg::PoseStamped& from,
                                  const geometry_msgs::msg::PoseStamped& to) const
{
  (void)from; (void)to;  // Suppress unused parameter warnings
  return true;  // Placeholder
}

std::vector<geometry_msgs::msg::PoseStamped> HDMapManager::get_lane_following_path(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal) const
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  
  if (!map_loaded_ || lanes_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "HD map not loaded or no lanes available");
    return path;
  }
  
  // Find nearest lanes to start and goal
  LaneInfo start_lane = get_nearest_lane(start.pose.position.x, start.pose.position.y);
  LaneInfo goal_lane = get_nearest_lane(goal.pose.position.x, goal.pose.position.y);
  
  if (start_lane.id == -1 || goal_lane.id == -1) {
    RCLCPP_WARN(node_->get_logger(), "Start or goal not near any lane");
    return path;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Planning lane path from lane %d to lane %d", start_lane.id, goal_lane.id);
  
  // If start and goal are in the same lane, create path along that lane
  if (start_lane.id == goal_lane.id) {
    return create_path_along_lane(start_lane, start, goal);
  }
  
  // Find lane sequence from start to goal
  std::vector<int> lane_sequence = find_lane_sequence(start_lane.id, goal_lane.id);
  
  if (lane_sequence.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No lane sequence found between start and goal");
    return path;
  }
  
  // Create path following the lane sequence
  for (size_t i = 0; i < lane_sequence.size(); ++i) {
    int lane_id = lane_sequence[i];
    
    // Find the lane info
    auto lane_it = std::find_if(lanes_.begin(), lanes_.end(), 
                               [lane_id](const LaneInfo& lane) { return lane.id == lane_id; });
    
    if (lane_it == lanes_.end()) continue;
    
    const LaneInfo& current_lane = *lane_it;
    
    // For first lane, start from vehicle position
    // For last lane, end at goal position  
    // For middle lanes, use full centerline
    
    geometry_msgs::msg::PoseStamped lane_start = start;
    geometry_msgs::msg::PoseStamped lane_goal = goal;
    
    if (i == 0) {
      // First lane: start from current position
      lane_start = start;
      if (lane_sequence.size() > 1) {
        // End at lane end if not the final lane
        lane_goal.pose.position.x = current_lane.centerline.back().first;
        lane_goal.pose.position.y = current_lane.centerline.back().second;
      } else {
        lane_goal = goal;
      }
    } else if (i == lane_sequence.size() - 1) {
      // Last lane: end at goal position
      lane_start.pose.position.x = current_lane.centerline.front().first;
      lane_start.pose.position.y = current_lane.centerline.front().second;
      lane_goal = goal;
    } else {
      // Middle lane: use full centerline
      lane_start.pose.position.x = current_lane.centerline.front().first;
      lane_start.pose.position.y = current_lane.centerline.front().second;
      lane_goal.pose.position.x = current_lane.centerline.back().first;
      lane_goal.pose.position.y = current_lane.centerline.back().second;
    }
    
    auto lane_path = create_path_along_lane(current_lane, lane_start, lane_goal);
    
    // Append to main path (avoid duplicating points at lane boundaries)
    if (path.empty()) {
      path = lane_path;
    } else {
      // Skip first point to avoid duplication
      path.insert(path.end(), lane_path.begin() + 1, lane_path.end());
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Created lane-following path with %zu waypoints", path.size());
  return path;
}

std::vector<int> HDMapManager::get_lane_sequence(const geometry_msgs::msg::PoseStamped& start,
                                                const geometry_msgs::msg::PoseStamped& goal) const
{
  LaneInfo start_lane = get_nearest_lane(start.pose.position.x, start.pose.position.y);
  LaneInfo goal_lane = get_nearest_lane(goal.pose.position.x, goal.pose.position.y);
  
  if (start_lane.id == -1 || goal_lane.id == -1) {
    return {};
  }
  
  return find_lane_sequence(start_lane.id, goal_lane.id);
}

std::vector<int> HDMapManager::find_lane_sequence(int start_lane_id, int goal_lane_id) const
{
  std::vector<int> sequence;
  
  if (start_lane_id == goal_lane_id) {
    sequence.push_back(start_lane_id);
    return sequence;
  }
  
  // Simple implementation: try to find connected path through successor relationships
  // For now, create a simple sequence based on lane IDs (can be enhanced later)
  
  // Find start and goal lanes
  const LaneInfo* start_lane = nullptr;
  const LaneInfo* goal_lane = nullptr;
  
  for (const auto& lane : lanes_) {
    if (lane.id == start_lane_id) start_lane = &lane;
    if (lane.id == goal_lane_id) goal_lane = &lane;
  }
  
  if (!start_lane || !goal_lane) {
    return sequence;
  }
  
  // Simple case: if goal is a successor of start
  auto it = std::find(start_lane->successor_ids.begin(), start_lane->successor_ids.end(), goal_lane_id);
  if (it != start_lane->successor_ids.end()) {
    sequence.push_back(start_lane_id);
    sequence.push_back(goal_lane_id);
    return sequence;
  }
  
  // Simple case: check if there's a direct path through one intermediate lane
  for (int intermediate_id : start_lane->successor_ids) {
    for (const auto& lane : lanes_) {
      if (lane.id == intermediate_id) {
        auto goal_it = std::find(lane.successor_ids.begin(), lane.successor_ids.end(), goal_lane_id);
        if (goal_it != lane.successor_ids.end()) {
          sequence.push_back(start_lane_id);
          sequence.push_back(intermediate_id);
          sequence.push_back(goal_lane_id);
          return sequence;
        }
        break;
      }
    }
  }
  
  // If no connected path found, create direct sequence for adjacent lanes
  if (abs(start_lane_id - goal_lane_id) <= 2) {
    if (start_lane_id < goal_lane_id) {
      for (int id = start_lane_id; id <= goal_lane_id; ++id) {
        sequence.push_back(id);
      }
    } else {
      for (int id = start_lane_id; id >= goal_lane_id; --id) {
        sequence.push_back(id);
      }
    }
  }
  
  return sequence;
}

std::vector<geometry_msgs::msg::PoseStamped> HDMapManager::create_path_along_lane(
  const LaneInfo& lane, 
  const geometry_msgs::msg::PoseStamped& start_pose,
  const geometry_msgs::msg::PoseStamped& goal_pose) const
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  
  if (lane.centerline.empty()) {
    return path;
  }
  
  // Find closest points on lane centerline to start and goal
  size_t start_idx = 0;
  size_t goal_idx = lane.centerline.size() - 1;
  
  double min_start_dist = std::numeric_limits<double>::max();
  double min_goal_dist = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < lane.centerline.size(); ++i) {
    double start_dist = std::sqrt(std::pow(lane.centerline[i].first - start_pose.pose.position.x, 2) +
                                 std::pow(lane.centerline[i].second - start_pose.pose.position.y, 2));
    
    double goal_dist = std::sqrt(std::pow(lane.centerline[i].first - goal_pose.pose.position.x, 2) +
                                std::pow(lane.centerline[i].second - goal_pose.pose.position.y, 2));
    
    if (start_dist < min_start_dist) {
      min_start_dist = start_dist;
      start_idx = i;
    }
    
    if (goal_dist < min_goal_dist) {
      min_goal_dist = goal_dist;
      goal_idx = i;
    }
  }
  
  // Ensure start_idx <= goal_idx
  if (start_idx > goal_idx) {
    std::swap(start_idx, goal_idx);
  }
  
  // Create path along lane centerline from start_idx to goal_idx
  for (size_t i = start_idx; i <= goal_idx; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = start_pose.header;
    pose.pose.position.x = lane.centerline[i].first;
    pose.pose.position.y = lane.centerline[i].second;
    pose.pose.position.z = start_pose.pose.position.z; // Keep same height
    
    // Calculate orientation based on lane direction
    if (i < lane.centerline.size() - 1) {
      double dx = lane.centerline[i+1].first - lane.centerline[i].first;
      double dy = lane.centerline[i+1].second - lane.centerline[i].second;
      double yaw = std::atan2(dy, dx);
      
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else if (i > 0) {
      // Use previous segment orientation for last point
      double dx = lane.centerline[i].first - lane.centerline[i-1].first;
      double dy = lane.centerline[i].second - lane.centerline[i-1].second;
      double yaw = std::atan2(dy, dx);
      
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      pose.pose.orientation = start_pose.pose.orientation;
    }
    
    path.push_back(pose);
  }
  
  return path;
}

bool HDMapManager::is_lane_change_safe(int from_lane_id, int to_lane_id, 
                                      const geometry_msgs::msg::PoseStamped& position) const
{
  (void)from_lane_id; (void)to_lane_id; (void)position;  // Suppress unused parameter warnings
  return true;  // Placeholder
}

bool HDMapManager::has_traffic_control_at_point(double x, double y, std::string& control_type) const
{
  (void)x; (void)y; (void)control_type;  // Suppress unused parameter warnings
  return false;  // Placeholder
}

std::vector<geometry_msgs::msg::PoseStamped> HDMapManager::optimize_path_to_lanes(
  const std::vector<geometry_msgs::msg::PoseStamped>& input_path) const
{
  return input_path;  // Placeholder
}

// Coordinate transformation (same as localization system)
std::pair<double, double> HDMapManager::transform_coordinates(double local_x, double local_y) const
{
  // Apply same transformation as localization system: subtract map origin offset
  double transformed_x = local_x - map_origin_offset_x_;
  double transformed_y = local_y - map_origin_offset_y_;
  return std::make_pair(transformed_x, transformed_y);
}

visualization_msgs::msg::MarkerArray HDMapManager::get_map_visualization() const
{
  visualization_msgs::msg::MarkerArray markers;
  // Placeholder implementation
  return markers;
}

// Additional feature parsing methods
void HDMapManager::parse_traffic_lights()
{
  traffic_lights_.clear();
  
  for (const auto& relation : osm_relations_) {
    const auto& tags = relation.second.tags;
    
    // Look for traffic light relations
    if (tags.find("subtype") != tags.end() && 
        (tags.at("subtype") == "traffic_light" || tags.at("subtype") == "red_yellow_green")) {
      
      TrafficLight traffic_light;
      traffic_light.id = relation.first;
      traffic_light.state = tags.count("subtype") ? tags.at("subtype") : "unknown";
      traffic_light.height = 5.0; // Default height
      
      // Find position from first way member
      for (const auto& member : relation.second.members) {
        if (member.type == "way" && osm_ways_.count(member.ref)) {
          const auto& way = osm_ways_.at(member.ref);
          if (!way.node_refs.empty() && osm_nodes_.count(way.node_refs[0])) {
            const auto& node = osm_nodes_.at(way.node_refs[0]);
            // Use the same coordinate system as lanes: node.x, node.y already contain the proper coordinates
            double local_x = node.x - map_origin_offset_x_;
            double local_y = node.y - map_origin_offset_y_;
            traffic_light.x = local_x;
            traffic_light.y = local_y;
            traffic_light.z = traffic_light.height;
            break;
          }
        }
      }
      
      traffic_lights_.push_back(traffic_light);
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Parsed %zu traffic lights", traffic_lights_.size());
}

void HDMapManager::parse_traffic_signs()
{
  traffic_signs_.clear();
  
  for (const auto& relation : osm_relations_) {
    const auto& tags = relation.second.tags;
    
    // Look for traffic sign relations
    if (tags.find("subtype") != tags.end() && 
        (tags.at("subtype") == "traffic_sign" || tags.at("subtype") == "stop_sign")) {
      
      TrafficSign traffic_sign;
      traffic_sign.id = relation.first;
      traffic_sign.sign_type = tags.at("subtype");
      traffic_sign.height = 3.0; // Default height
      
      // Extract value if available (e.g., speed limit)
      if (tags.count("value")) {
        traffic_sign.value = tags.at("value");
      }
      
      // Find position from first way member
      for (const auto& member : relation.second.members) {
        if (member.type == "way" && osm_ways_.count(member.ref)) {
          const auto& way = osm_ways_.at(member.ref);
          if (!way.node_refs.empty() && osm_nodes_.count(way.node_refs[0])) {
            const auto& node = osm_nodes_.at(way.node_refs[0]);
            // Use the same coordinate system as lanes
            double local_x = node.x - map_origin_offset_x_;
            double local_y = node.y - map_origin_offset_y_;
            traffic_sign.x = local_x;
            traffic_sign.y = local_y;
            traffic_sign.z = traffic_sign.height;
            break;
          }
        }
      }
      
      traffic_signs_.push_back(traffic_sign);
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Parsed %zu traffic signs", traffic_signs_.size());
}

void HDMapManager::parse_crosswalks()
{
  crosswalks_.clear();
  
  for (const auto& relation : osm_relations_) {
    const auto& tags = relation.second.tags;
    
    // Look for crosswalk relations
    if (tags.find("subtype") != tags.end() && tags.at("subtype") == "crosswalk") {
      
      Crosswalk crosswalk;
      crosswalk.id = relation.first;
      crosswalk.crossing_type = "crosswalk";
      crosswalk.has_traffic_light = false;
      
      // Extract boundary from way members
      for (const auto& member : relation.second.members) {
        if (member.type == "way" && osm_ways_.count(member.ref)) {
          const auto& way = osm_ways_.at(member.ref);
          for (long node_ref : way.node_refs) {
            if (osm_nodes_.count(node_ref)) {
              const auto& node = osm_nodes_.at(node_ref);
              // Use the same coordinate system as lanes
              double local_x = node.x - map_origin_offset_x_;
              double local_y = node.y - map_origin_offset_y_;
              crosswalk.boundary.emplace_back(local_x, local_y);
            }
          }
        }
      }
      
      if (!crosswalk.boundary.empty()) {
        crosswalks_.push_back(crosswalk);
      }
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Parsed %zu crosswalks", crosswalks_.size());
}

void HDMapManager::parse_road_markings()
{
  road_markings_.clear();
  
  for (const auto& relation : osm_relations_) {
    const auto& tags = relation.second.tags;
    
    // Look for road marking relations
    if (tags.find("subtype") != tags.end() && tags.at("subtype") == "road_marking") {
      
      RoadMarking road_marking;
      road_marking.id = relation.first;
      road_marking.marking_type = "solid"; // default
      road_marking.color = "white"; // default
      road_marking.width = 0.15; // default line width
      
      // Extract geometry from way members  
      for (const auto& member : relation.second.members) {
        if (member.type == "way" && osm_ways_.count(member.ref)) {
          const auto& way = osm_ways_.at(member.ref);
          for (long node_ref : way.node_refs) {
            if (osm_nodes_.count(node_ref)) {
              const auto& node = osm_nodes_.at(node_ref);
              // Use the same coordinate system as lanes
              double local_x = node.x - map_origin_offset_x_;
              double local_y = node.y - map_origin_offset_y_;
              road_marking.geometry.emplace_back(local_x, local_y);
            }
          }
        }
      }
      
      if (!road_marking.geometry.empty()) {
        road_markings_.push_back(road_marking);
      }
    }
  }
  
  // Also parse ways with road marking subtypes
  for (const auto& way : osm_ways_) {
    const auto& tags = way.second.tags;
    
    if (tags.find("subtype") != tags.end() && 
        (tags.at("subtype") == "solid" || tags.at("subtype") == "dashed")) {
      
      RoadMarking road_marking;
      road_marking.id = way.first;
      road_marking.marking_type = tags.at("subtype");
      road_marking.color = "white";
      road_marking.width = 0.15;
      
      for (long node_ref : way.second.node_refs) {
        if (osm_nodes_.count(node_ref)) {
          const auto& node = osm_nodes_.at(node_ref);
          // Use the same coordinate system as lanes
          double local_x = node.x - map_origin_offset_x_;
          double local_y = node.y - map_origin_offset_y_;
          road_marking.geometry.emplace_back(local_x, local_y);
        }
      }
      
      if (!road_marking.geometry.empty()) {
        road_markings_.push_back(road_marking);
      }
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Parsed %zu road markings", road_markings_.size());
}

void HDMapManager::parse_regulatory_elements()
{
  regulatory_elements_.clear();
  
  for (const auto& relation : osm_relations_) {
    const auto& tags = relation.second.tags;
    
    // Look for regulatory element relations
    if (tags.find("type") != tags.end() && tags.at("type") == "regulatory_element") {
      
      RegulatoryElement regulatory_element;
      regulatory_element.id = relation.first;
      
      if (tags.count("subtype")) {
        regulatory_element.subtype = tags.at("subtype");
      }
      
      // Extract member lane IDs based on roles
      for (const auto& member : relation.second.members) {
        if (member.type == "relation") {
          if (member.role == "yield") {
            regulatory_element.yield_lane_ids.push_back(member.ref);
          } else if (member.role == "right_of_way") {
            regulatory_element.right_of_way_lane_ids.push_back(member.ref);
          }
        }
      }
      
      regulatory_elements_.push_back(regulatory_element);
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Parsed %zu regulatory elements", regulatory_elements_.size());
}

}  // namespace awsim_path_planner
