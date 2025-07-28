#include "awsim_path_planner/astar_planner.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>

namespace awsim_path_planner
{

AStarPlanner::AStarPlanner(rclcpp::Node* node)
: node_(node),
  grid_resolution_(0.5),
  heuristic_weight_(1.0),
  search_radius_(100.0),
  obstacle_inflation_radius_(1.0)
{
  // Get A*-specific parameters
  if (node_->has_parameter("astar.heuristic_weight")) {
    heuristic_weight_ = node_->get_parameter("astar.heuristic_weight").as_double();
  }
  if (node_->has_parameter("astar.search_radius")) {
    search_radius_ = node_->get_parameter("astar.search_radius").as_double();
  }
  if (node_->has_parameter("astar.obstacle_inflation_radius")) {
    obstacle_inflation_radius_ = node_->get_parameter("astar.obstacle_inflation_radius").as_double();
  }
  
  RCLCPP_INFO(node_->get_logger(), "A* Planner initialized with heuristic weight: %.2f", heuristic_weight_);
}

std::vector<geometry_msgs::msg::PoseStamped> AStarPlanner::plan_path(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  const sensor_msgs::msg::PointCloud2::SharedPtr& obstacles)
{
  // Clear previous planning data
  clear_grid();
  explored_nodes_.clear();
  frontier_nodes_.clear();
  final_path_.clear();
  
  // Initialize vehicle-centered grid instead of start/goal centered
  initialize_vehicle_centered_grid(start);
  
  // Update obstacles from pointcloud if available
  if (obstacles) {
    update_obstacles_from_pointcloud(obstacles);
    inflate_obstacles();
    
    // Clear areas around start and goal positions to ensure they're not blocked
    double clear_radius = std::max(2.0, obstacle_inflation_radius_ * 1.5);
    clear_area_around_position(start.pose.position.x, start.pose.position.y, clear_radius);
    clear_area_around_position(goal.pose.position.x, goal.pose.position.y, clear_radius);
  }
  
  // Convert start and goal to grid coordinates
  GridNode start_node = world_to_grid(start.pose.position.x, start.pose.position.y);
  GridNode goal_node = world_to_grid(goal.pose.position.x, goal.pose.position.y);
  
  // Check if start and goal are valid
  if (!is_valid_node(start_node) || !is_valid_node(goal_node)) {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal position is invalid");
    return {};
  }
  
  if (is_obstacle(start_node) || is_obstacle(goal_node)) {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal position is occupied by obstacle");
    return {};
  }
  
  // Perform A* search
  std::vector<GridNode> grid_path = astar_search(start_node, goal_node);
  
  if (grid_path.empty()) {
    RCLCPP_WARN(node_->get_logger(), "A* search failed to find a path");
    return {};
  }
  
  // Convert grid path to world coordinates
  std::vector<geometry_msgs::msg::PoseStamped> world_path = reconstruct_path(grid_path);
  
  // Smooth the path
  world_path = smooth_path(world_path);
  
  // Store for visualization
  final_path_ = world_path;
  
  return world_path;
}

std::vector<GridNode> AStarPlanner::astar_search(const GridNode& start_node, const GridNode& goal_node)
{
  // Priority queue for open set (nodes to be evaluated)
  std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open_set;
  
  // Hash sets for efficient membership checking
  std::unordered_set<GridNode, GridNodeHash, GridNodeEqual> open_set_hash;
  std::unordered_set<GridNode, GridNodeHash, GridNodeEqual> closed_set;
  
  // Map to store the best path to each node
  std::unordered_map<GridNode, GridNode, GridNodeHash, GridNodeEqual> came_from;
  
  // Cost maps
  std::unordered_map<GridNode, double, GridNodeHash, GridNodeEqual> g_score;
  std::unordered_map<GridNode, double, GridNodeHash, GridNodeEqual> f_score;
  
  // Initialize start node
  GridNode start = start_node;
  start.g_cost = 0;
  start.h_cost = calculate_heuristic(start, goal_node);
  start.f_cost = start.g_cost + start.h_cost;
  
  open_set.push(start);
  open_set_hash.insert(start);
  g_score[start] = 0;
  f_score[start] = start.f_cost;
  
  int iterations = 0;
  const int max_iterations = 10000;
  
  while (!open_set.empty() && iterations < max_iterations) {
    iterations++;
    
    // Get node with lowest f_cost
    GridNode current = open_set.top();
    open_set.pop();
    open_set_hash.erase(current);
    
    // Add to explored nodes for visualization
    explored_nodes_.push_back(current);
    
    // Check if we reached the goal
    if (current.x == goal_node.x && current.y == goal_node.y) {
      RCLCPP_INFO(node_->get_logger(), "A* found path in %d iterations", iterations);
      
      // Reconstruct path
      std::vector<GridNode> path;
      GridNode path_node = current;
      
      while (came_from.find(path_node) != came_from.end()) {
        path.push_back(path_node);
        path_node = came_from[path_node];
      }
      path.push_back(start_node);
      
      std::reverse(path.begin(), path.end());
      return path;
    }
    
    closed_set.insert(current);
    
    // Explore neighbors
    std::vector<GridNode> neighbors = get_neighbors(current);
    
    for (GridNode& neighbor : neighbors) {
      // Skip if already evaluated or invalid
      if (closed_set.find(neighbor) != closed_set.end() || 
          !is_valid_node(neighbor) || is_obstacle(neighbor)) {
        continue;
      }
      
      // Calculate tentative g_score
      double tentative_g_score = g_score[current] + calculate_movement_cost(current, neighbor);
      
      bool is_better_path = false;
      
      // Check if this path to neighbor is better
      if (open_set_hash.find(neighbor) == open_set_hash.end()) {
        // Not in open set, so this is the first time we've seen it
        is_better_path = true;
      } else if (tentative_g_score < g_score[neighbor]) {
        // We've seen this node before, but this path is better
        is_better_path = true;
      }
      
      if (is_better_path) {
        // Record the best path
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;
        
        neighbor.g_cost = tentative_g_score;
        neighbor.h_cost = calculate_heuristic(neighbor, goal_node);
        neighbor.f_cost = neighbor.g_cost + neighbor.h_cost;
        f_score[neighbor] = neighbor.f_cost;
        
        if (open_set_hash.find(neighbor) == open_set_hash.end()) {
          open_set.push(neighbor);
          open_set_hash.insert(neighbor);
          frontier_nodes_.push_back(neighbor);  // For visualization
        }
      }
    }
  }
  
  RCLCPP_WARN(node_->get_logger(), "A* search exhausted after %d iterations", iterations);
  return {};  // No path found
}

void AStarPlanner::initialize_grid(const geometry_msgs::msg::PoseStamped& start, 
                                   const geometry_msgs::msg::PoseStamped& goal)
{
  // Calculate grid bounds based on start, goal, and search radius
  double min_x = std::min(start.pose.position.x, goal.pose.position.x) - search_radius_;
  double max_x = std::max(start.pose.position.x, goal.pose.position.x) + search_radius_;
  double min_y = std::min(start.pose.position.y, goal.pose.position.y) - search_radius_;
  double max_y = std::max(start.pose.position.y, goal.pose.position.y) + search_radius_;
  
  // Set grid origin (bottom-left corner)
  origin_x_ = min_x;
  origin_y_ = min_y;
  
  // Calculate grid dimensions
  grid_width_ = static_cast<int>((max_x - min_x) / grid_resolution_) + 1;
  grid_height_ = static_cast<int>((max_y - min_y) / grid_resolution_) + 1;
  
  // Initialize occupancy grid
  occupancy_grid_.assign(grid_height_, std::vector<int>(grid_width_, FREE_VALUE));
  
  RCLCPP_INFO(node_->get_logger(), "Initialized %dx%d grid with resolution %.2f m",
              grid_width_, grid_height_, grid_resolution_);
}

void AStarPlanner::initialize_vehicle_centered_grid(const geometry_msgs::msg::PoseStamped& vehicle_pose)
{
  // Create a square grid centered on the vehicle position
  double half_grid_size = search_radius_;
  
  // Set grid origin (bottom-left corner of vehicle-centered grid)
  origin_x_ = vehicle_pose.pose.position.x - half_grid_size;
  origin_y_ = vehicle_pose.pose.position.y - half_grid_size;
  
  // Calculate grid dimensions (square grid)
  grid_width_ = static_cast<int>((2.0 * half_grid_size) / grid_resolution_) + 1;
  grid_height_ = static_cast<int>((2.0 * half_grid_size) / grid_resolution_) + 1;
  
  // Initialize occupancy grid with unknown values (will be updated with sensor data)
  occupancy_grid_.assign(grid_height_, std::vector<int>(grid_width_, FREE_VALUE));
  
  RCLCPP_DEBUG(node_->get_logger(), 
    "Initialized vehicle-centered %dx%d grid at (%.2f, %.2f) with resolution %.2f m",
    grid_width_, grid_height_, origin_x_, origin_y_, grid_resolution_);
}

void AStarPlanner::update_obstacles_from_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud)
{
  // Convert ROS PointCloud2 to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  
  int obstacle_count = 0;
  
  // Process each point in the cloud
  for (const auto& point : pcl_cloud->points) {
    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    
    // Convert to grid coordinates
    GridNode grid_point = world_to_grid(point.x, point.y);
    
    // Mark as obstacle if within grid bounds
    if (is_valid_node(grid_point)) {
      occupancy_grid_[grid_point.y][grid_point.x] = OBSTACLE_VALUE;
      obstacle_count++;
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Added %d obstacle points from pointcloud", obstacle_count);
}

void AStarPlanner::update_grid_for_visualization(const geometry_msgs::msg::PoseStamped& vehicle_pose,
                                                 const sensor_msgs::msg::PointCloud2::SharedPtr& cloud)
{
  // Initialize vehicle-centered grid
  initialize_vehicle_centered_grid(vehicle_pose);
  
  // Update obstacles from point cloud if available
  if (cloud) {
    update_obstacles_from_pointcloud(cloud);
    inflate_obstacles();
  }
}

GridNode AStarPlanner::world_to_grid(double world_x, double world_y) const
{
  int grid_x = static_cast<int>((world_x - origin_x_) / grid_resolution_);
  int grid_y = static_cast<int>((world_y - origin_y_) / grid_resolution_);
  return GridNode(grid_x, grid_y);
}

std::pair<double, double> AStarPlanner::grid_to_world(const GridNode& node) const
{
  double world_x = origin_x_ + node.x * grid_resolution_;
  double world_y = origin_y_ + node.y * grid_resolution_;
  return {world_x, world_y};
}

double AStarPlanner::calculate_heuristic(const GridNode& current, const GridNode& goal) const
{
  // Euclidean distance heuristic
  double dx = goal.x - current.x;
  double dy = goal.y - current.y;
  return heuristic_weight_ * std::sqrt(dx * dx + dy * dy) * grid_resolution_;
}

double AStarPlanner::calculate_movement_cost(const GridNode& from, const GridNode& to) const
{
  double dx = std::abs(to.x - from.x);
  double dy = std::abs(to.y - from.y);
  
  // Diagonal movement costs more
  if (dx == 1 && dy == 1) {
    return DIAGONAL_COST * grid_resolution_;
  } else {
    return STRAIGHT_COST * grid_resolution_;
  }
}

bool AStarPlanner::is_valid_node(const GridNode& node) const
{
  return node.x >= 0 && node.x < grid_width_ && node.y >= 0 && node.y < grid_height_;
}

bool AStarPlanner::is_obstacle(const GridNode& node) const
{
  if (!is_valid_node(node)) {
    return true;  // Out of bounds is considered an obstacle
  }
  return occupancy_grid_[node.y][node.x] == OBSTACLE_VALUE;
}

std::vector<GridNode> AStarPlanner::get_neighbors(const GridNode& node) const
{
  std::vector<GridNode> neighbors;
  
  for (const auto& dir : directions_) {
    GridNode neighbor(node.x + dir.first, node.y + dir.second);
    if (is_valid_node(neighbor)) {
      neighbors.push_back(neighbor);
    }
  }
  
  return neighbors;
}

std::vector<geometry_msgs::msg::PoseStamped> AStarPlanner::reconstruct_path(
  const std::vector<GridNode>& grid_path) const
{
  std::vector<geometry_msgs::msg::PoseStamped> world_path;
  
  for (const auto& grid_node : grid_path) {
    auto [world_x, world_y] = grid_to_world(grid_node);
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node_->get_clock()->now();
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    
    // Set orientation based on direction to next waypoint
    if (&grid_node != &grid_path.back()) {
      auto next_it = std::next(std::find(grid_path.begin(), grid_path.end(), grid_node));
      auto [next_x, next_y] = grid_to_world(*next_it);
      
      double yaw = std::atan2(next_y - world_y, next_x - world_x);
      
      // Convert yaw to quaternion
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      // Last point keeps previous orientation
      if (!world_path.empty()) {
        pose.pose.orientation = world_path.back().pose.orientation;
      } else {
        pose.pose.orientation.w = 1.0;
      }
    }
    
    world_path.push_back(pose);
  }
  
  return world_path;
}

std::vector<geometry_msgs::msg::PoseStamped> AStarPlanner::smooth_path(
  const std::vector<geometry_msgs::msg::PoseStamped>& raw_path) const
{
  if (raw_path.size() <= 2) {
    return raw_path;
  }
  
  std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
  smoothed_path.push_back(raw_path.front());
  
  // Simple path smoothing: remove unnecessary waypoints
  for (size_t i = 1; i < raw_path.size() - 1; ++i) {
    const auto& prev = smoothed_path.back();
    const auto& curr = raw_path[i];
    const auto& next = raw_path[i + 1];
    
    // Check if we can skip current waypoint by going directly from prev to next
    bool can_skip = true;
    
    // Simple line-of-sight check (could be improved with proper collision checking)
    double dx = next.pose.position.x - prev.pose.position.x;
    double dy = next.pose.position.y - prev.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    int steps = static_cast<int>(distance / (grid_resolution_ * 0.5));
    
    for (int step = 1; step < steps; ++step) {
      double t = static_cast<double>(step) / steps;
      double check_x = prev.pose.position.x + t * dx;
      double check_y = prev.pose.position.y + t * dy;
      
      GridNode check_node = world_to_grid(check_x, check_y);
      if (is_obstacle(check_node)) {
        can_skip = false;
        break;
      }
    }
    
    if (!can_skip) {
      smoothed_path.push_back(curr);
    }
  }
  
  smoothed_path.push_back(raw_path.back());
  
  RCLCPP_INFO(node_->get_logger(), "Path smoothed from %zu to %zu waypoints",
              raw_path.size(), smoothed_path.size());
  
  return smoothed_path;
}

void AStarPlanner::clear_grid()
{
  occupancy_grid_.clear();
  grid_width_ = 0;
  grid_height_ = 0;
}

void AStarPlanner::inflate_obstacles()
{
  if (obstacle_inflation_radius_ <= 0) {
    return;
  }
  
  int inflation_cells = static_cast<int>(obstacle_inflation_radius_ / grid_resolution_);
  std::vector<std::vector<int>> inflated_grid = occupancy_grid_;
  
  for (int y = 0; y < grid_height_; ++y) {
    for (int x = 0; x < grid_width_; ++x) {
      if (occupancy_grid_[y][x] == OBSTACLE_VALUE) {
        // Inflate around this obstacle
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int new_x = x + dx;
            int new_y = y + dy;
            
            if (new_x >= 0 && new_x < grid_width_ && new_y >= 0 && new_y < grid_height_) {
              double distance = std::sqrt(dx * dx + dy * dy) * grid_resolution_;
              if (distance <= obstacle_inflation_radius_) {
                inflated_grid[new_y][new_x] = OBSTACLE_VALUE;
              }
            }
          }
        }
      }
    }
  }
  
  occupancy_grid_ = inflated_grid;
}

void AStarPlanner::clear_area_around_position(double world_x, double world_y, double radius)
{
  GridNode center = world_to_grid(world_x, world_y);
  int clear_cells = static_cast<int>(radius / grid_resolution_) + 1;
  
  for (int dy = -clear_cells; dy <= clear_cells; ++dy) {
    for (int dx = -clear_cells; dx <= clear_cells; ++dx) {
      int new_x = center.x + dx;
      int new_y = center.y + dy;
      
      if (is_valid_node({new_x, new_y})) {
        double distance = std::sqrt(dx * dx + dy * dy) * grid_resolution_;
        if (distance <= radius) {
          occupancy_grid_[new_y][new_x] = FREE_VALUE;
        }
      }
    }
  }
}

visualization_msgs::msg::MarkerArray AStarPlanner::get_search_visualization() const
{
  visualization_msgs::msg::MarkerArray markers;
  
  // Clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear_marker);
  
  // Explored nodes visualization
  if (!explored_nodes_.empty()) {
    visualization_msgs::msg::Marker explored_marker;
    explored_marker.header.frame_id = "map";
    explored_marker.header.stamp = node_->get_clock()->now();
    explored_marker.ns = "astar_explored";
    explored_marker.id = 0;
    explored_marker.type = visualization_msgs::msg::Marker::POINTS;
    explored_marker.action = visualization_msgs::msg::Marker::ADD;
    explored_marker.scale.x = grid_resolution_ * 0.8;
    explored_marker.scale.y = grid_resolution_ * 0.8;
    explored_marker.color.r = 1.0;
    explored_marker.color.g = 0.0;
    explored_marker.color.b = 0.0;
    explored_marker.color.a = 0.5;
    
    for (const auto& node : explored_nodes_) {
      auto [world_x, world_y] = grid_to_world(node);
      geometry_msgs::msg::Point point;
      point.x = world_x;
      point.y = world_y;
      point.z = 0.1;
      explored_marker.points.push_back(point);
    }
    
    markers.markers.push_back(explored_marker);
  }
  
  // Final path visualization
  if (!final_path_.empty()) {
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = node_->get_clock()->now();
    path_marker.ns = "astar_path";
    path_marker.id = 1;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.2;
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;
    
    for (const auto& pose : final_path_) {
      geometry_msgs::msg::Point point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.z = 0.2;
      path_marker.points.push_back(point);
    }
    
    markers.markers.push_back(path_marker);
  }
  
  return markers;
}

nav_msgs::msg::OccupancyGrid AStarPlanner::get_occupancy_grid() const
{
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = "map";
  grid_msg.header.stamp = node_->get_clock()->now();
  
  grid_msg.info.resolution = grid_resolution_;
  grid_msg.info.width = grid_width_;
  grid_msg.info.height = grid_height_;
  grid_msg.info.origin.position.x = origin_x_;
  grid_msg.info.origin.position.y = origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;
  
  // Flatten 2D grid to 1D vector
  grid_msg.data.reserve(grid_width_ * grid_height_);
  for (int y = 0; y < grid_height_; ++y) {
    for (int x = 0; x < grid_width_; ++x) {
      grid_msg.data.push_back(occupancy_grid_[y][x]);
    }
  }
  
  return grid_msg;
}

}  // namespace awsim_path_planner
