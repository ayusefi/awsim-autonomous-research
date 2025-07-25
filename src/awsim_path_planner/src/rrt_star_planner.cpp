#include "awsim_path_planner/rrt_star_planner.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>

namespace awsim_path_planner
{

RRTStarPlanner::RRTStarPlanner(rclcpp::Node* node)
: node_(node),
  root_node_(nullptr),
  max_iterations_(5000),
  step_size_(2.0),
  goal_tolerance_(2.0),
  rewiring_radius_(10.0),
  goal_bias_probability_(0.1),
  obstacle_check_resolution_(0.5),
  obstacle_radius_(1.0),
  rng_(std::random_device{}()),
  x_dist_(0.0, 1.0),
  y_dist_(0.0, 1.0),
  prob_dist_(0.0, 1.0)
{
  // Get RRT*-specific parameters
  if (node_->has_parameter("rrt_star.max_iterations")) {
    max_iterations_ = node_->get_parameter("rrt_star.max_iterations").as_int();
  }
  if (node_->has_parameter("rrt_star.step_size")) {
    step_size_ = node_->get_parameter("rrt_star.step_size").as_double();
  }
  if (node_->has_parameter("rrt_star.goal_tolerance")) {
    goal_tolerance_ = node_->get_parameter("rrt_star.goal_tolerance").as_double();
  }
  if (node_->has_parameter("rrt_star.rewiring_radius")) {
    rewiring_radius_ = node_->get_parameter("rrt_star.rewiring_radius").as_double();
  }
  
  RCLCPP_INFO(node_->get_logger(), "RRT* Planner initialized with %d max iterations, %.2f step size",
              max_iterations_, step_size_);
}

RRTStarPlanner::~RRTStarPlanner()
{
  clear_tree();
}

std::vector<geometry_msgs::msg::PoseStamped> RRTStarPlanner::plan_path(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  const sensor_msgs::msg::PointCloud2::SharedPtr& obstacles)
{
  // Clear previous planning data
  clear_tree();
  tree_edges_.clear();
  final_path_.clear();
  
  // Set planning bounds
  double margin = 50.0;  // 50m margin around start and goal
  min_x_ = std::min(start.pose.position.x, goal.pose.position.x) - margin;
  max_x_ = std::max(start.pose.position.x, goal.pose.position.x) + margin;
  min_y_ = std::min(start.pose.position.y, goal.pose.position.y) - margin;
  max_y_ = std::max(start.pose.position.y, goal.pose.position.y) + margin;
  
  // Update random distributions
  x_dist_ = std::uniform_real_distribution<double>(min_x_, max_x_);
  y_dist_ = std::uniform_real_distribution<double>(min_y_, max_y_);
  
  // Update obstacles from pointcloud if available
  if (obstacles) {
    update_obstacles_from_pointcloud(obstacles);
  }
  
  // Check if start and goal are collision-free
  if (!is_point_collision_free(start.pose.position.x, start.pose.position.y)) {
    RCLCPP_ERROR(node_->get_logger(), "Start position is in collision");
    return {};
  }
  
  if (!is_point_collision_free(goal.pose.position.x, goal.pose.position.y)) {
    RCLCPP_ERROR(node_->get_logger(), "Goal position is in collision");
    return {};
  }
  
  // Create start and goal nodes
  RRTNode start_node(start.pose.position.x, start.pose.position.y);
  RRTNode goal_node(goal.pose.position.x, goal.pose.position.y);
  
  // Perform RRT* search
  RRTNode* goal_reached = rrt_star_search(start_node, goal_node);
  
  if (!goal_reached) {
    RCLCPP_WARN(node_->get_logger(), "RRT* search failed to find a path");
    return {};
  }
  
  // Reconstruct path from goal to start
  std::vector<geometry_msgs::msg::PoseStamped> world_path = reconstruct_path(goal_reached);
  
  // Smooth the path
  world_path = smooth_path(world_path);
  
  // Store for visualization
  final_path_ = world_path;
  
  return world_path;
}

RRTNode* RRTStarPlanner::rrt_star_search(const RRTNode& start_node, const RRTNode& goal_node)
{
  // Initialize tree with start node
  root_node_ = add_node(start_node.x, start_node.y, 0.0);
  
  RRTNode* best_goal_node = nullptr;
  double best_goal_cost = std::numeric_limits<double>::infinity();
  
  for (int iteration = 0; iteration < max_iterations_; ++iteration) {
    // Sample random point (with goal biasing)
    auto [sample_x, sample_y] = (prob_dist_(rng_) < goal_bias_probability_) ?
      sample_goal_biased_point(goal_node) : sample_random_point();
    
    // Find nearest node in tree
    RRTNode* nearest = find_nearest_node(sample_x, sample_y);
    if (!nearest) continue;
    
    // Steer towards sample
    RRTNode* new_node_candidate = steer(*nearest, RRTNode(sample_x, sample_y));
    if (!new_node_candidate) continue;
    
    // Check if path is collision-free
    if (!is_path_collision_free(*nearest, *new_node_candidate)) {
      delete new_node_candidate;
      continue;
    }
    
    // Find nearby nodes for rewiring
    std::vector<RRTNode*> nearby_nodes = find_nodes_in_radius(
      new_node_candidate->x, new_node_candidate->y, rewiring_radius_);
    
    // Choose parent with minimum cost
    RRTNode* best_parent = nearest;
    double min_cost = nearest->cost + nearest->distance_to(*new_node_candidate);
    
    for (RRTNode* nearby : nearby_nodes) {
      double cost_through_nearby = nearby->cost + nearby->distance_to(*new_node_candidate);
      if (cost_through_nearby < min_cost &&
          is_path_collision_free(*nearby, *new_node_candidate)) {
        best_parent = nearby;
        min_cost = cost_through_nearby;
      }
    }
    
    // Add new node to tree
    RRTNode* new_node = add_node(new_node_candidate->x, new_node_candidate->y, 
                                min_cost, best_parent);
    delete new_node_candidate;  // Clean up temporary node
    
    // Store edge for visualization
    tree_edges_.push_back({best_parent, new_node});
    
    // Rewire nearby nodes
    rewire_tree(new_node, nearby_nodes);
    
    // Check if we reached the goal
    if (is_goal_reached(*new_node, goal_node)) {
      double goal_cost = new_node->cost + new_node->distance_to(goal_node);
      
      if (goal_cost < best_goal_cost) {
        // Create goal node connected to new_node
        if (best_goal_node) {
          // Remove previous goal node from tree
          auto it = std::find_if(tree_nodes_.begin(), tree_nodes_.end(),
            [best_goal_node](const std::unique_ptr<RRTNode>& node) {
              return node.get() == best_goal_node;
            });
          if (it != tree_nodes_.end()) {
            tree_nodes_.erase(it);
          }
        }
        
        best_goal_node = add_node(goal_node.x, goal_node.y, goal_cost, new_node);
        best_goal_cost = goal_cost;
        
        tree_edges_.push_back({new_node, best_goal_node});
        
        RCLCPP_INFO(node_->get_logger(), 
          "RRT* found path to goal in %d iterations with cost %.2f", 
          iteration + 1, best_goal_cost);
      }
    }
    
    // Early termination if we have a good solution
    if (best_goal_node && iteration > max_iterations_ * 0.8) {
      break;
    }
  }
  
  return best_goal_node;
}

RRTNode* RRTStarPlanner::add_node(double x, double y, double cost, RRTNode* parent)
{
  auto node = std::make_unique<RRTNode>(x, y, cost, parent);
  RRTNode* node_ptr = node.get();
  
  if (parent) {
    parent->children.push_back(node_ptr);
  }
  
  tree_nodes_.push_back(std::move(node));
  return node_ptr;
}

RRTNode* RRTStarPlanner::find_nearest_node(double x, double y) const
{
  if (tree_nodes_.empty()) return nullptr;
  
  RRTNode* nearest = tree_nodes_[0].get();
  double min_distance = nearest->distance_to(RRTNode(x, y));
  
  for (const auto& node : tree_nodes_) {
    double distance = node->distance_to(RRTNode(x, y));
    if (distance < min_distance) {
      min_distance = distance;
      nearest = node.get();
    }
  }
  
  return nearest;
}

std::vector<RRTNode*> RRTStarPlanner::find_nodes_in_radius(double x, double y, double radius) const
{
  std::vector<RRTNode*> nearby_nodes;
  RRTNode query_point(x, y);
  
  for (const auto& node : tree_nodes_) {
    if (node->distance_to(query_point) <= radius) {
      nearby_nodes.push_back(node.get());
    }
  }
  
  return nearby_nodes;
}

RRTNode* RRTStarPlanner::steer(const RRTNode& from, const RRTNode& to) const
{
  double distance = from.distance_to(to);
  
  if (distance <= step_size_) {
    return new RRTNode(to.x, to.y);
  }
  
  // Scale to step_size_
  double ratio = step_size_ / distance;
  double new_x = from.x + ratio * (to.x - from.x);
  double new_y = from.y + ratio * (to.y - from.y);
  
  return new RRTNode(new_x, new_y);
}

bool RRTStarPlanner::is_path_collision_free(const RRTNode& from, const RRTNode& to) const
{
  double distance = from.distance_to(to);
  int steps = static_cast<int>(distance / obstacle_check_resolution_) + 1;
  
  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    double check_x = from.x + t * (to.x - from.x);
    double check_y = from.y + t * (to.y - from.y);
    
    if (!is_point_collision_free(check_x, check_y)) {
      return false;
    }
  }
  
  return true;
}

bool RRTStarPlanner::is_point_collision_free(double x, double y) const
{
  // Check against obstacles
  for (const auto& obstacle : obstacles_) {
    double distance = std::sqrt((x - obstacle.first) * (x - obstacle.first) +
                               (y - obstacle.second) * (y - obstacle.second));
    if (distance < obstacle_radius_) {
      return false;
    }
  }
  
  // Check bounds
  if (x < min_x_ || x > max_x_ || y < min_y_ || y > max_y_) {
    return false;
  }
  
  return true;
}

void RRTStarPlanner::rewire_tree(RRTNode* new_node, const std::vector<RRTNode*>& nearby_nodes)
{
  for (RRTNode* nearby : nearby_nodes) {
    if (nearby == new_node || nearby == new_node->parent) continue;
    
    double cost_through_new = new_node->cost + new_node->distance_to(*nearby);
    
    if (cost_through_new < nearby->cost &&
        is_path_collision_free(*new_node, *nearby)) {
      
      // Remove nearby from its current parent's children
      if (nearby->parent) {
        auto& siblings = nearby->parent->children;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), nearby), siblings.end());
      }
      
      // Update nearby's parent and cost
      nearby->parent = new_node;
      nearby->cost = cost_through_new;
      new_node->children.push_back(nearby);
      
      // Update costs of all descendants
      std::function<void(RRTNode*)> update_descendants = [&](RRTNode* node) {
        for (RRTNode* child : node->children) {
          child->cost = node->cost + node->distance_to(*child);
          update_descendants(child);
        }
      };
      update_descendants(nearby);
      
      // Update visualization edges
      for (auto& edge : tree_edges_) {
        if (edge.second == nearby) {
          edge.first = new_node;
        }
      }
    }
  }
}

double RRTStarPlanner::calculate_path_cost(const RRTNode* node) const
{
  double cost = 0.0;
  while (node->parent) {
    cost += node->parent->distance_to(*node);
    node = node->parent;
  }
  return cost;
}

std::pair<double, double> RRTStarPlanner::sample_random_point()
{
  return {x_dist_(rng_), y_dist_(rng_)};
}

std::pair<double, double> RRTStarPlanner::sample_goal_biased_point(const RRTNode& goal)
{
  return {goal.x, goal.y};
}

bool RRTStarPlanner::is_goal_reached(const RRTNode& node, const RRTNode& goal) const
{
  return node.distance_to(goal) <= goal_tolerance_;
}

std::vector<geometry_msgs::msg::PoseStamped> RRTStarPlanner::reconstruct_path(const RRTNode* goal_node) const
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  
  const RRTNode* current = goal_node;
  while (current) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = node_->get_clock()->now();
    pose.pose.position.x = current->x;
    pose.pose.position.y = current->y;
    pose.pose.position.z = 0.0;
    
    // Set orientation based on direction to next waypoint
    if (current->parent) {
      double yaw = std::atan2(current->y - current->parent->y, 
                             current->x - current->parent->x);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      pose.pose.orientation.w = 1.0;
    }
    
    path.push_back(pose);
    current = current->parent;
  }
  
  // Reverse to get path from start to goal
  std::reverse(path.begin(), path.end());
  
  // Fix orientations to point forward
  for (size_t i = 0; i < path.size() - 1; ++i) {
    double dx = path[i + 1].pose.position.x - path[i].pose.position.x;
    double dy = path[i + 1].pose.position.y - path[i].pose.position.y;
    double yaw = std::atan2(dy, dx);
    
    path[i].pose.orientation.x = 0.0;
    path[i].pose.orientation.y = 0.0;
    path[i].pose.orientation.z = std::sin(yaw / 2.0);
    path[i].pose.orientation.w = std::cos(yaw / 2.0);
  }
  
  return path;
}

std::vector<geometry_msgs::msg::PoseStamped> RRTStarPlanner::smooth_path(
  const std::vector<geometry_msgs::msg::PoseStamped>& raw_path) const
{
  if (raw_path.size() <= 2) {
    return raw_path;
  }
  
  std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
  smoothed_path.push_back(raw_path.front());
  
  size_t i = 0;
  while (i < raw_path.size() - 1) {
    size_t farthest = i + 1;
    
    // Find farthest point we can reach directly
    for (size_t j = i + 2; j < raw_path.size(); ++j) {
      RRTNode from(raw_path[i].pose.position.x, raw_path[i].pose.position.y);
      RRTNode to(raw_path[j].pose.position.x, raw_path[j].pose.position.y);
      
      if (is_path_collision_free(from, to)) {
        farthest = j;
      } else {
        break;
      }
    }
    
    if (farthest != raw_path.size() - 1) {
      smoothed_path.push_back(raw_path[farthest]);
    }
    i = farthest;
  }
  
  smoothed_path.push_back(raw_path.back());
  
  RCLCPP_INFO(node_->get_logger(), "RRT* path smoothed from %zu to %zu waypoints",
              raw_path.size(), smoothed_path.size());
  
  return smoothed_path;
}

void RRTStarPlanner::update_obstacles_from_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud)
{
  obstacles_.clear();
  
  // Convert ROS PointCloud2 to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  
  // Downsample obstacles for efficiency
  for (size_t i = 0; i < pcl_cloud->points.size(); i += 10) {  // Every 10th point
    const auto& point = pcl_cloud->points[i];
    
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      obstacles_.emplace_back(point.x, point.y);
    }
  }
  
  RCLCPP_INFO(node_->get_logger(), "Updated RRT* obstacles: %zu points", obstacles_.size());
}

void RRTStarPlanner::clear_tree()
{
  tree_nodes_.clear();
  root_node_ = nullptr;
}

visualization_msgs::msg::MarkerArray RRTStarPlanner::get_tree_visualization() const
{
  visualization_msgs::msg::MarkerArray markers;
  
  // Clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear_marker);
  
  // Tree edges visualization
  if (!tree_edges_.empty()) {
    visualization_msgs::msg::Marker tree_marker;
    tree_marker.header.frame_id = "map";
    tree_marker.header.stamp = node_->get_clock()->now();
    tree_marker.ns = "rrt_star_tree";
    tree_marker.id = 0;
    tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree_marker.action = visualization_msgs::msg::Marker::ADD;
    tree_marker.scale.x = 0.05;  // Line width
    tree_marker.color.r = 0.5;
    tree_marker.color.g = 0.5;
    tree_marker.color.b = 1.0;
    tree_marker.color.a = 0.6;
    
    for (const auto& edge : tree_edges_) {
      geometry_msgs::msg::Point start_point;
      start_point.x = edge.first->x;
      start_point.y = edge.first->y;
      start_point.z = 0.05;
      
      geometry_msgs::msg::Point end_point;
      end_point.x = edge.second->x;
      end_point.y = edge.second->y;
      end_point.z = 0.05;
      
      tree_marker.points.push_back(start_point);
      tree_marker.points.push_back(end_point);
    }
    
    markers.markers.push_back(tree_marker);
  }
  
  // Final path visualization
  if (!final_path_.empty()) {
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = node_->get_clock()->now();
    path_marker.ns = "rrt_star_path";
    path_marker.id = 1;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.3;  // Line width
    path_marker.color.r = 0.0;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;
    
    for (const auto& pose : final_path_) {
      geometry_msgs::msg::Point point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.z = 0.3;
      path_marker.points.push_back(point);
    }
    
    markers.markers.push_back(path_marker);
  }
  
  return markers;
}

}  // namespace awsim_path_planner
