#ifndef AWSIM_PATH_PLANNER__RRT_STAR_PLANNER_HPP_
#define AWSIM_PATH_PLANNER__RRT_STAR_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <memory>
#include <random>

namespace awsim_path_planner
{

struct RRTNode
{
  double x, y;
  double cost;
  RRTNode* parent;
  std::vector<RRTNode*> children;
  
  RRTNode(double x_coord, double y_coord, double node_cost = 0.0, RRTNode* node_parent = nullptr)
    : x(x_coord), y(y_coord), cost(node_cost), parent(node_parent) {}
    
  double distance_to(const RRTNode& other) const {
    return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
  }
};

class RRTStarPlanner
{
public:
  explicit RRTStarPlanner(rclcpp::Node* node);
  ~RRTStarPlanner();

  // Main planning interface
  std::vector<geometry_msgs::msg::PoseStamped> plan_path(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const sensor_msgs::msg::PointCloud2::SharedPtr& obstacles = nullptr);
  
  // Configuration
  void set_max_iterations(int iterations) { max_iterations_ = iterations; }
  void set_step_size(double step_size) { step_size_ = step_size; }
  void set_goal_tolerance(double tolerance) { goal_tolerance_ = tolerance; }
  void set_rewiring_radius(double radius) { rewiring_radius_ = radius; }
  void set_obstacle_check_resolution(double resolution) { obstacle_check_resolution_ = resolution; }
  
  // Visualization
  visualization_msgs::msg::MarkerArray get_tree_visualization() const;

private:
  // Core RRT* algorithm
  RRTNode* rrt_star_search(const RRTNode& start_node, const RRTNode& goal_node);
  
  // Tree operations
  RRTNode* add_node(double x, double y, double cost = 0.0, RRTNode* parent = nullptr);
  RRTNode* find_nearest_node(double x, double y) const;
  std::vector<RRTNode*> find_nodes_in_radius(double x, double y, double radius) const;
  RRTNode* steer(const RRTNode& from, const RRTNode& to) const;
  
  // Path validation and optimization
  bool is_path_collision_free(const RRTNode& from, const RRTNode& to) const;
  bool is_point_collision_free(double x, double y) const;
  void rewire_tree(RRTNode* new_node, const std::vector<RRTNode*>& nearby_nodes);
  double calculate_path_cost(const RRTNode* node) const;
  
  // Sampling and goal biasing
  std::pair<double, double> sample_random_point();
  std::pair<double, double> sample_goal_biased_point(const RRTNode& goal);
  bool is_goal_reached(const RRTNode& node, const RRTNode& goal) const;
  
  // Path reconstruction
  std::vector<geometry_msgs::msg::PoseStamped> reconstruct_path(const RRTNode* goal_node) const;
  std::vector<geometry_msgs::msg::PoseStamped> smooth_path(
    const std::vector<geometry_msgs::msg::PoseStamped>& raw_path) const;
  
  // Obstacle handling
  void update_obstacles_from_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
  
  // Memory management
  void clear_tree();
  
  // ROS components
  rclcpp::Node* node_;
  
  // Tree structure
  std::vector<std::unique_ptr<RRTNode>> tree_nodes_;
  RRTNode* root_node_;
  
  // Planning bounds
  double min_x_, max_x_, min_y_, max_y_;
  
  // Algorithm parameters
  int max_iterations_;
  double step_size_;
  double goal_tolerance_;
  double rewiring_radius_;
  double goal_bias_probability_;
  double obstacle_check_resolution_;
  
  // Obstacle representation
  std::vector<std::pair<double, double>> obstacles_;
  double obstacle_radius_;
  
  // Random number generation
  mutable std::mt19937 rng_;
  mutable std::uniform_real_distribution<double> x_dist_;
  mutable std::uniform_real_distribution<double> y_dist_;
  mutable std::uniform_real_distribution<double> prob_dist_;
  
  // Visualization data
  mutable std::vector<std::pair<RRTNode*, RRTNode*>> tree_edges_;
  mutable std::vector<geometry_msgs::msg::PoseStamped> final_path_;
};

}  // namespace awsim_path_planner

#endif  // AWSIM_PATH_PLANNER__RRT_STAR_PLANNER_HPP_
