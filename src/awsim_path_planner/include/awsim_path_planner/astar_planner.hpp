#ifndef AWSIM_PATH_PLANNER__ASTAR_PLANNER_HPP_
#define AWSIM_PATH_PLANNER__ASTAR_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <memory>

namespace awsim_path_planner
{

struct GridNode
{
  int x, y;
  double g_cost;  // Cost from start
  double h_cost;  // Heuristic cost to goal
  double f_cost;  // Total cost (g + h)
  GridNode* parent;
  
  GridNode() : x(0), y(0), g_cost(0.0), h_cost(0.0), f_cost(0.0), parent(nullptr) {}
  
  GridNode(int x_coord, int y_coord) 
    : x(x_coord), y(y_coord), g_cost(0.0), h_cost(0.0), f_cost(0.0), parent(nullptr) {}
  
  bool operator>(const GridNode& other) const {
    return f_cost > other.f_cost;
  }
  
  bool operator==(const GridNode& other) const {
    return x == other.x && y == other.y;
  }
};

struct GridNodeHash
{
  std::size_t operator()(const GridNode& node) const {
    return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
  }
};

struct GridNodeEqual
{
  bool operator()(const GridNode& lhs, const GridNode& rhs) const {
    return lhs.x == rhs.x && lhs.y == rhs.y;
  }
};

class AStarPlanner
{
public:
  explicit AStarPlanner(rclcpp::Node* node);
  ~AStarPlanner() = default;

  // Main planning interface
  std::vector<geometry_msgs::msg::PoseStamped> plan_path(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const sensor_msgs::msg::PointCloud2::SharedPtr& obstacles = nullptr);
  
  // Configuration
  void set_grid_resolution(double resolution) { grid_resolution_ = resolution; }
  void set_heuristic_weight(double weight) { heuristic_weight_ = weight; }
  void set_search_radius(double radius) { search_radius_ = radius; }
  void set_obstacle_inflation_radius(double radius) { obstacle_inflation_radius_ = radius; }
  
  // Visualization
  visualization_msgs::msg::MarkerArray get_search_visualization() const;
  nav_msgs::msg::OccupancyGrid get_occupancy_grid() const;

private:
  // Core A* algorithm
  std::vector<GridNode> astar_search(const GridNode& start_node, const GridNode& goal_node);
  
  // Grid and coordinate utilities
  void initialize_grid(const geometry_msgs::msg::PoseStamped& start, 
                       const geometry_msgs::msg::PoseStamped& goal);
  void update_obstacles_from_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
  GridNode world_to_grid(double world_x, double world_y) const;
  std::pair<double, double> grid_to_world(const GridNode& node) const;
  
  // Cost and heuristic functions
  double calculate_heuristic(const GridNode& current, const GridNode& goal) const;
  double calculate_movement_cost(const GridNode& from, const GridNode& to) const;
  bool is_valid_node(const GridNode& node) const;
  bool is_obstacle(const GridNode& node) const;
  
  // Path processing
  std::vector<geometry_msgs::msg::PoseStamped> reconstruct_path(
    const std::vector<GridNode>& grid_path) const;
  std::vector<geometry_msgs::msg::PoseStamped> smooth_path(
    const std::vector<geometry_msgs::msg::PoseStamped>& raw_path) const;
  
  // Neighbor generation
  std::vector<GridNode> get_neighbors(const GridNode& node) const;
  
  // Grid management
  void clear_grid();
  void inflate_obstacles();
  
  // ROS components
  rclcpp::Node* node_;
  
  // Grid representation
  std::vector<std::vector<int>> occupancy_grid_;  // 0=free, 100=occupied, -1=unknown
  int grid_width_, grid_height_;
  double grid_resolution_;
  double origin_x_, origin_y_;  // World coordinates of grid origin
  
  // Algorithm parameters
  double heuristic_weight_;
  double search_radius_;
  double obstacle_inflation_radius_;
  
  // Visualization data
  mutable std::vector<GridNode> explored_nodes_;
  mutable std::vector<GridNode> frontier_nodes_;
  mutable std::vector<geometry_msgs::msg::PoseStamped> final_path_;
  
  // Constants
  static constexpr double DIAGONAL_COST = 1.414;  // sqrt(2)
  static constexpr double STRAIGHT_COST = 1.0;
  static constexpr int OBSTACLE_VALUE = 100;
  static constexpr int FREE_VALUE = 0;
  static constexpr int UNKNOWN_VALUE = -1;
  
  // Movement directions (8-connected)
  const std::vector<std::pair<int, int>> directions_ = {
    {-1, -1}, {-1, 0}, {-1, 1},
    {0, -1},           {0, 1},
    {1, -1},  {1, 0},  {1, 1}
  };
};

}  // namespace awsim_path_planner

#endif  // AWSIM_PATH_PLANNER__ASTAR_PLANNER_HPP_
