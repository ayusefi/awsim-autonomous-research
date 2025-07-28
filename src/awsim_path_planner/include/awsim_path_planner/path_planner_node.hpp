#ifndef AWSIM_PATH_PLANNER__PATH_PLANNER_NODE_HPP_
#define AWSIM_PATH_PLANNER__PATH_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// PCL headers for advanced ground filtering
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <memory>
#include <vector>
#include <string>

#include "awsim_path_planner/astar_planner.hpp"
#include "awsim_path_planner/rrt_star_planner.hpp"
#include "awsim_path_planner/hd_map_manager.hpp"

namespace awsim_path_planner
{

class PathPlannerNode : public rclcpp::Node
{
public:
  explicit PathPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PathPlannerNode() = default;

private:
  // Core planning methods
  void initialize_parameters();
  void setup_subscribers_and_publishers();
  bool plan_path(const geometry_msgs::msg::PoseStamped & start, 
                 const geometry_msgs::msg::PoseStamped & goal);
  
  // Callback functions
  void current_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void map_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void raw_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  // Utility functions
  void publish_path(const std::vector<geometry_msgs::msg::PoseStamped> & path);
  void publish_visualization_markers();
  void publish_occupancy_grid();
  void update_occupancy_grid();
  geometry_msgs::msg::PoseStamped transform_pose(
    const geometry_msgs::msg::PoseStamped & pose_in,
    const std::string & target_frame);
  
  // Point cloud processing functions - enhanced ground filtering
  sensor_msgs::msg::PointCloud2::SharedPtr filter_ground_points(
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud, bool is_map_data = false);
  
  // Advanced ground filtering methods
  pcl::PointIndices::Ptr ransac_ground_detection(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  pcl::PointIndices::Ptr progressive_morphological_filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  pcl::PointIndices::Ptr multi_hypothesis_ground_detection(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, bool is_map_data);
  
  sensor_msgs::msg::PointCloud2::SharedPtr combine_pointclouds(
    const sensor_msgs::msg::PointCloud2::SharedPtr & map_cloud,
    const sensor_msgs::msg::PointCloud2::SharedPtr & dynamic_cloud);

  // ROS 2 Components
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pointcloud_sub_;
  
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  
  // Timer for periodic occupancy grid publishing
  rclcpp::TimerBase::SharedPtr grid_timer_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Planning components
  std::unique_ptr<AStarPlanner> astar_planner_;
  std::unique_ptr<RRTStarPlanner> rrt_star_planner_;
  std::unique_ptr<HDMapManager> hd_map_manager_;
  
  // State variables
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  sensor_msgs::msg::PointCloud2::SharedPtr map_pointcloud_;
  sensor_msgs::msg::PointCloud2::SharedPtr raw_pointcloud_;
  sensor_msgs::msg::PointCloud2::SharedPtr combined_pointcloud_;
  bool has_current_pose_;
  bool has_goal_pose_;
  bool has_map_pointcloud_;
  bool has_raw_pointcloud_;
  
  // Parameters
  std::string planning_algorithm_;  // "astar" or "rrt_star"
  std::string map_frame_;
  std::string base_link_frame_;
  std::string hd_map_path_;
  double grid_resolution_;
  double planning_timeout_;
  double max_planning_range_;
  bool use_hd_map_constraints_;
  bool visualize_search_space_;
  
  // Ground filtering parameters - enhanced for advanced filtering
  double ground_filter_height_threshold_;
  double ground_filter_angle_threshold_;
  double dynamic_obstacle_max_range_;
  
  // RANSAC ground plane detection parameters
  double ransac_distance_threshold_;
  int ransac_max_iterations_;
  int min_ground_points_;
  double voxel_leaf_size_;
  
  // Progressive Morphological Filtering parameters
  bool use_progressive_morphological_;
  int pmf_max_window_size_;
  double pmf_slope_;
  double pmf_initial_distance_;
  double pmf_max_distance_;
};

}  // namespace awsim_path_planner

#endif  // AWSIM_PATH_PLANNER__PATH_PLANNER_NODE_HPP_
