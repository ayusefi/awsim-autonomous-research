#ifndef AWSIM_LOCALIZATION_NODE_HPP
#define AWSIM_LOCALIZATION_NODE_HPP

#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/gicp_omp.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace awsim_localization
{

class AwsimLocalizationNode : public rclcpp::Node
{
public:
  explicit AwsimLocalizationNode(const rclcpp::NodeOptions & options);
  ~AwsimLocalizationNode() = default;

private:
  // Algorithm selection
  std::string algorithm_type_;  // "ndt" or "icp"
  
  // Core localization algorithms
  std::unique_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt_;
  std::unique_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
  
  // Map coordinate transformation
  geometry_msgs::msg::Point map_origin_offset_;
  
  // Transform handling
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // State variables
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
  nav_msgs::msg::Odometry last_ground_truth_;
  bool pose_initialized_;
  bool map_loaded_;
  bool first_scan_;
  double fitness_score_;

  // Parameters
  std::string map_path_;
  std::string map_frame_;
  std::string base_link_frame_;
  std::string odom_frame_;
  std::string lidar_frame_;
  
  // Map centering parameters
  bool use_manual_map_center_;
  double manual_map_center_x_;
  double manual_map_center_y_;
  double manual_map_center_z_;
  
  // NDT parameters
  double ndt_resolution_;
  int ndt_max_iterations_;
  double ndt_transformation_epsilon_;
  double ndt_step_size_;
  int ndt_num_threads_;
  
  // ICP/GICP parameters
  int icp_max_iterations_;
  double icp_transformation_epsilon_;
  double icp_max_correspondence_distance_;
  double icp_euclidean_fitness_epsilon_;
  
  // Filtering parameters
  double voxel_leaf_size_;
  double lidar_max_range_;
  double lidar_min_range_;
  
  // Localization parameters
  double scan_matching_score_threshold_;
  bool publish_tf_;
  bool use_ground_truth_init_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_odom_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr map_publish_timer_;

  // Methods
  void declareParameters();
  void loadParameters();
  bool loadMap();
  
  // QoS profiles for AWSIM compatibility
  rclcpp::QoS createSensorQoS();
  rclcpp::QoS createGroundTruthQoS();
  rclcpp::QoS createLocalizationQoS();
  
  // Callbacks
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  
  // Core localization methods
  void performScanMatching(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan);
  void initializePose(const geometry_msgs::msg::Pose& pose);
  void updatePoseFromTransformation(const Eigen::Matrix4f& transformation, double fitness_score);
  
  // Publishing methods
  void publishTransform(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_msg);
  void publishVisualization();
  void publishMap();
  
  // Utility methods
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
};

}  // namespace awsim_localization

#endif  // AWSIM_LOCALIZATION_NODE_HPP
