#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <functional> // For std::hash

// Custom hash function for std::pair<int, int>
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1); // Combine hash values
    }
};

class GroundFilter : public rclcpp::Node {
public:
  GroundFilter();

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  // ROS communication
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_pub_;
  
  // Parameters
  double distance_threshold_;  // Maximum distance from plane (meters)
  double max_angle_deg_;      // Maximum allowed angle from horizontal (degrees)
  double max_angle_rad_;      // Maximum allowed angle from horizontal (radians)
  int min_inliers_;           // Minimum points to consider a valid plane
  int max_iterations_;        // Maximum RANSAC iterations
  double grid_size_;          // Grid cell size for ground modeling (meters)
  double height_threshold_;   // Height threshold for grid-based filtering (meters)
  double k_multiplier_;       // Multiplier for standard deviation threshold
  bool use_grid_filter_;      // Enable grid-based refinement
  double max_height_;         // Maximum height above the vehicle to consider a point (meters)
};