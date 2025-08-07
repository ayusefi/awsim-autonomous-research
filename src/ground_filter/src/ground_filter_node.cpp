#include "ground_filter/ground_filter.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/common/common.h>
#include <algorithm>
#include <numeric>

GroundFilter::GroundFilter() : Node("ground_filter") {
  // Declare parameters with default values
  this->declare_parameter("distance_threshold", 0.2);
  this->declare_parameter("max_angle", 15.0);  // degrees
  this->declare_parameter("min_inliers", 1000);
  this->declare_parameter("max_iterations", 1000);
  this->declare_parameter("grid_size", 1.0);    // meters
  this->declare_parameter("height_threshold", 0.3); // meters
  this->declare_parameter("k_multiplier", 2.0);
  this->declare_parameter("use_grid_filter", true);
  this->declare_parameter("max_height", 2.5); // meters

  // Get parameter values
  distance_threshold_ = this->get_parameter("distance_threshold").as_double();
  max_angle_deg_ = this->get_parameter("max_angle").as_double();
  min_inliers_ = this->get_parameter("min_inliers").as_int();
  max_iterations_ = this->get_parameter("max_iterations").as_int();
  grid_size_ = this->get_parameter("grid_size").as_double();
  height_threshold_ = this->get_parameter("height_threshold").as_double();
  k_multiplier_ = this->get_parameter("k_multiplier").as_double();
  use_grid_filter_ = this->get_parameter("use_grid_filter").as_bool();
  max_height_ = this->get_parameter("max_height").as_double();

  // Convert angle to radians
  max_angle_rad_ = max_angle_deg_ * M_PI / 180.0;

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensing/lidar/top/pointcloud_raw", rclcpp::SensorDataQoS(),
    std::bind(&GroundFilter::pointcloud_callback, this, std::placeholders::_1));

  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 10);
  nonground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("nonground_points", 10);
}

void GroundFilter::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  
  // Check if cloud has enough points
  if (cloud->points.size() < min_inliers_) {
    RCLCPP_WARN(this->get_logger(), "Point cloud too small (%zu points), skipping processing", cloud->points.size());
    return;
  }

  // Downsample cloud for faster processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel_grid.filter(*downsampled_cloud);

  // RANSAC plane segmentation for ground extraction
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  
  // Configure segmentation
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold_);
  seg.setMaxIterations(max_iterations_);
  
  // Set axis constraint (Z-axis for ground plane)
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(max_angle_rad_);
  
  seg.setInputCloud(downsampled_cloud);
  seg.segment(*ground_inliers, *coefficients);
  
  // Check if we found a valid plane
  if (ground_inliers->indices.size() < min_inliers_) {
    RCLCPP_WARN(this->get_logger(), 
                "No valid ground plane found (only %zu inliers, need %d). Treating all points as non-ground.",
                ground_inliers->indices.size(), min_inliers_);
    
    // Publish empty ground cloud and full cloud as non-ground
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*(pcl::PointCloud<pcl::PointXYZ>::Ptr)(new pcl::PointCloud<pcl::PointXYZ>), ground_msg);
    ground_msg.header = msg->header;
    ground_pub_->publish(ground_msg);
    
    sensor_msgs::msg::PointCloud2 nonground_msg;
    pcl::toROSMsg(*cloud, nonground_msg);
    nonground_msg.header = msg->header;
    nonground_pub_->publish(nonground_msg);
    return;
  }

  // Extract ground points from downsampled cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(downsampled_cloud);
  extract.setIndices(ground_inliers);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  extract.setNegative(false);
  extract.filter(*downsampled_ground_cloud);

  // Build grid model of ground
  std::unordered_map<std::pair<int, int>, std::vector<float>, pair_hash> grid_map;
  
  for (const auto& point : downsampled_ground_cloud->points) {
    int grid_x = static_cast<int>(std::floor(point.x / grid_size_));
    int grid_y = static_cast<int>(std::floor(point.y / grid_size_));
    grid_map[{grid_x, grid_y}].push_back(point.z);
  }

  // Compute statistics for each grid cell
  std::unordered_map<std::pair<int, int>, std::pair<float, float>, pair_hash> cell_stats; // mean, std_dev
  
  for (auto& cell : grid_map) {
    if (cell.second.empty()) continue;
    
    // Calculate mean
    float sum = std::accumulate(cell.second.begin(), cell.second.end(), 0.0f);
    float mean = sum / cell.second.size();
    
    // Calculate standard deviation
    float sq_sum = std::inner_product(cell.second.begin(), cell.second.end(), cell.second.begin(), 0.0f);
    float std_dev = std::sqrt(sq_sum / cell.second.size() - mean * mean);
    
    cell_stats[cell.first] = {mean, std_dev};
  }

  // Classify points in original cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr refined_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr refined_nonground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  for (const auto& point : cloud->points) {
    // Exclude points above max_height_ from all outputs
    if (point.z > max_height_ || point.z < -1.5) {
      continue;
    }
    int grid_x = static_cast<int>(std::floor(point.x / grid_size_));
    int grid_y = static_cast<int>(std::floor(point.y / grid_size_));
    auto key = std::make_pair(grid_x, grid_y);

    bool is_ground = false;

    if (use_grid_filter_ && cell_stats.find(key) != cell_stats.end()) {
      // Grid-based filtering
      auto stats = cell_stats[key];
      float mean_z = stats.first;
      float std_dev = stats.second;
      float threshold = std::max(height_threshold_, k_multiplier_ * std_dev);

      if (std::abs(point.z - mean_z) <= threshold) {
        is_ground = true;
      }
    } else {
      // Fallback to plane model
      float a = coefficients->values[0];
      float b = coefficients->values[1];
      float c = coefficients->values[2];
      float d = coefficients->values[3];

      if (std::abs(c) > 1e-5) { // Avoid division by zero
        float expected_z = (-a * point.x - b * point.y - d) / c;
        float distance = std::abs(point.z - expected_z);

        if (distance <= distance_threshold_) {
          is_ground = true;
        }
      }
    }

    if (is_ground) {
      refined_ground_cloud->points.push_back(point);
    } else {
      refined_nonground_cloud->points.push_back(point);
    }
  }
  
  // Publish results
  sensor_msgs::msg::PointCloud2 ground_msg;
  pcl::toROSMsg(*refined_ground_cloud, ground_msg);
  ground_msg.header = msg->header;
  ground_pub_->publish(ground_msg);
  
  sensor_msgs::msg::PointCloud2 nonground_msg;
  pcl::toROSMsg(*refined_nonground_cloud, nonground_msg);
  nonground_msg.header = msg->header;
  nonground_pub_->publish(nonground_msg);
  

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilter>());
  rclcpp::shutdown();
  return 0;
}