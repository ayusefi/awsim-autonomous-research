
#ifndef AWSIM_OBJECT_TRACKER_PERCEPTION_PIPELINE_HPP
#define AWSIM_OBJECT_TRACKER_PERCEPTION_PIPELINE_HPP

#include <vector>
#include <memory>
#include <map>
#include <string>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace awsim_object_tracker {

/**
 * @brief Represents a single object detection
 */
struct Detection {
  Eigen::Vector3d center;      // Center position [x, y, z]
  Eigen::Vector3d dimensions;  // Dimensions [length, width, height]
  Eigen::Quaterniond orientation; // Orientation of the detection
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;  // Point cloud of the detection
};

/**
 * @brief Perception pipeline for object detection from point clouds
 */
class PerceptionPipeline {
public:
  /**
   * @brief Configuration parameters for the perception pipeline
   */
  struct Config {
    double dbscan_eps;          // Maximum distance between points in a cluster
    int dbscan_min_points;      // Minimum points to form a cluster
    double filter_min_volume;   // Minimum bounding box volume in cubic meters
    double filter_max_volume;   // Maximum bounding box volume in cubic meters
    int filter_min_points;      // Minimum number of points in a cluster
    int filter_max_points;      // Maximum number of points in a cluster

    Config()
      : dbscan_eps(0.7),
        dbscan_min_points(10),
        filter_min_volume(1.0),
        filter_max_volume(100.0),
        filter_min_points(20),
        filter_max_points(5000) {}
  };

  /**
   * @brief Construct a new Perception Pipeline object
   * 
   * @param config Configuration parameters
   */
  explicit PerceptionPipeline(const Config& config = Config());

  /**
   * @brief Process a point cloud to detect objects
   * 
   * @param cloud Input point cloud (already non-ground points)
   * @return std::vector<Detection> List of detections
   */
  std::vector<Detection> processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

private:
  Config config_;

  /**
   * @brief Cluster points using DBSCAN
   */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterObjects(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  /**
   * @brief Filter clusters based on bounding box volume and point count
   */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filterClusters(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

  /**
   * @brief Create detections from clusters
   */
  std::vector<Detection> createDetections(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

  /**
   * @brief Calculate orientation of a cluster using PCA
   */
  Eigen::Quaterniond calculateOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
};

}  // namespace awsim_object_tracker

#endif  // AWSIM_OBJECT_TRACKER_PERCEPTION_PIPELINE_HPP