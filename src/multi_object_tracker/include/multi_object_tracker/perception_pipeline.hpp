#ifndef MULTI_OBJECT_TRACKER_PERCEPTION_PIPELINE_HPP
#define MULTI_OBJECT_TRACKER_PERCEPTION_PIPELINE_HPP

#include <vector>
#include <memory>
#include <map>
#include <string>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace multi_object_tracker {

/**
 * @brief Represents a single object detection
 */
struct Detection {
  Eigen::Vector3d center;      // Center position [x, y, z]
  Eigen::Vector3d dimensions;  // Dimensions [length, width, height]
  Eigen::Quaterniond orientation;  // Orientation as quaternion (estimated from point cloud)
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
    int filter_min_points;      // Minimum number of points in a cluster

    Config()
      : dbscan_eps(0.7),
        dbscan_min_points(10),
        filter_min_volume(1.0),
        filter_min_points(20) {}
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
   * 
   * @param cloud Input point cloud
   * @return std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> List of clusters
   */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterObjects(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  /**
   * @brief Filter clusters based on bounding box volume and point count
   * 
   * @param clusters List of clusters
   * @return std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> List of filtered clusters
   */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filterClusters(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

  /**
   * @brief Create detections from clusters
   * 
   * @param clusters List of clusters
   * @return std::vector<Detection> List of detections
   */
  std::vector<Detection> createDetections(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

private:
  /**
   * @brief Calculate orientation of a cluster using PCA
   * 
   * @param cluster Point cloud cluster
   * @return Eigen::Quaterniond Orientation as quaternion
   */
  Eigen::Quaterniond calculateOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
};

}  // namespace multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER_PERCEPTION_PIPELINE_HPP