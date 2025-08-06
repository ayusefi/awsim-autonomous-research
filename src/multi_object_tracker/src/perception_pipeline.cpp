#include "multi_object_tracker/perception_pipeline.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

namespace multi_object_tracker {

PerceptionPipeline::PerceptionPipeline(const Config& config) : config_(config) {}

std::vector<Detection> PerceptionPipeline::processPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  // std::cout << "PerceptionPipeline: Processing cloud with " << cloud->size() << " points" << std::endl;
  
  // Step 1: Cluster objects using DBSCAN
  auto clusters = clusterObjects(cloud);
  // std::cout << "PerceptionPipeline: Found " << clusters.size() << " clusters" << std::endl;
  
  // Step 2: Filter clusters based on size and point count
  auto filtered_clusters = filterClusters(clusters);
  // std::cout << "PerceptionPipeline: After filtering, " << filtered_clusters.size() << " clusters remain" << std::endl;
  
  // Step 3: Create detections from clusters
  auto detections = createDetections(filtered_clusters);
  // std::cout << "PerceptionPipeline: Created " << detections.size() << " detections" << std::endl;
  
  return detections;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PerceptionPipeline::clusterObjects(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  
  // std::cout << "clusterObjects: Input cloud has " << cloud->size() << " points" << std::endl;
  // std::cout << "clusterObjects: Using eps=" << config_.dbscan_eps << ", min_points=" << config_.dbscan_min_points << std::endl;
  
  // Create KDTree for the search method
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  
  // Set up EuclideanClusterExtraction (equivalent to DBSCAN)
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(config_.dbscan_eps);
  ec.setMinClusterSize(config_.dbscan_min_points);
  ec.setMaxClusterSize(25000);  // Large enough to include vehicles
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  
  // std::cout << "clusterObjects: Found " << cluster_indices.size() << " cluster indices" << std::endl;
  
  // Extract clusters
  for (const auto& indices : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(std::make_shared<pcl::PointIndices>(indices));
    extract.setNegative(false);
    extract.filter(*cluster);
    clusters.push_back(cluster);
    // std::cout << "clusterObjects: Cluster " << clusters.size()-1 << " has " << cluster->size() << " points" << std::endl;
  }
  
  // std::cout << "clusterObjects: Returning " << clusters.size() << " clusters" << std::endl;
  return clusters;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PerceptionPipeline::filterClusters(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_clusters;
  
  // std::cout << "filterClusters: Input " << clusters.size() << " clusters" << std::endl;
  // std::cout << "filterClusters: Min volume=" << config_.filter_min_volume << ", min points=" << config_.filter_min_points << std::endl;
  
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto& cluster = clusters[i];
    
    // Skip noise cluster (usually the last one with many points)
    if (cluster == clusters.back() && cluster->size() > 100) {
      // std::cout << "filterClusters: Skipping noise cluster " << i << " with " << cluster->size() << " points" << std::endl;
      continue;
    }
    
    // Calculate bounding box volume
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    double length = max_pt.x - min_pt.x;
    double width = max_pt.y - min_pt.y;
    double height = max_pt.z - min_pt.z;
    double volume = length * width * height;
    
    // std::cout << "filterClusters: Cluster " << i << " - points: " << cluster->size() 
    //           << ", volume: " << volume << " (L:" << length << " W:" << width << " H:" << height << ")" << std::endl;
    
    // Apply filters
    if (volume >= config_.filter_min_volume && cluster->size() >= config_.filter_min_points) {
      filtered_clusters.push_back(cluster);
      // std::cout << "filterClusters: Cluster " << i << " PASSED filters" << std::endl;
    } else {
      // std::cout << "filterClusters: Cluster " << i << " REJECTED - volume: " << volume 
      //           << " < " << config_.filter_min_volume << " OR points: " << cluster->size() 
      //           << " < " << config_.filter_min_points << std::endl;
    }
  }
  
  // std::cout << "filterClusters: Returning " << filtered_clusters.size() << " filtered clusters" << std::endl;
  return filtered_clusters;
}

std::vector<Detection> PerceptionPipeline::createDetections(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
  std::vector<Detection> detections;
  
  for (const auto& cluster : clusters) {
    Detection detection;
    
    // Get bounding box
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    // Calculate center
    detection.center = Eigen::Vector3d(
      (min_pt.x + max_pt.x) / 2.0,
      (min_pt.y + max_pt.y) / 2.0,
      (min_pt.z + max_pt.z) / 2.0
    );
    
    // Calculate dimensions
    detection.dimensions = Eigen::Vector3d(
      max_pt.x - min_pt.x,
      max_pt.y - min_pt.y,
      max_pt.z - min_pt.z
    );
    
    // Calculate orientation using PCA
    detection.orientation = calculateOrientation(cluster);
    
    // Store points
    detection.points = cluster;
    
    detections.push_back(detection);
  }
  
  return detections;
}

Eigen::Quaterniond PerceptionPipeline::calculateOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster) {
  if (cluster->size() < 3) {
    // Not enough points for PCA, return identity quaternion
    return Eigen::Quaterniond::Identity();
  }
  
  // Calculate centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);
  
  // Center the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr centered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::demeanPointCloud(*cluster, centroid, *centered_cloud);
  
  // Compute covariance matrix
  Eigen::Matrix3f covariance_matrix;
  pcl::computeCovarianceMatrixNormalized(*centered_cloud, centroid, covariance_matrix);
  
  // Perform eigenvalue decomposition
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
  Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();
  
  // Ensure right-handed coordinate system
  if (eigenvectors.determinant() < 0) {
    eigenvectors.col(0) *= -1;
  }
  
  // Create rotation matrix (eigenvectors are sorted by eigenvalue)
  // Use the two largest eigenvectors as primary directions
  Eigen::Matrix3d rotation_matrix = eigenvectors.cast<double>();
  
  // Convert rotation matrix to quaternion
  Eigen::Quaterniond orientation(rotation_matrix);
  orientation.normalize();
  
  return orientation;
}

}  // namespace multi_object_tracker