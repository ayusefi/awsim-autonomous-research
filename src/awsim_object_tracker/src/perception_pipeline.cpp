#include "awsim_object_tracker/perception_pipeline.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

namespace awsim_object_tracker {

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
  // std::cout << "filterClusters: Volume range=[" << config_.filter_min_volume << ", " << config_.filter_max_volume 
  //           << "], Points range=[" << config_.filter_min_points << ", " << config_.filter_max_points << "]" << std::endl;
  
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
    
    int point_count = static_cast<int>(cluster->size());
    
    // std::cout << "filterClusters: Cluster " << i << " - points: " << point_count 
    //           << ", volume: " << volume << " (L:" << length << " W:" << width << " H:" << height << ")" << std::endl;
    
    // Apply size filters (both minimum and maximum)
    bool volume_ok = (volume >= config_.filter_min_volume && volume <= config_.filter_max_volume);
    bool points_ok = (point_count >= config_.filter_min_points && point_count <= config_.filter_max_points);
    
    if (volume_ok && points_ok) {
      filtered_clusters.push_back(cluster);
      // std::cout << "filterClusters: Cluster " << i << " PASSED filters" << std::endl;
    } else {
      // std::cout << "filterClusters: Cluster " << i << " REJECTED - volume: " << volume 
      //           << " [" << config_.filter_min_volume << "-" << config_.filter_max_volume << "]"
      //           << " OR points: " << point_count 
      //           << " [" << config_.filter_min_points << "-" << config_.filter_max_points << "]" << std::endl;
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
    
    // Calculate centroid and initial bounding box (for center)
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    Eigen::Vector3d centroid((min_pt.x + max_pt.x) / 2.0,
                              (min_pt.y + max_pt.y) / 2.0,
                              (min_pt.z + max_pt.z) / 2.0);
    detection.center = centroid;
    
    // Compute orientation using PCA
    Eigen::Quaterniond q = calculateOrientation(cluster);
    detection.orientation = q;
    
    // Compute oriented bounding box dimensions: project points into local frame
    Eigen::Vector3d local_min(std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity());
    Eigen::Vector3d local_max(-std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity());
    Eigen::Matrix3d R = q.toRotationMatrix().transpose();  // world to local
    for (const auto& pt : *cluster) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      Eigen::Vector3d local = R * (p - centroid);
      local_min = local_min.cwiseMin(local);
      local_max = local_max.cwiseMax(local);
    }
    detection.dimensions = local_max - local_min;
    // Enforce that length (x) is the larger dimension for rectangle assumption
    if (detection.dimensions.x() < detection.dimensions.y()) {
      std::swap(detection.dimensions.x(), detection.dimensions.y());
    }
    
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
  Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();
  
  // Check if the object has a clear orientation (eigenvalues are sufficiently different)
  // If eigenvalues are too similar, the orientation is ambiguous
  double ratio_threshold = 0.3;  // Minimum ratio between largest and smallest eigenvalues
  if (eigenvalues(2) < ratio_threshold * eigenvalues(0)) {
    // Object is roughly spherical, orientation is ambiguous
    // Return a stable orientation aligned with coordinate axes
    return Eigen::Quaterniond::Identity();
  }
  
  // Ensure right-handed coordinate system
  if (eigenvectors.determinant() < 0) {
    eigenvectors.col(0) *= -1;
  }
  
  // For stability, prefer alignment with coordinate axes when possible
  // This reduces oscillation between similar orientations
  Eigen::Matrix3f rotation_matrix = eigenvectors;
  
  // Stabilize orientation by choosing consistent eigenvector directions
  // Prefer positive X and Y components in the main eigenvectors
  if (rotation_matrix(0, 2) < 0) {  // Z-axis (main direction) pointing backwards
    rotation_matrix.col(2) *= -1;
    rotation_matrix.col(1) *= -1;  // Maintain right-handed system
  }
  
  if (rotation_matrix(1, 1) < 0) {  // Y-axis pointing down
    rotation_matrix.col(1) *= -1;
    rotation_matrix.col(0) *= -1;  // Maintain right-handed system
  }
  
  // Convert rotation matrix to quaternion
  Eigen::Quaterniond orientation(rotation_matrix.cast<double>());
  orientation.normalize();
  
  return orientation;
}

}  // namespace awsim_object_tracker