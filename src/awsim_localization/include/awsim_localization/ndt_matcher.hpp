#ifndef AWSIM_LOCALIZATION__NDT_MATCHER_HPP_
#define AWSIM_LOCALIZATION__NDT_MATCHER_HPP_

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include "pclomp/ndt_omp.h"

namespace awsim_localization
{

struct NDTResult
{
  Eigen::Matrix4f transformation;
  double fitness_score;
  bool has_converged;
  int final_num_iteration;
  double transformation_probability;
};

class NDTMatcher
{
public:
  NDTMatcher();
  ~NDTMatcher() = default;
  
  // Configuration
  void setResolution(double resolution);
  void setMaxIterations(int max_iterations);
  void setTransformationEpsilon(double transformation_epsilon);
  void setStepSize(double step_size);
  void setNumThreads(int num_threads);
  
  // Map operations
  void setTargetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud);
  
  // Scan matching
  NDTResult matchScan(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity());
  
  // Quality metrics
  double getFitnessScore() const;
  bool hasConverged() const;
  int getFinalNumIteration() const;
  double getTransformationProbability() const;
  
  // Getters
  pcl::PointCloud<pcl::PointXYZ>::Ptr getAlignedCloud() const;

private:
  void preprocessCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  
  // Parameters
  double resolution_;
  int max_iterations_;
  double transformation_epsilon_;
  double step_size_;
  double voxel_leaf_size_;
  
  // Results
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;
  NDTResult last_result_;
};

}  // namespace awsim_localization

#endif  // AWSIM_LOCALIZATION__NDT_MATCHER_HPP_
