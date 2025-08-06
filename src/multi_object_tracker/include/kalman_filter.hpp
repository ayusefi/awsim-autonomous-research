
#ifndef MULTI_OBJECT_TRACKER_KALMAN_FILTER_HPP
#define MULTI_OBJECT_TRACKER_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <memory>

namespace multi_object_tracker {

/**
 * @brief Kalman Filter implementation for 3D object tracking
 * 
 * State vector: [x, y, z, vx, vy, vz] (position and velocity)
 * Measurement vector: [x, y, z] (position only)
 */
class KalmanFilter {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct a new Kalman Filter object
   * 
   * @param dt Time step between predictions (seconds)
   * @param process_noise Process noise variance
   * @param measurement_noise Measurement noise variance
   */
  KalmanFilter(double dt = 0.1, double process_noise = 1.0, double measurement_noise = 1.0);

  /**
   * @brief Initialize the filter with initial state
   * 
   * @param initial_position Initial position [x, y, z]
   * @param initial_velocity Initial velocity [vx, vy, vz] (default: zero)
   */
  void initialize(const Eigen::Vector3d& initial_position, 
                  const Eigen::Vector3d& initial_velocity = Eigen::Vector3d::Zero());

  /**
   * @brief Predict the next state using the motion model
   */
  void predict();

  /**
   * @brief Update the filter with a new measurement
   * 
   * @param measurement Measurement vector [x, y, z]
   */
  void update(const Eigen::Vector3d& measurement);

  /**
   * @brief Get current position estimate
   * 
   * @return Position vector [x, y, z]
   */
  Eigen::Vector3d getPosition() const;

  /**
   * @brief Get current velocity estimate
   * 
   * @return Velocity vector [vx, vy, vz]
   */
  Eigen::Vector3d getVelocity() const;

  /**
   * @brief Get position uncertainty (standard deviation)
   * 
   * @return Position uncertainty [σx, σy, σz]
   */
  Eigen::Vector3d getPositionUncertainty() const;

  /**
   * @brief Get velocity uncertainty (standard deviation)
   * 
   * @return Velocity uncertainty [σvx, σvy, σvz]
   */
  Eigen::Vector3d getVelocityUncertainty() const;

  /**
   * @brief Calculate Mahalanobis distance for data association
   * 
   * @param measurement Measurement vector [x, y, z]
   * @return Mahalanobis distance
   */
  double getInnovationDistance(const Eigen::Vector3d& measurement) const;

  /**
   * @brief Calculate Euclidean distance to measurement
   * 
   * @param measurement Measurement vector [x, y, z]
   * @return Euclidean distance
   */
  double getEuclideanDistance(const Eigen::Vector3d& measurement) const;

private:
  double dt_;  // Time step
  int state_dim_;  // State dimension (6)
  int measurement_dim_;  // Measurement dimension (3)

  // State vector [x, y, z, vx, vy, vz]
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // State transition matrix (constant velocity model)
  Eigen::MatrixXd F_;

  // Measurement matrix (we only measure position)
  Eigen::MatrixXd H_;

  // Process noise covariance matrix
  Eigen::MatrixXd Q_;

  // Measurement noise covariance matrix
  Eigen::MatrixXd R_;

  // Identity matrix for calculations
  Eigen::MatrixXd I_;

  /**
   * @brief Build the process noise covariance matrix Q
   * 
   * @param noise_variance Base noise variance
   * @return Process noise covariance matrix
   */
  Eigen::MatrixXd buildProcessNoiseMatrix(double noise_variance) const;
};

}  // namespace multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER_KALMAN_FILTER_HPP