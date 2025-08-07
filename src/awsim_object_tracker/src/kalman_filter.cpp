#include "awsim_object_tracker/kalman_filter.hpp"

#include <cmath>

namespace awsim_object_tracker {

KalmanFilter::KalmanFilter(double dt, double process_noise, double measurement_noise)
  : dt_(dt),
    state_dim_(6),
    measurement_dim_(3),
    x_(Eigen::VectorXd::Zero(state_dim_)),
    P_(Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 1000),
    F_(Eigen::MatrixXd::Zero(state_dim_, state_dim_)),
    H_(Eigen::MatrixXd::Zero(measurement_dim_, state_dim_)),
    Q_(Eigen::MatrixXd::Zero(state_dim_, state_dim_)),
    R_(Eigen::MatrixXd::Identity(measurement_dim_, measurement_dim_) * measurement_noise),
    I_(Eigen::MatrixXd::Identity(state_dim_, state_dim_)) {
  
  // Set up state transition matrix (constant velocity model)
  F_(0, 0) = 1.0; F_(0, 3) = dt_;
  F_(1, 1) = 1.0; F_(1, 4) = dt_;
  F_(2, 2) = 1.0; F_(2, 5) = dt_;
  F_(3, 3) = 1.0;
  F_(4, 4) = 1.0;
  F_(5, 5) = 1.0;

  // Set up measurement matrix (we only measure position)
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;
  H_(2, 2) = 1.0;

  // Build process noise covariance matrix
  Q_ = buildProcessNoiseMatrix(process_noise);
}

void KalmanFilter::initialize(const Eigen::Vector3d& initial_position, 
                              const Eigen::Vector3d& initial_velocity) {
  x_.segment<3>(0) = initial_position;
  x_.segment<3>(3) = initial_velocity;
  
  // Reset covariance matrix
  P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 1000;
  P_.block<3, 3>(3, 3) *= 100;  // Lower uncertainty for velocity
}

void KalmanFilter::predict() {
  // Predict state: x = F * x
  x_ = F_ * x_;
  
  // Predict covariance: P = F * P * F' + Q
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::Vector3d& measurement) {
  // Innovation (residual): y = z - H * x
  Eigen::VectorXd y = measurement - H_ * x_;
  
  // Innovation covariance: S = H * P * H' + R
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  
  // Kalman gain: K = P * H' * S^(-1)
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  // Update state: x = x + K * y
  x_ = x_ + K * y;
  
  // Update covariance: P = (I - K * H) * P
  P_ = (I_ - K * H_) * P_;
}

Eigen::Vector3d KalmanFilter::getPosition() const {
  return x_.segment<3>(0);
}

Eigen::Vector3d KalmanFilter::getVelocity() const {
  return x_.segment<3>(3);
}

Eigen::Vector3d KalmanFilter::getPositionUncertainty() const {
  return P_.diagonal().segment<3>(0).cwiseSqrt();
}

Eigen::Vector3d KalmanFilter::getVelocityUncertainty() const {
  return P_.diagonal().segment<3>(3).cwiseSqrt();
}

double KalmanFilter::getInnovationDistance(const Eigen::Vector3d& measurement) const {
  Eigen::VectorXd y = measurement - H_ * x_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  
  // Mahalanobis distance
  return std::sqrt(y.transpose() * S.inverse() * y);
}

double KalmanFilter::getEuclideanDistance(const Eigen::Vector3d& measurement) const {
  return (measurement - getPosition()).norm();
}

Eigen::MatrixXd KalmanFilter::buildProcessNoiseMatrix(double noise_variance) const {
  double dt = dt_;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  
  // Noise affects both position and velocity
  double q_pos = noise_variance * dt4 / 4;
  double q_vel = noise_variance * dt2;
  double q_pos_vel = noise_variance * dt3 / 2;
  
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
  
  Q(0, 0) = q_pos; Q(0, 3) = q_pos_vel;
  Q(1, 1) = q_pos; Q(1, 4) = q_pos_vel;
  Q(2, 2) = q_pos; Q(2, 5) = q_pos_vel;
  Q(3, 0) = q_pos_vel; Q(3, 3) = q_vel;
  Q(4, 1) = q_pos_vel; Q(4, 4) = q_vel;
  Q(5, 2) = q_pos_vel; Q(5, 5) = q_vel;
  
  return Q;
}

}  // namespace awsim_object_tracker