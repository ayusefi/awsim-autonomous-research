#include "awsim_trajectory_planner/trajectory_generator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>

namespace awsim_trajectory_planner
{

TrajectoryGenerator::TrajectoryGenerator(const TrajectoryGenerationParams& params)
: params_(params)
{
}

std::vector<Trajectory> TrajectoryGenerator::generateCandidateTrajectories(
  const nav_msgs::msg::Path& global_path,
  const VehicleState& current_state)
{
  std::vector<Trajectory> trajectories;

  if (global_path.poses.empty()) {
    return trajectories;
  }

  // Generate rollouts with different lateral offsets
  for (int i = 0; i < params_.num_rollouts; ++i) {
    double lateral_offset = 0.0;
    
    if (params_.num_rollouts > 1) {
      // Distribute lateral offsets symmetrically around the global path
      lateral_offset = -params_.max_lateral_offset + 
        (2.0 * params_.max_lateral_offset * i) / (params_.num_rollouts - 1);
    }

    Trajectory trajectory = generateRolloutTrajectory(global_path, current_state, lateral_offset);
    
    if (isTrajectoryValid(trajectory)) {
      trajectories.push_back(trajectory);
    }
  }

  return trajectories;
}

Trajectory TrajectoryGenerator::generateRolloutTrajectory(
  const nav_msgs::msg::Path& global_path,
  const VehicleState& current_state,
  double lateral_offset)
{
  Trajectory trajectory;
  auto reference_poses = sampleGlobalPath(global_path, current_state);

  if (reference_poses.empty()) {
    return trajectory;
  }

  // Pre-allocate trajectory capacity for better performance
  int max_points = static_cast<int>(params_.prediction_horizon / params_.time_step) + 1;
  trajectory.reserve(max_points);

  double time = 0.0;
  VehicleState state = current_state;
  double lookahead_distance = 5.0;  // e.g., 5 meters, adjustable or velocity-based

  while (time < params_.prediction_horizon) {
    TrajectoryPoint point;
    point.time = time;

    // Find the closest reference point (optimized search)
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < reference_poses.size(); ++i) {
      double dx = reference_poses[i].pose.position.x - state.x;
      double dy = reference_poses[i].pose.position.y - state.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_idx = i;
      }
    }

    // Find a target index ahead by lookahead_distance
    size_t target_idx = closest_idx;
    double accumulated_distance = 0.0;
    for (size_t i = closest_idx; i < reference_poses.size() - 1 && accumulated_distance < lookahead_distance; ++i) {
      double dx = reference_poses[i + 1].pose.position.x - reference_poses[i].pose.position.x;
      double dy = reference_poses[i + 1].pose.position.y - reference_poses[i].pose.position.y;
      accumulated_distance += std::sqrt(dx * dx + dy * dy);
      target_idx = i + 1;
    }

    if (target_idx < reference_poses.size()) {
      tf2::Quaternion q;
      tf2::fromMsg(reference_poses[target_idx].pose.orientation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      double reference_yaw = yaw;

      double target_x = reference_poses[target_idx].pose.position.x + 
                       lateral_offset * std::cos(reference_yaw + M_PI_2);
      double target_y = reference_poses[target_idx].pose.position.y + 
                       lateral_offset * std::sin(reference_yaw + M_PI_2);

      double dx = target_x - state.x;
      double dy = target_y - state.y;
      double target_yaw = std::atan2(dy, dx);

      double yaw_error = normalizeAngle(target_yaw - state.yaw);
      double max_yaw_rate = params_.max_curvature * state.velocity;
      double yaw_rate = std::max(-max_yaw_rate, std::min(max_yaw_rate, yaw_error / params_.time_step));

      state.yaw += yaw_rate * params_.time_step;
      state.yaw = normalizeAngle(state.yaw);

      double target_velocity = std::min(params_.max_velocity, 
                                       std::sqrt(dx * dx + dy * dy) / params_.time_step);
      double velocity_error = target_velocity - state.velocity;
      double acceleration = std::max(params_.max_deceleration, 
                                   std::min(params_.max_acceleration, velocity_error / params_.time_step));

      state.velocity += acceleration * params_.time_step;
      state.velocity = std::max(0.0, std::min(params_.max_velocity, state.velocity));

      state.x += state.velocity * std::cos(state.yaw) * params_.time_step;
      state.y += state.velocity * std::sin(state.yaw) * params_.time_step;

      point.x = state.x;
      point.y = state.y;
      point.yaw = state.yaw;
      point.velocity = state.velocity;
      point.acceleration = acceleration;
      point.curvature = yaw_rate / std::max(0.1, state.velocity);

      trajectory.push_back(point);
    }

    time += params_.time_step;
  }

  return trajectory;
}

std::vector<geometry_msgs::msg::PoseStamped> TrajectoryGenerator::sampleGlobalPath(
  const nav_msgs::msg::Path& global_path,
  const VehicleState& current_state) const
{
  std::vector<geometry_msgs::msg::PoseStamped> sampled_poses;

  if (global_path.poses.empty()) {
    return sampled_poses;
  }

  // Find the closest point on the global path to the current position (optimized)
  double min_dist_sq = std::numeric_limits<double>::max();
  size_t start_idx = 0;

  for (size_t i = 0; i < global_path.poses.size(); ++i) {
    double dx = global_path.poses[i].pose.position.x - current_state.x;
    double dy = global_path.poses[i].pose.position.y - current_state.y;
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      start_idx = i;
    }
  }

  // Sample points from the global path starting from the closest point
  double max_distance = params_.prediction_horizon * params_.max_velocity;
  double accumulated_distance = 0.0;

  for (size_t i = start_idx; i < global_path.poses.size() && accumulated_distance < max_distance; ++i) {
    sampled_poses.push_back(global_path.poses[i]);

    if (i > start_idx) {
      double dx = global_path.poses[i].pose.position.x - global_path.poses[i-1].pose.position.x;
      double dy = global_path.poses[i].pose.position.y - global_path.poses[i-1].pose.position.y;
      accumulated_distance += std::sqrt(dx * dx + dy * dy);
    }
  }

  return sampled_poses;
}

bool TrajectoryGenerator::isTrajectoryValid(const Trajectory& trajectory) const
{
  if (trajectory.empty()) {
    return false;
  }

  for (const auto& point : trajectory) {
    if (!satisfiesKinematicConstraints(point)) {
      return false;
    }
  }

  return true;
}

bool TrajectoryGenerator::satisfiesKinematicConstraints(const TrajectoryPoint& point) const
{
  // Check velocity constraints
  if (point.velocity < 0.0 || point.velocity > params_.max_velocity) {
    return false;
  }

  // Check acceleration constraints
  if (point.acceleration < params_.max_deceleration || point.acceleration > params_.max_acceleration) {
    return false;
  }

  // Check curvature constraints
  if (std::abs(point.curvature) > params_.max_curvature) {
    return false;
  }

  return true;
}

nav_msgs::msg::Path TrajectoryGenerator::trajectoryToPath(
  const Trajectory& trajectory, 
  const std::string& frame_id) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = rclcpp::Clock().now();

  for (const auto& point : trajectory) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;

    pose_stamped.pose.position.x = point.x;
    pose_stamped.pose.position.y = point.y;
    pose_stamped.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, point.yaw);
    pose_stamped.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(pose_stamped);
  }

  return path;
}

VehicleState TrajectoryGenerator::poseAndTwistToVehicleState(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::Twist& twist) const
{
  VehicleState state;

  state.x = pose.pose.position.x;
  state.y = pose.pose.position.y;

  // Extract yaw from quaternion
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  state.yaw = yaw;

  state.velocity = std::sqrt(twist.linear.x * twist.linear.x + 
                           twist.linear.y * twist.linear.y);

  // Approximate acceleration and curvature (could be improved with history)
  state.acceleration = 0.0;
  state.curvature = twist.angular.z / std::max(0.1, state.velocity);

  return state;
}

double TrajectoryGenerator::calculateCurvature(
  const TrajectoryPoint& p1, 
  const TrajectoryPoint& p2, 
  const TrajectoryPoint& p3) const
{
  // Calculate curvature using three points
  double dx1 = p2.x - p1.x;
  double dy1 = p2.y - p1.y;
  double dx2 = p3.x - p2.x;
  double dy2 = p3.y - p2.y;

  double cross_product = dx1 * dy2 - dy1 * dx2;
  double magnitude1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  double magnitude2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

  if (magnitude1 < 1e-6 || magnitude2 < 1e-6) {
    return 0.0;
  }

  return cross_product / (magnitude1 * magnitude2);
}

double TrajectoryGenerator::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace awsim_trajectory_planner
