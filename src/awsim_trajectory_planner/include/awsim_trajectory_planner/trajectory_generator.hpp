#ifndef AWSIM_TRAJECTORY_PLANNER__TRAJECTORY_GENERATOR_HPP_
#define AWSIM_TRAJECTORY_PLANNER__TRAJECTORY_GENERATOR_HPP_

#include <vector>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace awsim_trajectory_planner
{

struct VehicleState
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double curvature = 0.0;
};

struct TrajectoryPoint
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double curvature = 0.0;
  double time = 0.0;
};

using Trajectory = std::vector<TrajectoryPoint>;

struct TrajectoryGenerationParams
{
  double prediction_horizon = 5.0;
  double time_step = 0.1;
  int num_rollouts = 7;
  double max_lateral_offset = 3.0;
  double max_velocity = 15.0;
  double max_acceleration = 2.0;
  double max_deceleration = -4.0;
  double max_curvature = 0.2;
};

class TrajectoryGenerator
{
public:
  explicit TrajectoryGenerator(const TrajectoryGenerationParams& params);
  ~TrajectoryGenerator() = default;

  // Generate candidate trajectories
  std::vector<Trajectory> generateCandidateTrajectories(
    const nav_msgs::msg::Path& global_path,
    const VehicleState& current_state);

  // Convert trajectory to ROS path message
  nav_msgs::msg::Path trajectoryToPath(const Trajectory& trajectory, 
                                       const std::string& frame_id) const;

  // Convert ROS messages to internal representation
  VehicleState poseAndTwistToVehicleState(const geometry_msgs::msg::PoseStamped& pose,
                                          const geometry_msgs::msg::Twist& twist) const;

private:
  // Core trajectory generation methods
  Trajectory generateRolloutTrajectory(const nav_msgs::msg::Path& global_path,
                                       const VehicleState& current_state,
                                       double lateral_offset);

  std::vector<geometry_msgs::msg::PoseStamped> sampleGlobalPath(
    const nav_msgs::msg::Path& global_path,
    const VehicleState& current_state) const;

  // Constraint checking
  bool isTrajectoryValid(const Trajectory& trajectory) const;
  bool satisfiesKinematicConstraints(const TrajectoryPoint& point) const;
  
  // Utility functions
  double calculateCurvature(const TrajectoryPoint& p1, 
                           const TrajectoryPoint& p2, 
                           const TrajectoryPoint& p3) const;
  
  double normalizeAngle(double angle) const;
  
  TrajectoryGenerationParams params_;
};

}  // namespace awsim_trajectory_planner

#endif  // AWSIM_TRAJECTORY_PLANNER__TRAJECTORY_GENERATOR_HPP_
