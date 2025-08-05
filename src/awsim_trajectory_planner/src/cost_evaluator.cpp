#include "awsim_trajectory_planner/cost_evaluator.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace awsim_trajectory_planner
{

CostEvaluator::CostEvaluator(const CostEvaluationParams& params)
: params_(params)
{
}

std::vector<TrajectoryCost> CostEvaluator::evaluateTrajectories(
  const std::vector<Trajectory>& trajectories,
  const nav_msgs::msg::Path& global_path,
  const std::vector<Obstacle>& obstacles)
{
  std::vector<TrajectoryCost> costs;

  for (const auto& trajectory : trajectories) {
    TrajectoryCost cost;

    // Check collision first (early termination if collision detected)
    cost.is_collision_free = true;
    for (const auto& point : trajectory) {
      bool point_collision_free = true;
      for (const auto& obstacle : obstacles) {
        double dx = point.x - obstacle.x;
        double dy = point.y - obstacle.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < obstacle.radius + params_.safety_margin) {
          point_collision_free = false;
          cost.is_collision_free = false;
          break;
        }
      }
      if (!point_collision_free) {
        break;
      }
    }

    // If trajectory has collision, assign high cost and skip other calculations
    if (!cost.is_collision_free) {
      cost.obstacle_cost = 1e6;  // Very high cost for collision
      cost.path_deviation_cost = 0.0;
      cost.smoothness_cost = 0.0;
      cost.comfort_cost = 0.0;
      cost.total_cost = cost.obstacle_cost;
    } else {
      // Calculate individual cost components
      cost.obstacle_cost = calculateObstacleCost(trajectory, obstacles);
      cost.path_deviation_cost = calculatePathDeviationCost(trajectory, global_path);
      cost.smoothness_cost = calculateSmoothnessCost(trajectory);
      cost.comfort_cost = calculateComfortCost(trajectory);

      // Calculate weighted total cost
      cost.total_cost = 
        params_.obstacle_cost_weight * cost.obstacle_cost +
        params_.path_deviation_weight * cost.path_deviation_cost +
        params_.smoothness_weight * cost.smoothness_cost +
        params_.comfort_weight * cost.comfort_cost;
    }

    costs.push_back(cost);
  }

  return costs;
}

int CostEvaluator::selectBestTrajectory(const std::vector<TrajectoryCost>& costs)
{
  if (costs.empty()) {
    return -1;
  }

  int best_idx = -1;
  double min_cost = std::numeric_limits<double>::max();

  for (size_t i = 0; i < costs.size(); ++i) {
    // Prioritize collision-free trajectories
    if (costs[i].is_collision_free && costs[i].total_cost < min_cost) {
      min_cost = costs[i].total_cost;
      best_idx = static_cast<int>(i);
    }
  }

  // If no collision-free trajectory found, select the one with minimum collision cost
  if (best_idx == -1) {
    for (size_t i = 0; i < costs.size(); ++i) {
      if (costs[i].total_cost < min_cost) {
        min_cost = costs[i].total_cost;
        best_idx = static_cast<int>(i);
      }
    }
  }

  return best_idx;
}

double CostEvaluator::calculateObstacleCost(
  const Trajectory& trajectory,
  const std::vector<Obstacle>& obstacles)
{
  double total_cost = 0.0;

  for (const auto& point : trajectory) {
    double min_distance = distanceToNearestObstacle(point.x, point.y, obstacles);
    
    // Apply exponential cost function for proximity to obstacles
    if (min_distance < params_.safety_margin * 2.0) {
      double normalized_distance = min_distance / (params_.safety_margin * 2.0);
      total_cost += std::exp(-normalized_distance * 5.0);  // Exponential penalty
    }
  }

  return total_cost / trajectory.size();  // Average cost per point
}

double CostEvaluator::calculatePathDeviationCost(
  const Trajectory& trajectory,
  const nav_msgs::msg::Path& global_path)
{
  if (global_path.poses.empty() || trajectory.empty()) {
    return 0.0;
  }

  double total_deviation = 0.0;

  for (const auto& point : trajectory) {
    double deviation = distanceToPath(point.x, point.y, global_path);
    total_deviation += deviation * deviation;  // Quadratic penalty
  }

  return total_deviation / trajectory.size();
}

double CostEvaluator::calculateSmoothnessCost(const Trajectory& trajectory)
{
  if (trajectory.size() < 3) {
    return 0.0;
  }

  double total_cost = 0.0;

  for (size_t i = 1; i < trajectory.size() - 1; ++i) {
    // Calculate curvature change (second derivative of position)
    double curvature_change = calculateCurvatureChange(
      trajectory[i-1], trajectory[i], trajectory[i+1]);
    
    total_cost += curvature_change * curvature_change;

    // Add penalty for sharp velocity changes
    double velocity_change = std::abs(trajectory[i+1].velocity - trajectory[i-1].velocity);
    total_cost += velocity_change * velocity_change * 0.1;  // Lower weight for velocity smoothness
  }

  return total_cost / (trajectory.size() - 2);
}

double CostEvaluator::calculateComfortCost(const Trajectory& trajectory)
{
  if (trajectory.empty()) {
    return 0.0;
  }

  double total_cost = 0.0;

  for (const auto& point : trajectory) {
    // Penalize high accelerations (longitudinal discomfort)
    total_cost += point.acceleration * point.acceleration;

    // Penalize high curvatures (lateral discomfort)
    double lateral_acceleration = point.curvature * point.velocity * point.velocity;
    total_cost += lateral_acceleration * lateral_acceleration * 0.5;
  }

  return total_cost / trajectory.size();
}

double CostEvaluator::distanceToPath(double x, double y, const nav_msgs::msg::Path& path)
{
  if (path.poses.empty()) {
    return 0.0;
  }

  double min_distance = std::numeric_limits<double>::max();

  for (const auto& pose : path.poses) {
    double dx = x - pose.pose.position.x;
    double dy = y - pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    min_distance = std::min(min_distance, distance);
  }

  return min_distance;
}

double CostEvaluator::distanceToNearestObstacle(
  double x, double y,
  const std::vector<Obstacle>& obstacles)
{
  if (obstacles.empty()) {
    return std::numeric_limits<double>::max();
  }

  double min_distance = std::numeric_limits<double>::max();

  for (const auto& obstacle : obstacles) {
    double dx = x - obstacle.x;
    double dy = y - obstacle.y;
    double distance = std::sqrt(dx * dx + dy * dy) - obstacle.radius;
    min_distance = std::min(min_distance, std::max(0.0, distance));
  }

  return min_distance;
}

double CostEvaluator::calculateCurvatureChange(
  const TrajectoryPoint& p1,
  const TrajectoryPoint& p2,
  const TrajectoryPoint& p3)
{
  // Calculate change in heading angle (approximation of curvature change)
  double heading1 = std::atan2(p2.y - p1.y, p2.x - p1.x);
  double heading2 = std::atan2(p3.y - p2.y, p3.x - p2.x);
  
  double heading_change = heading2 - heading1;
  
  // Normalize angle difference
  while (heading_change > M_PI) {
    heading_change -= 2.0 * M_PI;
  }
  while (heading_change < -M_PI) {
    heading_change += 2.0 * M_PI;
  }

  return std::abs(heading_change);
}

geometry_msgs::msg::PoseStamped CostEvaluator::findClosestPoseOnPath(
  double x, double y, const nav_msgs::msg::Path& path)
{
  geometry_msgs::msg::PoseStamped closest_pose;

  if (path.poses.empty()) {
    return closest_pose;
  }

  double min_distance = std::numeric_limits<double>::max();
  size_t closest_idx = 0;

  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = x - path.poses[i].pose.position.x;
    double dy = y - path.poses[i].pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = i;
    }
  }

  return path.poses[closest_idx];
}

}  // namespace awsim_trajectory_planner
