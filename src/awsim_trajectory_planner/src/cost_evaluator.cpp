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
  costs.reserve(trajectories.size());

  for (const auto& trajectory : trajectories) {
    TrajectoryCost cost;

    // BINARY SAFETY CHECK: If ANY point violates safety margin, trajectory is REJECTED
    cost.is_collision_free = true;
    
    // Check discrete points using oriented bounding box collision detection
    for (const auto& point : trajectory) {
      for (const auto& obstacle : obstacles) {
        // Calculate minimum distance to oriented bounding box
        double distance = obstacle.distanceToPoint(point.x, point.y);
        
        if (distance < params_.safety_margin) {
          cost.is_collision_free = false;
          RCLCPP_DEBUG(rclcpp::get_logger("CostEvaluator"), 
                      "Trajectory REJECTED: point(%.2f,%.2f) too close to OBB (distance=%.2fm < safety_margin=%.2fm)", 
                      point.x, point.y, distance, params_.safety_margin);
          break;
        }
      }
      if (!cost.is_collision_free) {
        break; // Early termination - trajectory is rejected
      }
    }

    // If trajectory is still collision-free, check segments between points (continuous collision)
    if (cost.is_collision_free && trajectory.size() > 1) {
      for (size_t i = 0; i < trajectory.size() - 1; ++i) {
        if (isSegmentColliding(trajectory[i], trajectory[i+1], obstacles, params_.safety_margin)) {
          cost.is_collision_free = false;
          RCLCPP_DEBUG(rclcpp::get_logger("CostEvaluator"), 
                      "Trajectory REJECTED: segment between points %zu and %zu collides with obstacle", i, i+1);
          break;
        }
      }
    }

    // If trajectory violates safety margin, mark as OCCUPIED (not selectable)
    if (!cost.is_collision_free) {
      cost.obstacle_cost = std::numeric_limits<double>::infinity();
      cost.path_deviation_cost = std::numeric_limits<double>::infinity();
      cost.smoothness_cost = std::numeric_limits<double>::infinity();
      cost.comfort_cost = std::numeric_limits<double>::infinity();
      cost.total_cost = std::numeric_limits<double>::infinity();
      RCLCPP_DEBUG(rclcpp::get_logger("CostEvaluator"), "Trajectory marked as OCCUPIED due to safety violation");
    } else {
      // Trajectory is SAFE - calculate quality scores based on other factors
      cost.obstacle_cost = 0.0; // No obstacle penalty for safe trajectories
      cost.path_deviation_cost = calculatePathDeviationCost(trajectory, global_path);
      cost.smoothness_cost = calculateSmoothnessCost(trajectory);
      cost.comfort_cost = calculateComfortCost(trajectory);

      // Calculate weighted total cost (lower is better)
      cost.total_cost = 
        params_.path_deviation_weight * cost.path_deviation_cost +
        params_.smoothness_weight * cost.smoothness_cost +
        params_.comfort_weight * cost.comfort_cost;
        
      RCLCPP_DEBUG(rclcpp::get_logger("CostEvaluator"), 
                  "Safe trajectory scored: deviation=%.3f, smoothness=%.3f, comfort=%.3f, total=%.3f",
                  cost.path_deviation_cost, cost.smoothness_cost, cost.comfort_cost, cost.total_cost);
    }

    costs.push_back(cost);
  }

  return costs;
}

bool CostEvaluator::isSegmentColliding(
    const TrajectoryPoint& p1, 
    const TrajectoryPoint& p2,
    const std::vector<Obstacle>& obstacles,
    double safety_margin) const
{
    // Check multiple points along the segment using oriented bounding box collision
    const int steps = 10; // Increased resolution for better accuracy
    for (int i = 1; i < steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double x = p1.x + t * (p2.x - p1.x);
        double y = p1.y + t * (p2.y - p1.y);
        
        for (const auto& obstacle : obstacles) {
            // Use oriented bounding box distance calculation
            double distance = obstacle.distanceToPoint(x, y);
            if (distance < safety_margin) {
                return true;
            }
        }
    }
    return false;
}

int CostEvaluator::selectBestTrajectory(const std::vector<TrajectoryCost>& costs)
{
  if (costs.empty()) {
    return -1;
  }

  int best_idx = -1;
  double min_cost = std::numeric_limits<double>::max();

  // Only consider SAFE trajectories (collision-free within safety margin)
  for (size_t i = 0; i < costs.size(); ++i) {
    if (costs[i].is_collision_free && std::isfinite(costs[i].total_cost)) {
      if (costs[i].total_cost < min_cost) {
        min_cost = costs[i].total_cost;
        best_idx = static_cast<int>(i);
      }
    }
  }

  if (best_idx >= 0) {
    RCLCPP_DEBUG(rclcpp::get_logger("CostEvaluator"), 
                "Selected safe trajectory %d with cost %.3f", best_idx, min_cost);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("CostEvaluator"), 
               "No safe trajectories found - all violate safety margin");
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

  double min_dist_sq = std::numeric_limits<double>::max();

  // Use squared distance to avoid sqrt calculations
  for (const auto& pose : path.poses) {
    double dx = x - pose.pose.position.x;
    double dy = y - pose.pose.position.y;
    double dist_sq = dx * dx + dy * dy;
    min_dist_sq = std::min(min_dist_sq, dist_sq);
  }

  return std::sqrt(min_dist_sq);
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
    double distance = obstacle.distanceToPoint(x, y);
    min_distance = std::min(min_distance, distance);
    
    // Early termination if very close
    if (min_distance < 0.1) {
      return 0.0;
    }
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
