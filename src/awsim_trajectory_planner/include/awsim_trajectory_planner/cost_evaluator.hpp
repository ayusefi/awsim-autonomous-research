#ifndef AWSIM_TRAJECTORY_PLANNER__COST_EVALUATOR_HPP_
#define AWSIM_TRAJECTORY_PLANNER__COST_EVALUATOR_HPP_

#include <vector>
#include <nav_msgs/msg/path.hpp>
#include "awsim_trajectory_planner/trajectory_generator.hpp"
#include "awsim_trajectory_planner/obstacle_detector.hpp"

namespace awsim_trajectory_planner
{

struct CostEvaluationParams
{
  double obstacle_cost_weight = 1000.0;
  double path_deviation_weight = 10.0;
  double smoothness_weight = 1.0;
  double comfort_weight = 5.0;
  double safety_margin = 1.5;
  double collision_check_resolution = 0.2;
};

struct TrajectoryCost
{
  double total_cost = 0.0;
  double obstacle_cost = 0.0;
  double path_deviation_cost = 0.0;
  double smoothness_cost = 0.0;
  double comfort_cost = 0.0;
  bool is_collision_free = true;
};

class CostEvaluator
{
public:
  explicit CostEvaluator(const CostEvaluationParams& params);
  ~CostEvaluator() = default;

  // Main cost evaluation function
  std::vector<TrajectoryCost> evaluateTrajectories(
    const std::vector<Trajectory>& trajectories,
    const nav_msgs::msg::Path& global_path,
    const std::vector<Obstacle>& obstacles);

  // Select best trajectory based on cost
  int selectBestTrajectory(const std::vector<TrajectoryCost>& costs);

  // Individual cost components
  double calculateObstacleCost(const Trajectory& trajectory,
                              const std::vector<Obstacle>& obstacles);

  double calculatePathDeviationCost(const Trajectory& trajectory,
                                   const nav_msgs::msg::Path& global_path);

  double calculateSmoothnessCost(const Trajectory& trajectory);

  double calculateComfortCost(const Trajectory& trajectory);

private:
  // Utility functions
  double distanceToPath(double x, double y, const nav_msgs::msg::Path& path);
  
  double distanceToNearestObstacle(double x, double y,
                                  const std::vector<Obstacle>& obstacles);

  double calculateCurvatureChange(const TrajectoryPoint& p1,
                                 const TrajectoryPoint& p2,
                                 const TrajectoryPoint& p3);

  geometry_msgs::msg::PoseStamped findClosestPoseOnPath(
    double x, double y, const nav_msgs::msg::Path& path);

  CostEvaluationParams params_;
};

}  // namespace awsim_trajectory_planner

#endif  // AWSIM_TRAJECTORY_PLANNER__COST_EVALUATOR_HPP_
