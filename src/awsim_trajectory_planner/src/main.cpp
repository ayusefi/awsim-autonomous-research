#include "awsim_trajectory_planner/trajectory_planner_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<awsim_trajectory_planner::TrajectoryPlannerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
