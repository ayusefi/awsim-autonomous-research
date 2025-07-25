#include "awsim_path_planner/path_planner_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<awsim_path_planner::PathPlannerNode>();
  
  RCLCPP_INFO(node->get_logger(), "AWSIM Path Planner Node started");
  
  rclcpp::spin(node);
  
  RCLCPP_INFO(node->get_logger(), "AWSIM Path Planner Node shutting down");
  rclcpp::shutdown();
  
  return 0;
}
