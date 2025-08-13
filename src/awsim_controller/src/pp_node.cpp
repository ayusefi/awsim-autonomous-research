#include "awsim_controller/pure_pursuit_controller.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<awsim_controller::PurePursuitController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
