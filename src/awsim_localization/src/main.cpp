#include "awsim_localization_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<awsim_localization::AwsimLocalizationNode>(options);
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
