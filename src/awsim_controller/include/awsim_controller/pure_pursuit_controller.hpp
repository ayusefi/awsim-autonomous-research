#ifndef AWSIM_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define AWSIM_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

namespace awsim_controller
{

struct PurePursuitParams
{
  double lookahead_distance_min = 3.0;  // Minimum lookahead distance [m]
  double lookahead_distance_max = 20.0; // Maximum lookahead distance [m] 
  double lookahead_gain = 1.5;          // Lookahead distance gain based on velocity
  double wheelbase = 2.79;              // Vehicle wheelbase [m] (typical for sedan)
  double max_steering_angle = 0.7854;   // Max steering angle [rad] (45 degrees)
  double target_velocity = 10.0;        // Target velocity [m/s]
  double max_acceleration = 3.0;        // Max acceleration [m/s²]
  double max_deceleration = -5.0;       // Max deceleration [m/s²]
  double curve_decel_factor = 0.5;      // Deceleration factor for curves
  double curve_threshold = 0.3;         // Curvature threshold for curve detection [1/m]
  double velocity_tolerance = 0.5;      // Velocity tolerance for control [m/s]
  double path_tolerance = 0.1;          // Distance tolerance to path [m]
};

class PurePursuitController : public rclcpp::Node
{
public:
  explicit PurePursuitController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback functions
  void onPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onVelocity(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);
  
  // Timer callback
  void onTimer();
  
  // Core algorithm functions
  geometry_msgs::msg::Point findLookaheadPoint();
  double calculateSteeringAngle(const geometry_msgs::msg::Point & lookahead_point);
  double calculateTargetAcceleration();
  double calculatePathCurvature(size_t index, double window = 5.0);
  double calculateDistanceToPoint(const geometry_msgs::msg::Point & point);
  size_t findClosestPathIndex();
  
  // Utility functions
  double normalizeAngle(double angle);
  double calculateDistance(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);
  uint8_t determineGearCommand();
  
  // Visualization functions
  void publishVisualization(const geometry_msgs::msg::Point & lookahead_point);
  void publishPathVisualization();
  
  // Publishers and Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_vel_;
  
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_control_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State variables
  nav_msgs::msg::Path::SharedPtr current_path_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
  autoware_vehicle_msgs::msg::VelocityReport::SharedPtr current_velocity_;
  
  // Parameters
  PurePursuitParams params_;
  
  // Control state
  bool path_received_ = false;
  bool pose_received_ = false;
  bool velocity_received_ = false;
  double previous_steering_angle_ = 0.0;
  size_t closest_path_index_ = 0;
  
  // Constants
  static constexpr double CONTROL_FREQUENCY = 50.0;  // 50 Hz control loop
};

}  // namespace awsim_controller

#endif  // AWSIM_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_