#include "awsim_controller/pure_pursuit_controller.hpp"

using std::placeholders::_1;

namespace awsim_controller
{

PurePursuitController::PurePursuitController(const rclcpp::NodeOptions & options)
: Node("pure_pursuit_controller", options)
{
  // Declare parameters
  declare_parameter("lookahead_distance_min", params_.lookahead_distance_min);
  declare_parameter("lookahead_distance_max", params_.lookahead_distance_max);
  declare_parameter("lookahead_gain", params_.lookahead_gain);
  declare_parameter("wheelbase", params_.wheelbase);
  declare_parameter("max_steering_angle", params_.max_steering_angle);
  declare_parameter("target_velocity", params_.target_velocity);
  declare_parameter("max_acceleration", params_.max_acceleration);
  declare_parameter("max_deceleration", params_.max_deceleration);
  declare_parameter("curve_decel_factor", params_.curve_decel_factor);
  declare_parameter("curve_threshold", params_.curve_threshold);
  declare_parameter("velocity_tolerance", params_.velocity_tolerance);
  declare_parameter("path_tolerance", params_.path_tolerance);

  // Get parameters
  get_parameter("lookahead_distance_min", params_.lookahead_distance_min);
  get_parameter("lookahead_distance_max", params_.lookahead_distance_max);
  get_parameter("lookahead_gain", params_.lookahead_gain);
  get_parameter("wheelbase", params_.wheelbase);
  get_parameter("max_steering_angle", params_.max_steering_angle);
  get_parameter("target_velocity", params_.target_velocity);
  get_parameter("max_acceleration", params_.max_acceleration);
  get_parameter("max_deceleration", params_.max_deceleration);
  get_parameter("curve_decel_factor", params_.curve_decel_factor);
  get_parameter("curve_threshold", params_.curve_threshold);
  get_parameter("velocity_tolerance", params_.velocity_tolerance);
  get_parameter("path_tolerance", params_.path_tolerance);

  // Subscribers
  sub_path_ = create_subscription<nav_msgs::msg::Path>(
    "/planning/path", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    std::bind(&PurePursuitController::onPath, this, _1));
  sub_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/pose_with_covariance", 10,
    std::bind(&PurePursuitController::onPose, this, _1));
  sub_vel_ = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&PurePursuitController::onVelocity, this, _1));

  // Publishers
  rclcpp::QoS control_qos(rclcpp::KeepLast(1));
  control_qos.transient_local().reliable();
  pub_control_ = create_publisher<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", control_qos);
    
  rclcpp::QoS gear_qos(rclcpp::KeepLast(1));
  gear_qos.transient_local().reliable();
  pub_gear_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", gear_qos);
    
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/pure_pursuit/markers", rclcpp::QoS(rclcpp::KeepLast(1)));

  // Timer for control loop
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / CONTROL_FREQUENCY),
    std::bind(&PurePursuitController::onTimer, this));

  RCLCPP_INFO(get_logger(), "Pure Pursuit Controller initialized");
}

void PurePursuitController::onPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty path");
    return;
  }
  
  current_path_ = msg;
  path_received_ = true;
  closest_path_index_ = 0;  // Reset closest index
  
  RCLCPP_DEBUG(get_logger(), "Received path with %zu poses", msg->poses.size());
}

void PurePursuitController::onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg;
  pose_received_ = true;
}

void PurePursuitController::onVelocity(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  current_velocity_ = msg;
  velocity_received_ = true;
}

void PurePursuitController::onTimer()
{
  if (!path_received_ || !pose_received_ || !velocity_received_) {
    return;
  }

  if (current_path_->poses.empty()) {
    return;
  }

  // Find lookahead point
  geometry_msgs::msg::Point lookahead_point = findLookaheadPoint();
  
  // Calculate steering angle
  double steering_angle = calculateSteeringAngle(lookahead_point);
  
  // Calculate target acceleration
  double target_acceleration = calculateTargetAcceleration();
  
  // Create and publish control command
  auto control_msg = std::make_unique<autoware_control_msgs::msg::Control>();
  
  // Set timestamps
  auto now = get_clock()->now();
  control_msg->stamp = now;
  control_msg->control_time = now;
  
  // Lateral control
  control_msg->lateral.stamp = now;
  control_msg->lateral.control_time = now;
  control_msg->lateral.steering_tire_angle = steering_angle;
  control_msg->lateral.steering_tire_rotation_rate = 0.0;
  control_msg->lateral.is_defined_steering_tire_rotation_rate = false;
  
  // Longitudinal control
  control_msg->longitudinal.stamp = now;
  control_msg->longitudinal.control_time = now;
  control_msg->longitudinal.velocity = 0.0;  // Not used as specified
  control_msg->longitudinal.acceleration = target_acceleration/10;
  control_msg->longitudinal.jerk = 0.0;
  control_msg->longitudinal.is_defined_acceleration = true;
  control_msg->longitudinal.is_defined_jerk = false;
  
  pub_control_->publish(std::move(control_msg));
  
  // Create and publish gear command
  auto gear_msg = std::make_unique<autoware_vehicle_msgs::msg::GearCommand>();
  gear_msg->stamp = now;
  gear_msg->command = determineGearCommand();
  pub_gear_->publish(std::move(gear_msg));
  
  // Publish visualization
  publishVisualization(lookahead_point);
  publishPathVisualization();
  
  // Update previous values
  previous_steering_angle_ = steering_angle;
}

geometry_msgs::msg::Point PurePursuitController::findLookaheadPoint()
{
  // Calculate dynamic lookahead distance based on velocity
  double current_vel = current_velocity_->longitudinal_velocity;
  double lookahead_distance = std::clamp(
    params_.lookahead_gain * current_vel,
    params_.lookahead_distance_min,
    params_.lookahead_distance_max
  );
  
  // Find closest point on path
  size_t closest_idx = findClosestPathIndex();
  closest_path_index_ = closest_idx;
  
  // Find lookahead point
  geometry_msgs::msg::Point current_pos = current_pose_->pose.pose.position;
  
  for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
    double distance = calculateDistance(current_pos, current_path_->poses[i].pose.position);
    
    if (distance >= lookahead_distance) {
      return current_path_->poses[i].pose.position;
    }
  }
  
  // If no point found, return the last point
  return current_path_->poses.back().pose.position;
}

double PurePursuitController::calculateSteeringAngle(const geometry_msgs::msg::Point & lookahead_point)
{
  // Get current vehicle position and orientation
  const auto & current_pos = current_pose_->pose.pose.position;
  const auto & current_orientation = current_pose_->pose.pose.orientation;
  
  // Calculate vehicle heading
  double vehicle_yaw = tf2::getYaw(current_orientation);
  
  // Calculate angle to lookahead point
  double dx = lookahead_point.x - current_pos.x;
  double dy = lookahead_point.y - current_pos.y;
  double angle_to_target = std::atan2(dy, dx);
  
  // Calculate lateral error (cross-track error)
  double alpha = normalizeAngle(angle_to_target - vehicle_yaw);
  
  // Calculate distance to lookahead point
  double lookahead_dist = std::sqrt(dx * dx + dy * dy);
  
  // Pure pursuit steering angle calculation
  double steering_angle = std::atan(2.0 * params_.wheelbase * std::sin(alpha) / lookahead_dist);
  
  // Limit steering angle
  steering_angle = std::clamp(steering_angle, -params_.max_steering_angle, params_.max_steering_angle);
  
  return steering_angle;
}

double PurePursuitController::calculateTargetAcceleration()
{
  double current_vel = current_velocity_->longitudinal_velocity;
  
  // Calculate path curvature at current position
  double curvature = calculatePathCurvature(closest_path_index_);
  
  // Determine target velocity based on curvature
  double target_vel = params_.target_velocity;
  if (std::abs(curvature) > params_.curve_threshold) {
    // Reduce speed in curves
    target_vel *= params_.curve_decel_factor;
  }
  
  // Calculate velocity error
  double velocity_error = target_vel - current_vel;
  
  // Simple proportional controller for acceleration
  double kp_accel = 2.0;  // Proportional gain
  double target_acceleration = kp_accel * velocity_error;
  
  // Limit acceleration
  target_acceleration = std::clamp(
    target_acceleration,
    params_.max_deceleration,
    params_.max_acceleration
  );
  
  // Additional logic for path following
  double distance_to_end = 0.0;
  if (closest_path_index_ < current_path_->poses.size()) {
    for (size_t i = closest_path_index_; i < current_path_->poses.size() - 1; ++i) {
      distance_to_end += calculateDistance(
        current_path_->poses[i].pose.position,
        current_path_->poses[i + 1].pose.position
      );
    }
  }
  
  // Decelerate when approaching path end
  double decel_distance = 10.0;  // Start decelerating 10m before end
  if (distance_to_end < decel_distance && distance_to_end > 0.1) {
    double decel_factor = distance_to_end / decel_distance;
    target_acceleration = std::min(target_acceleration, -1.0 * (1.0 - decel_factor));
  }
  
  return target_acceleration;
}

double PurePursuitController::calculatePathCurvature(size_t index, double window)
{
  if (current_path_->poses.size() < 3 || index >= current_path_->poses.size()) {
    return 0.0;
  }
  
  // Find points within window
  std::vector<geometry_msgs::msg::Point> points;
  double total_distance = 0.0;
  
  // Add points before current index
  for (int i = static_cast<int>(index) - 1; i >= 0 && total_distance < window / 2; --i) {
    points.insert(points.begin(), current_path_->poses[i].pose.position);
    if (i < static_cast<int>(index)) {
      total_distance += calculateDistance(
        current_path_->poses[i].pose.position,
        current_path_->poses[i + 1].pose.position
      );
    }
  }
  
  // Add current point
  points.push_back(current_path_->poses[index].pose.position);
  
  // Add points after current index
  total_distance = 0.0;
  for (size_t i = index + 1; i < current_path_->poses.size() && total_distance < window / 2; ++i) {
    points.push_back(current_path_->poses[i].pose.position);
    total_distance += calculateDistance(
      current_path_->poses[i - 1].pose.position,
      current_path_->poses[i].pose.position
    );
  }
  
  if (points.size() < 3) {
    return 0.0;
  }
  
  // Calculate curvature using three points method
  size_t mid = points.size() / 2;
  if (mid == 0 || mid >= points.size() - 1) {
    return 0.0;
  }
  
  const auto & p1 = points[mid - 1];
  const auto & p2 = points[mid];
  const auto & p3 = points[mid + 1];
  
  double a = calculateDistance(p1, p2);
  double b = calculateDistance(p2, p3);
  double c = calculateDistance(p1, p3);
  
  if (a < 1e-6 || b < 1e-6 || c < 1e-6) {
    return 0.0;
  }
  
  double area = std::abs(
    (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y)
  ) / 2.0;
  
  double curvature = 4.0 * area / (a * b * c);
  
  return curvature;
}

size_t PurePursuitController::findClosestPathIndex()
{
  if (current_path_->poses.empty()) {
    return 0;
  }
  
  const auto & current_pos = current_pose_->pose.pose.position;
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  
  // Start search from previous closest index for efficiency
  size_t start_idx = std::max(0, static_cast<int>(closest_path_index_) - 10);
  size_t end_idx = std::min(current_path_->poses.size(), closest_path_index_ + 50);
  
  for (size_t i = start_idx; i < end_idx; ++i) {
    double distance = calculateDistance(current_pos, current_path_->poses[i].pose.position);
    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = i;
    }
  }
  
  return closest_idx;
}

double PurePursuitController::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double PurePursuitController::calculateDistance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double PurePursuitController::calculateDistanceToPoint(const geometry_msgs::msg::Point & point)
{
  return calculateDistance(current_pose_->pose.pose.position, point);
}

void PurePursuitController::publishVisualization(const geometry_msgs::msg::Point & lookahead_point)
{
  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  
  // Lookahead point marker
  visualization_msgs::msg::Marker lookahead_marker;
  lookahead_marker.header.frame_id = "map";
  lookahead_marker.header.stamp = get_clock()->now();
  lookahead_marker.ns = "pure_pursuit";
  lookahead_marker.id = 0;
  lookahead_marker.type = visualization_msgs::msg::Marker::SPHERE;
  lookahead_marker.action = visualization_msgs::msg::Marker::ADD;
  
  lookahead_marker.pose.position = lookahead_point;
  lookahead_marker.pose.orientation.w = 1.0;
  
  lookahead_marker.scale.x = 1.0;
  lookahead_marker.scale.y = 1.0;
  lookahead_marker.scale.z = 1.0;
  
  lookahead_marker.color.r = 1.0;
  lookahead_marker.color.g = 0.0;
  lookahead_marker.color.b = 0.0;
  lookahead_marker.color.a = 1.0;
  
  marker_array->markers.push_back(lookahead_marker);
  
  // Vehicle position marker
  visualization_msgs::msg::Marker vehicle_marker;
  vehicle_marker.header.frame_id = "map";
  vehicle_marker.header.stamp = get_clock()->now();
  vehicle_marker.ns = "pure_pursuit";
  vehicle_marker.id = 1;
  vehicle_marker.type = visualization_msgs::msg::Marker::ARROW;
  vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
  
  vehicle_marker.pose = current_pose_->pose.pose;
  
  vehicle_marker.scale.x = 2.0;
  vehicle_marker.scale.y = 0.5;
  vehicle_marker.scale.z = 0.5;
  
  vehicle_marker.color.r = 0.0;
  vehicle_marker.color.g = 1.0;
  vehicle_marker.color.b = 0.0;
  vehicle_marker.color.a = 1.0;
  
  marker_array->markers.push_back(vehicle_marker);
  
  // Lookahead distance circle
  visualization_msgs::msg::Marker circle_marker;
  circle_marker.header.frame_id = "map";
  circle_marker.header.stamp = get_clock()->now();
  circle_marker.ns = "pure_pursuit";
  circle_marker.id = 2;
  circle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  circle_marker.action = visualization_msgs::msg::Marker::ADD;
  
  circle_marker.pose.position = current_pose_->pose.pose.position;
  circle_marker.pose.orientation.w = 1.0;
  
  double lookahead_dist = calculateDistanceToPoint(lookahead_point);
  circle_marker.scale.x = lookahead_dist * 2.0;
  circle_marker.scale.y = lookahead_dist * 2.0;
  circle_marker.scale.z = 0.1;
  
  circle_marker.color.r = 0.0;
  circle_marker.color.g = 0.0;
  circle_marker.color.b = 1.0;
  circle_marker.color.a = 0.3;
  
  marker_array->markers.push_back(circle_marker);
  
  pub_markers_->publish(std::move(marker_array));
}

void PurePursuitController::publishPathVisualization()
{
  if (!current_path_ || current_path_->poses.empty()) {
    return;
  }
  
  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  
  // Path line strip
  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = "map";
  path_marker.header.stamp = get_clock()->now();
  path_marker.ns = "pure_pursuit_path";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  
  path_marker.pose.orientation.w = 1.0;
  
  path_marker.scale.x = 0.2;
  
  path_marker.color.r = 1.0;
  path_marker.color.g = 1.0;
  path_marker.color.b = 0.0;
  path_marker.color.a = 0.8;
  
  for (const auto & pose : current_path_->poses) {
    path_marker.points.push_back(pose.pose.position);
  }
  
  marker_array->markers.push_back(path_marker);
  
  pub_markers_->publish(std::move(marker_array));
}

uint8_t PurePursuitController::determineGearCommand()
{
  // For forward motion, use DRIVE
  // This could be enhanced with reverse logic if needed for parking/backing up
  double target_acceleration = calculateTargetAcceleration();
  double current_vel = current_velocity_->longitudinal_velocity;
  
  // If we need to go backwards (for parking scenarios)
  if (target_acceleration < -4.0 && std::abs(current_vel) < 0.5) {
    return autoware_vehicle_msgs::msg::GearCommand::REVERSE;
  }
  
  // For normal forward driving
  return autoware_vehicle_msgs::msg::GearCommand::DRIVE;
}

}  // namespace awsim_controller