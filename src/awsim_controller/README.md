# awsim_controller

A Pure Pursuit controller node for AWSIM that subscribes to a planning path and outputs Autoware control commands.

## Demo Video

🎥 **Watch the Pure Pursuit Controller in Action:**
[![Pure Pursuit Controller Demo](https://img.youtube.com/vi/uJs0FNT0_KQ/0.jpg)](https://www.youtube.com/watch?v=uJs0FNT0_KQ)

*Click the image above to see a demonstration of the Pure Pursuit controller navigating through the AWSIM simulator environment.*

## Overview

This package implements a Pure Pursuit lateral control algorithm for autonomous vehicle path following. The controller uses adaptive lookahead distance based on current velocity and provides smooth steering and velocity control.

## Topics

### Subscribed Topics:
- `/planning/path` (nav_msgs/Path) - Reference path to follow
- `/localization/pose_with_covariance` (geometry_msgs/PoseWithCovarianceStamped) - Current vehicle pose
- `/vehicle/status/velocity_status` (autoware_vehicle_msgs/VelocityReport) - Current vehicle velocity

### Published Topics:
- `/control/command/control_cmd` (autoware_control_msgs/Control) - Control commands (steering, acceleration)
- `/control/command/gear_cmd` (autoware_vehicle_msgs/GearCommand) - Gear commands
- `/pure_pursuit/markers` (visualization_msgs/MarkerArray) - Visualization markers for debugging

## Parameters

### Core Parameters:
- `lookahead_distance_min` (double, default 2.0) - Minimum lookahead distance in meters
- `lookahead_distance_max` (double, default 18.0) - Maximum lookahead distance in meters
- `lookahead_gain` (double, default 1.5) - Gain for velocity-based lookahead calculation
- `wheelbase` (double, default 2.7) - Vehicle wheelbase in meters
- `max_steering_angle` (double, default 0.6) - Maximum steering angle in radians

### Velocity Parameters:
- `target_velocity` (double, default 4.0) - Target velocity in m/s
- `max_acceleration` (double, default 2.0) - Maximum acceleration in m/s²
- `max_deceleration` (double, default -2.5) - Maximum deceleration in m/s²

### Advanced Parameters:
- `curve_decel_factor` (double) - Factor for deceleration in curves
- `curve_threshold` (double) - Threshold for curve detection
- `velocity_tolerance` (double) - Velocity tolerance for control
- `path_tolerance` (double) - Path following tolerance

## Launch

Start the Pure Pursuit controller:
```bash
ros2 launch awsim_controller pure_pursuit.launch.py
```

## Algorithm

The Pure Pursuit algorithm:
1. Finds the closest point on the reference path
2. Calculates adaptive lookahead distance: `Ld = lookahead_distance_min + lookahead_gain * velocity`
3. Finds the lookahead point on the path
4. Computes steering angle using Pure Pursuit geometry
5. Applies velocity control based on path curvature and target speed

## Configuration

Parameters can be configured in `config/pure_pursuit_params.yaml`. The controller supports dynamic parameter updates for real-time tuning.

## Notes

- The controller automatically handles path switching and recovery
- Visualization markers are published for debugging and monitoring
- Steering commands are smoothed to avoid abrupt changes
- The algorithm includes safety checks for maximum steering and acceleration limits
