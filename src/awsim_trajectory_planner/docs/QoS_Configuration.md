# QoS Configuration for AWSIM Trajectory Planner

## Overview

This document describes the Quality of Service (QoS) configuration for the trajectory planner node to ensure compatibility with the AWSIM simulation environment.

## Topic QoS Profiles

### Input Topics

| Topic | Message Type | Reliability | History Depth | Notes |
|-------|-------------|-------------|---------------|-------|
| `/planning/path` | `nav_msgs/msg/Path` | RELIABLE | KEEP_LAST(10) | Global path from path planner |
| `/localization/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RELIABLE | KEEP_LAST(10) | Vehicle pose from localization |
| `/sensing/lidar/top/pointcloud_raw` | `sensor_msgs/msg/PointCloud2` | BEST_EFFORT | KEEP_LAST(5) | LiDAR point cloud data |

### Output Topics

| Topic | Message Type | Reliability | History Depth | Notes |
|-------|-------------|-------------|---------------|-------|
| `/planning/trajectory_planner/local_trajectory` | `nav_msgs/msg/Path` | RELIABLE | KEEP_LAST(10) | Selected trajectory |
| `/planning/trajectory_planner/candidate_trajectories` | `nav_msgs/msg/Path` | RELIABLE | KEEP_LAST(10) | All candidate trajectories |
| `/planning/trajectory_planner/visualization` | `visualization_msgs/msg/MarkerArray` | RELIABLE | KEEP_LAST(10) | Visualization markers |

## Key QoS Considerations

### LiDAR Point Cloud (BEST_EFFORT)
- **Why BEST_EFFORT**: LiDAR data is high-frequency and large in size. Missing occasional frames is acceptable for trajectory planning as the next frame will arrive soon.
- **Benefit**: Reduced network overhead and better real-time performance.
- **Subscriber Configuration**: Matches the publisher's BEST_EFFORT reliability to ensure data flow.

### Path and Pose Data (RELIABLE)
- **Why RELIABLE**: Global path and pose information are critical for safe trajectory planning.
- **Benefit**: Ensures no data loss for safety-critical information.
- **Depth**: Uses KEEP_LAST(10) to maintain sufficient history while avoiding excessive buffering.

## Verification Commands

To verify QoS compatibility, you can check the topic information:

```bash
# Check global path topic
ros2 topic info -v /planning/path

# Check localization topic  
ros2 topic info -v /localization/pose_with_covariance

# Check LiDAR topic
ros2 topic info -v /sensing/lidar/top/pointcloud_raw

# Check trajectory planner outputs
ros2 topic info -v /planning/trajectory_planner/local_trajectory
ros2 topic info -v /planning/trajectory_planner/candidate_trajectories
ros2 topic info -v /planning/trajectory_planner/visualization
```

## Troubleshooting

### Common QoS Mismatches

1. **No data received from LiDAR**: 
   - Check if subscriber is using BEST_EFFORT reliability
   - Verify topic name matches exactly

2. **Intermittent path/pose data**:
   - Ensure RELIABLE reliability for both publisher and subscriber
   - Check network connectivity

3. **High latency**:
   - Consider reducing history depth for high-frequency topics
   - Monitor system resources

### Compatibility Matrix

| Publisher QoS | Subscriber QoS | Compatible |
|---------------|----------------|-----------|
| RELIABLE | RELIABLE | ✅ Yes |
| RELIABLE | BEST_EFFORT | ✅ Yes |
| BEST_EFFORT | RELIABLE | ❌ No |
| BEST_EFFORT | BEST_EFFORT | ✅ Yes |

## Implementation Details

The QoS profiles are configured in the `initializeSubscribersAndPublishers()` method:

```cpp
// For reliable communication (path, pose)
auto reliable_qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

// For best effort communication (LiDAR)  
auto best_effort_qos = rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
```

This ensures optimal performance while maintaining data integrity for safety-critical information.
