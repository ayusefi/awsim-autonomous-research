# AWSIM NDT Localization - Real-Time Vehicle Positioning for Autonomous Driving

> Achieve centimeter-level vehicle localization in AWSIM using NDT scan matching - your autonomous vehicle knows exactly where it is at all times!

## See It In Action

[![NDT Localization Demo](https://img.youtube.com/vi/b5t8RxWgBh8/0.jpg)](https://youtu.be/b5t8RxWgBh8)

*Watch the NDT localization system in action: real-time pose estimation and scan matching visualization in AWSIM simulation environment.*

## Quick Start

Ready to get localized? Here's how to run it:

```bash
# Step 1: Launch AWSIM simulation first
./AWSIM.x86_64

# Step 2: Start NDT localization with RViz visualization
ros2 launch awsim_localization awsim_localization.launch.py

# Step 3: Give initial pose from rviz and see the vehicle localized.
```

## What You Get

After launching, you'll see:

- 🎯 **Real-time localization** at 10 Hz with <10cm accuracy
- 📊 **Live RViz visualization** showing scan matching in action  
- 🗺️ **Centered map coordinates** eliminating RViz precision issues
- ⚡ **NDT fitness scores** indicating localization quality

### Your Localization Pipeline Looks Like This

```
LiDAR Scan → Point Cloud Filtering → NDT Scan Matching → Pose Estimation → TF Broadcasting
     ↓              ↓                      ↓                 ↓              ↓
   Raw data    Noise removal         Map alignment    Position update   Navigation ready
```

## How It Works

### The NDT Magic ✨

```cpp
// The core NDT alignment happens here
ndt_->align(*aligned_cloud, initial_guess);
fitness_score_ = ndt_->getFitnessScore();

// Result: Your vehicle knows its exact position!
current_pose_.pose.pose.position.x = transformation(0, 3);
current_pose_.pose.pose.position.y = transformation(1, 3);
```

### What Gets Processed
- **Input**: LiDAR point clouds from `/sensing/lidar/top/pointcloud_raw`
- **Reference**: Pre-built Shinjuku map (3M+ points → centered for precision)
- **Output**: 6DOF pose estimates with sub-10cm accuracy
- **Frequency**: Real-time at 10+ Hz depending on point cloud density

## Installation & Dependencies

You'll need these essentials:

```bash
# ROS2 dependencies
sudo apt install ros-humble-pcl-ros \
                 ros-humble-pcl-conversions \
                 ros-humble-tf2-geometry-msgs \
                 ros-humble-visualization-msgs

# Build the package
cd your_workspace
colcon build --packages-select awsim_localization
source install/setup.bash
```

**Requirements:**
- ROS2 (Humble)
- PCL (Point Cloud Library) 1.12+
- Eigen3 3.4+
- OpenMP for parallel processing
- AWSIM v1.3.1+ for simulation

## NDT OMP Library Credits

This implementation uses the high-performance NDT OMP (OpenMP) library for fast scan matching:

- **[ndt_omp](https://github.com/koide3/ndt_omp)** by Kenji Koide - The core NDT OpenMP implementation
- **[ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2)** by rsasaki0109 - ROS2 integration and examples


## Topics

### Subscribed Topics

- `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/PointCloud2) - Input LiDAR point cloud
- `/awsim/ground_truth/localization/kinematic_state` (nav_msgs/Odometry) - Ground truth pose for initialization (optional)
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) - Manual pose initialization

### Published Topics

- `/localization/pose_with_covariance` (geometry_msgs/PoseWithCovarianceStamped) - Estimated pose with covariance
- `/localization/kinematic_state` (nav_msgs/Odometry) - Estimated pose as odometry message
- `/localization/map` (sensor_msgs/PointCloud2) - Sparse map for visualization
- `/localization/markers` (visualization_msgs/MarkerArray) - Visualization markers
- `/localization/status` (std_msgs/String) - Localization status messages
- `/localization/ground_truth_pose` (geometry_msgs/PoseWithCovarianceStamped) - Ground truth for comparison

## Parameters

### Core Parameters

- `map_path` (string, default: path to shinjuku map) - Path to point cloud map file (.pcd)
- `map_frame` (string, default: "map") - Map coordinate frame
- `base_link_frame` (string, default: "base_link") - Vehicle base frame
- `odom_frame` (string, default: "odom") - Odometry frame

### NDT Parameters

- `ndt_resolution` (double, default: 2.0) - NDT voxel resolution in meters
- `ndt_max_iterations` (int, default: 35) - Maximum NDT iterations
- `ndt_transformation_epsilon` (double, default: 0.01) - Transformation convergence threshold
- `ndt_step_size` (double, default: 0.1) - NDT optimization step size
- `ndt_num_threads` (int, default: 4) - Number of threads for NDT processing

### Filtering Parameters

- `voxel_leaf_size` (double, default: 0.5) - Voxel grid filter leaf size
- `lidar_max_range` (double, default: 100.0) - Maximum LiDAR range
- `lidar_min_range` (double, default: 1.0) - Minimum LiDAR range

### Localization Parameters

- `scan_matching_score_threshold` (double, default: 2000.0) - Fitness score threshold
- `publish_tf` (bool, default: true) - Whether to publish TF transforms
- `use_ground_truth_init` (bool, default: true) - Initialize from ground truth

## Usage

### Basic Launch

```bash
ros2 launch awsim_localization awsim_localization.launch.py
```

### With Custom Parameters

```bash
ros2 launch awsim_localization awsim_localization.launch.py \
    map_path:=/path/to/your/map.pcd \
    ndt_resolution:=1.5 \
    use_ground_truth_init:=false
```

### Running with AWSIM

1. Start AWSIM simulation
2. Launch the localization node:
   ```bash
   ros2 launch awsim_localization awsim_localization.launch.py
   ```
3. Open RViz for visualization:
   ```bash
   rviz2 -d $(ros2 pkg prefix awsim_localization)/share/awsim_localization/rviz/awsim_localization.rviz
   ```

## Visualization

The package includes an RViz configuration that shows:
- Point cloud map (sparse visualization)
- Current LiDAR scan
- Estimated vehicle pose (red arrow)
- Ground truth pose (green arrow)
- NDT fitness score display
- Localization status markers

## Performance

The package is optimized for real-time performance:
- OpenMP parallelization for NDT computations
- Efficient point cloud filtering
- Sparse map publishing for visualization
- Configurable processing parameters

## Troubleshooting

### Common Issues

1. **Map not loading**: Check `map_path` parameter and file permissions
2. **No pose initialization**: Enable `use_ground_truth_init` or use `/initialpose` topic
3. **Poor localization**: Adjust `ndt_resolution` and `scan_matching_score_threshold`
4. **High CPU usage**: Reduce `ndt_num_threads` or increase filtering parameters

### Status Messages

- `SCAN_MATCHING_OK`: Successful localization
- `SCAN_MATCHING_FAILED_CONVERGENCE`: NDT didn't converge
- `SCAN_MATCHING_FAILED_SCORE`: Fitness score too high

## License

MIT License - see LICENSE file for details.

## Credits & References

### Core Libraries Used
- **[ndt_omp](https://github.com/koide3/ndt_omp)** - High-performance NDT implementation with OpenMP parallelization by Kenji Koide
- **[ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2)** - ROS2 integration and reference implementation by rsasaki0109
- **[AWSIM](https://github.com/tier4/AWSIM)** - Autonomous vehicle simulation environment by Tier IV

### Academic References
- Biber, Peter, and Wolfgang Straßer. "The normal distributions transform: A new approach to laser scan matching." *Proceedings 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2003)*. Vol. 3. IEEE, 2003.

### Implementation Notes
This package adapts the NDT OMP library specifically for AWSIM localization scenarios, with optimizations for:
- Real-time performance in simulation environments
- Coordinate system handling for large-scale maps  
- Integration with AWSIM's sensor data pipeline
- RViz visualization and debugging tools

**Acknowledgments:** Special thanks to Kenji Koide for the excellent NDT OMP implementation and rsasaki0109 for the ROS2 integration work that made this package possible.
