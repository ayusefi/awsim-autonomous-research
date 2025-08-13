# AWSIM Autonomous Research

My journey into autonomous driving development using the AWSIM simulator. This repository contains path planning systems, sensor processing, trajectory planning, vehicle control, and autonomous vehicle research implementations.

## 🎯 Recent Updates

- **✅ Pure Pursuit Controller**: Implemented adaptive Pure Pursuit algorithm with velocity-based lookahead distance
- **✅ MPC Removal**: Cleaned up repository by removing MPC controller dependencies and focusing on Pure Pursuit
- **✅ Trajectory Planning**: Added local trajectory planner with dynamic obstacle avoidance
- **✅ Complete Integration**: Full autonomous driving stack with perception, planning, and control
- **✅ Enhanced Documentation**: Comprehensive guides with demo videos for all components

## 🚗 What's Implemented

### Pure Pursuit Controller ✅
- **Adaptive lookahead distance** based on current velocity for smooth path following
- **Real-time steering control** with safety limits and smoothing
- **Velocity management** with curve-aware speed adjustments
- **ROS2 integration** with Autoware control messages
- **Comprehensive visualization** for debugging and monitoring

### Multi-Algorithm Path Planning System ✅
- **Three distinct planning algorithms**: A* (grid-based), RRT* (sampling-based), Route (lane-following)
- **Lanelet2 HD map integration** with traffic rules compliance
- **HD map processing**: lanelets with cached coordinate points
- **Live visualization** in RViz2 with path display and waypoint markers

### Trajectory Planning System ✅
- **Local trajectory generation** with dynamic obstacle avoidance
- **Smooth trajectory optimization** using spline interpolation
- **Real-time replanning** for dynamic environments
- **Integration with path planning** for complete navigation pipeline

### NDT Localization System ✅
- **Real-time vehicle positioning** using Normal Distributions Transform  
- **Centimeter-level accuracy** with pre-built map matching
- **AWSIM integration** with proper sensor transforms and timing
- **Live visualization** in RViz2 with pose tracking and fitness scores
- **Automatic initialization** from ground truth for seamless startup

### Sensor Data Processing ✅
- **Multi-sensor data logging** from AWSIM
- **Synchronized data capture** (LiDAR, cameras, IMU)
- **Real-time data analysis** and visualization
- **Point cloud filtering** and preprocessing

### Ground Filter Package ✅
- **Ground segmentation** from LiDAR point clouds using RANSAC and grid-based refinement
- **Separates ground and non-ground points** for downstream perception modules
- **Highly configurable** for different terrain types

### Object Tracker Package ✅
- **Real-time multi-object tracking** from LiDAR point clouds
- **DBSCAN clustering** for object detection
- **Kalman filter-based tracking** with occlusion handling
- **Smooth, robust trajectories** for vehicles, pedestrians, and cyclists

## Repository Structure
```
src/
├── awsim_controller/        # Pure Pursuit controller for autonomous vehicle control
├── awsim_path_planner/      # Multi-algorithm path planning with HD map integration
├── awsim_trajectory_planner/ # Local trajectory planning with obstacle avoidance
├── awsim_localization/      # NDT-based localization with pre-built maps
├── awsim_sensor_logger/     # Multi-sensor data logging and analysis  
├── awsim_object_tracker/    # Real-time multi-object tracking from LiDAR point clouds
├── ground_filter/           # Ground point filtering for LiDAR data
├── multi_object_tracker/    # Advanced multi-object tracking algorithms
├── multi_object_tracker_msgs/ # Custom message definitions for object tracking
├── awsim_bringup/           # Launch files and system integration
└── shinjuku_map/            # Lanelet2 HD map package

config/                      # Global parameter files and configurations
launch/                      # Top-level launch files for different scenarios
AWSIM_v1.3.1/               # AWSIM simulator binary

```

## 🚀 Quick Start

### Prerequisites
- ROS 2 Humble
- AWSIM simulator
- PCL library for point cloud processing

### Installation
```bash
# Clone and build
git clone https://github.com/ayusefi/awsim-autonomous-research.git
cd awsim-autonomous-research
colcon build
source install/setup.bash
```

## 🎮 Key Features

- 🎯 **Complete Autonomous Stack**: Perception → Planning → Control
- 🗺️ **HD Map Integration**: Lanelet2-based high-definition mapping
- 🚗 **Real-time Control**: Pure Pursuit controller with adaptive behavior  
- 📡 **Multi-sensor Fusion**: LiDAR, cameras, IMU integration
- 🎥 **Live Visualization**: RViz2 integration with real-time monitoring
- 📊 **Data Logging**: Comprehensive sensor data collection and analysis


### Launch Pure Pursuit Controller
```bash
# Start AWSIM and localization first, then launch Pure Pursuit controller
ros2 launch awsim_controller pure_pursuit.launch.py

# With custom parameters
ros2 launch awsim_controller pure_pursuit.launch.py \
    lookahead_distance_min:=3.0 \
    target_velocity:=5.0
```

### Launch NDT Localization
```bash
# Start AWSIM first, then launch localization with RViz
ros2 launch awsim_localization awsim_localization.launch.py

# For custom map or parameters
ros2 launch awsim_localization awsim_localization.launch.py \
    map_path:=/path/to/your/map.pcd \
    ndt_resolution:=1.5
```

### Launch Path Planning
```bash
# Start AWSIM and localization first, then launch path planner
ros2 launch awsim_path_planner path_planner.launch.py

# Choose specific algorithm (default: route)
ros2 launch awsim_path_planner path_planner.launch.py algorithm:=astar
ros2 launch awsim_path_planner path_planner.launch.py algorithm:=rrt_star
ros2 launch awsim_path_planner path_planner.launch.py algorithm:=route
```

### Launch Ground Filter
```bash
# Start AWSIM, then launch the ground filter node
ros2 launch ground_filter ground_filter.launch.py

# With custom parameters
ros2 launch ground_filter ground_filter.launch.py distance_threshold:=0.1 grid_size:=0.5
```

### Launch Object Tracker
```bash
# Start AWSIM, then launch the object tracker node
ros2 launch awsim_object_tracker tracker.launch.py
```

### Launch Trajectory Planner
```bash
# Start AWSIM and localization, then launch trajectory planner
ros2 launch awsim_trajectory_planner trajectory_planner.launch.py

# For dynamic obstacle avoidance mode
ros2 launch awsim_trajectory_planner trajectory_planner.launch.py enable_obstacles:=true
```

### Complete System Launch
```bash
# Launch the complete autonomous driving stack
ros2 launch awsim_bringup awsim_full.launch.py
```



## 📚 Documentation

- **[Pure Pursuit Controller Guide](src/awsim_controller/README.md)** - Adaptive Pure Pursuit controller with demo video
- **[Path Planning Guide](src/awsim_path_planner/README.md)** - Multi-algorithm path planning with HD map integration
- **[Trajectory Planning Guide](src/awsim_trajectory_planner/README.md)** - Local trajectory planning with obstacle avoidance
- **[NDT Localization Guide](src/awsim_localization/README.md)** - Real-time positioning with demonstration video
- **[Sensor Logger Guide](src/awsim_sensor_logger/README.md)** - Multi-sensor data collection and analysis
- **[Object Tracker Guide](src/awsim_object_tracker/README.md)** - Real-time multi-object tracking from LiDAR point clouds
- **[Ground Filter Guide](src/ground_filter/README.md)** - Ground point filtering for LiDAR data


## 📈 Future Research Directions

- **Behavior planning** with traffic light detection and lane changing
- **Advanced MPC controllers** for improved vehicle dynamics modeling
- **Model predictive path integral (MPPI)** for robust control in uncertain environments
- **Multi-sensor fusion** for robust localization (LiDAR + GNSS + IMU)
- **Deep learning integration** for semantic mapping and planning
- **Dynamic environment handling** for moving objects and pedestrians
- **Real-world deployment** and hardware-in-the-loop testing

⭐ **If you find this project helpful for your autonomous driving research, please give it a star!**

---
