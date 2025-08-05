# AWSIM Autonomous Research

My journey into autonomous driving development using the AWSIM simulator. This repository contains path planning systems, sensor processing, and autonomous vehicle research implementations.

## 🚗 What's Implemented

### Multi-Algorithm Path Planning System ✅
- **Three distinct planning algorithms**: A* (grid-based), RRT* (sampling-based), Route (lane-following)
- **Lanelet2 HD map integration** with traffic rules compliance
- **HD map processing**: lanelets with cached coordinate points
- **Live visualization** in RViz2 with path display and waypoint markers

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

## Repository Structure

```
src/
├── awsim_path_planner/    # Multi-algorithm path planning with HD map integration
├── awsim_localization/    # NDT-based localization with pre-built maps
├── awsim_sensor_logger/   # Multi-sensor data logging and analysis  
├── awsim_bringup/         # Launch files and system integration
└── config/                # Parameter files and configurations

shinjuku_map/              # HD map data with Lanelet2 format
```

## 🚀 Quick Start

### Prerequisites
- ROS 2 Humble
- AWSIM simulator
- PCL library for point cloud processing

### Installation
```bash
# Clone and build
git clone <this-repo>
cd awsim-autonomous-research
colcon build
source install/setup.bash
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


## 📚 Documentation

- **[Path Planning Guide](src/awsim_path_planner/README.md)** - Multi-algorithm path planning with HD map integration
- **[NDT Localization Guide](src/awsim_localization/README.md)** - Real-time positioning with demonstration video
- **[Sensor Logger Guide](src/awsim_sensor_logger/README.md)** - Multi-sensor data collection and analysis


## 📈 Future Research Directions

- **Behavior planning** with traffic light detection and lane changing
- **Advanced path optimization** with dynamic obstacle avoidance
- **Multi-sensor fusion** for robust localization (LiDAR + GNSS + IMU)
- **Deep learning integration** for semantic mapping and planning
- **Dynamic environment handling** for moving objects and pedestrians

⭐ **If you find this project helpful for your autonomous driving research, please give it a star!**

---
