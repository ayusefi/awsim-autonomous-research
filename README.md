# AWSIM Autonomous Research

My journey into autonomous driving development using the AWSIM simulator. This repository contains SLAM algorithms, sensor processing systems, and autonomous vehicle research implementations.

## 🚗 What's Implemented

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
├── awsim_localization/    # NDT-based localization with pre-built maps
├── awsim_sensor_logger/   # Multi-sensor data logging and analysis  
├── awsim_bringup/         # Launch files and system integration
└── config/                # Parameter files and configurations
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


## 📚 Documentation

- **[NDT Localization Guide](src/awsim_localization/README.md)** - Real-time positioning with demonstration video
- **[Sensor Logger Guide](src/awsim_sensor_logger/README.md)** - Multi-sensor data collection and analysis

## 🧪 Testing & Validation

### NDT Localization Testing
```bash
# Launch localization and monitor performance
ros2 launch awsim_localization awsim_localization.launch.py
ros2 topic hz /localization/pose_with_covariance
ros2 topic echo /localization/status

# Check NDT fitness scores (lower is better)
ros2 topic echo /localization/markers
```

## 📈 Future Research Directions

- **Advanced localization** with multi-sensor fusion (LiDAR + GNSS + IMU)
- **Deep learning integration** for semantic mapping
- **Multi-sensor fusion** with camera and IMU data
- **Dynamic environment handling** for moving objects

⭐ **If you find this project helpful for your autonomous driving research, please give it a star!**

---
