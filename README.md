# AWSIM Autonomous Research

My journey into autonomous driving development using the AWSIM simulator. This repo contains the experiments and algorithms I'm building to understand and implement autonomous vehicle systems.

## What I'm Working On

Currently focused on getting familiar with AWSIM's sensor data and building a solid foundation for autonomous driving algorithms. The main areas I'm exploring:

- Multi-sensor data integration from AWSIM
- SLAM algorithms for mapping and localization  
- Real-time perception systems
- Vehicle control strategies

## Repository Structure

```
src/
├── awsim_sensor_logger/    # Sensor data logging and analysis
├── awsim_bringup/         # Launch files and configurations
└── (more packages coming soon)
```

## Getting Started

You'll need ROS 2 Humble and the AWSIM simulator to run these experiments.

```bash
# Install dependencies
sudo apt install ros-humble-desktop-full
sudo apt install python3-colcon-common-extensions

# Build the workspace
colcon build --symlink-install
source install/setup.bash

# Run the sensor logger
ros2 launch awsim_bringup sensor_logger.launch.py
```

Make sure to start AWSIM first before running any nodes.

## Current Progress

- [x] Basic sensor data logging from AWSIM
- [ ] Sensor synchronization and calibration
- [ ] 2D SLAM implementation
- [ ] Object detection pipeline

## Tech Stack

- ROS 2 Humble
- C++17
- AWSIM v1.3.1

This is an ongoing learning project as I dive deeper into autonomous driving technology.

⭐ If you find this project helpful or interesting, please give it a star!
