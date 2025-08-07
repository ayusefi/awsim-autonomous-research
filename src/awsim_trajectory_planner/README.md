# AWSIM Trajectory Planner

[![Watch the performance example](https://img.youtube.com/vi/7OejdtVh5UU/0.jpg)](https://www.youtube.com/watch?v=7OejdtVh5UU)

The AWSIM Trajectory Planner is a local trajectory generation and obstacle avoidance package designed for autonomous vehicles in the AWSIM simulation environment.

## Overview

This package generates smooth, dynamically feasible, and collision-free trajectories for autonomous vehicles. It processes a global path and real-time sensor data to produce safe local trajectories for vehicle control.

## Features

- **Trajectory Generation**: Generates multiple candidate trajectories with lateral offsets.
- **Obstacle Detection**: Real-time detection using LiDAR point clouds.
- **Cost-based Selection**: Optimizes for safety, path adherence, smoothness, and comfort.
- **Kinematic Constraints**: Respects vehicle dynamics.
- **ROS 2 Integration**: Fully compatible with ROS 2 nodes and message interfaces.
- **Visualization**: Real-time trajectory visualization in RViz.

## Core Components

1. **TrajectoryPlannerNode**: Coordinates the planning pipeline.
2. **TrajectoryGenerator**: Creates candidate trajectories using a rollout method.
3. **ObstacleDetector**: Detects obstacles from sensor data.
4. **CostEvaluator**: Selects the optimal trajectory based on cost evaluation.

## Installation

```bash
# Clone the repository
cd ~/your_workspace/src
git clone <repository_url>

# Build the package
cd ~/your_workspace
colcon build --packages-select awsim_trajectory_planner

# Source the workspace
source install/setup.bash
```

## Configuration

Key parameters can be configured in `config/trajectory_planner_params.yaml`. These include trajectory generation settings, vehicle constraints, cost function weights, and safety margins.

## Usage

Launch the trajectory planner with:

```bash
ros2 launch awsim_trajectory_planner trajectory_planner.launch.py
```

## Future Enhancements

- Dynamic obstacle prediction.
- Integrated velocity profile optimization.
- Advanced vehicle dynamics models.
- Machine learning-based cost functions.
- Real-time parameter adaptation.

