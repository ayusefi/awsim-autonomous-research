# AWSIM Trajectory Planner

Local trajectory generation and obstacle avoidance for AWSIM autonomous research.

## Overview

This package implements a local trajectory planner that generates smooth, dynamically feasible, and collision-free trajectories for autonomous vehicles. It takes a global path and real-time sensor data as input and produces safe local trajectories for vehicle control.

## Features

- **Trajectory Generation**: Multiple candidate trajectory rollouts with lateral offsets
- **Obstacle Detection**: Real-time obstacle detection from LiDAR point clouds  
- **Cost-based Selection**: Multi-criteria optimization (safety, path adherence, smoothness, comfort)
- **Kinematic Constraints**: Respects vehicle dynamics limitations
- **ROS 2 Integration**: Full ROS 2 node with standard message interfaces
- **Visualization**: Real-time trajectory visualization in RViz

## Architecture

### Core Components

1. **TrajectoryPlannerNode**: Main ROS 2 node coordinating the planning pipeline
2. **TrajectoryGenerator**: Generates candidate trajectories using rollout method
3. **ObstacleDetector**: Processes point cloud data to detect obstacles
4. **CostEvaluator**: Evaluates and selects optimal trajectory

### Algorithm Overview

1. **Input Processing**: Receives global path, vehicle state, and sensor data
2. **Trajectory Generation**: Creates multiple candidate trajectories with different lateral offsets
3. **Obstacle Detection**: Extracts obstacles from LiDAR point cloud using clustering
4. **Cost Evaluation**: Computes costs for obstacle avoidance, path deviation, smoothness, and comfort
5. **Trajectory Selection**: Selects the trajectory with minimum total cost
6. **Output Publishing**: Publishes selected trajectory and visualization markers

## Dependencies

- ROS 2 Humble
- rclcpp
- nav_msgs
- geometry_msgs
- sensor_msgs
- visualization_msgs
- tf2 & tf2_ros
- tf2_geometry_msgs

## Installation

```bash
# Clone the repository (if not already done)
cd ~/your_workspace/src

# Build the package
cd ~/your_workspace
colcon build --packages-select awsim_trajectory_planner

# Source the workspace
source install/setup.bash
```

## Configuration

### Parameters

Key parameters can be configured in `config/trajectory_planner_params.yaml`:

#### Trajectory Generation
- `prediction_horizon`: Time horizon for trajectory generation (default: 5.0s)
- `time_step`: Time discretization step (default: 0.1s)
- `num_rollouts`: Number of candidate trajectories (default: 7)
- `max_lateral_offset`: Maximum lateral deviation from global path (default: 3.0m)

#### Vehicle Constraints
- `max_velocity`: Maximum vehicle velocity (default: 15.0 m/s)
- `max_acceleration`: Maximum acceleration (default: 2.0 m/s²)
- `max_deceleration`: Maximum deceleration (default: -4.0 m/s²)
- `max_curvature`: Maximum curvature (default: 0.2 1/m)

#### Cost Function Weights
- `obstacle_cost_weight`: Weight for obstacle avoidance (default: 1000.0)
- `path_deviation_weight`: Weight for staying close to global path (default: 10.0)
- `smoothness_weight`: Weight for trajectory smoothness (default: 1.0)
- `comfort_weight`: Weight for passenger comfort (default: 5.0)

#### Safety
- `safety_margin`: Minimum distance to obstacles (default: 1.5m)
- `collision_check_resolution`: Resolution for collision checking (default: 0.2m)

## Usage

### Basic Launch

```bash
# Launch the trajectory planner
ros2 launch awsim_trajectory_planner trajectory_planner.launch.py


## Tuning and Optimization

### Performance Tuning

- **Planning Frequency**: Adjust the timer frequency in the node (default: 10 Hz)
- **Trajectory Resolution**: Balance `time_step` and `prediction_horizon` for performance vs. accuracy
- **Number of Rollouts**: More rollouts provide better solutions but increase computation time

### Cost Function Tuning

Start with default weights and adjust based on observed behavior:

1. **High Obstacle Avoidance**: Increase `obstacle_cost_weight` if vehicle gets too close to obstacles
2. **Better Path Following**: Increase `path_deviation_weight` for tighter path following
3. **Smoother Trajectories**: Increase `smoothness_weight` for less jerky motion
4. **Passenger Comfort**: Increase `comfort_weight` to reduce accelerations

### Safety Tuning

- Increase `safety_margin` in crowded environments
- Adjust `max_velocity` and acceleration limits based on vehicle capabilities
- Tune `collision_check_resolution` for accuracy vs. performance trade-off

## Troubleshooting

### Common Issues

1. **No Trajectories Generated**
   - Check if global path is being received
   - Verify vehicle state (odometry) is available
   - Check parameter bounds and constraints

2. **Poor Obstacle Avoidance**
   - Verify LiDAR point cloud data
   - Check obstacle detection parameters
   - Increase `obstacle_cost_weight`

3. **Jerky Motion**
   - Increase `smoothness_weight`
   - Reduce `time_step` for higher resolution
   - Check vehicle constraint parameters

4. **Not Following Global Path**
   - Increase `path_deviation_weight`
   - Check global path validity
   - Verify coordinate frame consistency


## Algorithm Details

### Trajectory Generation Method

The planner uses a rollout-based approach:

1. **Reference Path Sampling**: Samples points from the global path within the prediction horizon
2. **Lateral Offset Generation**: Creates multiple trajectories with different lateral offsets
3. **Kinematic Integration**: Uses a simple bicycle model to integrate forward in time
4. **Constraint Checking**: Validates trajectories against kinematic and dynamic constraints

### Obstacle Detection Pipeline

1. **Point Cloud Filtering**: Filters points by height and range
2. **Clustering**: Groups nearby points using distance-based clustering
3. **Obstacle Representation**: Represents each cluster as a circular obstacle
4. **Collision Checking**: Performs efficient distance-based collision detection

### Cost Function Components

- **Obstacle Cost**: Exponential penalty based on distance to nearest obstacle
- **Path Deviation**: Quadratic penalty for deviation from global path
- **Smoothness**: Penalty for high curvature changes and velocity variations
- **Comfort**: Penalty for high accelerations (longitudinal and lateral)

## Future Enhancements

- **Dynamic Obstacles**: Prediction of moving obstacles
- **Velocity Planning**: Integrated velocity profile optimization
- **Advanced Vehicle Models**: More sophisticated vehicle dynamics
- **Machine Learning**: Learning-based cost function components
- **Real-time Parameter Adaptation**: Dynamic parameter tuning based on conditions

