# AWSIM Path Planner

A comprehensive path planning system for AWSIM that implements both A* and RRT* algorithms with HD map integration.

## Overview

This package proThe system provides comprehensive visualization including:
- **Current Pose**: Vehicle position with uncertainty
- **Planned Path**: Final path in green
- **Search Visualization**: 
  - A*: Explored nodes (red), search frontier (blue)
  - RRT*: Tree structure (blue), final path (green)
- **Occupancy Grid**: Combined static and dynamic obstacle map (A* only)
- **HD Map**: Lane centerlines and boundaries (if loaded)
- **Static Map**: Pre-built environment from PCD file
- **Dynamic Obstacles**: Filtered LiDAR data showing moving/temporary obstaclesanced path planning capabilities for autonomous vehicles in the AWSIM simulation environment. It includes:

- **A* Algorithm**: Grid-based optimal path planning with configurable heuristics
- **RRT* Algorithm**: Sampling-based asymptotically optimal path planning
- **HD Map Integration**: Integration with Lanelet2-formatted HD maps for lane-aware planning
- **Real-time Visualization**: Comprehensive RViz visualization of search progress and results
- **Obstacle Avoidance**: Dynamic obstacle detection using LiDAR point clouds

## Features

### Map-Based Global Planning
- **Static Map Integration**: Uses pre-built PCD map from `/localization/map` for global path planning
- **Dynamic Obstacle Detection**: Real-time obstacle detection from LiDAR point cloud with ground filtering
- **Combined Planning**: Intelligent combination of static map and dynamic obstacles for optimal paths

### A* Path Planner
- Grid-based search with configurable resolution
- Euclidean distance heuristic with adjustable weight
- Obstacle inflation for safe planning
- Path smoothing and optimization
- Real-time visualization of explored nodes and search frontier

### RRT* Path Planner
- Sampling-based approach with goal biasing
- Asymptotically optimal path refinement
- Dynamic rewiring for improved solutions
- Configurable step size and iteration limits
- Tree visualization with final path highlighting

### Advanced Point Cloud Processing
- **Ground Filtering**: Automatic removal of ground points to avoid false obstacles
- **Height-based Filtering**: Configurable height threshold for obstacle detection
- **Angle-based Filtering**: Slope analysis to distinguish ground from obstacles
- **Range Limiting**: Dynamic obstacle detection within configurable range

### HD Map Integration
- Support for Lanelet2 OSM format maps
- Lane-aware cost functions
- Boundary proximity penalties
- Speed limit consideration
- Path optimization to follow lane centerlines

## Installation

1. Make sure you have ROS 2 Humble installed
2. Install dependencies:
```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
```

3. Build the package:
```bash
cd /path/to/your/workspace
colcon build --packages-select awsim_path_planner
source install/setup.bash
```

## Usage

### Basic Path Planning

Launch the complete AWSIM system with path planning:

```bash
ros2 launch awsim_bringup awsim_path_planning.launch.py
```

### Algorithm Selection

Choose between A* and RRT* algorithms:

```bash
# Launch with A* algorithm (default)
ros2 launch awsim_bringup awsim_path_planning.launch.py planning_algorithm:=astar

# Launch with RRT* algorithm
ros2 launch awsim_bringup awsim_path_planning.launch.py planning_algorithm:=rrt_star
```

### Path Planner Only

Launch just the path planner node:

```bash
ros2 launch awsim_path_planner path_planner.launch.py
```

### Testing Different Algorithms

Compare A* and RRT* performance:

```bash
# Launch A* planner
ros2 launch awsim_path_planner algorithm_comparison.launch.py algorithm:=astar

# Launch RRT* planner  
ros2 launch awsim_path_planner algorithm_comparison.launch.py algorithm:=rrt_star
```

## Configuration

### A* Algorithm Parameters

```yaml
astar:
  heuristic_weight: 1.0         # A* heuristic weight (1.0 = A*, >1.0 = weighted A*)
  search_radius: 100.0          # Search radius around start/goal in meters
  obstacle_inflation_radius: 1.0 # Obstacle inflation radius in meters
```

### RRT* Algorithm Parameters

```yaml
rrt_star:
  max_iterations: 5000          # Maximum number of RRT* iterations
  step_size: 2.0               # Step size for tree extension in meters
  goal_tolerance: 2.0          # Goal reach tolerance in meters
  rewiring_radius: 10.0        # Radius for rewiring nearby nodes in meters
```

### General Parameters

```yaml
planning_algorithm: "astar"     # or "rrt_star"
grid_resolution: 0.5           # Grid resolution in meters (A* only)
planning_timeout: 5.0          # Maximum planning time in seconds
max_planning_range: 500.0      # Maximum planning distance in meters
visualize_search_space: true   # Show algorithm search progress

# Ground filtering parameters
ground_filter:
  height_threshold: 0.3        # Minimum height above ground to consider as obstacle (meters)
  angle_threshold: 15.0        # Minimum angle from horizontal to consider non-ground (degrees)

# Dynamic obstacle detection
dynamic_obstacle_max_range: 50.0  # Maximum range for dynamic obstacles (meters)
```

## Topics

### Subscribed Topics

- `/localization/pose_with_covariance` (geometry_msgs/PoseWithCovarianceStamped): Current vehicle pose
- `/planning/goal_pose` (geometry_msgs/PoseStamped): Goal pose from RViz 2D Nav Goal tool
- `/localization/map` (sensor_msgs/PointCloud2): Static map point cloud for global planning
- `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/PointCloud2): Raw LiDAR data for dynamic obstacle detection

### Published Topics

- `/planning/path` (nav_msgs/Path): Planned path waypoints
- `/planning/visualization_markers` (visualization_msgs/MarkerArray): Algorithm visualization markers
- `/planning/occupancy_grid` (nav_msgs/OccupancyGrid): Occupancy grid (A* only)

## Using the System

1. **Start the system**: Launch AWSIM with path planning
2. **Set goal**: Use RViz 2D Nav Goal tool to set a destination
3. **Monitor planning**: Watch the algorithm search in real-time
4. **View results**: The planned path will be displayed in green

### RViz Visualization

The system provides comprehensive visualization including:
- **Current Pose**: Vehicle position with uncertainty
- **Planned Path**: Final path in green
- **Search Visualization**: 
  - A*: Explored nodes (red), search frontier (blue)
  - RRT*: Tree structure (blue), final path (green)
- **Occupancy Grid**: Obstacle map (A* only)
- **HD Map**: Lane centerlines and boundaries (if loaded)
- **LiDAR Data**: Real-time point cloud

## Algorithm Comparison

### A* Algorithm
**Pros:**
- Guaranteed optimal path (with admissible heuristic)
- Deterministic results
- Fast for well-defined grid environments
- Excellent visualization of search process

**Cons:**
- Memory intensive for large areas
- Resolution-dependent performance
- Less suitable for high-dimensional spaces

**Best for:** Structured environments, when optimality is critical

### RRT* Algorithm
**Pros:**
- Handles high-dimensional spaces well
- Asymptotically optimal
- Good for complex obstacle environments
- Memory efficient

**Cons:**
- Probabilistically complete
- Can be slower to converge
- Results vary between runs

**Best for:** Complex environments, when fast approximate solutions are acceptable

## HD Map Integration

To use HD map features:

```bash
ros2 launch awsim_bringup awsim_path_planning.launch.py \
  hd_map_path:=/path/to/your/map.osm \
  planning_algorithm:=astar
```

The system will:
- Load lane information from the OSM file
- Apply lane-aware cost functions
- Optimize paths to follow lane centerlines
- Visualize lane boundaries and centerlines

## Troubleshooting

### Common Issues

1. **No path found**: 
   - Check if goal is reachable
   - Reduce obstacle inflation radius
   - Increase search radius (A*) or iterations (RRT*)

2. **Slow planning**:
   - Increase grid resolution (A*) for faster planning
   - Reduce max iterations (RRT*)
   - Decrease search radius

3. **Path too close to obstacles**:
   - Increase obstacle inflation radius
   - Adjust cost function weights

### Performance Tuning

For **faster planning**:
```yaml
# A* settings
grid_resolution: 1.0           # Larger grid cells
search_radius: 50.0           # Smaller search area

# RRT* settings  
max_iterations: 1000          # Fewer iterations
step_size: 5.0               # Larger steps
```

For **higher quality paths**:
```yaml
# A* settings
grid_resolution: 0.2          # Finer grid
obstacle_inflation_radius: 2.0 # More safety margin

# RRT* settings
max_iterations: 10000         # More iterations
rewiring_radius: 20.0        # Larger rewiring radius
```

## Development

### Adding New Algorithms

1. Create a new planner class inheriting from base interface
2. Implement the `plan_path()` method
3. Add algorithm selection in `PathPlannerNode`
4. Update configuration and launch files

### Testing

Run unit tests:
```bash
colcon test --packages-select awsim_path_planner
```

### Building from Source

```bash
git clone <repository-url>
cd awsim-autonomous-research
colcon build --packages-select awsim_path_planner
```

## License

This project is licensed under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## References

- [A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [RRT* Algorithm](https://journals.sagepub.com/doi/abs/10.1177/0278364911406761)
- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [AWSIM](https://github.com/tier4/AWSIM)
