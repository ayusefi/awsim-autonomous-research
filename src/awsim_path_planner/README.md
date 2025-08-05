# AWSIM Path Planner

> Multi-algorithm path planning system with lanelet2-based route planning and HD map visualization for AWSIM autonomous vehicle simulation

## Demo Videos

### A* Grid-Based Planning
[![AWSIM A* Path Planner Demo](https://img.youtube.com/vi/nA90nYrhmQ8/0.jpg)](https://www.youtube.com/watch?v=nA90nYrhmQ8)

*Grid-based A* pathfinding with obstacle avoidance using point cloud data*

### RRT* Sampling-Based Planning  
[![AWSIM RRT* Path Planner Demo](https://img.youtube.com/vi/1i7BVNgm2dM/0.jpg)](https://www.youtube.com/watch?v=1i7BVNgm2dM)

*RRT* sampling-based algorithm for complex obstacle environments*

### Route Planner with Lanelet2
[![AWSIM Route Planner Demo](https://img.youtube.com/vi/uDkqxIdpc84/0.jpg)](https://www.youtube.com/watch?v=uDkqxIdpc84)

*Lane-following route planning using HD map data with traffic rules integration*

## Current Status

- ✅ **A* Path Planning**: Grid-based pathfinding with obstacle avoidance
- ✅ **RRT* Path Planning**: Sampling-based algorithm for complex environments  
- ✅ **Route Planning**: Lane-following using lanelet2 with traffic rules
- ✅ **HD Map Visualization**: Lane boundaries, centerlines, traffic elements
- ✅ **Coordinate System**: Consistent coordinate transformation across components
- ✅ **Multi-Algorithm Support**: Configurable planning algorithm selection

## Quick Start

```bash
# Terminal 1: Start AWSIM
cd AWSIM_v1.3.1/
./AWSIM.x86_64

# Terminal 2: Launch path planner (default: route planning)
ros2 launch awsim_path_planner path_planner.launch.py

# Or specify algorithm:
ros2 launch awsim_path_planner path_planner.launch.py planning_algorithm:=astar
ros2 launch awsim_path_planner path_planner.launch.py planning_algorithm:=rrt_star  
ros2 launch awsim_path_planner path_planner.launch.py planning_algorithm:=route
```

In RViz2, use "2D Nav Goal" to set destinations. The route planner follows lane centerlines, while A* and RRT* provide direct geometric paths.

## Architecture

```
awsim_path_planner/
├── src/
│   ├── path_planner_node.cpp       # ROS2 node implementation  
│   ├── astar_planner.cpp           # A* algorithm implementation
│   ├── rrt_star_planner.cpp        # RRT* algorithm implementation
│   ├── route_planner.cpp           # Lanelet2 route planning
│   └── lanelet_visualizer_node.cpp # HD map visualization
├── include/awsim_path_planner/
│   ├── path_planner_node.hpp       # Main node header
│   ├── astar_planner.hpp           # A* algorithm header
│   ├── rrt_star_planner.hpp        # RRT* algorithm header
│   ├── route_planner.hpp           # Route planner header
│   └── lanelet_visualizer_node.hpp # Visualization header
├── launch/
│   ├── path_planner.launch.py      # Main system launch
│   └── sensor_logger.launch.py     # Data logging launch
├── config/                         # Configuration files
└── rviz/                          # RViz configurations
```


## Features

### Route Planner (Recommended)
- **Lane-following paths** using lanelet2 centerlines
- **Traffic rules integration** (German traffic rules)
- **Coordinate consistency** with localization system

### A* Planner
- **Grid-based pathfinding** with configurable resolution
- **Obstacle avoidance** using point cloud data
- **Occupancy grid** generation and visualization
- **Heuristic optimization** for efficient search

### RRT* Planner  
- **Sampling-based exploration** for complex environments
- **Asymptotically optimal** path solutions
- **Dynamic obstacle handling** capability
- **Tree visualization** for algorithm understanding

## Configuration

### Algorithm Selection
```yaml
# In path_planner.launch.py
planning_algorithm: "route"    # Options: astar, rrt_star, route
```

### Route Planner Parameters
```yaml
route_planner:
  traffic_rules_name: "german"          # Traffic rules to follow
  goal_search_radius: 150.0             # Goal search radius (meters)
  centerline_resolution: 0.5            # Path point spacing (meters)
  enable_lane_change: true              # Allow lane changes
```

### A* Planner Parameters  
```yaml
astar:
  grid_resolution: 0.5                  # Grid cell size (meters)
  heuristic_weight: 1.0                 # A* heuristic weight
  max_planning_range: 1000.0            # Maximum planning distance
```

## Dependencies

- ROS 2 Humble
- lanelet2_core, lanelet2_io, lanelet2_projection
- lanelet2_routing, lanelet2_traffic_rules  
- PCL (Point Cloud Library)
- tf2_geometry_msgs
- autoware_planning_msgs

