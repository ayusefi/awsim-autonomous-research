# AWSIM Path Planner

> Basic A* path planning with HD map visualization for AWSIM autonomous vehicle simulation

## Demo Video

[![AWSIM Path Planner Demo - A* Planning with Lanelet Visualization](https://img.youtube.com/vi/ga-dqycrPSM/0.jpg)](https://www.youtube.com/watch?v=ga-dqycrPSM)

*Click to watch: Current A* path planner with comprehensive HD map visualization showing lanelets, traffic lights, signs, and crosswalks*

## Current Status

- ✅ **A* Path Planning**: Grid-based pathfinding between points
- ✅ **HD Map Visualization**: Lane boundaries, traffic lights, signs, crosswalks
- ❌ **Lane Following**: Paths don't follow lane centerlines yet
- ❌ **Traffic Rules**: No consideration of speed limits or lane restrictions

## Quick Start

```bash
# Terminal 1: Start AWSIM
cd AWSIM_v1.3.1/
./AWSIM.x86_64

# Terminal 2: Launch path planner
ros2 launch awsim_path_planner path_planner.launch.py
```

In RViz2, use "2D Nav Goal" to set destinations. The planner generates direct A* paths (not lane-aware).

## Architecture

```
awsim_path_planner/
├── src/
│   ├── main.cpp                   # Main entry point
│   ├── path_planner_node.cpp      # ROS2 node implementation
│   ├── astar_planner.cpp          # A* algorithm implementation
│   ├── rrt_star_planner.cpp       # RRT* algorithm (alternative)
│   ├── hd_map_manager.cpp         # Lanelet2 OSM parsing
│   └── lanelet_visualizer_node.cpp # HD map visualization
├── include/awsim_path_planner/
│   ├── path_planner_node.hpp      # Main node header
│   ├── astar_planner.hpp          # A* algorithm header
│   ├── rrt_star_planner.hpp       # RRT* algorithm header
│   ├── hd_map_manager.hpp         # HD map interface
│   └── lanelet_visualizer_node.hpp # Visualization header
├── launch/
│   ├── path_planner.launch.py     # Main system launch
│   ├── lanelet_visualizer.launch.py # HD map visualization only
│   └── algorithm_comparison.launch.py # A* vs RRT* comparison
├── config/                        # Configuration files
└── rviz/                         # RViz configurations
```

### HD Map Data Available

- **15+ Lanelets** with centerlines and boundaries
- **164 Traffic Lights** with positions
- **450+ Traffic Signs** with speed limits
- **84 Crosswalks** with boundaries
- **1,600+ Road Markings**

## Known Limitations

- **Grid-based A***: Ignores lane boundaries and traffic rules
- **No Lane Following**: Paths cut through obstacles
- **No Traffic Integration**: Doesn't use parsed traffic light/sign data

## Next Steps

Replace A* grid search with lane-graph planning:

```cpp
// TODO: Implement lane-aware planning
class LaneAwarePlanner {
    std::vector<PoseStamped> plan_lane_path(start, goal);
    // Use parsed HD map data for lane following
};
```
