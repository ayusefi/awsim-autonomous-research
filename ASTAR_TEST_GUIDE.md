# A* Path Planner Basic Testing Guide

## 🎯 **Test Configuration**
- **Algorithm**: A* only (no RRT*)
- **HD Map**: Disabled (no lane constraints)
- **Ground Filtering**: Enabled (height + angle based)
- **Map Source**: Static PCD map + filtered dynamic obstacles

## 🚀 **Launch Commands**

### **Option 1: Path Planner Only**
```bash
ros2 launch awsim_path_planner path_planner.launch.py
```

### **Option 2: Full System with Localization**
```bash
ros2 launch awsim_bringup awsim_path_planning.launch.py
```

### **Option 3: Force A* Algorithm**
```bash
ros2 launch awsim_bringup awsim_path_planning.launch.py planning_algorithm:=astar
```

## 🔍 **Test Checklist**

### **1. System Startup**
- [ ] Path planner node starts without errors
- [ ] RViz opens with path planning configuration
- [ ] No HD map loading messages (should be disabled)
- [ ] "AWSIM Path Planner Node started" message appears

### **2. Topic Verification**
```bash
# Check required topics exist
ros2 topic list | grep -E "(planning|localization|sensing)"

# Expected topics:
# /localization/map               # Static map point cloud
# /localization/pose_with_covariance  # Vehicle pose
# /sensing/lidar/top/pointcloud_raw   # Raw LiDAR data
# /planning/goal_pose             # Goal from RViz
# /planning/path                  # Planned path output
# /planning/visualization_markers # A* search visualization
# /planning/occupancy_grid        # Grid map
```

### **3. Map Loading Verification**
```bash
# Check if static map is available
ros2 topic echo /localization/map --once | head -20

# Should show point cloud data with points
```

### **4. A* Algorithm Testing**

#### **Step 1: Set Goal in Open Area**
1. Open RViz
2. Use "2D Nav Goal" tool
3. Set goal in open area (no obstacles nearby)
4. Expected: Green path appears immediately

#### **Step 2: Set Goal Behind Obstacles**  
1. Set goal behind buildings/obstacles
2. Expected: A* finds path around obstacles
3. Check visualization: Red dots (explored nodes), blue dots (frontier)

#### **Step 3: Long Distance Planning**
1. Set goal far away (>200m)
2. Expected: Path plans across entire map area
3. Should use global map knowledge, not just local area

### **5. Ground Filtering Verification**

#### **Check Console Output**
```bash
# Look for ground filtering messages (enable debug if needed)
ros2 launch awsim_path_planner path_planner.launch.py --ros-args --log-level DEBUG

# Expected debug messages:
# "Ground filtering: X -> Y points"
# "Received and filtered raw point cloud"
# "Using combined point cloud for planning"
```

#### **Visual Verification**
- Paths should not be blocked by ground/floor
- Vehicle should be able to plan through open areas
- Only real obstacles (buildings, walls) should block paths

## 🔧 **Key Parameters (Current Settings)**

```yaml
# Basic settings
planning_algorithm: "astar"
use_hd_map_constraints: false     # DISABLED
hd_map_path: ""                   # EMPTY

# A* settings
grid_resolution: 0.5              # 50cm grid
search_radius: 100.0              # 100m search area
obstacle_inflation_radius: 1.5    # 1.5m safety margin

# Ground filtering
ground_filter.height_threshold: 0.3    # 30cm minimum height
ground_filter.angle_threshold: 15.0    # 15° slope threshold
dynamic_obstacle_max_range: 50.0       # 50m range for dynamic obstacles
```

## ✅ **Success Indicators**

### **Expected Behavior**
- ✅ A* search visualization shows red/blue dots spreading out
- ✅ Paths can plan across entire map (not just local area)
- ✅ Ground points don't block valid paths
- ✅ Path planning works in open areas
- ✅ Static obstacles (buildings) properly block paths
- ✅ Combined static + dynamic obstacle detection

### **Performance Metrics**
- ✅ Planning time: < 5 seconds for most goals
- ✅ Path quality: Reasonable routes around obstacles
- ✅ Visualization: Clear A* search progress
- ✅ No crashes or errors

## ❌ **Troubleshooting**

### **No Path Found**
```bash
# Check if goal is reachable
# Try closer goal first
# Verify map data is available
ros2 topic hz /localization/map
```

### **Path Blocked by Ground**
```bash
# Check ground filtering parameters
# Verify height_threshold (try 0.2 if still issues)
# Check console for filtering stats
```

### **Only Local Planning**
```bash
# Verify static map is being used
ros2 topic echo /localization/map --once

# Check max_planning_range parameter
# Should be 200m minimum for global planning
```

### **Poor Performance**
```bash
# Reduce grid_resolution to 1.0 for faster planning
# Reduce search_radius to 50.0 for local testing
# Check system resources
```

## 📊 **Expected Results**

With the current configuration, A* should demonstrate:
- **Global awareness**: Plans using entire Shinjuku map
- **Obstacle avoidance**: Routes around buildings/walls
- **Ground filtering**: No false obstacles from floor/ground
- **Real-time response**: < 5s planning for reasonable goals
- **Visual feedback**: Clear search visualization in RViz

This basic test validates the core path planning functionality before adding HD map constraints or lane following features!
