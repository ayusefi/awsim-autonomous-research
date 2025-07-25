# Path Planner Testing Guide

## 🚀 **Major Improvements Implemented**

### ✅ **Fixed Issues**
1. **Map-based Planning**: Now uses `/localization/map` topic (static PCD map) instead of limited raw point cloud
2. **Ground Filtering**: Automatically filters ground points from raw LiDAR to avoid false obstacles
3. **Combined Obstacle Detection**: Intelligently combines static map + filtered dynamic obstacles

### ✅ **Technical Enhancements**
- **Static Map Integration**: Global planning uses pre-built Shinjuku map PCD
- **Dynamic Obstacle Detection**: Real-time filtered LiDAR for moving obstacles
- **Ground Point Filtering**: Height + angle based filtering to distinguish ground from obstacles
- **Range-limited Processing**: Dynamic obstacles only within 50m for performance
- **Smart Point Cloud Combination**: Merges static + dynamic data for comprehensive planning

## 🧪 **Test Procedures**

### **Test 1: Basic System Verification**
```bash
# Launch the improved system
ros2 launch awsim_bringup awsim_path_planning.launch.py

# Check topics are working
ros2 topic list | grep -E "(map|planning|localization)"
ros2 topic hz /localization/map
ros2 topic hz /planning/path
```

### **Test 2: Ground Filtering Verification**
```bash
# Monitor raw vs filtered point clouds
ros2 topic echo /sensing/lidar/top/pointcloud_raw --once
# Should show all points including ground

# The system internally filters these - check logs:
ros2 launch awsim_bringup awsim_path_planning.launch.py --ros-args --log-level DEBUG
# Look for "Ground filtering: X -> Y points" messages
```

### **Test 3: Map-based Planning Test**
```bash
# Launch system and verify map loading
ros2 launch awsim_bringup awsim_path_planning.launch.py

# In RViz:
# 1. You should see the static map loaded
# 2. Set a goal using 2D Nav Goal tool
# 3. Path should plan using the full map, not just nearby obstacles
# 4. Path should avoid both static obstacles (buildings) and dynamic ones (if any)
```

### **Test 4: Algorithm Comparison with New Features**
```bash
# Test A* with map-based planning
ros2 launch awsim_bringup awsim_path_planning.launch.py planning_algorithm:=astar

# Test RRT* with map-based planning
ros2 launch awsim_bringup awsim_path_planning.launch.py planning_algorithm:=rrt_star

# Compare:
# - A*: Should show grid-based search across full map
# - RRT*: Should sample throughout map space, not just local area
```

## 📊 **Expected Improvements**

### **Before (Issues)**
- ❌ Planning only with immediate surroundings (raw LiDAR range ~100m)
- ❌ Ground points treated as obstacles
- ❌ No global map awareness
- ❌ Paths blocked by ground/floor detections

### **After (Fixed)**
- ✅ Global planning using full Shinjuku map
- ✅ Ground points automatically filtered out
- ✅ Static obstacles from map + dynamic from LiDAR
- ✅ Paths can plan across entire map area
- ✅ Better obstacle differentiation

## 🔧 **Configuration Parameters**

### **Ground Filtering Tuning**
```yaml
# Adjust these in path_planner_params.yaml if needed:
ground_filter:
  height_threshold: 0.3        # Points below this height considered ground
  angle_threshold: 15.0        # Angle from horizontal for ground detection

dynamic_obstacle_max_range: 50.0  # Range for dynamic obstacle detection
```

### **Performance Optimization**
- **Static map**: Loaded once, provides global context
- **Dynamic filtering**: Only processes nearby points (50m) for real-time performance
- **Combined processing**: Efficient merging of static + dynamic data

## 🎯 **Testing Checklist**

- [ ] System launches without errors
- [ ] Map point cloud visible in RViz
- [ ] Goal setting works (2D Nav Goal)
- [ ] Paths plan across longer distances
- [ ] No false obstacles from ground points
- [ ] Both A* and RRT* use full map
- [ ] Planning works in open areas
- [ ] Dynamic obstacles still detected
- [ ] Performance remains good

## 🔍 **Troubleshooting**

If issues occur:

1. **Check map loading**:
```bash
ros2 topic echo /localization/map --once
# Should show point cloud data
```

2. **Verify ground filtering**:
```bash
# Enable debug logging to see filtering statistics
ros2 launch awsim_path_planner path_planner.launch.py --ros-args --log-level DEBUG
```

3. **Test point cloud combination**:
```bash
# Check both static and dynamic inputs are available
ros2 topic hz /localization/map
ros2 topic hz /sensing/lidar/top/pointcloud_raw
```

The path planner now provides much more capable and robust planning using the full map context while maintaining real-time dynamic obstacle avoidance!
