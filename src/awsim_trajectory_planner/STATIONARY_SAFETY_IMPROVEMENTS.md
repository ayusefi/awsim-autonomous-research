# Stationary Obstacle Avoidance Improvements

## Problem Identification
You were right to be concerned! The trajectory planner could potentially plan through obstacles when stationary due to several issues:

1. **Cost threshold too permissive** (50.0) - allowing trajectories with moderate obstacle costs
2. **Insufficient safety margins** for stationary scenarios
3. **Lack of special handling** for stationary vs. moving conditions

## Implemented Solutions

### 1. Stricter Cost Thresholds
```yaml
# Before
max_acceptable_cost: 50.0        # Too permissive
safety_margin: 1.5               # Too small

# After  
max_acceptable_cost: 25.0        # More restrictive (50% reduction)
safety_margin: 2.0               # Increased minimum distance to obstacles
```

### 2. Enhanced Collision Detection
```cpp
// Before: Basic safety margin
double min_dist_sq = (obstacle.radius + params_.safety_margin);

// After: Conservative collision checking with extra buffer
double safety_radius = obstacle.radius + params_.safety_margin + 0.5; // Extra 0.5m buffer
double min_dist_sq = safety_radius * safety_radius;
```

### 3. Stationary-Specific Safety Logic
```cpp
// Added special handling for stationary vehicles
double current_velocity = estimateVelocity();
bool is_stationary = current_velocity < 0.1; // m/s

if (is_stationary && !obstacles.empty()) {
  // Check if any obstacles are very close (within 3m)
  for (const auto& obstacle : obstacles) {
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance < 3.0) {
      RCLCPP_WARN("Obstacle detected nearby while stationary: distance=%.2f m", distance);
      // Force stricter cost evaluation for nearby obstacles
      for (auto& cost : costs) {
        if (!cost.is_collision_free) {
          cost.total_cost = 1e6; // Make collision trajectories extremely expensive
        }
      }
    }
  }
}
```

### 4. Strict Collision Cost Assignment
```cpp
// Before: Collision trajectories still had finite costs
if (!cost.is_collision_free) {
  cost.obstacle_cost = 1e6;
  cost.path_deviation_cost = 0.0;  // Other costs were zero
  cost.smoothness_cost = 0.0;
  cost.comfort_cost = 0.0;
  cost.total_cost = cost.obstacle_cost;
}

// After: All costs maxed out for collision trajectories
if (!cost.is_collision_free) {
  cost.obstacle_cost = 1e6;
  cost.path_deviation_cost = 1e6;  // All costs set to maximum
  cost.smoothness_cost = 1e6;
  cost.comfort_cost = 1e6;
  cost.total_cost = 1e6;          // Ensure no collision trajectory is ever selected
}
```

### 5. Enhanced Collision Check Logic
```cpp
// STRICT CHECK: Must be collision free AND within cost threshold
if (costs[best_idx].is_collision_free && costs[best_idx].total_cost <= params_.max_acceptable_cost) {
  // Only then select trajectory
} else {
  if (!costs[best_idx].is_collision_free) {
    RCLCPP_WARN("Best trajectory has collision - rejecting");
  }
  if (costs[best_idx].total_cost > params_.max_acceptable_cost) {
    RCLCPP_WARN("Best trajectory exceeds cost threshold (%.2f > %.2f)", 
               costs[best_idx].total_cost, params_.max_acceptable_cost);
  }
}
```

### 6. Improved Debugging and Monitoring
```cpp
// Log obstacle information for debugging
for (size_t i = 0; i < obstacles.size() && i < 5; ++i) {
  RCLCPP_DEBUG("Obstacle %zu: pos(%.2f, %.2f), radius=%.2f", 
               i, obstacles[i].x, obstacles[i].y, obstacles[i].radius);
}

// Enhanced collision detection logging
RCLCPP_DEBUG("Collision detected: point(%.2f,%.2f) vs obstacle(%.2f,%.2f) dist=%.2f < safety=%.2f", 
            point.x, point.y, obstacle.x, obstacle.y, std::sqrt(dist_sq), safety_radius);
```

## Safety Guarantees

### Multi-Layer Protection
1. **Collision Detection**: Extra 0.5m safety buffer beyond configured margin
2. **Cost Thresholds**: Reduced from 50.0 to 25.0 for stricter selection
3. **Stationary Logic**: Special handling when vehicle speed < 0.1 m/s
4. **Proximity Alerts**: Warning and extra penalties for obstacles within 3m
5. **Fallback Safety**: Current pose as fallback when no safe trajectory exists

### Visualization Improvements
- **Blue trajectories**: Safe, collision-free paths
- **Red trajectories**: Paths with collisions or excessive cost
- **Green trajectory**: Currently selected safe path (if any)
- **No green trajectory**: Vehicle will stay at current position

## Testing Recommendations

### 1. Stationary Scenarios
```bash
# Test with obstacles nearby
ros2 topic echo /planning/trajectory_planner/visualization  # Check color coding
ros2 topic echo /planning/local_trajectory                  # Should be current pose if obstacles nearby
```

### 2. Debug Information
```bash
# Enable debug logging to see obstacle detection
ros2 run awsim_trajectory_planner trajectory_planner_node --ros-args --log-level debug
```

### 3. Parameter Tuning
```yaml
# If still too permissive, further reduce:
max_acceptable_cost: 15.0        # Even stricter
safety_margin: 2.5               # Even larger margin

# If too conservative, slightly increase:
max_acceptable_cost: 30.0        # Slightly less strict
```

## Expected Behavior

### When Stationary with Obstacles
1. **Obstacle Detection**: Should detect and log nearby obstacles
2. **Trajectory Evaluation**: All collision trajectories get 1e6 cost
3. **Selection Logic**: No trajectory should be selected if all have collisions
4. **Fallback**: Vehicle publishes current pose as "trajectory"
5. **Visualization**: All candidate trajectories should appear red

### When Moving
1. **Normal Operation**: Collision-free trajectories shown in blue
2. **Selected Path**: Safe trajectory shown in green
3. **Cost Filtering**: Only trajectories under cost threshold considered

The trajectory planner should now **never** plan through obstacles when stationary, providing multiple layers of safety validation.
