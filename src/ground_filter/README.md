

# Ground Filter Package

A ROS 2 package for filtering ground points from LiDAR point clouds using a combination of RANSAC plane segmentation and grid-based refinement.

## Overview

This package processes point cloud data to separate ground points from non-ground points. It uses a hybrid approach that first identifies the ground plane using RANSAC and then refines the classification using a grid-based method that better handles uneven terrain.

## How It Works

1. **Initial Plane Detection**: Uses RANSAC to identify the primary ground plane in the point cloud
2. **Grid-based Refinement**: Divides the ground into grid cells and computes local height statistics (mean and standard deviation)
3. **Point Classification**: Classifies each point based on its deviation from the local ground model
4. **Output**: Publishes separate point clouds for ground and non-ground points

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `distance_threshold` | 0.2 | Maximum distance from plane to consider a point as ground (meters) |
| `max_angle` | 15.0 | Maximum allowed angle from horizontal (degrees) |
| `min_inliers` | 1000 | Minimum number of points to consider a plane valid |
| `max_iterations` | 1000 | Maximum number of RANSAC iterations |
| `grid_size` | 1.0 | Grid cell size for ground modeling (meters) |
| `height_threshold` | 0.3 | Height threshold for grid-based filtering (meters) |
| `k_multiplier` | 2.0 | Multiplier for standard deviation threshold |
| `use_grid_filter` | true | Enable grid-based refinement |

## Topics

### Subscribed Topics
- `/sensing/lidar/top/pointcloud_raw` (sensor_msgs/msg/PointCloud2): Input point cloud data

### Published Topics
- `ground_points` (sensor_msgs/msg/PointCloud2): Filtered ground points
- `nonground_points` (sensor_msgs/msg/PointCloud2): Filtered non-ground points

## Usage

1. **Launch the node**:
   ```bash
   ros2 launch ground_filter ground_filter.launch.py
   ```

2. **With custom parameters**:
   ```bash
   ros2 launch ground_filter ground_filter.launch.py distance_threshold:=0.1 grid_size:=0.5
   ```

3. **View the results**:
   ```bash
   ros2 run rviz2 rviz2
   ```
   Add displays for the `/ground_points` and `/nonground_points` topics.

## Tuning Tips

- For flatter terrain, decrease `height_threshold` and `k_multiplier`
- For rougher terrain, increase `grid_size` to capture larger variations
- If processing is slow, increase the voxel grid leaf size in the code
- For better accuracy, decrease `distance_threshold` but ensure you have enough ground points

## Dependencies

- ROS 2 Humble
- PCL (Point Cloud Library)
- Eigen3

## License

This package is open source. Please see the LICENSE file for details.
```