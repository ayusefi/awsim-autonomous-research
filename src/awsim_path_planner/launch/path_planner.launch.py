#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    planning_algorithm_arg = DeclareLaunchArgument(
        'planning_algorithm',
        default_value='astar',  # Set to A* for basic testing
        description='Path planning algorithm: astar or rrt_star'
    )
    
    hd_map_path_arg = DeclareLaunchArgument(
        'hd_map_path',
        default_value='/home/abdullah/workspaces/career_sprint/awsim-autonomous-research/shinjuku_map/map/lanelet2_map.osm',
        description='Path to HD map file (Lanelet2 OSM format)'
    )
    
    use_hd_map_arg = DeclareLaunchArgument(
        'use_hd_map_constraints',
        default_value='true',
        description='Enable HD map constraints for path planning'
    )
    
    enforce_traffic_rules_arg = DeclareLaunchArgument(
        'enforce_traffic_rules',
        default_value='true',
        description='Enforce traffic rules from HD map'
    )
    
    grid_resolution_arg = DeclareLaunchArgument(
        'grid_resolution',
        default_value='0.5',
        description='Grid resolution for A* planning (meters)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    visualize_search_space_arg = DeclareLaunchArgument(
        'visualize_search_space',
        default_value='true',
        description='Visualize search space and algorithm progress'
    )
    
    # Get package share directory
    pkg_share = get_package_share_directory('awsim_path_planner')
    
    # Path planner node
    path_planner_node = Node(
        package='awsim_path_planner',
        executable='awsim_path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[{
            'planning_algorithm': LaunchConfiguration('planning_algorithm'),
            'map_frame': 'map',
            'base_link_frame': 'base_link',
            'hd_map_path': LaunchConfiguration('hd_map_path'),
            'grid_resolution': LaunchConfiguration('grid_resolution'),
            'planning_timeout': 5.0,
            'max_planning_range': 500.0,
            'use_hd_map_constraints': LaunchConfiguration('use_hd_map_constraints'),
            'enforce_traffic_rules': LaunchConfiguration('enforce_traffic_rules'),
            'lane_following_preference': 0.8,
            'lane_change_penalty': 10.0,
            'visualize_search_space': LaunchConfiguration('visualize_search_space'),
            
            # A* specific parameters
            'astar.heuristic_weight': 1.0,
            'astar.search_radius': 100.0,
            'astar.obstacle_inflation_radius': 1.0,
            
            # RRT* specific parameters
            'rrt_star.max_iterations': 5000,
            'rrt_star.step_size': 2.0,
            'rrt_star.goal_tolerance': 2.0,
            'rrt_star.rewiring_radius': 10.0,
            
            # Ground filtering parameters - fixed for better performance
            'ground_filter.height_threshold': 2.0,  # More permissive height threshold
            'ground_filter.angle_threshold': 45.0,  # More permissive angle
            'ground_filter.ransac_distance_threshold': 0.3,  # More permissive RANSAC
            'ground_filter.ransac_max_iterations': 500,  # Reduced iterations for speed
            'ground_filter.min_ground_points': 200,  # Lower minimum ground points
            'ground_filter.voxel_leaf_size': 0.2,  # Larger voxel size to prevent overflow
            'ground_filter.use_progressive_morphological': False,  # Disable for map data
            'ground_filter.pmf_max_window_size': 33,
            'ground_filter.pmf_slope': 1.0,
            'ground_filter.pmf_initial_distance': 0.5,
            'ground_filter.pmf_max_distance': 3.0,
            'dynamic_obstacle_max_range': 50.0,
            
            # Height filtering parameters for removing tree tops and vegetation
            'height_filter.enable_max_height_filter': True,  # Enable aggressive height filtering
            'height_filter.max_height_threshold': 0.5,  # 0.5m above ground - very aggressive filtering
            'height_filter.enable_vegetation_filter': True,  # Enable vegetation filtering  
            'height_filter.vegetation_filter_height': 0.5,  # 0.5m - aggressive vegetation filtering
        }],
        remappings=[
            ('/localization/pose_with_covariance', '/localization/pose_with_covariance'),
            ('/planning/goal_pose', '/planning/goal_pose'),
            ('/sensing/lidar/top/pointcloud_raw', '/sensing/lidar/top/pointcloud_raw'),
            ('/planning/path', '/planning/path'),
            ('/planning/visualization_markers', '/planning/visualization_markers'),
            ('/planning/occupancy_grid', '/planning/occupancy_grid'),
        ]
    )
    
    # RViz node
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'path_planner.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],  # Enable simulation time for AWSIM
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        planning_algorithm_arg,
        hd_map_path_arg,
        use_hd_map_arg,
        enforce_traffic_rules_arg,
        grid_resolution_arg,
        use_rviz_arg,
        visualize_search_space_arg,
        
        GroupAction([
            path_planner_node,
            rviz_node,
        ])
    ])
