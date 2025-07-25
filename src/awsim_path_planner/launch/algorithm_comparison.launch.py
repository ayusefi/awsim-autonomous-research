#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='astar',
        description='Planning algorithm: astar or rrt_star'
    )
    
    # A* Planner node
    astar_node = Node(
        package='awsim_path_planner',
        executable='awsim_path_planner_node',
        name='astar_planner_node',
        namespace='astar',
        output='screen',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('algorithm'), 'astar')
        ),
        parameters=[{
            'planning_algorithm': 'astar',
            'map_frame': 'map',
            'base_link_frame': 'base_link',
            'grid_resolution': 0.5,
            'planning_timeout': 5.0,
            'max_planning_range': 200.0,
            'use_hd_map_constraints': False,
            'visualize_search_space': True,
            
            # A* specific parameters
            'astar.heuristic_weight': 1.0,
            'astar.search_radius': 100.0,
            'astar.obstacle_inflation_radius': 1.5,
        }],
        remappings=[
            ('/localization/pose_with_covariance', '/localization/pose_with_covariance'),
            ('/planning/goal_pose', '/planning/goal_pose'),
            ('/sensing/lidar/top/pointcloud_raw', '/sensing/lidar/top/pointcloud_raw'),
            ('planning/path', 'astar/path'),
            ('planning/visualization_markers', 'astar/visualization_markers'),
            ('planning/occupancy_grid', 'astar/occupancy_grid'),
        ]
    )
    
    # RRT* Planner node
    rrt_star_node = Node(
        package='awsim_path_planner',
        executable='awsim_path_planner_node',
        name='rrt_star_planner_node',
        namespace='rrt_star',
        output='screen',
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration('algorithm'), 'rrt_star')
        ),
        parameters=[{
            'planning_algorithm': 'rrt_star',
            'map_frame': 'map',
            'base_link_frame': 'base_link',
            'grid_resolution': 0.5,
            'planning_timeout': 10.0,
            'max_planning_range': 200.0,
            'use_hd_map_constraints': False,
            'visualize_search_space': True,
            
            # RRT* specific parameters
            'rrt_star.max_iterations': 3000,
            'rrt_star.step_size': 3.0,
            'rrt_star.goal_tolerance': 3.0,
            'rrt_star.rewiring_radius': 15.0,
        }],
        remappings=[
            ('/localization/pose_with_covariance', '/localization/pose_with_covariance'),
            ('/planning/goal_pose', '/planning/goal_pose'),
            ('/sensing/lidar/top/pointcloud_raw', '/sensing/lidar/top/pointcloud_raw'),
            ('planning/path', 'rrt_star/path'),
            ('planning/visualization_markers', 'rrt_star/visualization_markers'),
            ('planning/occupancy_grid', 'rrt_star/occupancy_grid'),
        ]
    )
    
    return LaunchDescription([
        algorithm_arg,
        astar_node,
        rrt_star_node,
    ])
