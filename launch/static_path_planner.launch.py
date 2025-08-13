#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('awsim_path_planner')
    
    # Path to static mode configuration
    static_config_path = '/home/abdullah/workspaces/career_sprint/awsim-autonomous-research/config/static_path_planner_params.yaml'
    
    # Path planner node in static mode
    path_planner_node = Node(
        package='awsim_path_planner',
        executable='awsim_path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[static_config_path],
        remappings=[
            ('/localization/pose_with_covariance', '/localization/pose_with_covariance'),
            ('/planning/goal_pose', '/planning/goal_pose'),
            ('/sensing/lidar/top/pointcloud_raw', '/ground_filter/nonground_points'),
            ('/planning/path', '/planning/path'),
            ('/planning/route', '/planning/route'),
            ('/planning/visualization_markers', '/planning/visualization_markers'),
            ('/planning/occupancy_grid', '/planning/occupancy_grid'),
        ]
    )
    
    return LaunchDescription([
        path_planner_node,
    ])
