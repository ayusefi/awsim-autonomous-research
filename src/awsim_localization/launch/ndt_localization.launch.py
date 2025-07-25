#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """NDT-specific launch file for scan matching localization."""
    
    # Declare launch arguments
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/abdullah/workspaces/career_sprint/awsim-autonomous-research/shinjuku_map/map/pointcloud_map.pcd',
        description='Path to the PCD map file'
    )
    
    ndt_resolution_arg = DeclareLaunchArgument(
        'ndt_resolution',
        default_value='2.0',
        description='NDT grid resolution in meters'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )
    
    # Parameter file
    params_file = PathJoinSubstitution([
        FindPackageShare('awsim_localization'),
        'config',
        'scan_matching_params.yaml'
    ])
    
    # Scan matching node with NDT
    scan_matching_node = Node(
        package='awsim_localization',
        executable='scan_matching_node',
        name='scan_matching_node',
        parameters=[
            params_file,
            {
                'algorithm_type': 'ndt',
                'map_path': LaunchConfiguration('map_path'),
                'ndt_resolution': LaunchConfiguration('ndt_resolution'),
                'scan_matching_score_threshold': 2.0,  # Much lower threshold like working version
                'use_ground_truth_init': False
            }
        ],
        output='screen'
    )
    
    # RViz configuration
    rviz_config = PathJoinSubstitution([
        FindPackageShare('awsim_localization'),
        'rviz',
        'localization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Static transform publisher for velodyne_top to base_link
    # This transform places the LiDAR sensor on top of the vehicle
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_base_link_publisher',
        arguments=['0', '0', '2.0', '0', '0', '0', 'base_link', 'velodyne_top'],
        output='screen'
    )

    return LaunchDescription([
        map_path_arg,
        ndt_resolution_arg,
        use_rviz_arg,
        scan_matching_node,
        static_transform_publisher,
        rviz_node
    ])