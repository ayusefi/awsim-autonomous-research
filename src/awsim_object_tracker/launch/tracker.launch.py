#!/usr/bin/env python3
"""
Launch file for the Multi-Object Tracker Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/ground_filter/nonground_points',
            description='Input point cloud topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/awsim_object_tracker/tracked_objects',
            description='Output tracked objects topic'
        ),
        DeclareLaunchArgument(
            'marker_topic',
            default_value='/awsim_object_tracker/tracked_objects_markers',
            description='Visualization markers topic'
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='map',
            description='Target frame for point cloud transformation'
        ),
        DeclareLaunchArgument(
            'max_distance',
            default_value='3.0',
            description='Maximum distance for data association'
        ),
        DeclareLaunchArgument(
            'dt',
            default_value='0.1',
            description='Time step between frames'
        ),
        DeclareLaunchArgument(
            'dbscan_eps',
            default_value='1.7',
            description='DBSCAN epsilon parameter'
        ),
        DeclareLaunchArgument(
            'dbscan_min_points',
            default_value='10',
            description='DBSCAN minimum points parameter'
        ),
        DeclareLaunchArgument(
            'filter_min_volume',
            default_value='1.0',
            description='Minimum bounding box volume'
        ),
        DeclareLaunchArgument(
            'filter_max_volume',
            default_value='200.0',
            description='Maximum bounding box volume'
        ),
        DeclareLaunchArgument(
            'filter_min_points',
            default_value='20',
            description='Minimum points in a cluster'
        ),
        DeclareLaunchArgument(
            'filter_max_points',
            default_value='5000',
            description='Maximum points in a cluster'
        ),
        
        ComposableNodeContainer(
            name='tracker_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='awsim_object_tracker',
                    plugin='awsim_object_tracker::TrackerNode',
                    name='awsim_object_tracker_node',
                    parameters=[{
                        'input_topic': LaunchConfiguration('input_topic'),
                        'output_topic': LaunchConfiguration('output_topic'),
                        'marker_topic': LaunchConfiguration('marker_topic'),
                        'target_frame': LaunchConfiguration('target_frame'),
                        'max_distance': LaunchConfiguration('max_distance'),
                        'dt': LaunchConfiguration('dt'),
                        'dbscan_eps': LaunchConfiguration('dbscan_eps'),
                        'dbscan_min_points': LaunchConfiguration('dbscan_min_points'),
                        'filter_min_volume': LaunchConfiguration('filter_min_volume'),
                        'filter_max_volume': LaunchConfiguration('filter_max_volume'),
                        'filter_min_points': LaunchConfiguration('filter_min_points'),
                        'filter_max_points': LaunchConfiguration('filter_max_points'),
                    }],
                )
            ],
            output='screen',
        )
    ])