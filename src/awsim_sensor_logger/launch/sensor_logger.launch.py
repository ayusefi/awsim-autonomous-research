#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='/tmp/awsim_sensor_data',
        description='Directory to save collected sensor data'
    )
    
    logging_duration_arg = DeclareLaunchArgument(
        'logging_duration_sec',
        default_value='300',  # 5 minutes
        description='Duration for data collection in seconds'
    )
    
    sync_tolerance_arg = DeclareLaunchArgument(
        'sync_tolerance_sec',
        default_value='0.1',  # 100ms
        description='Synchronization tolerance in seconds'
    )
    
    save_pointclouds_arg = DeclareLaunchArgument(
        'save_pointclouds',
        default_value='true',
        description='Whether to save point cloud files'
    )
    
    save_images_arg = DeclareLaunchArgument(
        'save_images',
        default_value='true',
        description='Whether to save camera images'
    )
    
    save_raw_data_arg = DeclareLaunchArgument(
        'save_raw_data',
        default_value='true',
        description='Whether to save raw sensor data to CSV'
    )
    
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='10',
        description='Message queue size for synchronization'
    )

    # Create the sensor logger node
    sensor_logger_node = Node(
        package='awsim_sensor_logger',
        executable='sensor_logger_node',
        name='synchronized_sensor_logger',
        output='screen',
        parameters=[{
            'output_directory': LaunchConfiguration('output_directory'),
            'logging_duration_sec': LaunchConfiguration('logging_duration_sec'),
            'sync_tolerance_sec': LaunchConfiguration('sync_tolerance_sec'),
            'save_pointclouds': LaunchConfiguration('save_pointclouds'),
            'save_images': LaunchConfiguration('save_images'),
            'save_raw_data': LaunchConfiguration('save_raw_data'),
            'queue_size': LaunchConfiguration('queue_size'),
            'use_sim_time': True,  # Enable simulation time
        }],
        remappings=[
            # Add any topic remappings if needed
        ]
    )

    return LaunchDescription([
        output_dir_arg,
        logging_duration_arg,
        sync_tolerance_arg,
        save_pointclouds_arg,
        save_images_arg,
        save_raw_data_arg,
        queue_size_arg,
        sensor_logger_node
    ])
