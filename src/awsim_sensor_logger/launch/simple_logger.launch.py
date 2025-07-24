#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # Create the simple sensor logger node
    simple_sensor_logger_node = Node(
        package='awsim_sensor_logger',
        executable='simple_sensor_logger_node',
        name='simple_sensor_logger',
        output='screen',
        parameters=[{
            'output_directory': LaunchConfiguration('output_directory'),
            'logging_duration_sec': LaunchConfiguration('logging_duration_sec'),
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        output_dir_arg,
        logging_duration_arg,
        simple_sensor_logger_node
    ])
