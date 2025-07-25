#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    bringup_pkg_dir = FindPackageShare('awsim_bringup')
    slam_pkg_dir = FindPackageShare('awsim_slam')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM functionality'
    )
    
    enable_sensor_logger_arg = DeclareLaunchArgument(
        'enable_sensor_logger',
        default_value='false',
        description='Enable sensor data logging'
    )
    
    # Include sensor logger launch file (conditionally)
    sensor_logger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_pkg_dir, 'launch', 'sensor_logger.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=LaunchConfiguration('enable_sensor_logger')
    )
    
    # Include SLAM launch file (conditionally)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([slam_pkg_dir, 'launch', 'slam.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=LaunchConfiguration('enable_slam')
    )
    
    # Delay SLAM launch to ensure other nodes are ready
    delayed_slam_launch = TimerAction(
        period=2.0,
        actions=[slam_launch]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_slam_arg,
        enable_sensor_logger_arg,
        sensor_logger_launch,
        delayed_slam_launch,
    ])
