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
    localization_pkg_dir = FindPackageShare('awsim_localization')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    localization_algorithm_arg = DeclareLaunchArgument(
        'localization_algorithm',
        default_value='ndt',
        description='Localization algorithm: ndt or icp'
    )
    
    enable_sensor_logger_arg = DeclareLaunchArgument(
        'enable_sensor_logger',
        default_value='false',
        description='Enable sensor data logging'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
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
    
    # Include localization launch file
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([localization_pkg_dir, 'launch', 'scan_matching.launch.py'])
        ]),
        launch_arguments={
            'algorithm': LaunchConfiguration('localization_algorithm'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )
    
    # Delay localization launch to ensure other nodes are ready
    delayed_localization_launch = TimerAction(
        period=2.0,
        actions=[localization_launch]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        localization_algorithm_arg,
        enable_sensor_logger_arg,
        use_rviz_arg,
        sensor_logger_launch,
        delayed_localization_launch,
    ])
