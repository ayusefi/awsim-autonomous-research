#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Package directories
    bringup_pkg_dir = FindPackageShare('awsim_bringup')
    localization_pkg_dir = FindPackageShare('awsim_localization')
    slam_pkg_dir = FindPackageShare('awsim_slam')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    enable_3d_localization_arg = DeclareLaunchArgument(
        'enable_3d_localization',
        default_value='true',
        description='Enable 3D Monte Carlo Localization'
    )
    
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='false',
        description='Enable SLAM functionality (alternative to localization)'
    )
    
    enable_sensor_logger_arg = DeclareLaunchArgument(
        'enable_sensor_logger',
        default_value='false',
        description='Enable sensor data logging'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # 3D Monte Carlo Localization system
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([localization_pkg_dir, 'launch', 'full_3d_localization.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_rviz': LaunchConfiguration('enable_rviz'),
            'enable_sensor_logger': LaunchConfiguration('enable_sensor_logger'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_3d_localization'))
    )
    
    # SLAM system (alternative to localization)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([slam_pkg_dir, 'launch', 'slam.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_slam'))
    )
    
    # Standalone sensor logger (when not included in localization)
    sensor_logger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_pkg_dir, 'launch', 'sensor_logger.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_sensor_logger'))
    )
    
    # Delay SLAM launch to ensure localization doesn't conflict
    delayed_slam_launch = TimerAction(
        period=2.0,
        actions=[slam_launch]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_3d_localization_arg,
        enable_slam_arg,
        enable_sensor_logger_arg,
        enable_rviz_arg,
        
        # Core systems (mutually exclusive)
        localization_launch,
        delayed_slam_launch,
        
        # Supporting systems
        sensor_logger_launch,
    ])
