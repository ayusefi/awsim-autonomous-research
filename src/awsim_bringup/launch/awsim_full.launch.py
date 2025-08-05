#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    localization_pkg_dir = FindPackageShare('awsim_localization')
    route_planning_pkg_dir = FindPackageShare('awsim_path_planner')
    trajectory_planning_pkg_dir = FindPackageShare('awsim_trajectory_planner')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([localization_pkg_dir, 'launch', 'awsim_localization.launch.py'])
        ])
    )
    
    path_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([route_planning_pkg_dir, 'launch', 'path_planner.launch.py'])
        ])
    )

    trajectory_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([trajectory_planning_pkg_dir, 'launch', 'trajectory_planner.launch.py'])
        ])
    )

    # Delay path planner launch to ensure other nodes are ready
    delayed_path_planner_launch = TimerAction(
        period=2.0,
        actions=[path_planner_launch]
    )

    # Delay trajectory planner launch to ensure path planner is ready
    delayed_trajectory_planner_launch = TimerAction(
        period=4.0,
        actions=[trajectory_planner_launch]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        localization_launch,
        delayed_path_planner_launch,
        # delayed_trajectory_planner_launch,
    ])
