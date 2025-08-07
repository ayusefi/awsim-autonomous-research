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
    ground_filter_pkg_dir = FindPackageShare('ground_filter')
    localization_pkg_dir = FindPackageShare('awsim_localization')
    route_planning_pkg_dir = FindPackageShare('awsim_path_planner')
    trajectory_planning_pkg_dir = FindPackageShare('awsim_trajectory_planner')
    awsim_bringup_pkg_dir = FindPackageShare('awsim_bringup')
    awsim_object_tracker_pkg_dir = FindPackageShare('awsim_object_tracker')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # RViz configuration arguments
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Default RViz config in awsim_bringup package
    default_rviz_config = PathJoinSubstitution([
        awsim_bringup_pkg_dir,
        'rviz',
        'awsim.rviz'
    ])
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz configuration file'
    )
    
    ground_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ground_filter_pkg_dir, 'launch', 'ground_filter.launch.py'])
        ]),
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

    object_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([awsim_object_tracker_pkg_dir, 'launch', 'tracker.launch.py'])
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
    
    # RViz node with configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        launch_rviz_arg,
        rviz_config_arg,
        ground_filter_launch,
        localization_launch,
        delayed_path_planner_launch,
        object_tracker_launch,
        # delayed_trajectory_planner_launch,
        rviz_node,
    ])