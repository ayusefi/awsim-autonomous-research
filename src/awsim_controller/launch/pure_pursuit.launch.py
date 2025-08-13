#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('awsim_controller')
    
    # Launch arguments for common tuning/IO overrides
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation time'
    )
    
    path_topic = DeclareLaunchArgument(
        'path_topic', 
        default_value='/planning/trajectory_planner/local_trajectory',
        description='Topic name for path subscription'
    )
    
    pose_topic = DeclareLaunchArgument(
        'pose_topic', 
        default_value='/localization/pose_with_covariance',
        description='Topic name for pose subscription'
    )
    
    velocity_topic = DeclareLaunchArgument(
        'velocity_topic', 
        default_value='/vehicle/status/velocity_status',
        description='Topic name for velocity subscription'
    )
    
    control_topic = DeclareLaunchArgument(
        'control_topic',
        default_value='/control/command/control_cmd',
        description='Topic name for control command publication'
    )
    
    gear_topic = DeclareLaunchArgument(
        'gear_topic',
        default_value='/control/command/gear_cmd',
        description='Topic name for gear command publication'
    )
    
    visualization_topic = DeclareLaunchArgument(
        'visualization_topic',
        default_value='/pure_pursuit/markers',
        description='Topic name for visualization markers'
    )
    
    # Pure Pursuit Controller parameters
    lookahead_distance_min = DeclareLaunchArgument(
        'lookahead_distance_min',
        default_value='3.0',
        description='Minimum lookahead distance [m]'
    )
    
    lookahead_distance_max = DeclareLaunchArgument(
        'lookahead_distance_max',
        default_value='20.0',
        description='Maximum lookahead distance [m]'
    )
    
    lookahead_gain = DeclareLaunchArgument(
        'lookahead_gain',
        default_value='1.5',
        description='Lookahead distance gain based on velocity'
    )
    
    wheelbase = DeclareLaunchArgument(
        'wheelbase',
        default_value='2.79',
        description='Vehicle wheelbase [m]'
    )
    
    max_steering_angle = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.7854',
        description='Maximum steering angle [rad]'
    )
    
    target_velocity = DeclareLaunchArgument(
        'target_velocity',
        default_value='10.0',
        description='Target velocity [m/s]'
    )
    
    max_acceleration = DeclareLaunchArgument(
        'max_acceleration',
        default_value='3.0',
        description='Maximum acceleration [m/s²]'
    )
    
    max_deceleration = DeclareLaunchArgument(
        'max_deceleration',
        default_value='-5.0',
        description='Maximum deceleration [m/s²]'
    )
    
    curve_decel_factor = DeclareLaunchArgument(
        'curve_decel_factor',
        default_value='0.5',
        description='Deceleration factor for curves'
    )
    
    curve_threshold = DeclareLaunchArgument(
        'curve_threshold',
        default_value='0.3',
        description='Curvature threshold for curve detection [1/m]'
    )
    
    velocity_tolerance = DeclareLaunchArgument(
        'velocity_tolerance',
        default_value='0.5',
        description='Velocity tolerance for control [m/s]'
    )
    
    path_tolerance = DeclareLaunchArgument(
        'path_tolerance',
        default_value='0.1',
        description='Distance tolerance to path [m]'
    )
    
    # Pure Pursuit Controller Node
    pure_pursuit_node = Node(
        package='awsim_controller',
        executable='pure_pursuit_node',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lookahead_distance_min': LaunchConfiguration('lookahead_distance_min'),
            'lookahead_distance_max': LaunchConfiguration('lookahead_distance_max'),
            'lookahead_gain': LaunchConfiguration('lookahead_gain'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'target_velocity': LaunchConfiguration('target_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'max_deceleration': LaunchConfiguration('max_deceleration'),
            'curve_decel_factor': LaunchConfiguration('curve_decel_factor'),
            'curve_threshold': LaunchConfiguration('curve_threshold'),
            'velocity_tolerance': LaunchConfiguration('velocity_tolerance'),
            'path_tolerance': LaunchConfiguration('path_tolerance'),
        }],
        remappings=[
            ('/planning/path', LaunchConfiguration('path_topic')),
            ('/localization/pose_with_covariance', LaunchConfiguration('pose_topic')),
            ('/vehicle/status/velocity_status', LaunchConfiguration('velocity_topic')),
            ('/control/command/control_cmd', LaunchConfiguration('control_topic')),
            ('/control/command/gear_cmd', LaunchConfiguration('gear_topic')),
            ('/pure_pursuit/markers', LaunchConfiguration('visualization_topic')),
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        path_topic,
        pose_topic,
        velocity_topic,
        control_topic,
        gear_topic,
        visualization_topic,
        lookahead_distance_min,
        lookahead_distance_max,
        lookahead_gain,
        wheelbase,
        max_steering_angle,
        target_velocity,
        max_acceleration,
        max_deceleration,
        curve_decel_factor,
        curve_threshold,
        velocity_tolerance,
        path_tolerance,
        
        # Nodes
        pure_pursuit_node,
    ])


if __name__ == '__main__':
    generate_launch_description()