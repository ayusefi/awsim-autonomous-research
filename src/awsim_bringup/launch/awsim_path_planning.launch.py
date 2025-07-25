#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch AWSIM with path planning capabilities including A* and RRT* algorithms
    """
    
    # Declare arguments
    launch_awsim_arg = DeclareLaunchArgument(
        'launch_awsim', 
        default_value='true',
        description='Launch AWSIM executable'
    )
    
    use_localization_arg = DeclareLaunchArgument(
        'use_localization', 
        default_value='true',
        description='Use localization (vs ground truth pose)'
    )
    
    planning_algorithm_arg = DeclareLaunchArgument(
        'planning_algorithm',
        default_value='astar',
        description='Path planning algorithm: astar or rrt_star'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    hd_map_path_arg = DeclareLaunchArgument(
        'hd_map_path',
        default_value='',
        description='Path to HD map file (OSM format)'
    )
    
    # Get package share directories
    awsim_bringup_share = get_package_share_directory('awsim_bringup')
    path_planner_share = get_package_share_directory('awsim_path_planner')
    
    # Include localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('awsim_bringup'),
                'launch',
                'awsim_localization.launch.py'
            ])
        ]),
        launch_arguments={
            'launch_awsim': LaunchConfiguration('launch_awsim'),
            'use_rviz': 'false',  # We'll launch our own RViz
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_localization'))
    )
    
    # Path planner node
    path_planner_node = Node(
        package='awsim_path_planner',
        executable='awsim_path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[{
            'planning_algorithm': LaunchConfiguration('planning_algorithm'),
            'map_frame': 'map',
            'base_link_frame': 'base_link',
            'hd_map_path': '',  # Disabled for basic testing
            'grid_resolution': 0.5,
            'planning_timeout': 5.0,
            'max_planning_range': 200.0,
            'use_hd_map_constraints': False,  # Disabled for basic A* testing
            'visualize_search_space': True,
            
            # A* specific parameters
            'astar.heuristic_weight': 1.0,
            'astar.search_radius': 100.0,
            'astar.obstacle_inflation_radius': 1.5,
            
            # RRT* specific parameters
            'rrt_star.max_iterations': 3000,
            'rrt_star.step_size': 3.0,
            'rrt_star.goal_tolerance': 3.0,
            'rrt_star.rewiring_radius': 15.0,
            
            # Ground filtering parameters
            'ground_filter.height_threshold': 0.3,
            'ground_filter.angle_threshold': 15.0,
            'dynamic_obstacle_max_range': 50.0,
        }],
        remappings=[
            ('/localization/pose_with_covariance', '/localization/pose_with_covariance'),
            ('/planning/goal_pose', '/planning/goal_pose'),
            ('/sensing/lidar/top/pointcloud_raw', '/sensing/lidar/top/pointcloud_raw'),
        ]
    )
    
    # RViz node with path planning configuration
    rviz_config_path = os.path.join(path_planner_share, 'rviz', 'path_planner.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        launch_awsim_arg,
        use_localization_arg,
        planning_algorithm_arg,
        use_rviz_arg,
        hd_map_path_arg,
        
        # Launch groups
        GroupAction([
            localization_launch,
            path_planner_node,
            rviz_node,
        ])
    ])
