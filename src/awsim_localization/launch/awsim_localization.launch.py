#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('awsim_localization')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'algorithm_type',
            default_value='ndt', 
            description='Localization algorithm: ndt or icp'
        ),
        DeclareLaunchArgument(
            'map_path', 
            default_value='/home/abdullah/workspaces/career_sprint/awsim-autonomous-research/shinjuku_map/map/pointcloud_map.pcd',
            description='Path to the point cloud map file'
        ),
        DeclareLaunchArgument(
            'map_frame', 
            default_value='map',
            description='Map frame ID'
        ),
        DeclareLaunchArgument(
            'base_link_frame', 
            default_value='base_link',
            description='Base link frame ID'
        ),
        DeclareLaunchArgument(
            'odom_frame', 
            default_value='odom',
            description='Odometry frame ID'
        ),
        DeclareLaunchArgument(
            'lidar_frame', 
            default_value='velodyne_top',
            description='LiDAR sensor frame ID'
        ),
        DeclareLaunchArgument(
            'ndt_resolution', 
            default_value='2.0',
            description='NDT resolution'
        ),
        DeclareLaunchArgument(
            'ndt_max_iterations', 
            default_value='35',
            description='NDT maximum iterations'
        ),
        DeclareLaunchArgument(
            'ndt_transformation_epsilon', 
            default_value='0.01',
            description='NDT transformation epsilon'
        ),
        DeclareLaunchArgument(
            'ndt_step_size', 
            default_value='0.1',
            description='NDT step size'
        ),
        DeclareLaunchArgument(
            'ndt_num_threads', 
            default_value='4',
            description='Number of threads for NDT'
        ),
        DeclareLaunchArgument(
            'icp_max_iterations', 
            default_value='35',
            description='ICP/GICP maximum iterations (optimized from reference)'
        ),
        DeclareLaunchArgument(
            'icp_transformation_epsilon', 
            default_value='0.01',
            description='ICP/GICP transformation epsilon (optimized from reference)'
        ),
        DeclareLaunchArgument(
            'icp_max_correspondence_distance', 
            default_value='1.0',
            description='ICP/GICP maximum correspondence distance'
        ),
        DeclareLaunchArgument(
            'icp_euclidean_fitness_epsilon', 
            default_value='0.01',
            description='ICP/GICP euclidean fitness epsilon'
        ),
        DeclareLaunchArgument(
            'voxel_leaf_size', 
            default_value='0.5',
            description='Voxel grid leaf size (optimized for GICP from reference)'
        ),
        DeclareLaunchArgument(
            'lidar_max_range', 
            default_value='100.0',
            description='Maximum LiDAR range'
        ),
        DeclareLaunchArgument(
            'lidar_min_range', 
            default_value='1.0',
            description='Minimum LiDAR range'
        ),
        DeclareLaunchArgument(
            'scan_matching_score_threshold', 
            default_value='200.0',
            description='Scan matching fitness score threshold (optimized for GICP)'
        ),
        DeclareLaunchArgument(
            'publish_tf', 
            default_value='true',
            description='Whether to publish TF transforms'
        ),
        DeclareLaunchArgument(
            'use_ground_truth_init', 
            default_value='false',
            description='Whether to initialize pose from ground truth'
        ),
        DeclareLaunchArgument(
            'launch_rviz', 
            default_value='true',
            description='Whether to launch RViz2 for visualization'
        ),
        
        # Map centering parameters
        DeclareLaunchArgument(
            'use_manual_map_center', 
            default_value='true',
            description='Use manual map center coordinates instead of auto-calculated centroid'
        ),
        DeclareLaunchArgument(
            'manual_map_center_x', 
            default_value='0.0',
            description='Manual map center X coordinate (Shinjuku map default)'
        ),
        DeclareLaunchArgument(
            'manual_map_center_y', 
            default_value='0.0',
            description='Manual map center Y coordinate (Shinjuku map default)'
        ),
        DeclareLaunchArgument(
            'manual_map_center_z', 
            default_value='42.12',
            description='Manual map center Z coordinate (Shinjuku map default)'
        ),

        # AWSIM Localization Node
        Node(
            package='awsim_localization',
            executable='awsim_localization_node',
            name='awsim_localization_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # Enable simulation time for AWSIM
                'algorithm_type': LaunchConfiguration('algorithm_type'),
                'map_path': LaunchConfiguration('map_path'),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_link_frame': LaunchConfiguration('base_link_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'lidar_frame': LaunchConfiguration('lidar_frame'),
                'use_manual_map_center': LaunchConfiguration('use_manual_map_center'),
                'manual_map_center_x': LaunchConfiguration('manual_map_center_x'),
                'manual_map_center_y': LaunchConfiguration('manual_map_center_y'),
                'manual_map_center_z': LaunchConfiguration('manual_map_center_z'),
                'ndt_resolution': LaunchConfiguration('ndt_resolution'),
                'ndt_max_iterations': LaunchConfiguration('ndt_max_iterations'),
                'ndt_transformation_epsilon': LaunchConfiguration('ndt_transformation_epsilon'),
                'ndt_step_size': LaunchConfiguration('ndt_step_size'),
                'ndt_num_threads': LaunchConfiguration('ndt_num_threads'),
                'icp_max_iterations': LaunchConfiguration('icp_max_iterations'),
                'icp_transformation_epsilon': LaunchConfiguration('icp_transformation_epsilon'),
                'icp_max_correspondence_distance': LaunchConfiguration('icp_max_correspondence_distance'),
                'icp_euclidean_fitness_epsilon': LaunchConfiguration('icp_euclidean_fitness_epsilon'),
                'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
                'lidar_max_range': LaunchConfiguration('lidar_max_range'),
                'lidar_min_range': LaunchConfiguration('lidar_min_range'),
                'scan_matching_score_threshold': LaunchConfiguration('scan_matching_score_threshold'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'use_ground_truth_init': LaunchConfiguration('use_ground_truth_init'),
            }],
            remappings=[
                # Remap topics for AWSIM compatibility if needed
            ]
        ),

        # ============================================================================
        # AWSIM Static Transforms - Official Sensor Hierarchy
        # Based on: https://github.com/tier4/AWSIM/tree/main/Assets/AWSIM/Externals/RGLUnityPlugin/Resources/SensorKit
        # ============================================================================
        
        # 1. base_link -> sensor_kit_base_link (main sensor mount)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sensor_kit_static_tf',
            arguments=[
                '0.9', '0.0', '2.0',  # translation (AWSIM official)
                '-0.000363', '0.007508', '-0.018195', '0.999806',  # quaternion (roll:-0.001, pitch:0.015, yaw:-0.0364)
                LaunchConfiguration('base_link_frame'),  # parent frame
                'sensor_kit_base_link'  # child frame
            ]
        ),
        
        # 2. sensor_kit_base_link -> velodyne_top_base_link (LiDAR)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_kit_to_lidar_static_tf',
            arguments=[
                '0.0', '0.0', '0.0',  # translation (no offset from sensor kit)
                '0.000000', '0.000000', '0.708591', '0.705619',  # quaternion (yaw:1.575 rad = 90.25°)
                'sensor_kit_base_link',  # parent frame
                LaunchConfiguration('lidar_frame')  # child frame (velodyne_top)
            ]
        ),
        
        # 3. sensor_kit_base_link -> gnss_link (GNSS/GPS sensor)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_kit_to_gnss_static_tf',
            arguments=[
                '-0.1', '0.0', '-0.2',  # translation (behind and below sensor kit)
                '0.000000', '0.000000', '0.000000', '1.000000',  # quaternion (no rotation)
                'sensor_kit_base_link',  # parent frame
                'gnss_link'  # child frame
            ]
        ),
        
        # 4. sensor_kit_base_link -> imu_link (IMU - Tamagawa)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_kit_to_imu_static_tf',
            arguments=[
                '0.0', '0.0', '0.0',  # translation (co-located with sensor kit)
                '0.000001', '1.000000', '0.000001', '0.000000',  # quaternion (roll:180°, yaw:180°)
                'sensor_kit_base_link',  # parent frame
                'tamagawa/imu_link'  # child frame (AWSIM topic naming)
            ]
        ),
        
        # 5. sensor_kit_base_link -> traffic_light_left_camera_link (Left Camera)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_kit_to_camera_left_static_tf',
            arguments=[
                '0.05', '0.0175', '-0.1',  # translation (forward, left, down from sensor kit)
                '0.000000', '0.000000', '0.000000', '1.000000',  # quaternion (no rotation)
                'sensor_kit_base_link',  # parent frame
                'traffic_light_left_camera_link'  # child frame
            ]
        ),
        
        # 6. sensor_kit_base_link -> traffic_light_right_camera_link (Right Camera)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sensor_kit_to_camera_right_static_tf',
            arguments=[
                '0.05', '-0.0175', '-0.1',  # translation (forward, right, down from sensor kit)
                '0.000000', '0.000000', '0.000000', '1.000000',  # quaternion (no rotation)
                'sensor_kit_base_link',  # parent frame
                'traffic_light_right_camera_link'  # child frame
            ]
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],  # Enable simulation time for AWSIM
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'awsim_localization.rviz')],
            condition=IfCondition(LaunchConfiguration('launch_rviz'))
        ),
    ])
