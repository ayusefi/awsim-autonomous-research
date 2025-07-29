from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'hd_map_path',
            default_value='/home/abdullah/workspaces/career_sprint/awsim-autonomous-research/shinjuku_map/map/lanelet2_map.osm',
            description='Path to HD map file (Lanelet2 OSM format)'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame ID'
        ),
        DeclareLaunchArgument(
            'visualization_update_rate',
            default_value='0.5',
            description='Visualization update rate in Hz'
        ),
        DeclareLaunchArgument(
            'show_lane_boundaries',
            default_value='true',
            description='Show lane boundary lines'
        ),
        DeclareLaunchArgument(
            'show_centerlines',
            default_value='true',
            description='Show lane centerlines'
        ),
        DeclareLaunchArgument(
            'show_lane_ids',
            default_value='true',
            description='Show lane ID labels'
        ),
        DeclareLaunchArgument(
            'show_speed_limits',
            default_value='true',
            description='Show speed limit labels'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'max_visualization_distance',
            default_value='200.0',
            description='Maximum distance for visualization culling'
        ),
        DeclareLaunchArgument(
            'use_combined_markers',
            default_value='false',
            description='Use combined markers for better performance'
        ),
        DeclareLaunchArgument(
            'max_lanes_to_visualize',
            default_value='10000',
            description='Maximum number of lanes to visualize'
        ),
        DeclareLaunchArgument(
            'show_traffic_lights',
            default_value='true',
            description='Show traffic lights from HD map'
        ),
        DeclareLaunchArgument(
            'show_traffic_signs',
            default_value='true',
            description='Show traffic signs from HD map'
        ),
        DeclareLaunchArgument(
            'show_crosswalks',
            default_value='true',
            description='Show crosswalks from HD map'
        ),
        DeclareLaunchArgument(
            'show_road_markings',
            default_value='true',
            description='Show road markings from HD map'
        ),
        DeclareLaunchArgument(
            'show_regulatory_elements',
            default_value='true',
            description='Show regulatory elements from HD map'
        ),
        
        # Lanelet visualizer node
        Node(
            package='awsim_path_planner',
            executable='lanelet_visualizer_node',
            name='lanelet_visualizer_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'hd_map_path': LaunchConfiguration('hd_map_path'),
                'map_frame': LaunchConfiguration('map_frame'),
                'visualization_update_rate': LaunchConfiguration('visualization_update_rate'),
                'show_lane_boundaries': LaunchConfiguration('show_lane_boundaries'),
                'show_centerlines': LaunchConfiguration('show_centerlines'),
                'show_lane_ids': LaunchConfiguration('show_lane_ids'),
                'show_speed_limits': LaunchConfiguration('show_speed_limits'),
                'max_visualization_distance': LaunchConfiguration('max_visualization_distance'),
                'use_combined_markers': LaunchConfiguration('use_combined_markers'),
                'max_lanes_to_visualize': LaunchConfiguration('max_lanes_to_visualize'),
                'show_traffic_lights': LaunchConfiguration('show_traffic_lights'),
                'show_traffic_signs': LaunchConfiguration('show_traffic_signs'),
                'show_crosswalks': LaunchConfiguration('show_crosswalks'),
                'show_road_markings': LaunchConfiguration('show_road_markings'),
                'show_regulatory_elements': LaunchConfiguration('show_regulatory_elements'),
            }]
        ),
        
        # RViz node (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_lanelet_visualizer',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', '/home/abdullah/workspaces/career_sprint/awsim-autonomous-research/src/awsim_path_planner/rviz/lanelet_visualization.rviz'],
            condition=LaunchConfigurationEquals('launch_rviz', 'true')
        ),
    ])
