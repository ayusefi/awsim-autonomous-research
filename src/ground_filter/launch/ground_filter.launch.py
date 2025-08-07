from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    max_height_arg = DeclareLaunchArgument(
        'max_height',
        default_value='2.0',
        description='Maximum height above the vehicle to consider a point (meters)'
    )
    # Declare launch arguments with default values
    distance_threshold_arg = DeclareLaunchArgument(
        'distance_threshold',
        default_value='0.2',
        description='Maximum distance from plane to consider a point as ground (meters)'
    )
    
    max_angle_arg = DeclareLaunchArgument(
        'max_angle',
        default_value='15.0',
        description='Maximum allowed angle from horizontal (degrees)'
    )
    
    min_inliers_arg = DeclareLaunchArgument(
        'min_inliers',
        default_value='1000',
        description='Minimum number of points to consider a plane valid'
    )
    
    max_iterations_arg = DeclareLaunchArgument(
        'max_iterations',
        default_value='1000',
        description='Maximum number of RANSAC iterations'
    )
    
    grid_size_arg = DeclareLaunchArgument(
        'grid_size',
        default_value='1.0',
        description='Grid cell size for ground modeling (meters)'
    )
    
    height_threshold_arg = DeclareLaunchArgument(
        'height_threshold',
        default_value='0.3',
        description='Height threshold for grid-based filtering (meters)'
    )
    
    k_multiplier_arg = DeclareLaunchArgument(
        'k_multiplier',
        default_value='2.0',
        description='Multiplier for standard deviation threshold'
    )
    
    use_grid_filter_arg = DeclareLaunchArgument(
        'use_grid_filter',
        default_value='true',
        description='Enable grid-based refinement'
    )

    return LaunchDescription([
        # Declare all launch arguments
        distance_threshold_arg,
        max_angle_arg,
        min_inliers_arg,
        max_iterations_arg,
        grid_size_arg,
        height_threshold_arg,
        k_multiplier_arg,
        use_grid_filter_arg,
        max_height_arg,
        
        # Ground filter node
        Node(
            package='ground_filter',
            executable='ground_filter_node',
            name='ground_filter',
            output='screen',
            parameters=[
                {
                    'distance_threshold': LaunchConfiguration('distance_threshold'),
                    'max_angle': LaunchConfiguration('max_angle'),
                    'min_inliers': LaunchConfiguration('min_inliers'),
                    'max_iterations': LaunchConfiguration('max_iterations'),
                    'grid_size': LaunchConfiguration('grid_size'),
                    'height_threshold': LaunchConfiguration('height_threshold'),
                    'k_multiplier': LaunchConfiguration('k_multiplier'),
                    'use_grid_filter': LaunchConfiguration('use_grid_filter'),
                    'max_height': LaunchConfiguration('max_height'),
                }
            ],
            remappings=[
                ('/sensing/lidar/top/pointcloud_raw', '/sensing/lidar/top/pointcloud_raw'),
                ('ground_points', '/ground_filter/ground_points'),
                ('nonground_points', '/ground_filter/nonground_points')
            ]
        ),
    ])