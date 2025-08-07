from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('awsim_trajectory_planner'),
            'config',
            'trajectory_planner_params.yaml'
        ]),
        description='Path to trajectory planner parameters file'
    )

    # Trajectory planner node
    trajectory_planner_node = Node(
        package='awsim_trajectory_planner',
        executable='trajectory_planner_node',
        name='trajectory_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Input topics
            ('/planning/global_path', '/planning/path'),
            ('/localization/odometry', '/localization/pose_with_covariance'),
            ('/sensing/lidar/concatenated/pointcloud', '/ground_filter/nonground_points'),
            
            # Output topics
            ('/planning/local_trajectory', '/planning/trajectory_planner/local_trajectory'),
            ('/planning/candidate_trajectories', '/planning/trajectory_planner/candidate_trajectories'),
            ('/planning/trajectory_visualization', '/planning/trajectory_planner/visualization')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        trajectory_planner_node
    ])
