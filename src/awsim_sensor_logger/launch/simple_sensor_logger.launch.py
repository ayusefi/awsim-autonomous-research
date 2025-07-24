from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='/tmp/awsim_sensor_data',
        description='Directory to save sensor data'
    )
    
    logging_duration_arg = DeclareLaunchArgument(
        'logging_duration_sec',
        default_value='30',
        description='Duration to log data in seconds'
    )
    
    frame_skip_arg = DeclareLaunchArgument(
        'frame_skip',
        default_value='5',
        description='Frame skip rate (saves every N-th frame)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Simple sensor logger node
    simple_sensor_logger_node = Node(
        package='awsim_sensor_logger',
        executable='simple_sensor_logger_node',
        name='simple_sensor_logger',
        parameters=[{
            'output_directory': LaunchConfiguration('output_directory'),
            'logging_duration_sec': LaunchConfiguration('logging_duration_sec'),
            'frame_skip': LaunchConfiguration('frame_skip'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )

    return LaunchDescription([
        output_directory_arg,
        logging_duration_arg,
        frame_skip_arg,
        use_sim_time_arg,
        simple_sensor_logger_node
    ])
