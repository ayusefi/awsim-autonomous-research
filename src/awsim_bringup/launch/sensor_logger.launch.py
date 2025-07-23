from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='awsim_sensor_logger',
            executable='sensor_logger_node',
            name='sensor_logger',
            output='screen',
            parameters=[]
        )
    ])
