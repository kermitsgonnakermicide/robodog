from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_driver',
            executable='servo_driver_node',
            name='servo_driver',
            output='screen'
        )
    ])
