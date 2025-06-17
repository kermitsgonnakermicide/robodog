from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leg_walker',
            executable='walk_node',
            name='walk_node',
            output='screen',
            parameters=[{
                'L1': 0.1,
                'L2': 0.1
            }]
        )
    ])
