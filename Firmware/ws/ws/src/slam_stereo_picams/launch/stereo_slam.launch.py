from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = ['config/stereo_params.yaml']
    return LaunchDescription([
        Node(
            package='slam_stereo_picams',
            executable='stereo_slam_node',
            name='stereo_slam_node',
            output='screen',
            parameters=params
        )
    ])
