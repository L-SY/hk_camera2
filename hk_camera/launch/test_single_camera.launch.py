from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hk_camera',
            executable='test_single_camera',
            name='test_single_camera',
            output='screen',
        )
    ]) 