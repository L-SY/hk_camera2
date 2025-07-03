from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('hk_camera'),
        'config',
        'dual_camera.yaml'
    )
    return LaunchDescription([
        Node(
            package='hk_camera',
            executable='hk_camera_node_main',
            name='hk_camera_node',
            output='screen',
            parameters=[{'config_file': config_path}],
        ),
    ]) 