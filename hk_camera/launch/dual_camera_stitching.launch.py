from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hk_camera_share_dir = get_package_share_directory('hk_camera')
    
    config_file = os.path.join(
        hk_camera_share_dir,
        'config',
        'dual_camera_stitching.yaml'
    )
    
    homography_path = os.path.join(
        hk_camera_share_dir,
        'files',
        'H_right_to_left.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='hk_camera',
            executable='hk_camera_stitching_node_main',
            name='hk_camera_stitching_node',
            output='screen',
            parameters=[config_file],
        ),
    ]) 