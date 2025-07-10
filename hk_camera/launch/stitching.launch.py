from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hk_camera_share_dir = get_package_share_directory('hk_camera')
    
    # 基础拼接配置文件
    camera_stitching_config = os.path.join(
        hk_camera_share_dir,
        'config',
        'camera_stitching.yaml'
    )
    
    # 连续拼接配置文件
    continuous_stitching_config = os.path.join(
        hk_camera_share_dir,
        'config',
        'stitching.yaml'
    )
    
    return LaunchDescription([
        # 基础相机拼接节点
        Node(
            package='hk_camera',
            executable='camera_stitching_node_main',
            name='camera_stitching_node',
            output='screen',
            parameters=[camera_stitching_config],
        ),
        
        # 连续拼接节点（只使用自己的配置参数）
        Node(
            package='hk_camera',
            executable='stitching_node_main',
            name='stitching_node',
            output='screen',
            parameters=[continuous_stitching_config],
        ),
    ]) 