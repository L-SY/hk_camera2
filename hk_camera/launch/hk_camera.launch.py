from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hk_camera',
            executable='hk_camera_node_main',
            name='hk_camera_node',
            output='screen',
            parameters=[{
                'loop_rate_hz': 200,
                'camera_name': 'cs050',
                'serial_number': 'DA6320570',
                'exposure_auto': True,
                'exposure_min': 15,
                'exposure_max': 13319,
                'exposure_mode': 0,
                'exposure_value': 5000.0,
                'gain_auto': True,
                'gain_min': 0.0,
                'gain_max': 22.0,
                'gain_value': 6.0,
                'white_balance_auto': True,
                'roi_width': 2448,
                'roi_height': 2048,
                'roi_offset_x': 0,
                'roi_offset_y': 0
            }]
        )
    ]) 