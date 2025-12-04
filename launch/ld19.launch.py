from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_ld19',
            executable='ld19_node',
            name='ld19_lidar_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 230400,
                'frame_id': 'laser_frame',
                'scan_frequency': 10.0
            }]
        )
    ])