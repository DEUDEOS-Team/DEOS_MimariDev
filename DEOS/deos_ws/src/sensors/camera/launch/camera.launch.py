from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'device_id': 0,              # USB camera device ID
                'frame_width': 640,
                'frame_height': 480,
                'fps': 30
            }]
        )
    ])
