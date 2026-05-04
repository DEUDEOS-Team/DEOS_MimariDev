from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='realsense_d415_node',
            name='realsense_d415',
            output='screen',
            parameters=[{
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'color_info_topic': '/camera/color/camera_info',
                'depth_info_topic': '/camera/depth/camera_info',
                'frame_width': 640,
                'frame_height': 480,
                'fps': 30
            }]
        )
    ])
