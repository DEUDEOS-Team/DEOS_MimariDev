from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_bridge',
            executable='vision_bridge_node',
            name='vision_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/camera/image_raw',
                'output_topic': '/lane_walls',
                'confidence_threshold': 0.5,
                'max_distance': 5.0
            }]
        )
    ])
