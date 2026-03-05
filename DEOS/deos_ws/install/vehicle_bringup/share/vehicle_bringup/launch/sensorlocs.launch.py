import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        
        # --- SENSÖR KONUMLARI ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_publisher',
            arguments=['0.95','0.0','1.37','0','0','0','base_link','cloud']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            arguments=['0.95','0.0','1.37','0','0','0','base_link','camera_link']
        )
    ])
