import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

fusion_dir = get_package_share_directory('sensor_fusion')
map_path = os.path.join(fusion_dir, 'config', 'my_map.pcd')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcl_localization_ros2',
            executable='pcl_localization_node',
            name='pcl_localization',
            output='screen',
            parameters=[{
                'map_file': map_path, 
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'scan_topic': '/points_downsampled',
                'use_pcd_map': True,
                'resolution': 1.0,
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_yaw': 0.0
            }]
        )
    ])
