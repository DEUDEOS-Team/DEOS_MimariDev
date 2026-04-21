import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_lidar',
            output='screen',
            parameters=[{
            
                'scanner_type': 'sick_multiscan',
                'hostname': '192.168.0.1',
                'udp_receiver_ip': '192.168.0.100' #RASPİ IP GİRİLECEK !!
                }] 
        ),

        Node(
            package='pcl_ros',
            executable='filter_voxel_grid_node',
            name='voxel_grid_filter',
            output='screen',
            parameters=[{
                'filter_field_name': 'z',
                'filter_limit_min': -0.5, 
                'filter_limit_max': 2.0,  
                'filter_limit_negative': False,
                'leaf_size': 0.2,         
                'input_frame': 'cloud',   
                'output_frame': 'base_link'
            }],
            remappings=[
                ('input', '/cloud'),            
                ('output', '/points_downsampled') 
            ]
        )
    ])

