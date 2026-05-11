from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for SICK Multiscan165 LiDAR
    
    Configuration:
    - Hostname: 192.168.0.1 (factory default, change if modified)
    - TCP Port: 2115 (SICK IPC protocol)
    - Frame ID: laser (used in ROS 2 transforms)
    """
    
    # Launch arguments with defaults
    hostname_arg = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.0.1',
        description='IP address of SICK Multiscan165'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='TF frame ID for the LiDAR'
    )
    
    return LaunchDescription([
        hostname_arg,
        frame_id_arg,
        
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_multiscan165',
            parameters=[{
                'hostname': LaunchConfiguration('hostname'),
                'scanner_type': 'sick_multiscan165',
                'frame_id': LaunchConfiguration('frame_id'),
            }],
            output='screen',
            respawn=True,
            respawn_delay=2,
        )
    ])
