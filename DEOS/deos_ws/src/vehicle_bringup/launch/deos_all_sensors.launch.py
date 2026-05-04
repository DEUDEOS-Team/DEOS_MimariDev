from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Master launch file for DEOS - launches all sensor nodes
    """
    
    return LaunchDescription([
        # RealSense D415 Camera
        Node(
            package='camera',
            executable='realsense_d415_node',
            name='realsense_d415_node',
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        
        # Quectel L86 EVB GPS
        Node(
            package='imu',
            executable='gps_node',
            name='gps_node',
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        
        # 6-DOF IMU
        Node(
            package='imu',
            executable='imu_node',
            name='imu_node',
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        
        # SICK Multiscan165 LiDAR
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_multiscan165',
            parameters=[{
                'hostname': '192.168.0.1',
                'scanner_type': 'sick_multiscan165',
                'frame_id': 'laser',
            }],
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
        
        # Vision Bridge (Lane Detection)
        Node(
            package='vision_bridge',
            executable='vision_bridge_node',
            name='vision_bridge_node',
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),
    ])
