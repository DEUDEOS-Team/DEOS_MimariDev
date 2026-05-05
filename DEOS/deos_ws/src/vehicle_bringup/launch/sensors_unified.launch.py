from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    lidar_hostname_arg = DeclareLaunchArgument(
        'lidar_hostname',
        default_value='10.10.10.211',
        description='IP address of SICK Multiscan165 LiDAR'
    )
    
    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='laser',
        description='TF frame ID for LiDAR'
    )
    
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='USB camera device ID'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='Camera frame rate in Hz'
    )
    
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for IMU'
    )
    
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='mpu9250',
        description='IMU type'
    )
    
    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for GPS'
    )
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable SICK Multiscan165 LiDAR'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable RealSense D415 Camera'
    )
    
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='Enable IMU sensor'
    )
    
    enable_gps_arg = DeclareLaunchArgument(
        'enable_gps',
        default_value='true',
        description='Enable GPS sensor'
    )
    
    lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_multiscan165',
        parameters=[{
            'hostname': LaunchConfiguration('lidar_hostname'),
            'scanner_type': 'sick_multiscan',
            'frame_id': LaunchConfiguration('lidar_frame_id'),
            'udp_receiver_ip': '10.10.10.210',
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )
    
    camera_node = Node(
        package='camera',
        executable='realsense_d415_node',
        name='realsense_d415',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('camera_device'),
            'fps': LaunchConfiguration('camera_fps'),
            'frame_width': 640,
            'frame_height': 480,
        }],
        respawn=True,
        respawn_delay=2,
        condition=IfCondition(LaunchConfiguration('enable_camera')),
    )
    
    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baudrate': 115200,
            'frame_id': 'imu_link',
            'imu_type': LaunchConfiguration('imu_type'),
        }],
        respawn=True,
        respawn_delay=2,
        condition=IfCondition(LaunchConfiguration('enable_imu')),
    )
    
    gps_node = Node(
        package='imu',
        executable='gps_node',
        name='gps_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('gps_port'),
            'baudrate': 9600,
            'frame_id': 'gps_link',
        }],
        respawn=True,
        respawn_delay=2,
        condition=IfCondition(LaunchConfiguration('enable_gps')),
    )
    
    return LaunchDescription([
        lidar_hostname_arg,
        lidar_frame_id_arg,
        camera_device_arg,
        camera_fps_arg,
        imu_port_arg,
        imu_type_arg,
        gps_port_arg,
        enable_lidar_arg,
        enable_camera_arg,
        enable_imu_arg,
        enable_gps_arg,
        lidar_node,
        camera_node,
        imu_node,
        gps_node,
    ])
