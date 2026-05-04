from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Unified Sensor Launch File - Start all sensors with custom parameters
    
    Usage:
        ros2 launch vehicle_bringup sensors_unified.launch.py \\
            lidar_hostname:=10.10.10.211 \\
            imu_port:=/dev/ttyUSB1 \\
            gps_port:=/dev/ttyUSB0 \\
            camera_device:=0
    
    Published Topics:
    - /cloud (PointCloud2) - LiDAR 3D point cloud
    - /camera/color/image_raw (Image) - RGB camera frame
    - /imu/data (Imu) - Inertial measurement data
    - /gps/fix (NavSatFix) - GPS location
    """
    
    # LIDAR PARAMETERS
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
    
    # CAMERA PARAMETERS
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='USB camera device ID'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='Camera frame rate (Hz)'
    )
    
    # IMU PARAMETERS
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for IMU'
    )
    
    imu_baudrate_arg = DeclareLaunchArgument(
        'imu_baudrate',
        default_value='115200',
        description='IMU serial baudrate'
    )
    
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='mpu9250',
        description='IMU type (mpu9250, vectornav, xsens)'
    )
    
    # GPS PARAMETERS
    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for GPS'
    )
    
    gps_baudrate_arg = DeclareLaunchArgument(
        'gps_baudrate',
        default_value='9600',
        description='GPS serial baudrate'
    )
    
    # ENABLE/DISABLE FLAGS
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
    
    # LIDAR NODE - Publishes /cloud
    lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_multiscan165',
        parameters=[{
            'hostname': LaunchConfiguration('lidar_hostname'),
            'scanner_type': 'sick_multiscan165',
            'frame_id': LaunchConfiguration('lidar_frame_id'),
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
        condition=IfCondition(LaunchConfiguration('enable_lidar')),
    )
    
    # CAMERA NODE - Publishes /camera/color/image_raw
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
    
    # IMU NODE - Publishes /imu/data
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
    
    # GPS NODE - Publishes /gps/fix
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
        imu_baudrate_arg,
        imu_type_arg,
        gps_port_arg,
        gps_baudrate_arg,
        enable_lidar_arg,
        enable_camera_arg,
        enable_imu_arg,
        enable_gps_arg,
        lidar_node,
        camera_node,
        imu_node,
        gps_node,
    ])
