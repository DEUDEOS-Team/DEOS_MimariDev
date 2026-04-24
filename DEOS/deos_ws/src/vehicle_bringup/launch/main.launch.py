from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    DEOS Main Launch File - Complete Autonomous Vehicle Stack
    
    Launches:
    1. Sensors (Camera, GPS, IMU, LiDAR)
    2. Localization (PCL localization)
    
    Data Flow:
    /camera/color/image_raw (30 Hz) → Lane detection → /lane_walls (obstacles)
    /gps/fix (5-10 Hz) → Mission planning → /goal_pose
    /imu/data (100 Hz) ──┐
    /scan_fullframe (20 Hz) ├→ PCL Localization → /odometry/icp
    /cloud_unstructured (20 Hz) ┘
    
    /lane_walls + /odometry/icp → Local navigation → /cmd_vel
    /cmd_vel → Vehicle controller → Motors/Steering
    """
    
    # Sensor Nodes
    camera_node = Node(
        package='camera',
        executable='realsense_d415_node',
        name='realsense_d415_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    gps_node = Node(
        package='imu',
        executable='gps_node',
        name='gps_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_multiscan165',
        parameters=[{
            'hostname': '192.168.0.1',
            'scanner_type': 'sick_multiscan',
            'frame_id': 'laser',
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    # Localization - PCL based (scan matching)
    pcl_localization_node = Node(
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        name='pcl_localization_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    return LaunchDescription([
        # === SENSORS (Raw Data Acquisition) ===
        camera_node,           # RGB frames: /camera/color/image_raw (30 Hz)
        gps_node,              # GPS location: /gps/fix (5-10 Hz)
        imu_node,              # Inertial data: /imu/data (100 Hz)
        lidar_node,            # 3D scans: /scan_fullframe, /cloud_unstructured_fullframe (20 Hz)
        
        # === LOCALIZATION (Position Estimation) ===
        pcl_localization_node, # Scan matching: /odometry/icp, /tf (20 Hz)
        
        # === PIPELINE ===
        # Camera → Lane detection → /lane_walls (obstacles to avoid)
        # LiDAR + IMU → PCL localization → /odometry/icp (vehicle position)
        # Lane walls + Position → Local navigation → /cmd_vel (steering commands)
        # Steering commands → Vehicle controller → Motors/Steering servos
    ])
