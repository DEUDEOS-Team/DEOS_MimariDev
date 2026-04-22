import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Get Package Directories
    bringup_dir = get_package_share_directory('vehicle_bringup')
    lidar_dir = get_package_share_directory('lidar') 
    fusion_dir = get_package_share_directory('sensor_fusion')
    camera_dir = get_package_share_directory('camera')
    imu_dir = get_package_share_directory('imu')
    perception_dir = get_package_share_directory('perception')

    # 2. Sensor Launches
    
    # A. Vehicle Sensor Locations (TF - Transform Frame)
    sensor_locs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'sensorlocs.launch.py'))
    )

    # B. Camera Capture
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(camera_dir, 'launch', 'camera.launch.py'))
    )

    # C. Lidar Driver + Voxel Filter
    lidar_filters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_dir, 'launch', 'lidarfilters.launch.py'))
    )

    # D. IMU + GPS Drivers
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(imu_dir, 'launch', 'sensors.launch.py'))
    )

    # 3. Perception Layer
    
    # E. Vision Processing (Lane detection)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(perception_dir, 'launch', 'perception.launch.py'))
    )

    # F. NDT Localization (Lidar-based map matching)
    mappings_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fusion_dir, 'launch', 'mappings.launch.py'))
    )

    # G. Sensor Fusion (EKF + NavSAT) - Fuses Lidar + GPS + IMU
    sensor_fus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fusion_dir, 'launch', 'sensorfus.launch.py'))
    )

    # 4. Start All Nodes
    return LaunchDescription([
        sensor_locs_launch,      # TF setup
        camera_launch,           # Camera driver
        lidar_filters_launch,    # Lidar + filtering
        sensors_launch,          # GPS + IMU drivers
        perception_launch,       # Vision processing
        mappings_launch,         # Localization
        sensor_fus_launch        # Multi-sensor fusion
    ])
