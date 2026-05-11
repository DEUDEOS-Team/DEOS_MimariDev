import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Test architecture with mock sensors (no hardware needed)"""
    
    bringup_dir = get_package_share_directory('vehicle_bringup')
    lidar_dir = get_package_share_directory('lidar') 
    fusion_dir = get_package_share_directory('sensor_fusion')
    imu_dir = get_package_share_directory('imu')

    # A. Vehicle Sensor Locations (TF)
    sensor_locs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'sensorlocs.launch.py'))
    )

    # B. Mock Sensors (simulates GPS, IMU, Lidar)
    mock_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(imu_dir, 'launch', 'mock_sensors.launch.py'))
    )

    # C. Lidar Filter (uses mock lidar data from mock_sensors)
    lidar_filters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_dir, 'launch', 'lidarfilters.launch.py'))
    )

    # D. NDT Localization
    mappings_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fusion_dir, 'launch', 'mappings.launch.py'))
    )

    # E. Sensor Fusion (EKF)
    sensor_fus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fusion_dir, 'launch', 'sensorfus.launch.py'))
    )

    return LaunchDescription([
        sensor_locs_launch,
        mock_sensors_launch,      # Mock GPS + IMU + Lidar
        lidar_filters_launch,
        mappings_launch,
        sensor_fus_launch
    ])
