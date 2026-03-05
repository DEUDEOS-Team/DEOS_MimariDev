import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Sistemdeki Paketlerin Yollarını Bul
    bringup_dir = get_package_share_directory('vehicle_bringup')
    
    # Not: Eğer lidarfilters.py dosyasını sensors klasörü altında 'lidar' adında 
    # bir pakete koyduysan burası 'lidar' kalmalı. Başka yere koyduysan adını değiştir.
    lidar_dir = get_package_share_directory('lidar') 
    
    fusion_dir = get_package_share_directory('sensor_fusion')

    # 2. Alt Launch Dosyalarını Dahil Et (Include)
    
    # A. Aracın İskeleti (Sensör Konumları - TF)
    sensor_locs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'sensorlocs.launch.py'))
    )

    # B. Lidar Sürücüsü ve Voxel Filtresi
    lidar_filters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_dir, 'launch', 'lidarfilters.launch.py'))
    )

    # C. NDT Harita Eşleştirme (Lokalizasyon)
    mappings_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fusion_dir, 'launch', 'mappings.launch.py'))
    )

    # D. EKF ve GPS (Sensör Füzyonu)
    sensor_fus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fusion_dir, 'launch', 'sensorfus.launch.py'))
    )

    # 3. Bütün Düğümleri Aynı Anda Başlat
    return LaunchDescription([
        sensor_locs_launch,
        lidar_filters_launch,
        mappings_launch,
        sensor_fus_launch
    ])