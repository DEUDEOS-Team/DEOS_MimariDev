## DEOS-Mimari — ROS2 Algorithm Pipeline Kurulumu (Jazzy)

Bu doküman `docs/superpowers/specs/2026-05-01-deos-ros2-algorithm-adaptation-design.md` tasarımına göre
repoya eklenen node/paketlerle **çalışır pipeline** kurulumunu özetler.

### 1) Paketler / Node’lar

- **Ortak algoritma kütüphanesi**: `DEOS/deos_ws/src/deos_algorithms`
  - `safety_logic`, `traffic_sign_logic`, `traffic_light_logic`, `obstacle_logic`, `slalom_logic`,
    `parking_logic`, `geojson_mission_reader`, `waypoint_manager`, `mission_manager`, `perception_fusion`
- **Perception (kamera)**: `vision_bridge`
  - `stereo_detector_node` → `/perception/stereo_detections` (`std_msgs/String`, JSON)
  - `perception_fusion_node` → `/perception/*` (`Bool/Float32`)
- **Perception (LiDAR)**: `sensor_fusion`
  - `lidar_obstacle_node` → `/perception/lidar_obstacles` (`std_msgs/String`, JSON)
- **Planning**: `mission_planning`
  - `mission_planning_node` → `/planning/*`
- **Control**: `vehicle_controller`
  - `vehicle_controller_node` → `/cmd_vel` + `/safety/emergency_stop`

### 2) Launch

Ana launch: `DEOS/deos_ws/src/vehicle_bringup/launch/main.launch.py`

- `mission_file` parametresi:
  - boşsa waypoint takibi pasif
  - doluysa GeoJSON görev dosyasından takip başlar

Örnek:

```bash
ros2 launch vehicle_bringup main.launch.py mission_file:=/home/pi/rota.geojson
```

### 3) Docker ile çalıştırma (Linux/Raspberry Pi önerilir)

> Windows üzerinde bu repodaki `docker-compose.yml` içindeki `devices:` ve `network_mode: host`
> pratikte Linux host için tasarlanmıştır.

Repo kökünden:

```bash
docker compose up -d micro_ros_agent deos
docker compose exec deos bash
```

Container içinde:

```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch vehicle_bringup main.launch.py mission_file:=/ros2_ws/src/<rota>.geojson
```

### 4) micro-ROS Agent (STM32/UDP)

`docker-compose.yml` içinde:
- `micro_ros_agent`: `udp4 -p 8888`
- `network_mode: host`

STM32 tarafında agent hedefi:
- **IP**: host (Raspberry Pi) ethernet IP
- **Port**: `8888`

### 5) Topic haritası (hızlı kontrol)

- **Stereo**: `/perception/stereo_detections` (JSON)
- **LiDAR**: `/perception/lidar_obstacles` (JSON)
- **Karar**:
  - `/perception/emergency_stop` (`std_msgs/Bool`)
  - `/perception/speed_cap` (`std_msgs/Float32`, 0..1)
  - `/perception/steering_override` (`std_msgs/Float32`, -1..+1)
  - `/perception/has_steering_override` (`std_msgs/Bool`)
- **Plan**:
  - `/planning/steering_ref` (`std_msgs/Float32`, -1..+1)
  - `/planning/speed_limit` (`std_msgs/Float32`, 0..1)
  - `/planning/current_task` (`std_msgs/String`)
  - `/planning/arrived` (`std_msgs/Bool`)
- **Çıkış**:
  - `/cmd_vel` (`geometry_msgs/Twist`)
  - `/safety/emergency_stop` (`std_msgs/Bool`)

CLI kontrol:

```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /perception/emergency_stop
```

