# DEOS-Mimari

Bu repo, Raspberry Pi üzerinde çalışan **ROS 2 Jazzy** tabanlı bir otonom araç yazılım mimarisini içerir.
Sensörlerden gelen veriler (kamera/derinlik, LiDAR, GPS, IMU) ROS 2 graph’ında işlenir; karar ve planlama katmanları
çıktıyı **standart mesaj tipleriyle** üretir; kontrol çıktısı **micro-ROS (STM32) üzerinden UDP** ile araca aktarılır.

Bu README “nasıl kurulur”dan çok **mimariyi** (paketler, node’lar, topic sözleşmeleri, veri akışı, öncelikler) açıklar.

> Kurulum/çalıştırma adımları için ayrıca `SETUP_ROS2_PIPELINE.md` dosyasına bakın.
>
> Raspberry Pi’de güç verildiğinde otomatik kalkış (systemd + Docker) için `OPS_AUTOSTART.md` dosyasına bakın.

## Raspberry Pi: repo yolu ve systemd

Otomatik bringup/health yapısı (`ops/systemd/*.service`, `ops/scripts/*`) **sabit dosya yolları** kullanır; böylece servisler her boot’ta aynı komutlarla çalışır.

### Önerilen konum

- Repo kökünü cihazda **`/opt/deos`** altına koyun (klon veya kopya).
- `docker compose` komutları repo kökünden (`docker-compose.yml` burada olduğu için) çalıştırılır; `WorkingDirectory=/opt/deos` buna göre ayarlıdır.

Bu varsayılan, dokümantasyondaki (`OPS_AUTOSTART.md`) kopyalama yollarıyla uyumludur.

### Yolu değiştirebilir miyiz?

Evet. İki pratik yöntem:

1. **Sembolik bağlantı (en az değişiklik)**  
   Repoyu örneğin `/home/pi/DEOS-Mimari` altında tutup systemd’nin beklediği yolu bağlayın:

   ```bash
   sudo ln -sfn /home/pi/DEOS-Mimari /opt/deos
   ```

   Unit dosyalarını değiştirmeniz gerekmez (gerçek dosyalar `readlink -f /opt/deos` ile çözülür).

2. **Unit ve ortam değişkenlerini güncelleme**  
   Repoyu başka bir dizine (`/veri/deos` gibi) koyacaksanız şunları **aynı köke** işaret edecek şekilde düzenleyin:

   - `/etc/systemd/system/deos-bringup.service` içindeki `WorkingDirectory=`, `ExecStart=`, `ExecStop=` yolları (`ExecStart` mutlaka o kopyadaki `ops/scripts/deos_compose_up.sh` dosyasına gitsin)
   - `/etc/systemd/system/deos-health.service` içindeki `WorkingDirectory=` ve `Environment=REPO_ROOT=...`

   `deos_compose_up.sh`, `REPO_ROOT` ortam değişkeni verilmedikçe repo kökünü **scriptin bulunduğu yoldan** türetir; böylece `ExecStart` doğru kopyayı gösterdiği sürece `docker compose` her zaman o kopyanın `docker-compose.yml` dosyasını kullanır.

   Ardından:

   ```bash
   sudo systemctl daemon-reload
   sudo systemctl restart deos-bringup deos-health
   ```

İsteğe bağlı: her iki serviste de aynı kökü sabitlemek için `Environment=REPO_ROOT=/yeni/yol` kullanılabilir (`deos-health` zaten `REPO_ROOT` ile `docker compose exec` yollarını üretir).

## Mimari özet

Yüksek seviye katmanlar:

- **Sensors (ROS driver’lar)**: kamera + depth, LiDAR, GPS, IMU
- **Perception (algı + karar kısıtları)**:
  - kamera/derinlikten JSON tespitler (stub/yer tutucu)
  - LiDAR point cloud’dan engel kümeleri (JSON)
  - tüm logic modüllerinin çıktısını **kısıt** olarak yayınlama (`speed_cap`, `emergency_stop`, opsiyonel `steering_override`)
- **Planning (görev/rota)**:
  - GeoJSON görev dosyasından waypoint takibi
  - `/planning/steering_ref` ve `/planning/speed_limit`
- **Control (arbitraj + çıktı)**:
  - algı + planlamayı öncelik kurallarıyla birleştirir
  - `/cmd_vel` ve `/safety/emergency_stop`
- **Actuation (STM32)**:
  - micro-ROS client olarak ROS 2 graph’a katılır, Twist/Bool tüketir

## Veri akışı (pipeline)

```
RealSense D415 (RGB + Depth) ──┐
                               ├─> vision_bridge/stereo_detector_node
                               │      └─> /perception/stereo_detections (std_msgs/String, JSON)
SICK LiDAR (PointCloud2) ──────┤
                               ├─> sensor_fusion/lidar_obstacle_node
                               │      └─> /perception/lidar_obstacles (std_msgs/String, JSON)
IMU (sensor_msgs/Imu) ─────────┘
                                       │
                                       └─> vision_bridge/perception_fusion_node
                                              ├─> /perception/emergency_stop (std_msgs/Bool)
                                              ├─> /perception/speed_cap (std_msgs/Float32)
                                              ├─> /perception/steering_override (std_msgs/Float32)
                                              └─> /perception/has_steering_override (std_msgs/Bool)

GPS (sensor_msgs/NavSatFix) + IMU ───────────> mission_planning/mission_planning_node
                                              ├─> /planning/steering_ref (std_msgs/Float32)
                                              ├─> /planning/speed_limit (std_msgs/Float32)
                                              ├─> /planning/current_task (std_msgs/String)
                                              └─> /planning/arrived (std_msgs/Bool)

/perception/* + /planning/* ──────────────────> vehicle_controller/vehicle_controller_node
                                              ├─> /cmd_vel (geometry_msgs/Twist)  -> STM32
                                              └─> /safety/emergency_stop (std_msgs/Bool) -> STM32
```

## Mesaj sözleşmesi (topics)

### Perception iç JSON (yalnızca Raspberry Pi tarafı)

`/perception/stereo_detections` (`std_msgs/String`) örnek JSON:

```json
[
  {
    "class_name": "pedestrian",
    "confidence": 0.92,
    "bbox_px": [10, 20, 50, 120],
    "distance_m": 6.1,
    "lateral_m": -0.2
  }
]
```

`/perception/lidar_obstacles` (`std_msgs/String`) örnek JSON:

```json
[
  {
    "kind": "cone",
    "confidence": 0.7,
    "distance_m": 4.8,
    "lateral_m": 0.5,
    "bbox_px": null
  }
]
```

Bu JSON kullanımının nedeni: özel mesaj tipleri STM32 micro-ROS firmware’ine ek “message generation” gerektirebilir.
JSON sadece Linux/RPi tarafında kalır; STM32’ye gidenler standart tiplerdir.

### STM32’ye giden kontrol çıktısı (standart mesajlar)

- `/cmd_vel` (`geometry_msgs/Twist`)
  - `linear.x`: hız (m/s)
  - `angular.z`: direksiyon/turn rate (rad/s) — bu projede “steering oranı” ölçeklenerek kullanılır
- `/safety/emergency_stop` (`std_msgs/Bool`)

## Paket yapısı (ROS2 workspace)

ROS2 workspace: `DEOS/deos_ws/`

`DEOS/deos_ws/src/` altında ana gruplar:

- `sensors/` (kamera, lidar, imu/gps)
- `perception/`
  - `obstacle_detection/vision_bridge/` (`vision_bridge` paketi)
  - `sensor_fusion/` (`sensor_fusion` paketi)
- `planning/`
  - `mission_planning/` (`mission_planning` paketi)
  - `local_navigation/` (mevcut Nav2/TEB yapılandırmaları)
- `control/`
  - `vehicle_controller/` (`vehicle_controller` paketi)
  - `micro_ros_agent/` (micro-ROS ile ilgili yardımcı paket)
- `vehicle_bringup/` (launch’lar)
- `pcl_localization_ros2/` (PCL tabanlı lokalizasyon, üçüncü parti)
- **Yeni** `deos_algorithms/` (saf Python algorithm library)

## `deos_algorithms` (ortak algoritma kütüphanesi)

`DEOS/deos_ws/src/deos_algorithms` paketi, ROS bağımsız “saf Python” logic modüllerini barındırır.
Node’lar sadece adaptör/IO görevi görür; karar mantığı burada yaşar.

Başlıca modüller:

- `safety_logic`: koridor + hysteresis ile temel yavaşla/dur
- `obstacle_logic`: dinamik (yaya) / statik (koni/bariyer) davranış
- `traffic_sign_logic`, `traffic_light_logic`: kısıt üretimi
- `slalom_logic`: konilerle slalom steering override
- `parking_logic`: park manevrası state’i (şimdilik perception tarafında park tespiti stub)
- `geojson_mission_reader`, `waypoint_manager`, `mission_manager`: görev/rota takibi
- `perception_fusion`: stereo + lidar + imu adaptasyonu (liste üretimi)

## Öncelik/Arbitraj (kontrol mantığı)

`vehicle_controller_node` şu temel kuralları uygular:

- **Acil fren**: `/perception/emergency_stop == True` veya algı timeout → `/cmd_vel` sıfır
- **Hız**: `min(/perception/speed_cap, /planning/speed_limit)`
- **Direksiyon**:
  - `has_steering_override` true ise `steering_override`
  - değilse `planning/steering_ref`

## micro-ROS (STM32) + UDP 8888

Bu repoda ROS2 tarafı Docker ile `network_mode: host` olarak çalışacak şekilde tasarlanmıştır.
STM32 ile Ethernet üzerinden **UDP + micro-ROS** haberleşmesi için yaklaşım:

1. `micro-ROS Agent` ayrı container.
2. `deos` ve `micro_ros_agent` aynı `ROS_DOMAIN_ID`.
3. İki container da Raspberry Pi üzerinde `network_mode: host`.
4. STM32, Raspberry Pi Ethernet IP’sine `UDP 8888` üzerinden bağlanır.

`docker-compose.yml` servisleri:

- `deos`: ana ROS2 stack
- `micro_ros_agent`: `microros/micro-ros-agent:jazzy` (`udp4 -p 8888`)

Veri akışı:

`STM32 (micro-ROS client)` -> `UDP/8888` -> `micro_ros_agent` -> `ROS 2 DDS graph` -> `deos`

## Çalıştırma (mimari doğrulama)

Docker ile (Linux/Raspberry Pi):

```bash
docker compose up -d micro_ros_agent deos
```

`deos` container içinde:

```bash
ros2 topic list
ros2 node list
ros2 launch vehicle_bringup main.launch.py mission_file:=/path/to/rota.geojson
```

