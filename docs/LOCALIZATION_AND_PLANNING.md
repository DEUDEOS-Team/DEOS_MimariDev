## Lokalizasyon ve Rota Planlama Dokümantasyonu (DEOS-Mimari)

Bu doküman, bu repodaki **lokalizasyon**, **global rota/görev planlama** ve **lokal planlama (davranış/kaçınma)** akışını “hangi node ne yapıyor, hangi topic’leri üretip tüketiyor, nasıl debug edilir?” perspektifinde açıklar.

> Kapsam: `DEOS/deos_ws/src/*` altındaki çalışan pipeline.

---

## 1) Büyük resim (pipeline)

Bu repo şu yaklaşımı kullanır:

- **Global planlama**: GeoJSON görev noktaları + GPS/IMU ile “hangi waypoint’e gidiyorum” ve “direksiyon referansı” üretir.
- **Lokal planlama / davranış**: Perception çıktılarından (engel/ışık/tabela/park) **hız tavanı** ve bazı durumlarda **direksiyon override** üretir.
- **Kontrol**: Planning + Perception kısıtlarını birleştirip `/cmd_vel` yayınlar; STM32’ye ayrıca `/safety/emergency_stop` pulse’ları basar.

Özet dataflow:

- `/gps/fix` + `/imu/data` → `mission_planning_node` → `/planning/*`
- Stereo/LiDAR/IMU → `perception_fusion_node` → `/perception/*`
- `/planning/*` + `/perception/*` → `vehicle_controller_node` → `/cmd_vel`, `/safety/emergency_stop`

---

## 2) Lokalizasyon (PCL Localization)

### 2.1 Amaç

`pcl_localization_ros2` paketi; LiDAR pointcloud’u, harita (PCD) ve (opsiyonel) odom/IMU yardımıyla aracı harita üzerinde **map frame**’e oturtmak için tasarlanmıştır. TF olarak **`map → odom`** transform’u yayınlar.

> Not: Bu repoda global planlama şu an doğrudan `pcl_localization` kullanmıyor; planning tarafı GPS/IMU ile çalışıyor. Lokalizasyon, harita tabanlı sürüşe geçmek için hazır bir bileşen olarak repo içinde duruyor.

### 2.2 Node / dosyalar

- **Paket**: `DEOS/deos_ws/src/pcl_localization_ros2`
- **Launch**: `pcl_localization_ros2/launch/pcl_localization.launch.py`
- **Parametre**: `pcl_localization_ros2/param/localization.yaml`

### 2.3 Topic’ler

`pcl_localization_component.cpp`’dan özet:

- **Sub**
  - `cloud` (`sensor_msgs/PointCloud2`) — launch’ta genelde `/points_raw` → `cloud` remap
  - `map` (`sensor_msgs/PointCloud2`) — harita (opsiyonel)
  - `initialpose` (`geometry_msgs/PoseStamped`) — ilk poz
  - `odom` (`nav_msgs/Odometry`) — (opsiyonel)
  - `imu` (`sensor_msgs/Imu`) — (opsiyonel)
- **Pub**
  - `pcl_pose` (`geometry_msgs/PoseStamped`) — lokalizasyon poz çıktısı
  - `path` (`nav_msgs/Path`) — takip edilen yol izi
  - `initial_map` (`sensor_msgs/PointCloud2`) — ilk harita yayını
- **TF**
  - `map` → `odom` (global_frame_id → odom_frame_id)

### 2.4 Önemli parametreler

`localization.yaml` içinde:

- `registration_method`: `"NDT"` / `"GICP"`
- `ndt_resolution`, `ndt_step_size`, `transform_epsilon`
- `use_pcd_map`, `map_path`
- `set_initial_pose`, `initial_pose_x/y/z/qx/qy/qz/qw`
- `use_odom`, `use_imu`
- `global_frame_id` (default `"map"`), `odom_frame_id` (default `"odom"`), `base_frame_id` (default `"base_link"`)

### 2.5 Debug / doğrulama

Örnek komutlar:

```bash
ros2 topic list | grep pcl
ros2 topic echo /pcl_localization/pcl_pose
ros2 run tf2_ros tf2_echo map odom
```

Harita yolu doğru mu?

- `map_path` yanlışsa lokalizasyon “harita yok” durumuna düşer; log’lardan anlaşılır.

---

## 3) Global planlama (Görev/Waypoint takibi)

### 3.1 Şartname ile ilişki

2026 şartnamesi GeoJSON içinde kritik noktaları verir:

- Start
- Görev noktaları (gorev_1, gorev_2, …)
- **Park giriş** (`park_giris`): park edilecek yer değil; bu noktadan sonra araç uygun bir park slotu bulup park etmelidir.

Bu repo, `GeoJsonMissionReader` ile GeoJSON’u parse eder ve `WaypointManager` ile sıradaki noktayı takip eder.

### 3.2 Node

- **Node**: `planning/mission_planning/mission_planning_node.py`
- **Girdiler**:
  - `/gps/fix` (`sensor_msgs/NavSatFix`)
  - `/imu/data` (`sensor_msgs/Imu`) → heading çıkarılır
  - `/perception/park_complete` (`std_msgs/Bool`) → park bitti bilgisi
  - `/perception/turn_permissions` (`std_msgs/String`, JSON) → tabela bazlı dönüş kısıtları
- **Çıktılar**:
  - `/planning/steering_ref` (`std_msgs/Float32`, -1..+1)
  - `/planning/speed_limit` (`std_msgs/Float32`, 0..1)
  - `/planning/current_task` (`std_msgs/String`)
  - `/planning/arrived` (`std_msgs/Bool`)
  - `/planning/park_mode` (`std_msgs/Bool`)
  - `/planning/park_remaining_s` (`std_msgs/Float32`)

### 3.3 GeoJSON parse mantığı

- Dosya: `deos_algorithms/geojson_mission_reader.py`
- `properties.task` alanı yoksa/uygunsuzsa `properties.name` üzerinden eşleme yapılır:
  - `start` → `TaskType.START`
  - `gorev_*` → `TaskType.CHECKPOINT`
  - `park_giris` → **`TaskType.PARK_ENTRY`**

### 3.4 WaypointManager (global direksiyon referansı)

- Dosya: `deos_algorithms/waypoint_manager.py`
- Hesaplar:
  - Haversine mesafe
  - Bearing (hedefe yön)
  - Heading error
  - Cross-track error (bir önceki wp → mevcut wp hattına göre)
- Çıkış:
  - `steering_ref = clamp(bearing_error * gain - xte * gain, -1..+1)`
  - `speed_limit_ratio = waypoint.speed_limit_ratio`
  - `arrived = dist <= arrival_radius_m`

### 3.5 MissionManager (task seviyesinde davranış)

- Dosya: `deos_algorithms/mission_manager.py`
- Pickup/Dropoff:
  - arrived olunca 15–20 s arası “hold” uygular (`speed_cap_ratio=0`)
- Park:
  - `PARK_ENTRY` arrived olunca `park_mode=True` başlar
  - park modunda hız limiti düşer (`speed_cap_ratio <= 0.5`)
  - 180 s aşılırsa güvenli dur (park görevi timeout)
  - Perception’dan `park_complete=True` gelince park modu kapanır, waypoint advance edilir

### 3.6 Tabela bazlı “dönüş kısıtı” uygulaması

- `TrafficSignLogic` turn_permissions üretir (sol/sağ/düz izin + forced_direction).
- Bu bilgi `perception_fusion_node` tarafından `/perception/turn_permissions` olarak publish edilir.
- `mission_planning_node` waypoint’e **yaklaşırken** (varsayılan 8 m) şu şekilde uygular:
  - `forced_direction=left/right/straight` → `steering_ref` bias + hız çarpanı
  - `pass_left/pass_right` → küçük bias
  - yasak dönüşlerde steer “yumuşatılır” ve hız düşürülür

Bu, “tam bir graph-based replanning” değildir; amaç, tabelayı görmezden gelmemek ve güvenli davranış üretmektir.

---

## 4) Lokal planlama (Perception tabanlı davranış ve override)

### 4.1 Node

- **Node**: `perception/obstacle_detection/vision_bridge/perception_fusion_node.py`
- **Girdiler**:
  - `/perception/stereo_detections` (`std_msgs/String`, JSON)
  - `/perception/lidar_obstacles` (`std_msgs/String`, JSON)
  - `/imu/data` (`sensor_msgs/Imu`)
  - `/planning/park_mode` (`std_msgs/Bool`)
  - `/hardware/motion_enable` (`std_msgs/Bool`) — fail-safe stop/continue
- **Çıktılar**:
  - `/perception/emergency_stop` (`std_msgs/Bool`)
  - `/perception/speed_cap` (`std_msgs/Float32`)
  - `/perception/steering_override` (`std_msgs/Float32`)
  - `/perception/has_steering_override` (`std_msgs/Bool`)
  - `/perception/park_complete` (`std_msgs/Bool`)
  - `/perception/turn_permissions` (`std_msgs/String`, JSON)

### 4.2 Engel bazlı lokal “yeniden yönelim”

- `ObstacleLogic`:
  - statik engelde `suggest_lane_change=True` + `avoidance_direction` üretir
  - yol kapalı durumunda `road_blocked=True`
- `perception_fusion_node`:
  - `suggest_lane_change=True` ise küçük bir direksiyon bias’ı ile override üretir
  - `road_blocked=True` ise `speed_cap=0` ile güvenli durdurur (global replanning gerektirir)

### 4.3 Park modu (slot seçimi + manevra)

- `MissionManager` park modunu başlatır (`/planning/park_mode=True`)
- `perception_fusion_node` park modunda:
  - `ParkingLogic` steering/speed_ratio değerlerini local override olarak kullanır
  - uygun slot yoksa (`no_eligible_spot=True`) hız tavanını 0’a indirir
  - `park_complete=True` yayınlar

### 4.4 Işık/tabela hız kısıtları

Lokal kısıt birleşimi:

- `speed_cap = min(obstacle.speed_cap_ratio, sign.speed_cap_ratio, light.speed_cap_ratio)`
- `emergency_stop = obstacle.emergency_stop OR light.must_stop OR sign.must_stop_soon`

---

## 5) Kontrol katmanı (planning + perception birleşimi)

- Node: `control/vehicle_controller/vehicle_controller_node.py`
- Birleştirme:
  - `speed_ratio = min(/perception/speed_cap, /planning/speed_limit)`
  - direksiyon: perception override varsa onu kullanır
  - emergency veya perception timeout veya STM32 motion_enable hold → `/cmd_vel` kesilir ve `/safety/emergency_stop` pulse basılır

---

## 6) Çalıştırma / Debug hızlı rehber

### 6.1 Planning kontrol

```bash
ros2 topic echo /planning/current_task
ros2 topic echo /planning/steering_ref
ros2 topic echo /planning/speed_limit
ros2 topic echo /planning/park_mode
ros2 topic echo /planning/park_remaining_s
```

### 6.2 Perception kontrol

```bash
ros2 topic echo /perception/emergency_stop
ros2 topic echo /perception/speed_cap
ros2 topic echo /perception/has_steering_override
ros2 topic echo /perception/steering_override
ros2 topic echo /perception/turn_permissions
ros2 topic echo /perception/park_complete
```

### 6.3 Fail-safe (STM32 motion_enable)

```bash
ros2 topic pub /hardware/motion_enable std_msgs/msg/Bool "{data: false}" -1
ros2 topic pub /hardware/motion_enable std_msgs/msg/Bool "{data: true}" -1
```

---

## 7) Mevcut sınırlamalar ve sonraki iyileştirmeler

- **Gerçek global replanning yok**: `road_blocked=True` durumunda planlama yeni bir yol üretmiyor; sadece güvenli duruyor.
  - İyileştirme: GeoJSON’dan yol grafı veya şerit merkez hattı çıkarıp “segment değiştirme / alternatif yol” üretmek.
- **Map tabanlı (PCL) lokalizasyon planning’de kullanılmıyor**:
  - İyileştirme: GPS→map dönüşümü (ENU/UTM) + `map→base_link` ile waypoint takibini harita frame’e taşımak.
- **Şerit bazlı local planner yok**:
  - İyileştirme: lane centerline + obstacle occupancy ile kısa horizon path üretimi (pure pursuit / stanley + kısıtlar).

