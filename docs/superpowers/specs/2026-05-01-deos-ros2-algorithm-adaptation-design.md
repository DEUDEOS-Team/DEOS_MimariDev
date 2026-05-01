# DEOS — Robotaksi Algoritmalarının ROS2 Mimarisine Uyarlanması

**Tarih:** 2026-05-01  
**Sensörler:** Stereo Kamera (RealSense D415), LiDAR (SICK multiScan165), GPS (NMEA serial)  
**Hedef Platform:** Raspberry Pi + STM32 (micro-ROS over UDP)  
**ROS2 Dağıtımı:** Jazzy

---

## 1. Kapsam

Robotaksi projesindeki ROS-bağımsız saf-Python algoritma modüllerini
(`safety_logic`, `traffic_sign_logic`, `traffic_light_logic`, `obstacle_logic`,
`slalom_logic`, `parking_logic`, `geojson_mission_reader`, `waypoint_manager`,
`mission_manager`, `perception_fusion`) mevcut DEOS-Mimari ROS2 paket yapısına
entegre eden node ve topic tasarımı.

YOLO nesne tespiti şimdilik stub olarak bırakılır; ileride yalnızca
`stereo_detector_node` içinde değişiklik yapılarak eklenir.

---

## 2. Mimari Genel Görünüm

```
SENSÖRLER
  /camera/color/image_raw   Image        30 Hz   RealSense D415
  /camera/depth/image_raw   Image        30 Hz   RealSense D415
  /points_downsampled       PointCloud2  20 Hz   SICK multiScan165 → voxel filtre
  /gps/fix                  NavSatFix     5 Hz   GPS NMEA serial
  /imu/data                 Imu         100 Hz   IMU serial

ALGI KATMANI  (node-içi, STM32 görmez)
  /perception/stereo_detections   std_msgs/String (JSON)   30 Hz
  /perception/lidar_obstacles     std_msgs/String (JSON)   20 Hz

KARAR KATMANI
  /perception/emergency_stop        std_msgs/Bool
  /perception/speed_cap             std_msgs/Float32   0.0–1.0
  /perception/steering_override     std_msgs/Float32  -1.0–+1.0
  /perception/has_steering_override std_msgs/Bool

PLANLAMA KATMANI
  /planning/steering_ref   std_msgs/Float32  -1.0–+1.0
  /planning/speed_limit    std_msgs/Float32   0.0–1.0
  /planning/current_task   std_msgs/String
  /planning/arrived        std_msgs/Bool

KONTROL ÇIKTI → STM32 via micro-ROS UDP
  /cmd_vel                geometry_msgs/Twist   (linear.x=hız m/s, angular.z=direksiyon rad/s)
  /safety/emergency_stop  std_msgs/Bool
```

Neden standart mesaj tipleri: micro-ROS firmware'e önceden derlenmeden gelmeyen
tipler STM32 tarafında görünmez. `geometry_msgs/Twist`, `std_msgs/Bool`,
`std_msgs/Float32` zaten micro-ROS client kütüphanesinde gömülü gelir.
Node-içi JSON taşıma yalnızca Raspberry Pi üzerinde kalır.

---

## 3. Paket Yapısı

```
deos_ws/src/
├── deos_algorithms/                    YENİ — paylaşılan algoritma kütüphanesi
│   ├── package.xml
│   ├── setup.cfg / setup.py
│   └── deos_algorithms/
│       ├── __init__.py
│       ├── safety_logic.py
│       ├── traffic_sign_logic.py
│       ├── traffic_light_logic.py
│       ├── obstacle_logic.py
│       ├── slalom_logic.py
│       ├── parking_logic.py
│       ├── geojson_mission_reader.py
│       ├── waypoint_manager.py
│       ├── mission_manager.py
│       ├── perception_fusion.py
│       └── sensors/
│           ├── __init__.py
│           └── types.py
│
├── perception/obstacle_detection/vision_bridge/
│   └── vision_bridge/
│       ├── stereo_detector_node.py     GÜNCELLEME (YOLO stub + derinlik örnekleme)
│       └── perception_fusion_node.py   YENİ
│
├── perception/sensor_fusion/
│   └── sensor_fusion/
│       └── lidar_obstacle_node.py      YENİ
│
├── planning/mission_planning/
│   └── mission_planning/
│       └── mission_planning_node.py    YENİ
│
└── control/vehicle_controller/
    └── vehicle_controller/
        └── vehicle_controller_node.py  YENİ
```

---

## 4. Node Tasarımları

### 4.1 `stereo_detector_node`

**Paket:** `vision_bridge`
**Not:** Mevcut `vision_bridge_node.py` (LaneToWallNode — Canny stub) bu node
tarafından işlevsel olarak yerini alır. Dosya silinmez, ama `main.launch.py`'de
artık başlatılmaz.

**Abonelikler:** `/camera/color/image_raw`, `/camera/depth/image_raw`  
**Yayınlar:** `/perception/stereo_detections` (String JSON)

**İç akış:**
1. RGB frame geldiğinde `yolo_detect(rgb)` çağrılır → şimdilik boş liste döner.
2. Her tespit için `depth_image[cy, cx]` örneklenir → `distance_m`.
3. Bbox merkezi ile görüntü merkezi farkından `lateral_m` hesaplanır (pinhole model).
4. `StereoBbox` listesi JSON olarak yayımlanır.

**YOLO entegrasyon noktası:** `yolo_detect()` fonksiyonu, modelin yüklendiği tek
yerdir. Gerçek model eklendiğinde bu fonksiyon dışındaki hiçbir şey değişmez.

---

### 4.2 `lidar_obstacle_node`

**Paket:** `sensor_fusion`  
**Abonelikler:** `/points_downsampled` (PointCloud2)  
**Yayınlar:** `/perception/lidar_obstacles` (String JSON)

**İç akış:**
1. PointCloud2 → numpy xy dizisi (z filtresi 0.1–2.0 m uygulanmış gelir).
2. Öklid kümeleme (epsilon=0.5 m, min_points=5) → her küme bir engel adayı.
3. Küme boyutuna göre `kind` tahmini:
   - bbox çapı < 0.4 m → `"cone"`
   - bbox çapı 0.4–1.5 m → `"unknown"`
   - bbox çapı > 1.5 m → `"barrier"`
4. Küme merkezi → `distance_m`, `lateral_m`.
5. `LidarObstacle` listesi JSON olarak yayımlanır.

---

### 4.3 `perception_fusion_node`

**Paket:** `vision_bridge`  
**Abonelikler:** `/perception/stereo_detections`, `/perception/lidar_obstacles`, `/imu/data`  
**Yayınlar:** `/perception/emergency_stop`, `/perception/speed_cap`,
`/perception/steering_override`, `/perception/has_steering_override`

**İç akış:**
1. Stereo + LiDAR JSON'ları okunur, `perception_fusion.fuse()` ile `PerceptionFrame` oluşturulur.
2. `safety_logic.decide()` → `SafetyDecision`
3. `traffic_sign_logic.update()` → `TrafficSignState`
4. `traffic_light_logic.update()` → `TrafficLightState`
5. `obstacle_logic.update()` → `ObstacleState`
6. `slalom_logic.update()` (yalnızca cone tespitleri) → `SlalomState`
7. `parking_logic.update()` → `ParkState`
8. Karar önceliği (aşağıya bakınız) uygulanır, topic'ler yayımlanır.

**Karar önceliği (fusion node içi):**
1. `ObstacleState.emergency_stop` → emergency_stop = True
2. `TrafficLightState.must_stop` → emergency_stop = True
3. `TrafficSignState.must_stop_soon` → emergency_stop = True
4. `speed_cap` = min(safety, traffic_sign, traffic_light, obstacle)
5. `steering_override` = SlalomState.steering (aktifse) ya da ParkState.steering

---

### 4.4 `mission_planning_node`

**Paket:** `mission_planning`  
**Parametre:** `mission_file` (GeoJSON dosya yolu)  
**Abonelikler:** `/gps/fix` (NavSatFix), `/imu/data` (Imu)  
**Yayınlar:** `/planning/steering_ref`, `/planning/speed_limit`,
`/planning/current_task`, `/planning/arrived`

**İç akış:**
1. Başlangıçta `geojson_mission_reader.read_file(mission_file)` → `MissionPlan`.
2. Her `/gps/fix` callback'te `GpsPosition(lat, lon)` oluşturulur.
3. `/imu/data`'dan quaternion → `heading_deg` hesaplanır.
4. `mission_manager.update(pos, now_s)` → `WaypointState + MissionDecision`.
5. Sonuçlar ilgili topic'lere yayımlanır.

**GPS timeout politikası:**

| Süre | Davranış |
|------|----------|
| 0 – 2 s | Son geçerli `steering_ref` ve `speed_limit` korunur, araç devam eder |
| 2 – 5 s | `speed_limit × 0.5` uygulanır, yön korunur |
| > 5 s | `speed_limit` her saniye 0.2 azalır, 0.0'a ulaşınca sabit kalır |

---

### 4.5 `vehicle_controller_node`

**Paket:** `vehicle_controller`  
**Abonelikler:** Tüm `/perception/*` ve `/planning/*` topic'leri  
**Yayınlar:** `/cmd_vel` (Twist), `/safety/emergency_stop` (Bool)  
**Parametreler:** `max_speed_mps` (varsayılan: 3.0), `max_steer_rads` (varsayılan: 1.0)

**Öncelik hiyerarşisi:**

```python
# 1. Acil fren — watchdog veya algı kararı
if emergency_stop or perception_timeout:
    cmd_vel → (0.0, 0.0)
    /safety/emergency_stop → True
    return

# 2. Hız = en kısıtlayıcı
speed = min(perception_speed_cap, planning_speed_limit)

# 3. Direksiyon — algı manevrası öncelikli
if has_steering_override:
    steer = perception_steering_override   # slalom / park
else:
    steer = planning_steering_ref          # waypoint takibi

# 4. /cmd_vel yayımla
Twist.linear.x  = speed × max_speed_mps
Twist.angular.z = steer × max_steer_rads
```

**Watchdog:** Her 50 ms timer callback'te `/perception/emergency_stop` yaşını
kontrol eder. 200 ms'den eski ise sistem timeout acil frenine geçer.

---

## 5. Hata Durumları ve Timeout Güvenceleri

| Kaynak | Timeout Eşiği | Davranış |
|--------|--------------|----------|
| `/camera/depth/image_raw` | 500 ms | Stereo tespitler dondurulur, sadece LiDAR aktif |
| `/points_downsampled` | 500 ms | LiDAR engeller temizlenir, sadece kamera aktif |
| `/perception/*` (watchdog) | **200 ms** | `vehicle_controller` acil fren uygular |
| `/gps/fix` | 2 s / 5 s | Kademeli hız düşürme (bkz. §4.4) |
| `/imu/data` | 500 ms | `heading_deg = 0.0` varsayılır |

**Sensör degradasyon politikası:**

- Kamera YOK + LiDAR VAR: safety_logic LiDAR mesafesiyle çalışır; tabela/ışık/slalom/park devre dışı.
- Kamera VAR + LiDAR YOK: stereo derinlikten mesafe tahmini; tüm modüller çalışır, mesafe daha gürültülü.
- GPS YOK: kademeli yavaşlama (bkz. §4.4), son waypoint korunur.

**STM32 tarafı beklentisi:** `/cmd_vel` en az 10 Hz gelmezse STM32 firmware
kendi watchdog'u ile motoru durdurur. Bu tasarım belgesi yalnızca ROS2 tarafını
kapsar; firmware davranışı ayrıca tanımlanır.

---

## 6. Launch Dosyası Güncellemesi

`vehicle_bringup/launch/main.launch.py` aşağıdaki node'ları ekleyecek şekilde
güncellenir:

```
mevcut:  camera, gps, imu, lidar, vision_bridge, pcl_localization
eklenecek:
  perception/lidar_obstacle_node
  perception/perception_fusion_node
  planning/mission_planning_node     (mission_file parametresiyle)
  control/vehicle_controller_node    (max_speed_mps, max_steer_rads parametreleriyle)
```

---

## 7. Kapsam Dışı

- STM32 firmware ve micro-ROS istemci kodu
- YOLO model eğitimi veya ağırlık dosyası
- Nav2 yığını (mevcut `local_navigation` paketi dokunulmadan kalır)
- PCL lokalizasyon (mevcut `pcl_localization_ros2` dokunulmadan kalır)
