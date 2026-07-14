# DEOS — Deterministik Otonom Araç Sistemi

Raspberry Pi 5 üzerinde çalışan, ROS 2 Jazzy tabanlı tam otonom araç yazılımı.  
Robotaksi Binek Otonom Araç Yarışması için geliştirilmiştir.

---

## Mimari Genel Bakış

```
Sensörler (20–100 Hz)
    │
    ▼
┌─────────────────────────────────────────────┐
│              Algı Katmanı                   │
│  Kamera → YOLOv8 (Hailo-8)                 │
│  LiDAR  → DBSCAN Kümeleme                  │
│  Kamera → Şerit Segmentasyonu (HEF)        │
│             │                               │
│        Algı Füzyonu                         │
│  (emergency_stop, speed_cap, steer, park)   │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│             Lokalizasyon                    │
│  GPS + IMU → EKF → odom/ekf                │
│  LiDAR ICP → odom/icp                      │
│  Level-3 Füzyon → odom/final               │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│               Planlama                      │
│  GeoJSON Görev + Dijkstra → steering_ref   │
│  Waypoint takip, park modu, görev geçişi   │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│               Kontrol                       │
│  Vehicle Controller (blending) → cmd_vel   │
│  STM32 Bridge → speed_delta, steering_deg  │
└──────────────────┬──────────────────────────┘
                   │
             STM32 Motor Kontrolcü
         (servo + sürücü motoru PWM)
```

---

## Donanım

| Bileşen | Model | Arayüz |
|---|---|---|
| Bilgisayar | Raspberry Pi 5 | — |
| Yapay Zeka Hızlandırıcı | Hailo-8 HAT | PCIe `/dev/hailo0` |
| Stereo Kamera | Intel RealSense D415 | USB 3.1 |
| LiDAR | SICK Multiscan165 | Ethernet `192.168.0.1` |
| GPS | Quectel L86 EVB | Serial `/dev/ttyUSB0` 9600 baud |
| IMU | 6-DOF | Serial `/dev/ttyUSB1` |
| Motor Kontrolcü | STM32 | microROS UDP 8888 |

---

## ROS 2 Paket Yapısı

```
DEOS/deos_ws/src/
│
├── deos_algorithms/          # Pure-Python karar motoru (ROS bağımsız)
├── deos_logging/             # Ortak loglama (DeosLogger)
│
├── sensors/
│   ├── camera/               # RealSense D415 sürücüsü
│   ├── imu/                  # GPS node + IMU node
│   └── lidar/                # SICK Multiscan165 sarmalayıcısı
│
├── perception/
│   ├── obstacle_detection/
│   │   └── vision_bridge/    # YOLOv8 tespiti + algı füzyonu
│   ├── lane_tracking/        # Şerit segmentasyonu + kontrol referansı
│   └── sensor_fusion/        # LiDAR engel kümeleme + EKF + Level-3 odometri
│
├── planning/
│   └── mission_planning/     # GeoJSON görev + Dijkstra rota takibi
│
├── control/
│   ├── vehicle_controller/   # cmd_vel blending + STM32 köprüsü
│   └── deos_failsafe/        # Sağlık validasyonu, acil durdurma
│
├── pcl_localization_ros2/    # C++ ICP tabanlı LiDAR SLAM
└── vehicle_bringup/          # Ana launch dosyası, tüm sistemi başlatır
```

---

## Katmanlar

### 1. Sensör Katmanı

Her sensör bağımsız bir node olarak çalışır ve ham veriyi `/deos/sensors/` altında yayınlar:

| Node | Topic | Frekans |
|---|---|---|
| `realsense_d415_node` | `/deos/sensors/camera/color/image_raw` | 30 Hz |
| `realsense_d415_node` | `/deos/sensors/camera/depth/image_raw` | 30 Hz |
| `gps_node` | `/deos/sensors/gps/fix` | 5–10 Hz |
| `imu_node` | `/deos/sensors/imu/data` | 100 Hz |
| `sick_multiscan165` | `/deos/sensors/lidar/cloud_unstructured_fullframe` | 20 Hz |

---

### 2. Algı Katmanı

Üç paralel algı zinciri bulunur:

**2a. Nesne Tespiti** (`stereo_detector_node`)  
Kamera görüntüsünü alır, Hailo-8 üzerindeki YOLOv8 modeliyle nesne tespiti yapar. Çıktı olarak sınır kutuları (bounding box), sınıf adı ve güven skoru gönderir.  
Topic: `/deos/perception/stereo/detections`

**2b. Şerit Tespiti** (`lane_detection_node` + `lane_control_node`)  
Hailo-8 üzerindeki segmentasyon modeliyle şerit piksellerini bulur, merkez çizgisini çıkarır ve direksiyon referansı üretir.  
Topics: `/deos/perception/lane/center_pts`, `/deos/lane/steering_ref`

**2c. LiDAR Engel Kümeleme** (`lidar_obstacle_node`)  
PointCloud2 verisine DBSCAN kümeleme uygular. Araç koridoruna (±3 m) giren kümeleri engel olarak bildirir.  
Topic: `/deos/perception/lidar/obstacles`

**2d. Algı Füzyonu** (`perception_fusion_node`)  
Üç kaynaktan gelen veriyi birleştirir ve `deos_algorithms` modüllerini kullanarak tek bir karar üretir:

| Çıktı Topic | Tür | Açıklama |
|---|---|---|
| `fusion/emergency_stop` | Bool | Acil durdurma |
| `fusion/speed_cap` | Float32 | 0–1 hız sınırı |
| `fusion/steering_override` | Float32 | Engel kaçınma direksiyonu |
| `fusion/has_steering_override` | Bool | Override aktif mi |
| `fusion/turn_permissions` | String | Dönüş kısıtları (JSON) |
| `fusion/park_complete` | Bool | Park tamamlandı |
| `fusion/green_elapsed_s` | Float32 | Yeşil ışık geçen süre |
| `fusion/decision_debug` | String | Karar gerekçeleri |

---

### 3. Lokalizasyon Katmanı

Üç katmanlı odometri füzyonu:

```
GPS + IMU → EKF (robot_localization) → /odom/ekf
LiDAR     → ICP SLAM (pcl_localization_ros2) → /odom/icp
                        │
                  final_odom_node
          (ICP < 0.2s taze ise ICP, aksi EKF)
                        │
                  /odom/final  ← Planlama ve kontrol bu topic'i kullanır
```

---

### 4. Planlama Katmanı

`mission_planning_node`, GeoJSON formatındaki görev dosyasını okur ve Dijkstra algoritmasıyla optimal rotayı hesaplar.

**Desteklenen Görev Türleri**: `START`, `CHECKPOINT`, `STOP`, `PARK_ENTRY`, `PARK`, `PICKUP`, `DROPOFF`

**Çıktılar**:
- `/deos/planning/steering_ref` — Mevcut waypoint'e yönelen referans açı (rad)
- `/deos/planning/speed_limit` — Rota hız sınırı (0–1)
- `/deos/planning/current_task` — Aktif görev adı
- `/deos/planning/arrived` — Waypoint'e ulaşıldı sinyali
- `/deos/planning/park_mode` — Park manevrası aktif

---

### 5. Karar Motoru (`deos_algorithms`)

ROS bağımsız, pure-Python modüller. `perception_fusion_node` tarafından kullanılır.

**Öncelik Sırası** (yüksekten düşüğe):

| Öncelik | Modül | Tetikleyen Durum |
|---|---|---|
| 1 | — | `motion_enable` timeout, tüm sensör kaybı |
| 2 | `obstacle_logic` | Engel acil durdurmasi, yol kapalı |
| 3 | `decision_arbiter` (lane) | Şerit ihlali / kaçınma kısıtı |
| 4 | `traffic_light_logic` | Kırmızı/sarı ışık |
| 5 | `traffic_sign_logic` | Dur tabelası, yön kısıtı |
| 6 | `parking_logic` | Park manevrası |
| 7 | `slalom_logic` | Koni slalom |

**Engel Davranışları** (`obstacle_logic`):
- `CLEAR` — Yol açık
- `DYNAMIC_SLOW` — Yaya 10 m'de, yavaşla
- `DYNAMIC_WAIT` — Yaya 5 m'de, dur ve bekle; 0.4 s sonra lateral açıksa yanından geç
- `STATIC_AVOID` — Koni/bariyer 7 m'de, şerit değiştir (commit: 15 frame)
- `EMERGENCY_STOP` — Kritik mesafe (<1.5 m)

---

### 6. Kontrol Katmanı

**`vehicle_controller_node`** — Birden fazla kaynaktan gelen direksiyon ve hız referanslarını birleştirir (blending):

```
Planlama  → steering_ref (GPS takip)
Şerit     → steering_ref (lane centering)
Algı      → steering_override (engel kaçınması, yüksek öncelik)
Failsafe  → speed_cap, emergency_stop

→ Sonuç: cmd_vel (Twist: v_x, omega_z)
```

**`stm32_bridge_node`** — `cmd_vel`'i microROS üzerinden STM32'ye iletir:
- `/deos/actuators/stm32/speed_delta_mps`
- `/deos/actuators/stm32/steering_deg`

**`failsafe_supervisor_node`** — Tüm kritik node'ların ve sensörlerin sağlığını izler. Timeout veya anormallik durumunda `emergency_stop` gönderir.

---

### 7. Çalışma Ortamı

Sistem Docker konteyneri içinde çalışır:

**Konteyner: `epic_torvalds`**
- Base image: `ros:jazzy` (Ubuntu 24.04, aarch64)
- `network_mode: host` — LiDAR Ethernet erişimi için
- `privileged: true` — USB, serial, PCIe cihaz erişimi
- `ipc: host` — ROS paylaşımlı bellek

**Volume Mount'ları**:
```
./DEOS/deos_ws/src → /ros2_ws/src        # Geliştirme sırasında canlı mount
./models           → /ros2_ws/models      # HEF model dosyaları
./missions         → /ros2_ws/missions    # GeoJSON görev dosyaları
./logs             → /ros2_ws/logs        # Node çıktı logları
./ops/scripts      → /ros2_ws/ops/scripts # Operasyon araçları
```

**microROS Agent**: `microros/micro-ros-agent:jazzy` — Ayrı konteyner, UDP 8888'den STM32 bağlantısını dinler.

---

### 8. Otomatik Başlatma

Pi açılışında sistem otomatik olarak ayağa kalkar:

```
systemd: deos-compose.service
    └─ docker compose up
         ├─ epic_torvalds (deos_autostart.sh)
         │   ├─ colcon build
         │   ├─ ros2 launch vehicle_bringup main.launch.py
         │   └─ camera_stream_server.py (port 8080)
         └─ deos_micro_ros_agent

systemd: deos-log-monitor.service
    └─ log_monitor_server.py (port 9000)

systemd: deos-health.service
    └─ health_monitor.py (10 s aralık)
```

**İzleme Arayüzleri**:
- `http://<pi-ip>:8080` — Canlı kamera görüntüsü (MJPEG)
- `http://<pi-ip>:9000` — Gerçek zamanlı node logları

---

## Hızlı Başlangıç

```bash
# 1. Repoyu çek
git clone <repo-url> && cd DEOS_MimariDev

# 2. Model dosyalarını yerleştir
# models/detection.hef  → YOLOv8 nesne tespiti
# models/lane_seg.hef   → Şerit segmentasyonu

# 3. Görev dosyasını hazırla
# missions/mission.geojson

# 4. Image'ı derle (ilk kez)
docker compose build

# 5. Sistemi başlat
docker compose up -d

# 6. Logları izle
docker exec epic_torvalds tail -f /ros2_ws/logs/vehicle_controller.log
```

---

## Topic Şeması

```
/deos/
  sensors/
    camera/{color,depth}/image_raw
    imu/data
    gps/fix
    lidar/cloud_unstructured_fullframe
  perception/
    stereo/detections
    lidar/obstacles
    lane/center_pts
    fusion/{emergency_stop,speed_cap,steering_override,...}
  localization/
    odom/{ekf,icp,final}
  lane/
    steering_ref
    speed_limit
  planning/
    steering_ref, speed_limit, current_task, arrived, park_mode
  hardware/
    motion_enable, autonomy_enable
  control/
    cmd_vel
  actuators/
    stm32/{speed_delta_mps,steering_deg}
  failsafe/
    out/{emergency_stop,speed_cap,diagnostics}
```
