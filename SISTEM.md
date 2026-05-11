# DEOS otonom araç sistemi

Bu belge, **DEOS-Mimari** deposundaki yazılımın ne işe yaradığını, hangi bileşenlerden oluştuğunu ve çalışma zamanında nasıl bir araya geldiğini özetler. Teknik sözleşme detayları (topic listeleri, JSON şemaları) için [`README.md`](README.md); kurulum ve çalıştırma komutları için [`SETUP_ROS2_PIPELINE.md`](SETUP_ROS2_PIPELINE.md) kullanılır.

## Sistemin amacı

Raspberry Pi üzerinde **ROS 2 Jazzy** ile sensör verisini toplayıp işleyen, görev ve güvenlik kısıtlarına göre hız/direksiyon kararı üreten ve çıktıyı **STM32** tarafına **micro-ROS** üzerinden ileten bir **otonom araç yazılım yığınıdır**. Yarışma senaryosunda hedef, araç açıldığında mümkün olduğunca **insan müdahalesi olmadan** stack’in ayağa kalkmasıdır.

## Yüksek seviye yerleşim

| Katman | Ne yapar | Nerede |
|--------|-----------|--------|
| Linux + Docker | İzolasyon, tekrarlanabilir ortam, cihaz erişimi | Raspberry Pi |
| ROS 2 graph | Node’lar, topic’ler, mesajlar | `deos` container (host network) |
| micro-ROS Agent | UDP üzerinden MCU client’ı DDS graph’a köprüler | Ayrı container, host network |
| STM32 firmware | `/cmd_vel`, `/safety/emergency_stop` tüketir, aktüatör | Ethernet ile Pi’ye bağlı MCU |

**Önemli:** STM32’nin ROS’taki görünen adı (ör. `stm32_node`) pratikte eşleşmeyi tanımlamaz; sözleşme **`/cmd_vel`** ve **`/safety/emergency_stop`** topic’leri, mesaj tipleri ve QoS ile kurulur.

## Çalışma zamanı (özet akış)

1. **Docker Compose** iki servisi başlatır: `deos` (ana workspace) ve `micro_ros_agent` (UDP 8888 dinleyici). Tanım: [`docker-compose.yml`](docker-compose.yml).
2. `deos` container’ı varsayılan olarak etkileşimli `bash` ile açılır; stack’i çalıştırmak için içeriden `ros2 launch` çağrılır. Tam otomasyon için `command` satırının launch başlatacak şekilde özelleştirilmesi veya systemd/`docker exec` ile komut verilmesi gerekir (yarışma profili).
3. **`vehicle_bringup`** paketindeki `main.launch.py` sensörleri, algı/planlama/kontrol pipeline’ını ve PCL lokalizasyon node’unu tek seferde başlatır.
4. STM32, Ethernet üzerinden Pi’deki **micro-ROS Agent**’a bağlanır; agent ROS 2 DDS domain’inde diğer node’larla aynı grafikte yer alır.

## ROS 2 workspace yapısı

Kaynak ağaç: `DEOS/deos_ws/src/`.

- **Sensör sürücüleri** (`sensors/`): RealSense, SICK LiDAR, GPS, IMU vb.
- **Algı** (`perception/`): `vision_bridge`, `sensor_fusion` paketleri.
- **Planlama** (`planning/`): `mission_planning` ve ilgili yapılandırmalar.
- **Kontrol** (`control/`): `vehicle_controller`.
- **Bringup** (`vehicle_bringup/`): Ana launch dosyası.
- **Lokalizasyon** (`pcl_localization_ros2/`): PCL tabanlı lokalizasyon.
- **Ortak algoritmalar** (`deos_algorithms/`): ROS’tan bağımsız Python mantık kütüphanesi; node’lar ince adaptör olarak kullanır.

## Ana launch ile başlayan node’lar

`vehicle_bringup/launch/main.launch.py` özetle şunları başlatır:

| Node adı (ör.) | Paket / executable | Rol |
|----------------|---------------------|-----|
| `realsense_d415_node` | `camera` | RGB + derinlik |
| `gps_node`, `imu_node` | `imu` | Konum / ataletsel |
| `sick_multiscan165` | `sick_scan_xd` | LiDAR (Ethernet, örnek IP yapılandırması) |
| `stereo_detector_node` | `vision_bridge` | Görüntüden tespit (JSON string topic) |
| `lidar_obstacle_node` | `sensor_fusion` | Nokta bulutundan engel listesi (JSON string topic) |
| `perception_fusion_node` | `vision_bridge` | Algı birleştirme, hız tavanı / acil durdurma kısıtları |
| `pcl_localization_node` | `pcl_localization_ros2` | Lokalizasyon |
| `mission_planning_node` | `mission_planning` | GeoJSON görev, planlama çıktıları |
| `vehicle_controller_node` | `vehicle_controller` | Arbitraj, `/cmd_vel` ve güvenlik bool |

`mission_file` launch argümanı ile görev dosyası (GeoJSON) verilir; boş bırakılabilir, davranış paket içi varsayılanlara bağlıdır.

## Veri akışı (mantıksal)

Sensör ve algı çıktıları birleştirilir; planlama direksiyon/hız referansı üretir; kontrolcü bunları güvenlik kısıtlarıyla birleştirip STM32’nin dinlediği standart mesajlara çevirir. Çizgisel şema ve topic isimleri [`README.md`](README.md) içindeki “Veri akışı (pipeline)” bölümündedir.

**Geliştirme notu:** `lidar_obstacle_node` kaynakta `/points_downsampled` topic’ine abone olacak şekilde yazılmıştır. `main.launch.py` içinde bu isimde bir yayın yoksa, ya voxel filtre / dönüşüm zinciri launch’a eklenmeli ya da node parametreleri/driver topic’leri hizalanmalıdır. Aksi halde LiDAR tabanlı engel çıktısı üretilmez.

## Docker ve ağ

- `network_mode: host` ve `ipc: host`, sensörler ve LiDAR Ethernet trafiği için tipik tercihtir.
- `devices` ile RealSense (USB) ve seri portlar (GPS/IMU) container’a aktarılır.
- `micro_ros_agent` servisi `udp4 -p 8888 -v6` ile dinler; STM32 **localhost değil**, aynı L2/L3 ağda Pi’nin adresine UDP ile bağlanır.

## İlgili belgeler

| Belge | İçerik |
|-------|--------|
| [`README.md`](README.md) | Mimari derinliği, mesaj sözleşmesi, JSON örnekleri, paket listesi |
| [`SETUP_ROS2_PIPELINE.md`](SETUP_ROS2_PIPELINE.md) | Kurulum, build, çalıştırma |
| [`DEOS Architecture.html`](../DEOS%20Architecture.html) (repo üst dizininde) | Görsel mimari, systemd/health kavramları |

Bu dosya bilinçli olarak kısa tutulur; değişiklik yapılan tek şey yeni **`SISTEM.md`** eklenmesidir.
