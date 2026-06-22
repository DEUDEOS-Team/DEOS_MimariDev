# DEOS — Karar / Kontrol / Güvenlik (IYVS) metin şablonu ve bakım rehberi

Bu dosya, resmî mimari belgede kullanılacak **İddia — Yöntem — Veri — Sonuç** dörtlemesini **iki katmana** böler:

| Katman | Amaç | Ne zaman değişir? |
|--------|------|-------------------|
| **A — Süreç (donuk metin)** | Algoritma *ne yapar*, hangi güvenlik ilkeleri geçerli (koddan bağımsız dil). | Nadiren; yalnızca tasarım ilkesi değişirse. |
| **B — Uygulama haritası (güncellenir)** | Hangi paket/düğüm/topic/parametre bunu taşır. | Kod veya topic ağacı değişince **sadece B** güncellenir. |

Resmî Word metninde: **A** paragraflarını aynen taşıyabilirsiniz; **B**’yi tablo veya kısa madde listesi olarak ekleyin. Kod revizyonunda Word’ü baştan yazmak yerine **B tablosunu** güncellemeniz yeterlidir.

**Senkron kaynak (tek doğruluk):** `DEOS/deos_ws/src/deos_algorithms/deos_algorithms/ros_topic_layout.py`  
**Launch gerçeği:** `DEOS/deos_ws/src/vehicle_bringup/launch/main.launch.py`

---

## Genel şablon (her modül için kopyala-yapıştır)

### X.Y.Z [Modül adı]

**İddia (A):**  
[Ölçülebilir, tek cümlelik davranış veya güvence.]

**Yöntem — süreç (A):**  
[Deterministik öncelik / arbitraj / izleme-fail-safe gibi *ilke*; paket adı vermeden.]

**Yöntem — uygulama haritası (B) — güncellenir**

| Bileşen | Konum (repo) | Not |
|---------|----------------|-----|
| … | `...` | … |

**Veri (B ağırlıklı):**  
[Topic kökü `{{DEOS_ROOT}}` varsayılan `/deos`, mesaj tipleri, tipik timeout/parametre *isimleri*; sayısal değerleri mümkünse tabloya.]

**Sonuç (B ağırlıklı):**  
[Doğrulama: senaryo kodu, rosbag, pytest; metrik kutusu: “doldurulacak”.]

---

## 5.4.x — Karar modülü

### İddia (A)

Çok kaynaklı algı ve kural çıktıları birleştirildiğinde araç **tek ve tekrarlanabilir** bir davranış kümesi (acil durdurma, hız tavanı, isteğe bağlı direksiyon üst yazımı) üretir; **acil durdurma** diğer tüm kısıtlardan önceliklidir.

### Yöntem — süreç (A)

Karar birleştirme **deterministik öncelik** ile yapılır: acil durdurma birleşiminde baskınlık, hız tavanlarında indirgeme (min), direksiyon üst yazımında öncelik sırasına göre seçim uygulanır; şerit sınırları yalnızca seçilen kaçınma davranışlarında ek kısıt olarak devreye girebilir.

### Yöntem — uygulama haritası (B) — güncellenir

| Rol | Bileşen | Konum |
|-----|---------|--------|
| ROS düğümü | `perception_fusion_node` | `DEOS/deos_ws/src/perception/obstacle_detection/vision_bridge/vision_bridge/perception_fusion_node.py` |
| Öncelik çekirdeği | `DecisionArbiter` | `DEOS/deos_ws/src/deos_algorithms/deos_algorithms/decision_arbiter.py` |
| Kural modülleri (ör.) | `TrafficSignLogic`, `TrafficLightLogic`, `ObstacleLogic`, `SlalomLogic`, `ParkingLogic` | `DEOS/deos_ws/src/deos_algorithms/deos_algorithms/` |
| Topic sözleşmesi | `build_deos_topics` | `DEOS/deos_ws/src/deos_algorithms/deos_algorithms/ros_topic_layout.py` |

### Veri (B)

- Girdi topic’leri `ros_topic_layout.py` içindeki `perception_*`, `planning_park_mode`, sensör ve donanım bayrakları ile uyumlu tam yollar.  
- `deos_root` launch argümanı (varsayılan `/deos`) tüm ağacı önekler.  
- İzlenebilirlik: `perception_fusion_decision_debug` (String) — içerik formatı kodla birlikte evrilebilir.

### Sonuç (B)

| Senaryo kimliği | Beklenen çıktılar | Kanıt |
|-----------------|-------------------|--------|
| *(doldurulacak)* | `emergency_stop`, `speed_cap`, `has_steering_override` | rosbag / `decision_debug` |
| … | … | `deos_algorithms/test/...` |

---

## 5.4.x — Kontrol modülü

### İddia (A)

Planlama ve (veri taze ise) şerit referansları, algı ve güvenlik kısıtları ile birleştirilerek **tek nihai hareket komutu** üretilir; **manuel sürüş** veya **donanım hareket izni yok** iken otonom hareket komutu güvenli biçimde kesilir veya güvenlik çıkışı kontrollü üretilir.

### Yöntem — süreç (A)

Hız tarafında kısıtlar **indirgeme** ile birleştirilir; direksiyonda üst yazım varsa öncelik verilir, aksi halde plan/şerit referansı kullanılır; acil durumda güvenlik mesajı **darbeli yayın politikası** ile üretilebilir; aktüasyon tarafında köprü düğümü ile birim dönüşümü yapılır.

### Yöntem — uygulama haritası (B) — güncellenir

| Rol | Bileşen | Konum |
|-----|---------|--------|
| Arbitraj + Twist | `vehicle_controller_node` | `DEOS/deos_ws/src/control/vehicle_controller/vehicle_controller/vehicle_controller_node.py` |
| STM32 köprüsü | `stm32_bridge_node` | `DEOS/deos_ws/src/control/vehicle_controller/vehicle_controller/stm32_bridge_node.py` |
| Topic sözleşmesi | `build_deos_topics` | `ros_topic_layout.py` |

### Veri (B)

- Çıktı: `control_cmd_vel`, `safety_emergency_stop` (`ros_topic_layout` anahtarları).  
- Parametre *isimleri* (değerler revizyonla değişebilir): `max_speed_mps`, `max_steer_rads`, `PERCEPTION_TIMEOUT_S`, `lane_timeout_s`, `hardware_motion_enable_timeout_s`, `safety_emergency_stop_pulse_count`, `use_lane_control`.

### Sonuç (B)

| Test | Beklenen gözlem | Kanıt |
|------|-----------------|--------|
| Algı timeout | `cmd_vel` sıfır; estop politikası | rosbag |
| `autonomy_enable=false` | `cmd_vel` yok (spam yok) | echo |
| Köprü | STM32 topic’leri `cmd_vel` ile tutarlı ölçek | saha log |

---

## 5.4.x — Güvenlik modülü

### İddia (A)

Kritik veri akışları izlenerek risk artıkça önce **kısıtlayıcı** çıktı, gerekirse **acil durdurma** üretilir; acil durdurma durumu **manuel sıfırlama** ile normale dönebilir.

### Yöntem — süreç (A)

Modül sağlığı özetlenir; çoklu uyarıda düşük hız profiline geçiş; kritik hatalarda acil durdurma; açılışta yanlış pozitifleri azaltmak için **başlangıç toleransı** uygulanabilir.

### Yöntem — uygulama haritası (B) — güncellenir

| Rol | Bileşen | Konum |
|-----|---------|--------|
| Süpervizör düğümü | `failsafe_supervisor_node` | `DEOS/deos_ws/src/control/deos_failsafe/deos_failsafe/failsafe_supervisor_node.py` |
| Karar çekirdeği | `FailSafeDecisionCore` | `DEOS/deos_ws/src/control/deos_failsafe/deos_failsafe/decision_engine_core.py` |
| FSM | `AutonomousFSM`, `SystemState` | `DEOS/deos_ws/src/control/deos_failsafe/deos_failsafe/fsm.py` |
| Topic sözleşmesi | `failsafe_out_*`, `failsafe_in_fsm_reset` | `ros_topic_layout.py` |

### Veri (B)

- İzlenen girişler: kamera, LiDAR bulutu, IMU, stereo string, plan hız/direksiyon, `cmd_vel` (süpervizör parametreleriyle seçilir).  
- Parametre *isimleri*: `sensor_timeout_s`, `perception_timeout_s`, `planning_timeout_s`, `startup_grace_s`, plan doğrulama eşikleri (`planning_max_speed_mps`, vb.).

### Sonuç (B)

| Olay | Beklenen FSM / çıktı | Kanıt |
|------|----------------------|--------|
| Tek sensör kesintisi | WARNING veya hız tavanı | `failsafe/out/diagnostics` |
| Çoklu kritik | EMERGENCY_STOP | echo + reset |
| Manuel reset | NORMAL | `failsafe/in/fsm_reset` |

---

## Bakım checklist’i (kod değişince)

1. `ros_topic_layout.py` diff’ine bak: **B** tablolarındaki topic adları güncellendi mi?  
2. `main.launch.py`: yeni düğüm eklendiyse **B** haritasına satır ekle.  
3. **A** metnini yalnızca *tasarım ilkesi* değiştiyse güncelle (ör. öncelik sırası ilkesi).  
4. **Sonuç** tablolarına saha/rosbag referansı ekle; metrik hücrelerini doldur.

---

*Dosya sürümü: metin şablonu; kodla birlikte revize edin. Son güncelleme: 2026-05-13.*
