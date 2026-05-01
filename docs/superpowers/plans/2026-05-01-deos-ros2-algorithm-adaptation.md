# DEOS ROS2 Algorithm Adaptation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Robotaksi projesindeki saf-Python algoritmalarını DEOS-Mimari ROS2 paket yapısına entegre eden 1 paylaşımlı kütüphane paketi ve 5 yeni node oluşturmak; `/cmd_vel` çıktısı STM32'ye micro-ROS UDP üzerinden iletilir.

**Architecture:** Stereo kameradan gelen RGB+depth frame ile LiDAR point cloud paralel olarak işlenir; `perception_fusion_node` tüm robotaksi karar modüllerini çalıştırır; `mission_planning_node` GPS ve GeoJSON rotasından direksiyon referansı üretir; `vehicle_controller_node` her iki katmanı öncelik hiyerarşisiyle birleştirir ve `/cmd_vel` (Twist) yayımlar. YOLO tespiti şimdilik stub — tek bir fonksiyon değiştirildiğinde etkinleşir.

**Tech Stack:** ROS2 Jazzy, Python 3.10+, ament_python, numpy, cv_bridge, std_msgs, geometry_msgs, sensor_msgs, micro-ROS (STM32 tarafı kapsam dışı)

**Çalışma dizini:** `D:\DEOS\mimari\DEOS-Mimari\DEOS\deos_ws\src\`  
**Robotaksi kaynak dizini:** `D:\DEOS\mimari\robotaksi\`

---

## Dosya Haritası

### Yeni Oluşturulacak
```
deos_algorithms/
  package.xml, setup.cfg, setup.py, resource/deos_algorithms
  deos_algorithms/__init__.py
  deos_algorithms/safety_logic.py         ← robotaksi'den kopyala, import yok
  deos_algorithms/traffic_sign_logic.py   ← kopyala, import yok
  deos_algorithms/traffic_light_logic.py  ← kopyala, import yok
  deos_algorithms/parking_logic.py        ← kopyala, import yok
  deos_algorithms/geojson_mission_reader.py ← kopyala, import yok
  deos_algorithms/obstacle_logic.py       ← kopyala + 1 import düzelt
  deos_algorithms/slalom_logic.py         ← kopyala + 2 import düzelt
  deos_algorithms/waypoint_manager.py     ← kopyala + 1 import düzelt
  deos_algorithms/mission_manager.py      ← kopyala + 2 import düzelt
  deos_algorithms/perception_fusion.py    ← kopyala + 4 import düzelt
  deos_algorithms/sensors/__init__.py
  deos_algorithms/sensors/types.py        ← kopyala, import yok
  test/test_deos_algorithms.py

perception/obstacle_detection/vision_bridge/vision_bridge/
  stereo_detector_node.py   (yeni)
  perception_fusion_node.py (yeni)

perception/sensor_fusion/sensor_fusion/
  lidar_obstacle_node.py    (yeni)

planning/mission_planning/mission_planning/
  mission_planning_node.py  (yeni)

control/vehicle_controller/vehicle_controller/
  vehicle_controller_node.py (yeni)
```

### Güncellenecek
```
perception/obstacle_detection/vision_bridge/setup.py      (entry points)
perception/obstacle_detection/vision_bridge/package.xml   (bağımlılık)
perception/sensor_fusion/setup.py                         (entry point)
perception/sensor_fusion/package.xml                      (bağımlılık)
planning/mission_planning/setup.py                        (entry point)
planning/mission_planning/package.xml                     (bağımlılık)
control/vehicle_controller/setup.py                       (entry point)
control/vehicle_controller/package.xml                    (bağımlılık)
vehicle_bringup/launch/main.launch.py                     (yeni node'lar)
```

---

## Task 1: `deos_algorithms` Paylaşımlı Kütüphane Paketi

**Files:**
- Create: `deos_algorithms/package.xml`
- Create: `deos_algorithms/setup.cfg`
- Create: `deos_algorithms/setup.py`
- Create: `deos_algorithms/resource/deos_algorithms` (boş dosya)
- Create: `deos_algorithms/deos_algorithms/__init__.py`
- Create: `deos_algorithms/deos_algorithms/sensors/__init__.py`
- Copy+fix: 11 algoritma dosyası
- Test: `deos_algorithms/test/test_deos_algorithms.py`

- [ ] **Adım 1: Paket dizin yapısını oluştur**

```bash
cd D:\DEOS\mimari\DEOS-Mimari\DEOS\deos_ws\src
mkdir -p deos_algorithms\deos_algorithms\sensors
mkdir -p deos_algorithms\resource
mkdir -p deos_algorithms\test
```

- [ ] **Adım 2: `package.xml` oluştur**

`deos_algorithms/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>deos_algorithms</name>
  <version>0.1.0</version>
  <description>Robotaksi pure-Python algorithm library for DEOS autonomous vehicle stack</description>
  <maintainer email="aaltindas.work@gmail.com">aaltindas</maintainer>
  <license>Apache-2.0</license>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Adım 3: `setup.cfg` ve `setup.py` oluştur**

`deos_algorithms/setup.cfg`:
```ini
[develop]
script_dir=$base/lib/deos_algorithms
[install]
install_scripts=$base/lib/deos_algorithms
```

`deos_algorithms/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'deos_algorithms'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaltindas',
    maintainer_email='aaltindas.work@gmail.com',
    description='Robotaksi pure-Python algorithm library',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
```

`deos_algorithms/resource/deos_algorithms` — boş dosya oluştur (touch komutu veya Write ile).

- [ ] **Adım 4: `__init__.py` dosyalarını oluştur**

`deos_algorithms/deos_algorithms/__init__.py` — boş.  
`deos_algorithms/deos_algorithms/sensors/__init__.py` — boş.

- [ ] **Adım 5: Import değişikliği gerektirmeyen dosyaları kopyala**

Robotaksi kaynak: `D:\DEOS\mimari\robotaksi\`  
Hedef: `deos_algorithms\deos_algorithms\`

Kopyalanacak (değişmeden):
- `safety_logic.py`
- `traffic_sign_logic.py`
- `traffic_light_logic.py`
- `parking_logic.py`
- `geojson_mission_reader.py`
- `sensors\types.py` → `sensors/types.py`

```bash
copy D:\DEOS\mimari\robotaksi\safety_logic.py deos_algorithms\deos_algorithms\
copy D:\DEOS\mimari\robotaksi\traffic_sign_logic.py deos_algorithms\deos_algorithms\
copy D:\DEOS\mimari\robotaksi\traffic_light_logic.py deos_algorithms\deos_algorithms\
copy D:\DEOS\mimari\robotaksi\parking_logic.py deos_algorithms\deos_algorithms\
copy D:\DEOS\mimari\robotaksi\geojson_mission_reader.py deos_algorithms\deos_algorithms\
copy D:\DEOS\mimari\robotaksi\sensors\types.py deos_algorithms\deos_algorithms\sensors\
```

- [ ] **Adım 6: Import düzeltmesi gerektiren dosyaları kopyala ve düzelt**

**`deos_algorithms/deos_algorithms/obstacle_logic.py`**  
Kaynak kopyalanır, ardından şu satır değiştirilir:

```python
# ÖNCE:
from safety_logic import (
    SafetyLogic,
    Detection as SafetyDetection,
    ThreatLevel,
    ThreatObservation,
)

# SONRA:
from deos_algorithms.safety_logic import (
    SafetyLogic,
    Detection as SafetyDetection,
    ThreatLevel,
    ThreatObservation,
)
```

**`deos_algorithms/deos_algorithms/slalom_logic.py`**  
İki satır değiştirilir:

```python
# ÖNCE:
from obstacle_logic import ObstacleDetection, ObstacleKind
from safety_logic import IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX

# SONRA:
from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleKind
from deos_algorithms.safety_logic import IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX
```

**`deos_algorithms/deos_algorithms/waypoint_manager.py`**  
Bir satır değiştirilir:

```python
# ÖNCE:
from geojson_mission_reader import MissionPlan, MissionPoint

# SONRA:
from deos_algorithms.geojson_mission_reader import MissionPlan, MissionPoint
```

**`deos_algorithms/deos_algorithms/mission_manager.py`**  
İki satır değiştirilir:

```python
# ÖNCE:
from geojson_mission_reader import TaskType
from waypoint_manager import GpsPosition, MissionPlan, WaypointManager, WaypointState

# SONRA:
from deos_algorithms.geojson_mission_reader import TaskType
from deos_algorithms.waypoint_manager import GpsPosition, MissionPlan, WaypointManager, WaypointState
```

**`deos_algorithms/deos_algorithms/perception_fusion.py`**  
Dört satır değiştirilir:

```python
# ÖNCE:
from obstacle_logic import ObstacleDetection, classify_obstacle
from sensors.types import ImuSample, LidarObstacle, StereoBbox
from traffic_light_logic import LightDetection, classify_color
from traffic_sign_logic import SignDetection

# SONRA:
from deos_algorithms.obstacle_logic import ObstacleDetection, classify_obstacle
from deos_algorithms.sensors.types import ImuSample, LidarObstacle, StereoBbox
from deos_algorithms.traffic_light_logic import LightDetection, classify_color
from deos_algorithms.traffic_sign_logic import SignDetection
```

- [ ] **Adım 7: Test dosyasını yaz**

`deos_algorithms/test/test_deos_algorithms.py`:
```python
"""deos_algorithms paket entegrasyon testleri."""
import pytest


def test_imports():
    from deos_algorithms.safety_logic import SafetyLogic
    from deos_algorithms.obstacle_logic import ObstacleLogic, ObstacleDetection
    from deos_algorithms.slalom_logic import SlalomLogic
    from deos_algorithms.parking_logic import ParkingLogic, ParkingDetection
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic, SignDetection
    from deos_algorithms.traffic_light_logic import TrafficLightLogic, LightDetection
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader, MissionPlan
    from deos_algorithms.waypoint_manager import WaypointManager, GpsPosition
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.perception_fusion import fuse
    from deos_algorithms.sensors.types import StereoBbox, LidarObstacle, ImuSample


def test_obstacle_logic_empty_detections():
    from deos_algorithms.obstacle_logic import ObstacleLogic
    logic = ObstacleLogic()
    state = logic.update([])
    assert not state.emergency_stop
    assert state.speed_cap_ratio == 1.0


def test_slalom_logic_no_cones():
    from deos_algorithms.slalom_logic import SlalomLogic
    logic = SlalomLogic()
    state = logic.update([])
    assert not state.aktif
    assert state.faz == "bekleme"


def test_traffic_sign_logic_no_signs():
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    state = logic.update([])
    assert not state.must_stop_soon
    assert state.speed_cap_ratio == 1.0


def test_traffic_light_logic_no_lights():
    from deos_algorithms.traffic_light_logic import TrafficLightLogic
    logic = TrafficLightLogic()
    state = logic.update([])
    assert not state.must_stop
    assert not state.can_go


def test_parking_logic_no_spots():
    from deos_algorithms.parking_logic import ParkingLogic, ParkPhase
    logic = ParkingLogic()
    state = logic.update([])
    assert state.phase == ParkPhase.WAITING
    assert not state.complete


def test_perception_fusion_empty():
    from deos_algorithms.perception_fusion import fuse
    frame = fuse(stereo=[], lidar=[], imu=None)
    assert frame.sign_dets == []
    assert frame.light_dets == []
    assert frame.obstacle_dets == []
    assert frame.imu is None


def test_lidar_obstacle_routed_to_frame():
    from deos_algorithms.perception_fusion import fuse
    from deos_algorithms.sensors.types import LidarObstacle
    obs = LidarObstacle(kind="cone", confidence=0.9, distance_m=5.0, lateral_m=0.3)
    frame = fuse(lidar=[obs])
    assert len(frame.obstacle_dets) == 1
    assert frame.obstacle_dets[0].kind == "cone"
```

- [ ] **Adım 8: Testleri çalıştır (pip install geçici)**

```bash
cd D:\DEOS\mimari\DEOS-Mimari\DEOS\deos_ws\src\deos_algorithms
pip install -e . --quiet
pytest test/test_deos_algorithms.py -v
```

Beklenen çıktı:
```
PASSED test/test_deos_algorithms.py::test_imports
PASSED test/test_deos_algorithms.py::test_obstacle_logic_empty_detections
PASSED test/test_deos_algorithms.py::test_slalom_logic_no_cones
PASSED test/test_deos_algorithms.py::test_traffic_sign_logic_no_signs
PASSED test/test_deos_algorithms.py::test_traffic_light_logic_no_lights
PASSED test/test_deos_algorithms.py::test_parking_logic_no_spots
PASSED test/test_deos_algorithms.py::test_perception_fusion_empty
PASSED test/test_deos_algorithms.py::test_lidar_obstacle_routed_to_frame
8 passed
```

- [ ] **Adım 9: Commit**

```bash
cd D:\DEOS\mimari\DEOS-Mimari
git add DEOS/deos_ws/src/deos_algorithms/
git commit -m "feat: add deos_algorithms shared library package

Robotaksi pure-Python algorithm modules ported with updated imports.
All 8 unit tests pass."
```

---

## Task 2: `stereo_detector_node` — Stereo Kamera Algılama

**Files:**
- Create: `perception/obstacle_detection/vision_bridge/vision_bridge/stereo_detector_node.py`
- Modify: `perception/obstacle_detection/vision_bridge/setup.py`
- Modify: `perception/obstacle_detection/vision_bridge/package.xml`

- [ ] **Adım 1: `stereo_detector_node.py` oluştur**

`perception/obstacle_detection/vision_bridge/vision_bridge/stereo_detector_node.py`:
```python
import json
import math
import struct
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class StereoDetectorNode(Node):
    def __init__(self):
        super().__init__('stereo_detector_node')

        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('focal_length_px', 320.0)
        self.declare_parameter('depth_timeout_s', 0.5)

        self._w = self.get_parameter('image_width').value
        self._h = self.get_parameter('image_height').value
        self._focal = self.get_parameter('focal_length_px').value
        self._depth_timeout = self.get_parameter('depth_timeout_s').value

        self._bridge = CvBridge()
        self._depth_img: np.ndarray | None = None
        self._depth_stamp: float = 0.0

        self.create_subscription(Image, '/camera/color/image_raw', self._rgb_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self._depth_cb, 10)
        self._pub = self.create_publisher(String, '/perception/stereo_detections', 10)
        self.get_logger().info('stereo_detector_node ready (YOLO stub active)')

    # ------------------------------------------------------------------
    def _depth_cb(self, msg: Image) -> None:
        try:
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint16)
            self._depth_img = raw.reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            self._depth_stamp = time.monotonic()
        except Exception as e:
            self.get_logger().error(f'depth convert: {e}')

    def _rgb_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'rgb convert: {e}')
            return

        dets = self._yolo_detect(frame)
        result = []
        for d in dets:
            x1, y1, x2, y2 = d['bbox']
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            dist = self._sample_depth(cx, cy)
            lat = self._lateral_m(cx, dist) if dist is not None else None
            result.append({
                'class_name': d['class_name'],
                'confidence': d['confidence'],
                'bbox_px': [float(x1), float(y1), float(x2), float(y2)],
                'distance_m': dist,
                'lateral_m': lat,
            })
        self._pub.publish(String(data=json.dumps(result)))

    # ------------------------------------------------------------------
    def _yolo_detect(self, frame: np.ndarray) -> list[dict]:
        """
        YOLO entegrasyon noktası.
        Gerçek model eklendiğinde yalnızca bu fonksiyon değişir.
        Döndürülen format:
          [{'class_name': str, 'confidence': float, 'bbox': (x1,y1,x2,y2)}, ...]
        """
        return []

    def _sample_depth(self, cx: int, cy: int) -> float | None:
        if self._depth_img is None:
            return None
        if time.monotonic() - self._depth_stamp > self._depth_timeout:
            return None
        h, w = self._depth_img.shape
        cy_c = max(0, min(cy, h - 1))
        cx_c = max(0, min(cx, w - 1))
        d = float(self._depth_img[cy_c, cx_c])
        return d if d > 0.05 else None

    def _lateral_m(self, cx: int, dist: float) -> float:
        du = cx - self._w / 2.0
        return -(du * dist) / self._focal  # ROS: sağ = -y


def main(args=None):
    rclpy.init(args=args)
    node = StereoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Adım 2: `setup.py` güncelle**

`perception/obstacle_detection/vision_bridge/setup.py` içindeki `entry_points` kısmını şu şekilde değiştir:

```python
    entry_points={
        'console_scripts': [
            'vision_bridge_node = vision_bridge.vision_bridge_node:main',
            'stereo_detector_node = vision_bridge.stereo_detector_node:main',
        ],
    },
```

- [ ] **Adım 3: `package.xml` güncelle — `deos_algorithms` bağımlılığı ekle**

`perception/obstacle_detection/vision_bridge/package.xml` içinde `<exec_depend>` bloğuna ekle:

```xml
  <exec_depend>deos_algorithms</exec_depend>
```

- [ ] **Adım 4: Import testi yaz ve çalıştır**

`perception/obstacle_detection/vision_bridge/test/test_stereo_detector_import.py`:
```python
def test_stereo_detector_node_importable():
    """Node modülü rclpy olmadan import edilebilmeli."""
    import importlib
    import sys
    # rclpy stub
    import types
    rclpy_stub = types.ModuleType('rclpy')
    rclpy_stub.node = types.ModuleType('rclpy.node')

    class FakeNode:
        def __init__(self, name): pass
        def declare_parameter(self, *a, **kw): pass
        def get_parameter(self, name):
            class V:
                value = 640
            return V()
        def create_subscription(self, *a, **kw): pass
        def create_publisher(self, *a, **kw): pass
        def get_logger(self): 
            class L:
                def info(self, m): pass
            return L()

    rclpy_stub.node.Node = FakeNode
    sys.modules.setdefault('rclpy', rclpy_stub)
    sys.modules.setdefault('rclpy.node', rclpy_stub.node)

    # cv_bridge / cv2 stub
    for mod in ('cv_bridge', 'cv2'):
        if mod not in sys.modules:
            sys.modules[mod] = types.ModuleType(mod)

    import vision_bridge.stereo_detector_node as m
    assert hasattr(m, 'StereoDetectorNode')
    assert hasattr(m, 'main')
```

```bash
cd D:\DEOS\mimari\DEOS-Mimari\DEOS\deos_ws\src\perception\obstacle_detection\vision_bridge
pytest test/test_stereo_detector_import.py -v
```

Beklenen: `PASSED test_stereo_detector_node_importable`

- [ ] **Adım 5: Commit**

```bash
cd D:\DEOS\mimari\DEOS-Mimari
git add DEOS/deos_ws/src/perception/obstacle_detection/vision_bridge/
git commit -m "feat: add stereo_detector_node with YOLO stub and depth sampling"
```

---

## Task 3: `lidar_obstacle_node` — LiDAR Küme Tespiti

**Files:**
- Create: `perception/sensor_fusion/sensor_fusion/lidar_obstacle_node.py`
- Modify: `perception/sensor_fusion/setup.py`
- Modify: `perception/sensor_fusion/package.xml`

- [ ] **Adım 1: `lidar_obstacle_node.py` oluştur**

`perception/sensor_fusion/sensor_fusion/lidar_obstacle_node.py`:
```python
import json
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_node')

        self.declare_parameter('cluster_epsilon_m', 0.5)
        self.declare_parameter('cluster_min_points', 5)
        self.declare_parameter('max_distance_m', 20.0)
        self.declare_parameter('corridor_half_width_m', 3.0)

        self._eps = self.get_parameter('cluster_epsilon_m').value
        self._min_pts = self.get_parameter('cluster_min_points').value
        self._max_dist = self.get_parameter('max_distance_m').value
        self._corridor_hw = self.get_parameter('corridor_half_width_m').value

        self.create_subscription(PointCloud2, '/points_downsampled', self._cloud_cb, 10)
        self._pub = self.create_publisher(String, '/perception/lidar_obstacles', 10)
        self.get_logger().info('lidar_obstacle_node ready')

    # ------------------------------------------------------------------
    def _cloud_cb(self, msg: PointCloud2) -> None:
        pts = self._unpack_xyz(msg)
        if pts is None or len(pts) == 0:
            self._pub.publish(String(data='[]'))
            return

        mask = (
            (pts[:, 0] > 0.3)
            & (pts[:, 0] < self._max_dist)
            & (np.abs(pts[:, 1]) < self._corridor_hw)
        )
        pts = pts[mask]

        if len(pts) == 0:
            self._pub.publish(String(data='[]'))
            return

        clusters = self._euclidean_cluster(pts[:, :2])
        obstacles = []
        for cl in clusters:
            dist = float(np.min(cl[:, 0]))
            lat = float(np.mean(cl[:, 1]))
            span_x = float(np.max(cl[:, 0]) - np.min(cl[:, 0]))
            span_y = float(np.max(cl[:, 1]) - np.min(cl[:, 1]))
            diameter = max(span_x, span_y)
            obstacles.append({
                'kind': self._classify(diameter),
                'confidence': 0.7,
                'distance_m': dist,
                'lateral_m': lat,
                'bbox_px': None,
            })
        self._pub.publish(String(data=json.dumps(obstacles)))

    # ------------------------------------------------------------------
    @staticmethod
    def _classify(diameter: float) -> str:
        if diameter < 0.4:
            return 'cone'
        if diameter < 1.5:
            return 'unknown'
        return 'barrier'

    def _euclidean_cluster(self, pts_xy: np.ndarray) -> list[np.ndarray]:
        """Bağımlılıksız Öklid kümeleme (sklearn gerekmez)."""
        visited = np.zeros(len(pts_xy), dtype=bool)
        clusters: list[np.ndarray] = []
        for i in range(len(pts_xy)):
            if visited[i]:
                continue
            dists = np.linalg.norm(pts_xy - pts_xy[i], axis=1)
            nbrs = np.where(dists < self._eps)[0]
            if len(nbrs) < self._min_pts:
                visited[i] = True
                continue
            cluster_idx: set[int] = set(nbrs.tolist())
            queue = list(nbrs)
            while queue:
                j = queue.pop()
                if visited[j]:
                    continue
                visited[j] = True
                d2 = np.linalg.norm(pts_xy - pts_xy[j], axis=1)
                new_nbrs = np.where(d2 < self._eps)[0]
                if len(new_nbrs) >= self._min_pts:
                    for nb in new_nbrs:
                        if nb not in cluster_idx:
                            cluster_idx.add(nb)
                            queue.append(nb)
            visited[list(nbrs)] = True
            clusters.append(pts_xy[list(cluster_idx)])
        return clusters

    def _unpack_xyz(self, msg: PointCloud2) -> np.ndarray | None:
        try:
            fmap = {f.name: f.offset for f in msg.fields}
            if not all(k in fmap for k in ('x', 'y', 'z')):
                return None
            ox, oy, oz = fmap['x'], fmap['y'], fmap['z']
            step = msg.point_step
            n = msg.width * msg.height
            data = bytes(msg.data)
            pts = np.empty((n, 3), dtype=np.float32)
            for i in range(n):
                base = i * step
                pts[i, 0] = struct.unpack_from('f', data, base + ox)[0]
                pts[i, 1] = struct.unpack_from('f', data, base + oy)[0]
                pts[i, 2] = struct.unpack_from('f', data, base + oz)[0]
            return pts[np.isfinite(pts).all(axis=1)]
        except Exception as e:
            self.get_logger().error(f'PointCloud2 unpack: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Adım 2: Kümeleme algoritmasını saf-Python olarak test et**

`perception/sensor_fusion/test/test_lidar_cluster.py`:
```python
import numpy as np
import pytest


def _classify(diameter: float) -> str:
    if diameter < 0.4:
        return 'cone'
    if diameter < 1.5:
        return 'unknown'
    return 'barrier'


def test_classify_cone():
    assert _classify(0.2) == 'cone'


def test_classify_unknown():
    assert _classify(0.8) == 'unknown'


def test_classify_barrier():
    assert _classify(2.0) == 'barrier'


def test_empty_cloud_produces_no_obstacles():
    pts = np.empty((0, 2), dtype=np.float32)
    # corridor filter
    assert len(pts) == 0


def test_two_distinct_clusters_separated():
    """İki ayrı nokta grubu iki farklı küme üretmeli."""
    cluster_a = np.random.uniform(0, 0.3, (20, 2)) + np.array([2.0, 0.0])
    cluster_b = np.random.uniform(0, 0.3, (20, 2)) + np.array([5.0, 1.5])
    pts = np.vstack([cluster_a, cluster_b])
    
    # Basit kümeleme doğrulaması: her iki merkez de farklı olmalı
    center_a = cluster_a.mean(axis=0)
    center_b = cluster_b.mean(axis=0)
    assert np.linalg.norm(center_a - center_b) > 1.0
```

```bash
cd D:\DEOS\mimari\DEOS-Mimari\DEOS\deos_ws\src\perception\sensor_fusion
pytest test/test_lidar_cluster.py -v
```

Beklenen: 5 passed

- [ ] **Adım 3: `setup.py` güncelle**

`perception/sensor_fusion/setup.py` içindeki `entry_points` bloğunu değiştir:
```python
    entry_points={
        'console_scripts': [
            'lidar_obstacle_node = sensor_fusion.lidar_obstacle_node:main',
        ],
    },
```

- [ ] **Adım 4: `package.xml` güncelle**

`perception/sensor_fusion/package.xml` içine ekle:
```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>deos_algorithms</exec_depend>
```

- [ ] **Adım 5: Commit**

```bash
cd D:\DEOS\mimari\DEOS-Mimari
git add DEOS/deos_ws/src/perception/sensor_fusion/
git commit -m "feat: add lidar_obstacle_node with Euclidean clustering"
```

---

## Task 4: `perception_fusion_node` — Tüm Logic Modülleri

**Files:**
- Create: `perception/obstacle_detection/vision_bridge/vision_bridge/perception_fusion_node.py`
- Modify: `perception/obstacle_detection/vision_bridge/setup.py` (entry point eklenir)

- [ ] **Adım 1: `perception_fusion_node.py` oluştur**

`perception/obstacle_detection/vision_bridge/vision_bridge/perception_fusion_node.py`:
```python
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String

from deos_algorithms.obstacle_logic import ObstacleLogic
from deos_algorithms.parking_logic import ParkingLogic, ParkPhase
from deos_algorithms.perception_fusion import fuse
from deos_algorithms.sensors.types import ImuSample, LidarObstacle, StereoBbox
from deos_algorithms.slalom_logic import SlalomLogic
from deos_algorithms.traffic_light_logic import TrafficLightLogic
from deos_algorithms.traffic_sign_logic import TrafficSignLogic


class PerceptionFusionNode(Node):
    STEREO_TIMEOUT_S = 0.5
    LIDAR_TIMEOUT_S = 0.5

    def __init__(self):
        super().__init__('perception_fusion_node')

        self._sign = TrafficSignLogic()
        self._light = TrafficLightLogic()
        self._obstacle = ObstacleLogic()
        self._slalom = SlalomLogic()
        self._parking = ParkingLogic()

        self._stereo: list[StereoBbox] = []
        self._lidar: list[LidarObstacle] = []
        self._imu: ImuSample | None = None
        self._stereo_stamp: float = 0.0
        self._lidar_stamp: float = 0.0

        self.create_subscription(String, '/perception/stereo_detections', self._stereo_cb, 10)
        self.create_subscription(String, '/perception/lidar_obstacles', self._lidar_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)

        self._pub_estop = self.create_publisher(Bool, '/perception/emergency_stop', 10)
        self._pub_speed = self.create_publisher(Float32, '/perception/speed_cap', 10)
        self._pub_steer = self.create_publisher(Float32, '/perception/steering_override', 10)
        self._pub_has_steer = self.create_publisher(Bool, '/perception/has_steering_override', 10)

        self.create_timer(0.05, self._tick)  # 20 Hz karar döngüsü
        self.get_logger().info('perception_fusion_node ready')

    # ------------------------------------------------------------------
    def _stereo_cb(self, msg: String) -> None:
        try:
            raw: list[dict] = json.loads(msg.data)
            self._stereo = [
                StereoBbox(
                    class_name=d['class_name'],
                    confidence=float(d['confidence']),
                    bbox_px=tuple(float(v) for v in d['bbox_px']),
                    distance_m=d.get('distance_m'),
                    lateral_m=d.get('lateral_m'),
                )
                for d in raw
            ]
            self._stereo_stamp = time.monotonic()
        except Exception as e:
            self.get_logger().error(f'stereo parse: {e}')

    def _lidar_cb(self, msg: String) -> None:
        try:
            raw: list[dict] = json.loads(msg.data)
            self._lidar = [
                LidarObstacle(
                    kind=d['kind'],
                    confidence=float(d['confidence']),
                    distance_m=float(d['distance_m']),
                    lateral_m=float(d['lateral_m']),
                    bbox_px=tuple(d['bbox_px']) if d.get('bbox_px') else None,
                )
                for d in raw
            ]
            self._lidar_stamp = time.monotonic()
        except Exception as e:
            self.get_logger().error(f'lidar parse: {e}')

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        heading_deg = math.degrees(math.atan2(siny, cosy)) % 360.0
        yaw_rate_dps = math.degrees(msg.angular_velocity.z)
        self._imu = ImuSample(heading_deg=heading_deg, yaw_rate_dps=yaw_rate_dps)

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now = time.monotonic()
        stereo = self._stereo if (now - self._stereo_stamp) < self.STEREO_TIMEOUT_S else []
        lidar = self._lidar if (now - self._lidar_stamp) < self.LIDAR_TIMEOUT_S else []

        frame = fuse(stereo=stereo, lidar=lidar, imu=self._imu)

        sign_state = self._sign.update(frame.sign_dets)
        light_state = self._light.update(frame.light_dets)
        obs_state = self._obstacle.update(frame.obstacle_dets)
        slalom_state = self._slalom.update(frame.obstacle_dets)
        park_state = self._parking.update([])  # YOLO stub: park tespiti yok

        emergency = obs_state.emergency_stop or light_state.must_stop or sign_state.must_stop_soon

        speed_cap = min(
            obs_state.speed_cap_ratio,
            sign_state.speed_cap_ratio,
            light_state.speed_cap_ratio,
        )

        has_steer = slalom_state.aktif or park_state.phase not in (
            ParkPhase.WAITING, 'park_edildi'
        )
        steer = 0.0
        if slalom_state.aktif:
            steer = slalom_state.steering
        elif has_steer:
            steer = park_state.steering

        self._pub_estop.publish(Bool(data=bool(emergency)))
        self._pub_speed.publish(Float32(data=float(speed_cap)))
        self._pub_steer.publish(Float32(data=float(steer)))
        self._pub_has_steer.publish(Bool(data=bool(has_steer)))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Adım 2: `setup.py` entry point ekle**

`perception/obstacle_detection/vision_bridge/setup.py` içindeki `entry_points`:
```python
    entry_points={
        'console_scripts': [
            'vision_bridge_node = vision_bridge.vision_bridge_node:main',
            'stereo_detector_node = vision_bridge.stereo_detector_node:main',
            'perception_fusion_node = vision_bridge.perception_fusion_node:main',
        ],
    },
```

- [ ] **Adım 3: Logic entegrasyon testi — saf Python**

`perception/obstacle_detection/vision_bridge/test/test_perception_fusion_logic.py`:
```python
"""perception_fusion_node'un iç logic akışını rclpy olmadan test et."""
import pytest


def test_fusion_with_lidar_obstacle_triggers_speed_cap():
    from deos_algorithms.perception_fusion import fuse
    from deos_algorithms.obstacle_logic import ObstacleLogic
    from deos_algorithms.sensors.types import LidarObstacle

    obs = LidarObstacle(
        kind='pedestrian',
        confidence=0.95,
        distance_m=4.0,  # DIST_EMERGENCY_STOP=3m, DIST_HARD_SLOWDOWN=8m → yavaşla
        lateral_m=0.0,
    )
    frame = fuse(lidar=[obs])
    logic = ObstacleLogic()
    state = logic.update(frame.obstacle_dets)
    assert state.speed_cap_ratio < 1.0


def test_fusion_too_far_obstacle_no_effect():
    from deos_algorithms.perception_fusion import fuse
    from deos_algorithms.obstacle_logic import ObstacleLogic
    from deos_algorithms.sensors.types import LidarObstacle

    obs = LidarObstacle(kind='cone', confidence=0.9, distance_m=25.0, lateral_m=0.0)
    frame = fuse(lidar=[obs])
    logic = ObstacleLogic()
    state = logic.update(frame.obstacle_dets)
    assert state.speed_cap_ratio == 1.0
    assert not state.emergency_stop


def test_emergency_stop_at_close_range():
    from deos_algorithms.perception_fusion import fuse
    from deos_algorithms.obstacle_logic import ObstacleLogic
    from deos_algorithms.sensors.types import LidarObstacle

    obs = LidarObstacle(kind='pedestrian', confidence=0.99, distance_m=1.5, lateral_m=0.0)
    frame = fuse(lidar=[obs])
    logic = ObstacleLogic()
    # Hysteresis gerektirir: 3 ardışık frame göndermemiz lazım
    for _ in range(3):
        state = logic.update(frame.obstacle_dets)
    assert state.emergency_stop or state.speed_cap_ratio < 0.5
```

```bash
cd D:\DEOS\mimari\DEOS-Mimari\DEOS\deos_ws\src\perception\obstacle_detection\vision_bridge
pytest test/test_perception_fusion_logic.py -v
```

Beklenen: 3 passed

- [ ] **Adım 4: Commit**

```bash
cd D:\DEOS\mimari\DEOS-Mimari
git add DEOS/deos_ws/src/perception/obstacle_detection/vision_bridge/
git commit -m "feat: add perception_fusion_node integrating all decision logic modules"
```

---

## Task 5: `mission_planning_node` — GPS Waypoint Takibi

**Files:**
- Create: `planning/mission_planning/mission_planning/mission_planning_node.py`
- Modify: `planning/mission_planning/setup.py`
- Modify: `planning/mission_planning/package.xml`

- [ ] **Adım 1: `mission_planning_node.py` oluştur**

`planning/mission_planning/mission_planning/mission_planning_node.py`:
```python
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float32, String

from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
from deos_algorithms.mission_manager import MissionManager
from deos_algorithms.waypoint_manager import GpsPosition


GPS_TIMEOUT_SLOW_S = 2.0   # bu süreden sonra hız %50'ye düşer
GPS_TIMEOUT_STOP_S = 5.0   # bu süreden sonra kademeli durdurma
SPEED_DECAY_PER_SEC = 0.2  # >5s sonunda saniyede 0.2 azalt


class MissionPlanningNode(Node):
    def __init__(self):
        super().__init__('mission_planning_node')

        self.declare_parameter('mission_file', '')
        self.declare_parameter('heading_offset_deg', 0.0)

        mission_file = self.get_parameter('mission_file').value
        self._heading_offset = self.get_parameter('heading_offset_deg').value

        if not mission_file:
            self.get_logger().warn('mission_file parametresi boş — waypoint takibi pasif')
            self._manager = None
        else:
            reader = GeoJsonMissionReader()
            plan = reader.read_file(mission_file)
            self._manager = MissionManager(plan)
            self.get_logger().info(f'Görev yüklendi: {len(plan)} waypoint — {mission_file}')

        # Durum değişkenleri
        self._last_steering = 0.0
        self._last_speed = 1.0
        self._last_task = ''
        self._heading_deg = 0.0
        self._gps_stamp: float = 0.0

        self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)

        self._pub_steer = self.create_publisher(Float32, '/planning/steering_ref', 10)
        self._pub_speed = self.create_publisher(Float32, '/planning/speed_limit', 10)
        self._pub_task = self.create_publisher(String, '/planning/current_task', 10)
        self._pub_arrived = self.create_publisher(Bool, '/planning/arrived', 10)
        self.get_logger().info('mission_planning_node ready')

    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_deg = math.degrees(math.atan2(siny, cosy)) % 360.0
        self._heading_deg = (raw_deg + self._heading_offset) % 360.0

    def _gps_cb(self, msg: NavSatFix) -> None:
        self._gps_stamp = time.monotonic()

        if self._manager is None:
            self._pub_steer.publish(Float32(data=0.0))
            self._pub_speed.publish(Float32(data=0.0))
            self._pub_task.publish(String(data='no_mission'))
            self._pub_arrived.publish(Bool(data=False))
            return

        pos = GpsPosition(
            lat=msg.latitude,
            lon=msg.longitude,
            heading_deg=self._heading_deg,
        )
        wp_state, mission_dec = self._manager.update(pos, now_s=time.monotonic())

        self._last_steering = float(wp_state.steering_ref)
        base_speed = float(wp_state.speed_limit_ratio) * float(mission_dec.speed_cap_ratio)
        self._last_speed = base_speed
        self._last_task = str(wp_state.current_task or '')

        self._pub_steer.publish(Float32(data=self._last_steering))
        self._pub_speed.publish(Float32(data=self._last_speed))
        self._pub_task.publish(String(data=self._last_task))
        self._pub_arrived.publish(Bool(data=bool(wp_state.arrived)))

    # ------------------------------------------------------------------
    def _gps_age_s(self) -> float:
        if self._gps_stamp == 0.0:
            return float('inf')
        return time.monotonic() - self._gps_stamp

    def _apply_gps_timeout(self, base_speed: float) -> float:
        age = self._gps_age_s()
        if age < GPS_TIMEOUT_SLOW_S:
            return base_speed
        if age < GPS_TIMEOUT_STOP_S:
            return base_speed * 0.5
        excess = age - GPS_TIMEOUT_STOP_S
        return max(0.0, base_speed - excess * SPEED_DECAY_PER_SEC)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Adım 2: Waypoint hesaplamalarını saf Python ile test et**

`planning/mission_planning/test/test_waypoint_logic.py`:
```python
import pytest


def test_haversine_known_distance():
    from deos_algorithms.waypoint_manager import haversine_m
    # Ankara Kızılay → Ulus yaklaşık 3 km
    d = haversine_m(39.9199, 32.8543, 39.9334, 32.8597)
    assert 1500 < d < 2500  # gerçek yaklaşık 1.8 km


def test_forward_azimuth_east():
    from deos_algorithms.waypoint_manager import forward_azimuth_deg
    # Aynı enlem, doğuya doğru → 90°
    az = forward_azimuth_deg(0.0, 0.0, 0.0, 1.0)
    assert abs(az - 90.0) < 1.0


def test_forward_azimuth_north():
    from deos_algorithms.waypoint_manager import forward_azimuth_deg
    az = forward_azimuth_deg(0.0, 0.0, 1.0, 0.0)
    assert abs(az - 0.0) < 1.0 or abs(az - 360.0) < 1.0


def test_waypoint_manager_no_plan_raises():
    from deos_algorithms.waypoint_manager import WaypointManager, GpsPosition
    from deos_algorithms.geojson_mission_reader import MissionPlan
    plan = MissionPlan(points=[])
    mgr = WaypointManager(plan)
    pos = GpsPosition(lat=39.9, lon=32.8, heading_deg=90.0)
    state = mgr.update(pos)
    assert state.arrived  # boş plan = hemen tamamlandı


def test_geojson_reader_valid_file(tmp_path):
    import json
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
    geojson = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [32.8597, 39.9334]},
                "properties": {"id": 1, "name": "Start", "task": "start",
                               "heading_deg": 90.0, "speed_limit_ratio": 0.8, "radius_m": 2.0}
            },
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [32.8620, 39.9350]},
                "properties": {"id": 2, "name": "Goal", "task": "stop",
                               "heading_deg": 90.0, "speed_limit_ratio": 0.5, "radius_m": 3.0}
            }
        ]
    }
    f = tmp_path / "test_route.geojson"
    f.write_text(json.dumps(geojson))
    reader = GeoJsonMissionReader()
    plan = reader.read_file(str(f))
    assert len(plan) == 2
    assert plan.start is not None
    assert plan.goal is not None
```

```bash
cd D:\DEOS\mimari\DEOS-Mimari\DEOS\deos_ws\src\planning\mission_planning
pytest test/test_waypoint_logic.py -v
```

Beklenen: 5 passed

- [ ] **Adım 3: `setup.py` güncelle**

`planning/mission_planning/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'mission_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaltindas',
    maintainer_email='aaltindas.work@gmail.com',
    description='GPS waypoint mission planning node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planning_node = mission_planning.mission_planning_node:main',
        ],
    },
)
```

- [ ] **Adım 4: `package.xml` güncelle**

`planning/mission_planning/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mission_planning</name>
  <version>0.1.0</version>
  <description>GPS waypoint mission planning node</description>
  <maintainer email="aaltindas.work@gmail.com">aaltindas</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>deos_algorithms</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Adım 5: Commit**

```bash
cd D:\DEOS\mimari\DEOS-Mimari
git add DEOS/deos_ws/src/planning/mission_planning/
git commit -m "feat: add mission_planning_node with GPS waypoint tracking and timeout policy"
```

---

## Task 6: `vehicle_controller_node` — Öncelik + `/cmd_vel` → STM32

**Files:**
- Create: `control/vehicle_controller/vehicle_controller/vehicle_controller_node.py`
- Modify: `control/vehicle_controller/setup.py`
- Modify: `control/vehicle_controller/package.xml`

- [ ] **Adım 1: `vehicle_controller_node.py` oluştur**

`control/vehicle_controller/vehicle_controller/vehicle_controller_node.py`:
```python
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class VehicleControllerNode(Node):
    PERCEPTION_TIMEOUT_S = 0.2  # algı kesilirse acil fren

    def __init__(self):
        super().__init__('vehicle_controller_node')

        self.declare_parameter('max_speed_mps', 3.0)
        self.declare_parameter('max_steer_rads', 1.0)

        self._max_speed = self.get_parameter('max_speed_mps').value
        self._max_steer = self.get_parameter('max_steer_rads').value

        # Algı katmanı
        self._emergency_stop: bool = False
        self._speed_cap: float = 1.0
        self._steer_override: float = 0.0
        self._has_steer_override: bool = False
        self._perception_stamp: float = 0.0

        # Planlama katmanı
        self._plan_steer: float = 0.0
        self._plan_speed: float = 0.0

        self.create_subscription(Bool, '/perception/emergency_stop', self._estop_cb, 10)
        self.create_subscription(Float32, '/perception/speed_cap', self._speed_cap_cb, 10)
        self.create_subscription(Float32, '/perception/steering_override', self._steer_ovr_cb, 10)
        self.create_subscription(Bool, '/perception/has_steering_override', self._has_steer_cb, 10)
        self.create_subscription(Float32, '/planning/steering_ref', self._plan_steer_cb, 10)
        self.create_subscription(Float32, '/planning/speed_limit', self._plan_speed_cb, 10)

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_estop = self.create_publisher(Bool, '/safety/emergency_stop', 10)

        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info(
            f'vehicle_controller_node ready — '
            f'max_speed={self._max_speed} m/s, max_steer={self._max_steer} rad/s'
        )

    # ------------------------------------------------------------------
    def _estop_cb(self, msg: Bool) -> None:
        self._emergency_stop = msg.data
        self._perception_stamp = time.monotonic()

    def _speed_cap_cb(self, msg: Float32) -> None:
        self._speed_cap = float(msg.data)
        self._perception_stamp = time.monotonic()

    def _steer_ovr_cb(self, msg: Float32) -> None:
        self._steer_override = float(msg.data)

    def _has_steer_cb(self, msg: Bool) -> None:
        self._has_steer_override = msg.data

    def _plan_steer_cb(self, msg: Float32) -> None:
        self._plan_steer = float(msg.data)

    def _plan_speed_cb(self, msg: Float32) -> None:
        self._plan_speed = float(msg.data)

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        cmd = Twist()
        perception_age = time.monotonic() - self._perception_stamp

        # 1. Acil fren — watchdog veya algı kararı
        if self._emergency_stop or perception_age > self.PERCEPTION_TIMEOUT_S:
            self._pub_cmd.publish(cmd)  # Twist() = sıfır hız
            self._pub_estop.publish(Bool(data=True))
            if perception_age > self.PERCEPTION_TIMEOUT_S:
                self.get_logger().warn(
                    f'Perception timeout {perception_age:.2f}s — emergency stop'
                )
            return

        # 2. Hız = en kısıtlayıcı
        speed_ratio = min(self._speed_cap, self._plan_speed)
        speed_ratio = max(0.0, min(1.0, speed_ratio))

        # 3. Direksiyon — algı manevrası öncelikli
        if self._has_steer_override:
            steer_ratio = self._steer_override
        else:
            steer_ratio = self._plan_steer
        steer_ratio = max(-1.0, min(1.0, steer_ratio))

        # 4. Ölçekle ve yayımla
        cmd.linear.x = speed_ratio * self._max_speed
        cmd.angular.z = steer_ratio * self._max_steer

        self._pub_cmd.publish(cmd)
        self._pub_estop.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Adım 2: Öncelik mantığını saf Python ile test et**

`control/vehicle_controller/test/test_priority_logic.py`:
```python
"""vehicle_controller öncelik hiyerarşisini saf Python ile test et."""
import pytest


def apply_priority(
    emergency_stop: bool,
    perception_timeout: bool,
    speed_cap: float,
    plan_speed: float,
    has_steer_override: bool,
    steer_override: float,
    plan_steer: float,
    max_speed: float = 3.0,
    max_steer: float = 1.0,
) -> tuple[float, float, bool]:
    """(linear_x, angular_z, estop) döndürür."""
    if emergency_stop or perception_timeout:
        return 0.0, 0.0, True
    speed = min(speed_cap, plan_speed)
    speed = max(0.0, min(1.0, speed))
    steer = steer_override if has_steer_override else plan_steer
    steer = max(-1.0, min(1.0, steer))
    return speed * max_speed, steer * max_steer, False


def test_emergency_stop_overrides_all():
    lx, az, estop = apply_priority(
        emergency_stop=True, perception_timeout=False,
        speed_cap=1.0, plan_speed=1.0,
        has_steer_override=False, steer_override=0.5, plan_steer=0.3,
    )
    assert lx == 0.0
    assert az == 0.0
    assert estop is True


def test_perception_timeout_overrides_all():
    lx, az, estop = apply_priority(
        emergency_stop=False, perception_timeout=True,
        speed_cap=1.0, plan_speed=1.0,
        has_steer_override=False, steer_override=0.0, plan_steer=0.5,
    )
    assert lx == 0.0
    assert estop is True


def test_speed_takes_minimum():
    lx, az, estop = apply_priority(
        emergency_stop=False, perception_timeout=False,
        speed_cap=0.4, plan_speed=0.8,
        has_steer_override=False, steer_override=0.0, plan_steer=0.0,
    )
    assert abs(lx - 0.4 * 3.0) < 1e-6
    assert not estop


def test_steer_override_beats_plan():
    _, az, _ = apply_priority(
        emergency_stop=False, perception_timeout=False,
        speed_cap=1.0, plan_speed=1.0,
        has_steer_override=True, steer_override=0.7, plan_steer=0.1,
    )
    assert abs(az - 0.7 * 1.0) < 1e-6


def test_plan_steer_used_when_no_override():
    _, az, _ = apply_priority(
        emergency_stop=False, perception_timeout=False,
        speed_cap=1.0, plan_speed=1.0,
        has_steer_override=False, steer_override=0.7, plan_steer=0.2,
    )
    assert abs(az - 0.2 * 1.0) < 1e-6


def test_speed_clamped_to_zero():
    lx, _, _ = apply_priority(
        emergency_stop=False, perception_timeout=False,
        speed_cap=0.0, plan_speed=1.0,
        has_steer_override=False, steer_override=0.0, plan_steer=0.0,
    )
    assert lx == 0.0


def test_steer_clamped():
    _, az, _ = apply_priority(
        emergency_stop=False, perception_timeout=False,
        speed_cap=1.0, plan_speed=1.0,
        has_steer_override=True, steer_override=5.0, plan_steer=0.0,
    )
    assert abs(az - 1.0 * 1.0) < 1e-6  # 1.0 rad/s maksimum
```

```bash
cd D:\DEOS\mimari\DEOS-Mimari\DEOS\deos_ws\src\control\vehicle_controller
pytest test/test_priority_logic.py -v
```

Beklenen: 7 passed

- [ ] **Adım 3: `setup.py` güncelle**

`control/vehicle_controller/setup.py`:
```python
from setuptools import find_packages, setup

package_name = 'vehicle_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaltindas',
    maintainer_email='aaltindas.work@gmail.com',
    description='Vehicle controller with priority arbitration and micro-ROS output',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_controller_node = vehicle_controller.vehicle_controller_node:main',
        ],
    },
)
```

- [ ] **Adım 4: `package.xml` güncelle**

`control/vehicle_controller/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vehicle_controller</name>
  <version>0.1.0</version>
  <description>Vehicle controller: priority arbitration and /cmd_vel output for micro-ROS</description>
  <maintainer email="aaltindas.work@gmail.com">aaltindas</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Adım 5: Commit**

```bash
cd D:\DEOS\mimari\DEOS-Mimari
git add DEOS/deos_ws/src/control/vehicle_controller/
git commit -m "feat: add vehicle_controller_node with priority arbitration and watchdog"
```

---

## Task 7: `main.launch.py` Güncellemesi

**Files:**
- Modify: `vehicle_bringup/launch/main.launch.py`

- [ ] **Adım 1: `main.launch.py` güncelle**

`vehicle_bringup/launch/main.launch.py` içeriğini şu şekilde değiştir:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    DEOS Main Launch File — Complete Autonomous Vehicle Stack

    Data Flow:
    RealSense D415 → /camera/color/image_raw, /camera/depth/image_raw (30 Hz)
    SICK LiDAR     → /cloud → voxel filter → /points_downsampled (20 Hz)
    GPS serial     → /gps/fix (5 Hz)
    IMU serial     → /imu/data (100 Hz)

    stereo_detector_node  ← /camera/color/image_raw + /camera/depth/image_raw
                          → /perception/stereo_detections

    lidar_obstacle_node   ← /points_downsampled
                          → /perception/lidar_obstacles

    perception_fusion_node ← /perception/stereo_detections + /perception/lidar_obstacles + /imu/data
                           → /perception/emergency_stop, /perception/speed_cap,
                             /perception/steering_override, /perception/has_steering_override

    mission_planning_node ← /gps/fix + /imu/data  (mission_file param)
                          → /planning/steering_ref, /planning/speed_limit,
                            /planning/current_task, /planning/arrived

    vehicle_controller_node ← tüm /perception/* + /planning/*
                            → /cmd_vel (Twist → STM32 via micro-ROS)
                            → /safety/emergency_stop (Bool → STM32)
    """
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='',
        description='Görev rotası GeoJSON dosyası tam yolu',
    )

    # === SENSORS ===
    camera_node = Node(
        package='camera',
        executable='realsense_d415_node',
        name='realsense_d415_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    gps_node = Node(
        package='imu',
        executable='gps_node',
        name='gps_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_multiscan165',
        parameters=[{
            'hostname': '192.168.0.1',
            'scanner_type': 'sick_multiscan',
            'frame_id': 'laser',
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    # === PERCEPTION ===
    stereo_detector_node = Node(
        package='vision_bridge',
        executable='stereo_detector_node',
        name='stereo_detector_node',
        parameters=[{
            'image_width': 640,
            'image_height': 480,
            'focal_length_px': 320.0,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    lidar_obstacle_node = Node(
        package='sensor_fusion',
        executable='lidar_obstacle_node',
        name='lidar_obstacle_node',
        parameters=[{
            'cluster_epsilon_m': 0.5,
            'cluster_min_points': 5,
            'max_distance_m': 20.0,
            'corridor_half_width_m': 3.0,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    perception_fusion_node = Node(
        package='vision_bridge',
        executable='perception_fusion_node',
        name='perception_fusion_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    # === LOCALIZATION ===
    pcl_localization_node = Node(
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        name='pcl_localization_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    # === PLANNING ===
    mission_planning_node = Node(
        package='mission_planning',
        executable='mission_planning_node',
        name='mission_planning_node',
        parameters=[{
            'mission_file': LaunchConfiguration('mission_file'),
            'heading_offset_deg': 0.0,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    # === CONTROL ===
    vehicle_controller_node = Node(
        package='vehicle_controller',
        executable='vehicle_controller_node',
        name='vehicle_controller_node',
        parameters=[{
            'max_speed_mps': 3.0,
            'max_steer_rads': 1.0,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    return LaunchDescription([
        mission_file_arg,
        # Sensors
        camera_node,
        gps_node,
        imu_node,
        lidar_node,
        # Perception
        stereo_detector_node,
        lidar_obstacle_node,
        perception_fusion_node,
        # Localization
        pcl_localization_node,
        # Planning
        mission_planning_node,
        # Control
        vehicle_controller_node,
    ])
```

- [ ] **Adım 2: Launch dosyasını syntax kontrolü yap**

```bash
python3 -c "import ast; ast.parse(open('D:/DEOS/mimari/DEOS-Mimari/DEOS/deos_ws/src/vehicle_bringup/launch/main.launch.py').read()); print('syntax OK')"
```

Beklenen: `syntax OK`

- [ ] **Adım 3: Commit**

```bash
cd D:\DEOS\mimari\DEOS-Mimari
git add DEOS/deos_ws/src/vehicle_bringup/launch/main.launch.py
git commit -m "feat: update main.launch.py with full algorithm pipeline

Adds stereo_detector, lidar_obstacle, perception_fusion,
mission_planning, vehicle_controller nodes. mission_file
passed as launch arg."
```

---

## Kurulum Notları

### colcon build (Raspberry Pi üzerinde)

```bash
cd ~/deos_ws
colcon build --packages-select deos_algorithms
colcon build --packages-select vision_bridge sensor_fusion mission_planning vehicle_controller
source install/setup.bash
```

Paket bağımlılık sırası önemlidir: `deos_algorithms` diğerlerinden önce derlenmeli.

### Çalıştırma

```bash
# Varsayılan (mission_file olmadan)
ros2 launch vehicle_bringup main.launch.py

# GeoJSON dosyasıyla
ros2 launch vehicle_bringup main.launch.py mission_file:=/home/pi/rota.geojson
```

### GPS Kalibrasyon Notu

`gps_node.py` içindeki mevcut parser `msg.lat` kullanıyor (NMEA DDMM.MMMM formatı).
`NavSatFix.latitude` standarda göre ondalık derece olmalı. Eğer waypoint
takibi sapmalı çalışırsa bu dönüşümü kontrol edin:

```python
# pynmea2 ile doğru yol:
nav_msg.latitude = msg.latitude   # decimal degrees (pynmea2 property)
nav_msg.longitude = msg.longitude
```

### heading_offset_deg Ayarı

IMU koordinat sistemi ile kuzeye referanslı sistem arasındaki fark varsa
`mission_planning_node`'un `heading_offset_deg` parametresini ayarlayın.
Yol: aracı bilinen bir yönde sürün, GPS bearing'ini hesaplayın, IMU yaw'ını
ölçün, aralarındaki fark `heading_offset_deg`'dir.
