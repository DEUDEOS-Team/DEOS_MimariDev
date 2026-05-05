"""
Bu dosya `deos_algorithms` paketinin davranışını **pytest** ile doğrular.

Amaç: Kod yazmayı bilmeyen biri bile `pytest -v` çıktısındaki test isimlerinden
“ne kontrol edildiğini” anlayabilsin.

Çalıştırma:
  cd DEOS/deos_ws/src/deos_algorithms
  python -m pytest -q
  python -m pytest -vv   # daha okunur isimler + daha detaylı çıktı
"""

import sys
from pathlib import Path

import pytest


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_imports():
    """Paket modülleri import edilebiliyor mu? (kurulum bozuksa ilk burada patlar)"""
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


def test_obstacle_logic_empty():
    """Engel yokken araç hız tavanı serbest kalmalı ve acil durdurma olmamalı."""
    from deos_algorithms.obstacle_logic import ObstacleLogic

    logic = ObstacleLogic()
    state = logic.update([])
    assert state.speed_cap_ratio == 1.0, "Engel yokken hız tavanı 1.0 (tam) olmalı."
    assert not state.emergency_stop, "Engel yokken acil durdurma false olmalı."


def test_slalom_no_cones():
    """Koni yokken slalom modu aktif olmamalı (bekleme fazında kalmalı)."""
    from deos_algorithms.slalom_logic import SlalomLogic

    logic = SlalomLogic()
    state = logic.update([])
    assert state.faz == "bekleme", "Koni yokken faz 'bekleme' olmalı."
    assert not state.aktif, "Koni yokken slalom aktif olmamalı."


@pytest.mark.parametrize(
    "frames, beklenen",
    [
        pytest.param(2, "NONE", id="TR:2_kare_goruntu_yeterli_degil"),
        pytest.param(3, "HARD_SLOW", id="TR:3_kare_goruntu_yeterli"),
    ],
)
def test_safety_logic_yaya_onay_kareleri(frames: int, beklenen: str):
    """
    Güvenlik modülü tek karede panik yapmamak için birkaç kare üst üste görme ister.

    Senaryo: Aynı yaya kutusu birkaç kare üst üste geliyor.
    Beklenti: 2 karede henüz tehdit yok; 3 karede yavaşlama seviyesi oluşur.
    """
    from deos_algorithms.safety_logic import Detection, SafetyLogic, ThreatLevel

    logic = SafetyLogic()
    det = Detection(
        x1=600,
        y1=300,
        x2=680,
        y2=700,
        class_name="pedestrian",
        confidence=0.95,
        estimated_distance_m=6.0,
        estimated_lateral_m=0.0,
    )

    decision = None
    for _ in range(frames):
        decision = logic.analyze([det]).decision

    assert decision is not None
    assert decision.threat_level.name == beklenen, f"Beklenen tehdit seviyesi {beklenen}, gelen: {decision.threat_level.name}"


def test_obstacle_logic_yaya_yakinda_bekleme():
    """
    Yaya çok yakınsa (mesafe eşiğinin altında) sistem 'bekle' moduna geçebilir.

    Not: Bu modül içerde güvenlik onayı (birkaç kare) kullandığı için aynı tespiti
    birkaç kez üst üste veriyoruz.
    """
    from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleLogic

    logic = ObstacleLogic()
    ped = ObstacleDetection(
        kind="pedestrian",
        confidence=0.95,
        bbox_px=(610, 250, 700, 700),
        estimated_distance_m=4.0,
        estimated_lateral_m=0.0,
    )

    state = None
    for _ in range(5):
        state = logic.update([ped])

    assert state is not None
    assert state.waiting_for_dynamic_obstacle is True, "Yakın yaya için bekleme modu bekleniyor."
    assert state.speed_cap_ratio == 0.0, "Bekleme modunda hız tavanı 0 olmalı."


def test_slalom_iki_koni_kapi_baslangici():
    """
    İki koni sağ/sol kapı gibi duruyorsa slalom 'başlangıç kapısı' moduna geçebilir.

    Beklenti: aktif olur ve hedef_taraf 'kapi' olur.
    """
    from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleKind
    from deos_algorithms.slalom_logic import SlalomLogic

    slalom = SlalomLogic()
    cone_left = ObstacleDetection(
        kind=ObstacleKind.CONE,
        confidence=0.9,
        bbox_px=(900, 300, 980, 650),
        estimated_distance_m=2.0,
        estimated_lateral_m=0.8,
    )
    cone_right = ObstacleDetection(
        kind=ObstacleKind.CONE,
        confidence=0.9,
        bbox_px=(200, 300, 280, 650),
        estimated_distance_m=2.0,
        estimated_lateral_m=-0.8,
    )

    state = slalom.update([cone_left, cone_right])
    assert state.aktif is True, "İki koni varken slalom aktif olmalı."
    assert state.hedef_taraf == "kapi", "Zıt tarafta iki koni varsa hedef_taraf 'kapi' olmalı."


def test_perception_fusion_kisa_senaryo():
    """
    Kamera+Lidar girdileri birleştirilince doğru 'kovalara' ayrılmalı.

    Beklenti:
    - 'red' ışık -> ışık listesine
    - 'koni' -> engel listesine
    - STOP tabelası -> tabela listesine
    - lidar engeli -> engel listesine
    """
    from deos_algorithms.perception_fusion import fuse
    from deos_algorithms.sensors.types import ImuSample, LidarObstacle, StereoBbox

    stereo = [
        StereoBbox(class_name="red", confidence=0.9, bbox_px=(0, 0, 1, 1), distance_m=20.0),
        StereoBbox(class_name="koni", confidence=0.9, bbox_px=(10, 10, 20, 20), distance_m=5.0, lateral_m=0.2),
        StereoBbox(class_name="dur tabelası", confidence=0.9, bbox_px=(5, 5, 6, 6), distance_m=8.0),
    ]
    lidar = [LidarObstacle(kind="barrier", confidence=0.8, distance_m=6.0, lateral_m=-0.5)]
    imu = ImuSample(heading_deg=10.0)

    frame = fuse(stereo=stereo, lidar=lidar, imu=imu)
    assert len(frame.light_dets) == 1, "Bir adet trafik ışığı bekleniyor."
    assert len(frame.sign_dets) == 1, "Bir adet tabela bekleniyor."
    assert len(frame.obstacle_dets) == 2, "Koni + lidar engeli -> iki engel bekleniyor."
    assert frame.imu is not None and frame.imu.heading_deg == 10.0, "IMU bilgisi aynen taşınmalı."

