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


def test_parking_detections_from_signs():
    """Park / park yasak tabelaları ParkingDetection listesine ayrışmalı."""
    from deos_algorithms.perception_fusion import parking_detections_from_signs
    from deos_algorithms.traffic_sign_logic import SignClass, SignDetection

    signs = [
        SignDetection(class_name=SignClass.PARKING_AREA, confidence=0.9, bbox_px=(0, 0, 10, 20)),
        SignDetection(class_name=SignClass.NO_PARKING, confidence=0.85, bbox_px=(5, 5, 15, 25)),
    ]
    dets = parking_detections_from_signs(signs)
    assert len(dets) == 2
    assert dets[0].parking_allowed is True
    assert dets[1].parking_allowed is False


def test_parking_only_allows_parking_sign_spot():
    """Yalnızca parking_allowed=True adaylar manevra hedefi olur; yasak slotta ilerleme yok."""
    from deos_algorithms.parking_logic import ParkingDetection, ParkingLogic, ParkPhase

    logic = ParkingLogic()
    banned = ParkingDetection(bbox_px=(400, 100, 900, 900), confidence=0.95, parking_allowed=False)
    st = logic.update([banned])
    assert st.phase == ParkPhase.WAITING
    assert st.no_eligible_spot is True
    assert "yasak" in st.reason.lower()


def test_parking_prefers_allowed_over_closer_forbidden():
    """Yasak slot kameraya daha yakın olsa bile izinli slot seçilmeli."""
    from deos_algorithms.parking_logic import ParkingDetection, ParkingLogic, ParkPhase

    logic = ParkingLogic()
    # y2 büyük = daha yakın; yasak daha yakın
    banned = ParkingDetection(bbox_px=(100, 100, 200, 1100), confidence=0.95, parking_allowed=False)
    allowed = ParkingDetection(bbox_px=(500, 100, 780, 500), confidence=0.95, parking_allowed=True)
    st = logic.update([banned, allowed])
    assert st.phase == ParkPhase.APPROACHING
    assert st.distance_m == pytest.approx(6.0, rel=0, abs=0.05)


def _confirm_light(logic, det, now0: float, frames: int = 2, **kwargs):
    st = None
    for i in range(frames):
        st = logic.update([det], now=now0 + i * 0.1, **kwargs)
    return st


def test_traffic_light_yellow_ilkgorus_yavaslama():
    """İlk görünen sarı: yavaşlama (prepare_to_stop), tam hız yok."""
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic()
    y = LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=15.0)
    st = _confirm_light(logic, y, 1000.0)
    assert st is not None
    assert st.prepare_to_stop is True
    assert st.prepare_to_move is False
    assert st.can_go is False
    assert st.speed_cap_ratio == pytest.approx(0.4)


def test_traffic_light_yellow_duruyorken_tavan_sifir():
    """Sarıda anlık hız ~0 verilirse durmaya devam (hız tavanı 0)."""
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic()
    y = LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=10.0)
    st = _confirm_light(logic, y, 2000.0, vehicle_speed_mps=0.0)
    assert st.speed_cap_ratio == pytest.approx(0.0)


def test_traffic_light_kirmizi_sonra_sari_hazirlik():
    """Kırmızı onayından sonra sarı: harekete hazırlık (prepare_to_move), dur zorunluluğu yok."""
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic()
    r = LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=12.0)
    y = LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=8.0)
    _confirm_light(logic, r, 3000.0)
    st = _confirm_light(logic, y, 3001.0)
    assert st.must_stop is False
    assert st.prepare_to_move is True
    assert st.prepare_to_stop is False
    assert st.speed_cap_ratio == pytest.approx(0.25)


def test_traffic_light_kirmizi_sonra_sari_duruyorken_sifir():
    """Kırmızı→sarı hazırlıkta da araç duruyorsa tavan 0 kalmalı."""
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic()
    r = LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=12.0)
    y = LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=8.0)
    _confirm_light(logic, r, 4000.0)
    st = _confirm_light(logic, y, 4001.0, vehicle_speed_mps=0.02)
    assert st.prepare_to_move is True
    assert st.speed_cap_ratio == pytest.approx(0.0)


def test_traffic_light_yesil_sonra_sari_yavaslama():
    """Yeşilden sonra sarı: yine yavaşlama dalı (prepare_to_stop), hazırlık dalı değil."""
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic()
    g = LightDetection(color=LightColor.GREEN, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=20.0)
    y = LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=10.0)
    _confirm_light(logic, g, 5000.0)
    st = _confirm_light(logic, y, 5001.0)
    assert st.prepare_to_stop is True
    assert st.prepare_to_move is False
    assert st.speed_cap_ratio == pytest.approx(0.4)

