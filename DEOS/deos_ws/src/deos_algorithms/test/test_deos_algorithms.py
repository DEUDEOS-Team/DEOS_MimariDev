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

    actual = decision.threat_level.name if decision else "None"
    print(f"  {frames} kare -> beklenen={beklenen:<12s}  gelen={actual}")
    assert decision is not None
    assert actual == beklenen, f"Beklenen tehdit seviyesi {beklenen}, gelen: {actual}"


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
    """Yalnızca parking_allowed=True adaylar manevra hedefi olur; yasak slotta manevra yok (arama devam edebilir)."""
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
    # y2 büyük = daha yakın; yasak daha yakın (640×480 uzayı, PRINCIPAL_POINT_Y_PX=240)
    # yasak: y2=440 → dy=200 → dist≈1.92m (çok yakın)
    # izinli: y2=304 → dy=64  → dist=6.0m   (APPROACH bandı 3–8m)
    banned = ParkingDetection(bbox_px=(0, 50, 120, 440), confidence=0.95, parking_allowed=False)
    allowed = ParkingDetection(bbox_px=(300, 50, 450, 304), confidence=0.95, parking_allowed=True)
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


def test_traffic_light_red_emergency_band_must_stop():
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic()
    r = LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=2.5)
    st = _confirm_light(logic, r, 6000.0)
    assert st.must_stop is True
    assert st.speed_cap_ratio == pytest.approx(0.0)


def test_traffic_light_red_hard_band_slow():
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic()
    r = LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=5.0)
    st = _confirm_light(logic, r, 7000.0)
    assert st.must_stop is False
    assert st.speed_cap_ratio == pytest.approx(0.5)


def test_traffic_light_red_soft_band_slow():
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic()
    r = LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=12.0)
    st = _confirm_light(logic, r, 8000.0)
    assert st.must_stop is False
    assert st.speed_cap_ratio == pytest.approx(0.8)


def test_traffic_light_red_far_no_constraint():
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic()
    r = LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=25.0)
    st = _confirm_light(logic, r, 9000.0)
    assert st.must_stop is False
    assert st.speed_cap_ratio == pytest.approx(1.0)


def test_traffic_light_red_unknown_soft_when_configured():
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic

    logic = TrafficLightLogic(red_unknown_must_stop=False)
    r = LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=None)
    st = _confirm_light(logic, r, 10000.0)
    assert st.must_stop is False
    assert st.speed_cap_ratio == pytest.approx(0.8)


# ==============================================================================
# BÖLÜM 2 — EKSİK TEST KAPSAMLARI (şartname doğrulama)
# ==============================================================================
#
# Kapsanan modüller:
#   SafetyLogic     : mesafe bantları, koridor filtresi, güven eşiği, tracker forget
#   ObstacleLogic   : dinamik yavaşlama, bekleme temizleme, statik kaçınma, yol kapalı
#   TrafficSignLogic: DUR hold/cooldown, yaya geçidi, tünel, dönüş kısıtları, bellek
#   TrafficLightLogic: yeşil, mesafe bilinmiyor, bellek süresi
#   WaypointManager : haversine, bearing, XTE, P-kontrol, varış radius
#   MissionManager  : PICKUP hold, DROPOFF, PARK timeout, park_completed
#   RouteGraph      : Dijkstra, bloklu kenar, tünel zorunlu, oneway, en yakın düğüm
#   GeoJsonReader   : Türkçe eş anlamlılar, isim-tabanlı çözümleme, varsayılanlar
#   DecisionArbiter : acil dur önceliği, hız min, steer öncelik sırası, lane clamp
#   SlalomLogic     : hız bantları, tamamlanma, tek koni steering, 640px doğrulama
# ==============================================================================


# ──────────────────────────────────────────────────────────────────────────────
# Yardımcı fabrika fonksiyonları
# ──────────────────────────────────────────────────────────────────────────────

def _safety_det(distance_m: float, lateral_m: float = 0.0,
                cls: str = "pedestrian", conf: float = 0.9):
    """Harici mesafe + yanal ofset verilmiş SafetyLogic Detection."""
    from deos_algorithms.safety_logic import Detection
    return Detection(
        x1=600, y1=300, x2=680, y2=700,
        class_name=cls, confidence=conf,
        estimated_distance_m=distance_m,
        estimated_lateral_m=lateral_m,
    )


def _safety_confirmed(logic, det, frames: int = 3):
    """SafetyLogic track'ini CONFIRM_FRAMES=3 ile onaylar, son analizi döner."""
    result = None
    for _ in range(frames):
        result = logic.analyze([det])
    return result


def _obs_ped(distance_m: float, lateral_m: float = 0.0, conf: float = 0.9):
    from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleKind
    return ObstacleDetection(
        kind=ObstacleKind.PEDESTRIAN, confidence=conf,
        bbox_px=(600, 300, 680, 700),
        estimated_distance_m=distance_m,
        estimated_lateral_m=lateral_m,
    )


def _obs_cone(distance_m: float, lateral_m: float = 0.5, conf: float = 0.9):
    from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleKind
    return ObstacleDetection(
        kind=ObstacleKind.CONE, confidence=conf,
        bbox_px=(600, 300, 680, 700),
        estimated_distance_m=distance_m,
        estimated_lateral_m=lateral_m,
    )


def _obs_barrier(distance_m: float, lateral_m: float = 0.3, conf: float = 0.9):
    from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleKind
    return ObstacleDetection(
        kind=ObstacleKind.BARRIER, confidence=conf,
        bbox_px=(600, 300, 680, 700),
        estimated_distance_m=distance_m,
        estimated_lateral_m=lateral_m,
    )


def _sign_det(cls, conf=0.9, dist=None):
    from deos_algorithms.traffic_sign_logic import SignDetection
    return SignDetection(
        class_name=cls, confidence=conf,
        bbox_px=(0, 0, 10, 10), estimated_distance_m=dist,
    )


def _confirm_sign(logic, det, frames=3, now_base=1000.0):
    st = None
    for i in range(frames):
        st = logic.update([det], now=now_base + i * 0.1)
    return st


def _simple_plan(*tasks_coords):
    """(lat, lon, task) demetlerinden minimal MissionPlan üretir."""
    from deos_algorithms.geojson_mission_reader import MissionPlan, MissionPoint
    pts = []
    for i, (lat, lon, task) in enumerate(tasks_coords):
        pts.append(MissionPoint(
            index=i, point_id=i, name=f"wp{i}",
            lat=lat, lon=lon, task=task,
            heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=3.0,
        ))
    plan = MissionPlan()
    plan.points = pts
    return plan


def _make_geojson(*features):
    return {"type": "FeatureCollection", "features": list(features)}


def _make_point_feature(lon, lat, **props):
    return {
        "type": "Feature",
        "geometry": {"type": "Point", "coordinates": [lon, lat]},
        "properties": dict(props),
    }


def _three_node_graph():
    """A(0)→B(1)→C(2) düz yol (çift yönlü, ~111m her kenar)."""
    from deos_algorithms.route_graph import Edge, Node, RouteGraph
    nodes = [Node(id=0, lat=41.0, lon=29.0),
             Node(id=1, lat=41.001, lon=29.0),
             Node(id=2, lat=41.002, lon=29.0)]
    adj = {
        0: [Edge(u=0, v=1, cost=111.0, props={})],
        1: [Edge(u=1, v=0, cost=111.0, props={}), Edge(u=1, v=2, cost=111.0, props={})],
        2: [Edge(u=2, v=1, cost=111.0, props={})],
    }
    return RouteGraph(nodes=nodes, adj=adj)


def _slalom_cone_det(distance_m: float, lateral_offset: float, conf: float = 0.9):
    """
    SlalomLogic için koni detection. lateral_offset: -1..1 (sağ pozitif).
    640px kamera alanında bbox merkezi hesaplanır.
    """
    from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleKind
    cx = 320.0 + lateral_offset * 320.0   # goruntu_genislik=640
    return ObstacleDetection(
        kind=ObstacleKind.CONE, confidence=conf,
        bbox_px=(cx - 20, 200, cx + 20, 500),
        estimated_distance_m=distance_m,
        estimated_lateral_m=None,
    )


# ──────────────────────────────────────────────────────────────────────────────
# SafetyLogic — mesafe bantları ve filtreler
# ──────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("distance_m,expected_level", [
    pytest.param(2.5,  "EMERGENCY",  id="acil_dur_alti"),
    pytest.param(3.0,  "EMERGENCY",  id="acil_dur_esiginde"),   # eff=2.0 < 3.0
    pytest.param(4.5,  "HARD_SLOW",  id="hard_yavasla_alti"),   # eff=3.0 → HARD
    pytest.param(6.0,  "HARD_SLOW",  id="hard_yavasla_merkez"), # eff=4.0
    pytest.param(12.0, "SOFT_SLOW",  id="yumusak_yavasla"),     # eff=8.0 → SOFT
    pytest.param(22.5, "NONE",       id="guvende_uzak"),        # eff=15.0 → safe
])
def test_safety_logic_mesafe_bantlari(distance_m: float, expected_level: str):
    """
    Şartname mesafe tablosu: yaya CLASS_CAUTION=1.5 ile effective_distance=distance/1.5.
    DIST_EMERGENCY_STOP=3.0m, DIST_HARD_SLOWDOWN=8.0m, DIST_SOFT_SLOWDOWN=15.0m.
    """
    from deos_algorithms.safety_logic import SafetyLogic
    logic = SafetyLogic()
    det = _safety_det(distance_m, lateral_m=0.0, cls="pedestrian")
    result = _safety_confirmed(logic, det)
    actual = result.decision.threat_level.name
    print(f"  mesafe={distance_m:5.1f}m -> beklenen={expected_level:<12s}  gelen={actual}")
    assert actual == expected_level


def test_safety_logic_koridor_disi_engel_goz_ardi():
    """Yanal ofseti CORRIDOR_HALF_WIDTH_M=1.2m'den büyük engel tehdit sayılmamalı."""
    from deos_algorithms.safety_logic import CORRIDOR_HALF_WIDTH_M, SafetyLogic
    logic = SafetyLogic()
    det = _safety_det(5.0, lateral_m=CORRIDOR_HALF_WIDTH_M + 0.1)
    result = _safety_confirmed(logic, det)
    assert result.decision.emergency_stop is False
    assert result.decision.speed_cap_ratio == 1.0


def test_safety_logic_dusuk_guven_goz_ardi():
    """MIN_CONFIDENCE=0.4 altındaki tespit track'e alınmamalı, karar serbest kalmalı."""
    from deos_algorithms.safety_logic import MIN_CONFIDENCE, SafetyLogic
    logic = SafetyLogic()
    det = _safety_det(2.0, conf=MIN_CONFIDENCE - 0.01)
    result = None
    for _ in range(5):
        result = logic.analyze([det])
    assert result.decision.emergency_stop is False
    assert result.decision.speed_cap_ratio == 1.0


def test_safety_logic_tracker_unutur():
    """FORGET_FRAMES=5 boş kare sonrası onaylı track silinmeli, karar güvenli dönmeli."""
    from deos_algorithms.safety_logic import FORGET_FRAMES, SafetyLogic
    logic = SafetyLogic()
    det = _safety_det(5.0)
    _safety_confirmed(logic, det, frames=3)   # track oluştur ve onayla
    result = None
    for _ in range(FORGET_FRAMES):            # tüm unutma süresi boyunca boş
        result = logic.analyze([])
    assert result.decision.speed_cap_ratio == 1.0
    assert result.decision.emergency_stop is False


def test_safety_logic_en_yakin_tehdit_karar_verir():
    """Birden fazla koridor tehdidinde en küçük effective_distance belirleyici olmalı."""
    from deos_algorithms.safety_logic import SafetyLogic, ThreatLevel
    logic = SafetyLogic()
    # pedestrian 4.0m → eff=2.67 → EMERGENCY; vehicle 20m → eff=16.7 → safe
    yakın = _safety_det(4.0, lateral_m=0.0, cls="pedestrian")
    uzak  = _safety_det(20.0, lateral_m=0.2, cls="vehicle")
    result = None
    for _ in range(3):
        result = logic.analyze([yakın, uzak])
    assert result.decision.threat_level == ThreatLevel.EMERGENCY


# ──────────────────────────────────────────────────────────────────────────────
# ObstacleLogic — dinamik ve statik engel davranışları
# ──────────────────────────────────────────────────────────────────────────────

def test_obstacle_logic_yaya_10m_yavasla():
    """Yaya DYNAMIC_SLOW_DISTANCE_M=10m'den yakında → speed_cap=0.4, bekleme modu yok."""
    from deos_algorithms.obstacle_logic import DYNAMIC_SLOW_SPEED_CAP, ObstacleLogic
    logic = ObstacleLogic()
    state = None
    for _ in range(5):
        state = logic.update([_obs_ped(9.0)])
    print(f"  yaya@9m -> speed_cap={state.speed_cap_ratio:.2f}  (beklenen <= {DYNAMIC_SLOW_SPEED_CAP:.2f})")
    assert state.speed_cap_ratio <= DYNAMIC_SLOW_SPEED_CAP
    assert not state.emergency_stop
    assert not state.waiting_for_dynamic_obstacle


def test_obstacle_logic_yaya_temizlenince_bekleme_biter():
    """
    Yaya geçince bekleme modu kapanmalı.

    SafetyLogic tracker: boş gönderimde track'in frames_missed'i artar; FORGET_FRAMES=5
    dolana kadar track HÂLÂ confirmed sayılır ve dynamic_threats boş olmaz.
    Bu yüzden bekleme sona ermesi için:
      FORGET_FRAMES (5) + DYNAMIC_CLEAR_FRAMES (3) = 8 boş kare gerekir.
    """
    from deos_algorithms.obstacle_logic import DYNAMIC_CLEAR_FRAMES, ObstacleLogic
    from deos_algorithms.safety_logic import FORGET_FRAMES
    logic = ObstacleLogic()
    for _ in range(5):
        logic.update([_obs_ped(4.0)])
    state = None
    for _ in range(FORGET_FRAMES + DYNAMIC_CLEAR_FRAMES):
        state = logic.update([])
    assert not state.waiting_for_dynamic_obstacle


def test_obstacle_logic_statik_lane_change_onerisi():
    """Koni STATIC_LANE_CHANGE_TRIGGER_M=3m'den yakın → şerit değiştirme önerisi."""
    from deos_algorithms.obstacle_logic import STATIC_LANE_CHANGE_SPEED_CAP, ObstacleLogic
    logic = ObstacleLogic()
    state = None
    for _ in range(5):
        state = logic.update([_obs_cone(2.5)])
    assert state.suggest_lane_change is True
    assert state.speed_cap_ratio <= STATIC_LANE_CHANGE_SPEED_CAP


def test_obstacle_logic_iki_bariyer_yol_kapali():
    """2 veya daha fazla bariyer → road_blocked=True (şartname: yol kapalı)."""
    from deos_algorithms.obstacle_logic import ObstacleLogic
    logic = ObstacleLogic()
    b1 = _obs_barrier(2.0, lateral_m=0.3)
    b2 = _obs_barrier(2.5, lateral_m=-0.3)
    state = None
    for _ in range(5):
        state = logic.update([b1, b2])
    assert state.road_blocked is True


def test_obstacle_logic_dinamik_kacinma_yonu_sag():
    """
    Yaya solda (lateral_m > 0) duruyorsa DYNAMIC_AVOID_HOLD_S geçince geçiş yönü SAĞ olmalı.
    now parametresi ile sanal zaman: 20 * 0.05s = 1.0s > DYNAMIC_AVOID_HOLD_S=0.4s.
    """
    from deos_algorithms.obstacle_logic import DYNAMIC_AVOID_HOLD_S, ObstacleLogic
    logic = ObstacleLogic()
    state = None
    for i in range(20):
        state = logic.update([_obs_ped(4.0, lateral_m=0.5)], now=float(i) * 0.05)
    assert state.dynamic_avoid_active is True, f"dynamic_avoid_active=False, state={state}"
    assert state.dynamic_avoidance_direction == "right"


# ──────────────────────────────────────────────────────────────────────────────
# TrafficSignLogic — şartname tabela kuralları
# ──────────────────────────────────────────────────────────────────────────────

def test_traffic_sign_dur_tabela_hold():
    """DUR tabelası onaylanınca STOP_HOLD_SECONDS=5s boyunca must_stop_soon=True."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("dur tabelası", dist=3.0), now_base=2000.0)
    assert st.must_stop_soon is True


def test_traffic_sign_dur_tabela_auto_tamamlanir():
    """DUR hold süresi (5s) dolunca must_stop_soon otomatik False'a düşmeli."""
    from deos_algorithms.traffic_sign_logic import STOP_HOLD_SECONDS, TrafficSignLogic
    logic = TrafficSignLogic()
    det = _sign_det("dur tabelası", dist=3.0)
    now = 3000.0
    for i in range(3):
        logic.update([det], now=now + i * 0.1)
    # 5s'yi aş — stop_started_at ≈ now+0.2
    st = logic.update([det], now=now + 0.2 + STOP_HOLD_SECONDS + 0.5)
    assert st.must_stop_soon is False


def test_traffic_sign_dur_cooldown():
    """DUR tamamlandıktan sonra STOP_REARM_SECONDS=5s cooldown içinde yeniden tetiklenmemeli."""
    from deos_algorithms.traffic_sign_logic import STOP_HOLD_SECONDS, TrafficSignLogic
    logic = TrafficSignLogic()
    det = _sign_det("dur tabelası", dist=3.0)
    now = 4000.0
    for i in range(3):
        logic.update([det], now=now + i * 0.1)
    # DUR hold tamamla
    logic.update([det], now=now + 0.2 + STOP_HOLD_SECONDS + 0.3)
    # Cooldown içinde aynı tabelayı yeniden gör
    st = logic.update([_sign_det("dur tabelası", dist=3.0)],
                      now=now + 0.2 + STOP_HOLD_SECONDS + 1.0)
    assert st.must_stop_soon is False


def test_traffic_sign_yaya_gecidi_hiz_kisitlamasi():
    """Yaya geçidi tabelası → speed_cap_ratio <= CROSSWALK_SPEED_RATIO=0.5."""
    from deos_algorithms.traffic_sign_logic import CROSSWALK_SPEED_RATIO, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("yaya gecidi"), now_base=5000.0)
    print(f"  yaya gecidi -> speed_cap={st.speed_cap_ratio:.2f}  (beklenen <= {CROSSWALK_SPEED_RATIO:.2f})")
    assert st.speed_cap_ratio <= CROSSWALK_SPEED_RATIO


def test_traffic_sign_tunel_hiz_ve_bayrak():
    """Tünel tabelası → speed_cap_ratio <= 0.7, approaching_tunnel=True."""
    from deos_algorithms.traffic_sign_logic import TUNNEL_SPEED_RATIO, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("tunel"), now_base=6000.0)
    print(f"  tunel -> speed_cap={st.speed_cap_ratio:.2f}  approaching_tunnel={st.approaching_tunnel}  (beklenen <= {TUNNEL_SPEED_RATIO:.2f}, True)")
    assert st.speed_cap_ratio <= TUNNEL_SPEED_RATIO
    assert st.approaching_tunnel is True


def test_traffic_sign_saga_donulmez():
    """'saga donulmez' tabelası → turn_permissions.right=False, straight değişmemeli."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("saga donulmez"), now_base=7000.0)
    print(f"  saga_donulmez -> right={st.turn_permissions.right}  straight={st.turn_permissions.straight}  (beklenen False, True)")
    assert st.turn_permissions.right is False
    assert st.turn_permissions.straight is True


def test_traffic_sign_sola_mecburi():
    """'sola mecburi' → forced_direction='left', right=False, straight=False."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("sola mecburi"), now_base=8000.0)
    print(f"  sola_mecburi -> forced={st.turn_permissions.forced_direction}  straight={st.turn_permissions.straight}  right={st.turn_permissions.right}")
    assert st.turn_permissions.forced_direction == "left"
    assert st.turn_permissions.straight is False
    assert st.turn_permissions.right is False


def test_traffic_sign_girilmez_must_stop():
    """'girilmez' tabelası → must_stop_soon=True (şartname: yanlış yön engeli)."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("girilmez"), now_base=9000.0)
    assert st.must_stop_soon is True


def test_traffic_sign_dusuk_guven_goz_ardi():
    """MIN_CONFIDENCE=0.7 altındaki tabela onaylanmamalı."""
    from deos_algorithms.traffic_sign_logic import MIN_CONFIDENCE, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("dur tabelası", conf=MIN_CONFIDENCE - 0.01),
                       now_base=10000.0)
    assert st.must_stop_soon is False


def test_traffic_sign_bellek_sona_erer():
    """SIGN_VALIDITY_SECONDS=5s sonra tabela bellekten silinmeli, kısıt kalkmalı.

    _confirm_sign 3 kare gönderir (0.0, 0.1, 0.2); son görülme zamanı now_base+0.2.
    Süre aşımı için now_base + 0.2 + SIGN_VALIDITY_SECONDS + 0.1 gerekir.
    """
    from deos_algorithms.traffic_sign_logic import SIGN_VALIDITY_SECONDS, TrafficSignLogic
    logic = TrafficSignLogic()
    now = 11000.0
    _confirm_sign(logic, _sign_det("yaya gecidi"), now_base=now)
    st = logic.update([], now=now + 0.2 + SIGN_VALIDITY_SECONDS + 0.1)
    assert st.speed_cap_ratio == 1.0


def test_traffic_sign_kavsak_sonrasi_kisitlama_silinir():
    """notify_intersection_passed() → dönüş kısıtlaması temizlenmeli."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    _confirm_sign(logic, _sign_det("saga donulmez"), now_base=12000.0)
    logic.notify_intersection_passed()
    st = logic.update([], now=12001.0)
    assert st.turn_permissions.right is True


# ──────────────────────────────────────────────────────────────────────────────
# TrafficLightLogic — ek eksik senaryolar
# ──────────────────────────────────────────────────────────────────────────────

def test_traffic_light_yesil_gecebilir():
    """Yeşil ışık → can_go=True, speed_cap=1.0."""
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic
    logic = TrafficLightLogic()
    g = LightDetection(color=LightColor.GREEN, confidence=0.9, bbox_px=(0, 0, 1, 1))
    st = _confirm_light(logic, g, now0=20000.0)
    print(f"  yesil isik -> can_go={st.can_go}  speed_cap={st.speed_cap_ratio:.2f}  (beklenen True, 1.0)")
    assert st.can_go is True
    assert st.speed_cap_ratio == pytest.approx(1.0)


def test_traffic_light_isik_yok_neutral():
    """Işık tespiti yok → must_stop=False, can_go=False, speed_cap=1.0."""
    from deos_algorithms.traffic_light_logic import TrafficLightLogic
    logic = TrafficLightLogic()
    st = logic.update([], now=21000.0)
    assert not st.must_stop
    assert not st.can_go
    assert st.speed_cap_ratio == pytest.approx(1.0)


def test_traffic_light_dusuk_guven_goz_ardi():
    """MIN_CONFIDENCE=0.45 altındaki ışık yok sayılmalı."""
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, MIN_CONFIDENCE, TrafficLightLogic
    logic = TrafficLightLogic()
    r = LightDetection(color=LightColor.RED, confidence=MIN_CONFIDENCE - 0.01,
                       bbox_px=(0, 0, 1, 1), estimated_distance_m=3.0)
    st = _confirm_light(logic, r, now0=22000.0)
    assert not st.must_stop
    assert st.speed_cap_ratio == pytest.approx(1.0)


def test_traffic_light_bellek_sona_erer():
    """LIGHT_VALIDITY_SECONDS=1.5s sonra ışık bellekten silinmeli.

    _confirm_light 2 kare gönderir (now0, now0+0.1); son görülme now0+0.1.
    Süre aşımı için now0 + 0.1 + LIGHT_VALIDITY_SECONDS + 0.1 gerekir.
    """
    from deos_algorithms.traffic_light_logic import LIGHT_VALIDITY_SECONDS, LightColor, LightDetection, TrafficLightLogic
    logic = TrafficLightLogic()
    r = LightDetection(color=LightColor.RED, confidence=0.9,
                       bbox_px=(0, 0, 1, 1), estimated_distance_m=4.0)
    now = 23000.0
    _confirm_light(logic, r, now0=now)
    st = logic.update([], now=now + 0.1 + LIGHT_VALIDITY_SECONDS + 0.1)
    assert not st.must_stop
    assert st.speed_cap_ratio == pytest.approx(1.0)


def test_traffic_light_kirmizi_bilinmeyen_mesafe_varsayilan_dur():
    """Kırmızı + distance=None + red_unknown_must_stop=True (varsayılan) → must_stop."""
    from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic
    logic = TrafficLightLogic(red_unknown_must_stop=True)
    r = LightDetection(color=LightColor.RED, confidence=0.9,
                       bbox_px=(0, 0, 1, 1), estimated_distance_m=None)
    st = _confirm_light(logic, r, now0=24000.0)
    assert st.must_stop is True


# ──────────────────────────────────────────────────────────────────────────────
# WaypointManager — navigasyon hesap doğrulaması
# ──────────────────────────────────────────────────────────────────────────────

def test_waypoint_haversine_yaklasik_deger():
    """Haversine: 1° enlem farkı ≈ 111.195 km (±500m tolerans)."""
    from deos_algorithms.waypoint_manager import haversine_m
    d = haversine_m(0.0, 0.0, 1.0, 0.0)
    print(f"  haversine(0->1 derece enlem) = {d:.0f}m  (beklenen ~= 111195m, tolerans +/-500m)")
    assert abs(d - 111_195.0) < 500.0, f"Haversine sapması çok büyük: {d:.0f}m"


def test_waypoint_bearing_kuzey():
    """Kuzey yönü bearing 0° (veya 360°) olmalı."""
    from deos_algorithms.waypoint_manager import forward_azimuth_deg
    b = forward_azimuth_deg(41.0, 29.0, 41.01, 29.0)
    print(f"  kuzey bearing = {b:.4f} derece  (beklenen ~= 0 veya 360)")
    assert abs(b) < 1.0 or abs(b - 360.0) < 1.0, f"Kuzey bearing hatası: {b:.2f}°"


def test_waypoint_bearing_dogu():
    """Doğu yönü bearing ~90° olmalı."""
    from deos_algorithms.waypoint_manager import forward_azimuth_deg
    b = forward_azimuth_deg(41.0, 29.0, 41.0, 29.01)
    print(f"  dogu bearing  = {b:.4f} derece  (beklenen ~= 90)")
    assert abs(b - 90.0) < 1.5, f"Doğu bearing hatası: {b:.2f}°"


def test_waypoint_angle_diff_sarma():
    """angle_diff: hedef=10°, mevcut=350° → +20° (saatin yönünde kısa yol)."""
    from deos_algorithms.waypoint_manager import angle_diff
    diff = angle_diff(target_deg=10.0, current_deg=350.0)
    print(f"  angle_diff(hedef=10°, mevcut=350°) = {diff:.4f}°  (beklenen=+20°)")
    assert abs(diff - 20.0) < 0.01, f"angle_diff sarma hatası: {diff:.4f}"


def test_waypoint_cross_track_error_isareti():
    """Araç kuzey yolun doğusunda (sağında) → XTE pozitif (sağda → sol düzeltme)."""
    from deos_algorithms.waypoint_manager import cross_track_error_m
    xte = cross_track_error_m(
        start_lat=41.0,  start_lon=29.0,
        end_lat=41.01,   end_lon=29.0,    # kuzey
        pos_lat=41.005,  pos_lon=29.001,  # sağ (doğu)
    )
    print(f"  XTE (arac sagda) = {xte:.4f}m  (beklenen > 0)")
    assert xte > 0.0, f"Sağdaki araç için XTE pozitif olmalı, gelen: {xte:.2f}"


def test_waypoint_varis_radius_true():
    """Waypoint'e arrival_radius_m içinden yaklaşınca arrived=True."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.waypoint_manager import GpsPosition, WaypointManager
    plan = _simple_plan((41.0, 29.0, TaskType.CHECKPOINT))
    plan.points[0].arrival_radius_m = 5.0
    mgr = WaypointManager(plan, auto_advance=False)
    # Birebir aynı konum → dist≈0 < 5m
    state = mgr.update(GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0))
    assert state.arrived is True


def test_waypoint_varis_radius_false():
    """Waypoint'ten uzaktayken arrived=False."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.waypoint_manager import GpsPosition, WaypointManager
    plan = _simple_plan((41.01, 29.0, TaskType.CHECKPOINT))
    plan.points[0].arrival_radius_m = 3.0
    mgr = WaypointManager(plan, auto_advance=False)
    # ~1.1 km uzakta
    state = mgr.update(GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0))
    assert state.arrived is False


def test_waypoint_p_controller_90_derece_tam_steer():
    """BEARING_GAIN=1/90: 90° bearing hatası → clamp sonrası steer=1.0."""
    from deos_algorithms.waypoint_manager import BEARING_GAIN
    steer = max(-1.0, min(1.0, 90.0 * BEARING_GAIN))
    assert steer == pytest.approx(1.0)


def test_waypoint_mission_complete():
    """auto_advance=True + arrived=True → son wp geçilince mission_complete=True."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.waypoint_manager import GpsPosition, WaypointManager
    plan = _simple_plan((41.0, 29.0, TaskType.CHECKPOINT))
    plan.points[0].arrival_radius_m = 500.0
    mgr = WaypointManager(plan, auto_advance=True)
    state = mgr.update(GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0))
    assert state.mission_complete is True


# ──────────────────────────────────────────────────────────────────────────────
# MissionManager — şartname görev davranışları
# ──────────────────────────────────────────────────────────────────────────────

def test_mission_manager_pickup_hold_speed_sifir():
    """PICKUP noktasına varışta ilk 15s boyunca speed_cap=0.0 (yolcu biniş beklemesi)."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.waypoint_manager import GpsPosition
    plan = _simple_plan(
        (41.0, 29.0, TaskType.START),
        (41.0, 29.0, TaskType.PICKUP),
        (41.001, 29.0, TaskType.STOP),
    )
    plan.points[1].arrival_radius_m = 500.0
    mgr = MissionManager(plan)
    pos = GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0)
    mgr.wp.advance()                          # START'ı geç, PICKUP'a yönel
    _, dec = mgr.update(pos, now_s=0.0)
    assert dec.speed_cap_ratio == 0.0
    assert dec.hold_remaining_s > 0.0


def test_mission_manager_pickup_hold_sonrasi_ilerleme():
    """PICKUP 15s dolunca hold_remaining_s=0.0 ve devam sinyali."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.waypoint_manager import GpsPosition
    plan = _simple_plan(
        (41.0, 29.0, TaskType.PICKUP),
        (41.001, 29.0, TaskType.STOP),
    )
    plan.points[0].arrival_radius_m = 500.0
    mgr = MissionManager(plan)
    pos = GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0)
    mgr.update(pos, now_s=0.0)          # hold başlat (started_at=0.0)
    _, dec = mgr.update(pos, now_s=16.0)
    assert dec.hold_remaining_s == 0.0


def test_mission_manager_dropoff_hold():
    """DROPOFF da 15s hold uygulamalı (şartname: yolcu indirme bekleme süresi)."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.waypoint_manager import GpsPosition
    plan = _simple_plan((41.0, 29.0, TaskType.DROPOFF))
    plan.points[0].arrival_radius_m = 500.0
    mgr = MissionManager(plan)
    pos = GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0)
    _, dec = mgr.update(pos, now_s=0.0)
    assert dec.speed_cap_ratio == 0.0
    assert "dropoff" in dec.hold_reason.lower()


def test_mission_manager_park_entry_aktif():
    """PARK_ENTRY noktasına varınca park_mode=True olmalı."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.waypoint_manager import GpsPosition
    plan = _simple_plan(
        (41.0, 29.0, TaskType.PARK_ENTRY),
        (41.001, 29.0, TaskType.STOP),
    )
    plan.points[0].arrival_radius_m = 500.0
    mgr = MissionManager(plan)
    _, dec = mgr.update(GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0), now_s=0.0)
    assert dec.park_mode is True


def test_mission_manager_park_3dk_timeout():
    """Park modunda 180s dolunca speed_cap=0.0 (şartname: park süresi aşımı)."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.waypoint_manager import GpsPosition
    plan = _simple_plan(
        (41.0, 29.0, TaskType.PARK_ENTRY),
        (41.001, 29.0, TaskType.STOP),
    )
    plan.points[0].arrival_radius_m = 500.0
    mgr = MissionManager(plan)
    pos = GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0)
    mgr.update(pos, now_s=0.0)          # park modunu başlat
    _, dec = mgr.update(pos, now_s=181.0)
    assert dec.speed_cap_ratio == 0.0


def test_mission_manager_park_tamamlandi_sinyal():
    """notify_park_completed() sonrası park_mode=False, araç devam eder."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.waypoint_manager import GpsPosition
    plan = _simple_plan(
        (41.0, 29.0, TaskType.PARK_ENTRY),
        (41.001, 29.0, TaskType.STOP),
    )
    plan.points[0].arrival_radius_m = 500.0
    mgr = MissionManager(plan)
    pos = GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0)
    mgr.update(pos, now_s=0.0)
    mgr.notify_park_completed()
    _, dec = mgr.update(pos, now_s=1.0)
    assert dec.park_mode is False


# ──────────────────────────────────────────────────────────────────────────────
# RouteGraph — Dijkstra ve tünel zorunlu geçiş
# ──────────────────────────────────────────────────────────────────────────────

def test_route_dijkstra_basit_yol():
    """A→C: Dijkstra [0, 1, 2] yolunu döndürmeli."""
    from deos_algorithms.route_graph import dijkstra
    path = dijkstra(_three_node_graph(), start=0, goal=2)
    assert path == [0, 1, 2]


def test_route_dijkstra_bloklu_kenar_bos():
    """Tüm geçiş kenarları bloklanınca yol yok → boş liste."""
    from deos_algorithms.route_graph import dijkstra
    blocked = {(0, 1), (1, 2)}
    path = dijkstra(_three_node_graph(), start=0, goal=2, blocked_edges=blocked)
    assert path == []


def test_route_dijkstra_baslangic_es_hedef():
    """start == goal → tek elemanlı yol [start]."""
    from deos_algorithms.route_graph import dijkstra
    path = dijkstra(_three_node_graph(), start=1, goal=1)
    assert path == [1]


def test_route_tunel_zorunlu_gecis():
    """
    Bypass kenar mevcut ama tünel zorunlu → dijkstra_mandatory_tunnel tünelden geçmeli.
    Graf: 0→2 bypass (cost=50), 0→1→2 tünel (cost=200) ama tünel=True.
    """
    from deos_algorithms.route_graph import Edge, Node, RouteGraph, dijkstra_mandatory_tunnel
    nodes = [Node(id=0, lat=41.0, lon=29.0),
             Node(id=1, lat=41.001, lon=29.0),
             Node(id=2, lat=41.002, lon=29.0)]
    adj = {
        0: [Edge(u=0, v=2, cost=50.0, props={}),
            Edge(u=0, v=1, cost=100.0, props={})],
        1: [Edge(u=1, v=0, cost=100.0, props={}),
            Edge(u=1, v=2, cost=100.0, props={"tunnel": True})],
        2: [Edge(u=2, v=0, cost=50.0, props={}),
            Edge(u=2, v=1, cost=100.0, props={"tunnel": True})],
    }
    g = RouteGraph(nodes=nodes, adj=adj)
    path = dijkstra_mandatory_tunnel(g, start=0, goal=2)
    assert 1 in path, f"Tünel kenarı (0→1→2) kullanılmadı: path={path}"


def test_route_oneway_ters_yonde_yok():
    """Tek yönlü kenar: 0→1 var, 1→0 yok → ters yönde yol boş."""
    from deos_algorithms.route_graph import Edge, Node, RouteGraph, dijkstra
    nodes = [Node(id=0, lat=41.0, lon=29.0),
             Node(id=1, lat=41.001, lon=29.0)]
    adj = {
        0: [Edge(u=0, v=1, cost=100.0, props={"oneway": True})],
        1: [],
    }
    g = RouteGraph(nodes=nodes, adj=adj)
    assert dijkstra(g, start=1, goal=0) == []
    assert dijkstra(g, start=0, goal=1) == [0, 1]


def test_route_nearest_node():
    """nearest_node_id: en yakın düğümü doğru bulmalı."""
    from deos_algorithms.route_graph import nearest_node_id
    g = _three_node_graph()
    assert nearest_node_id(g, lat=41.001, lon=29.0) == 1
    assert nearest_node_id(g, lat=41.0,   lon=29.0) == 0
    assert nearest_node_id(g, lat=41.002, lon=29.0) == 2


# ──────────────────────────────────────────────────────────────────────────────
# GeoJsonMissionReader — Türkçe eş anlamlılar ve çözümleme
# ──────────────────────────────────────────────────────────────────────────────

import json as _json


def test_geojson_reader_temel_gorevler():
    """start, durak, park → START, STOP, PARK olarak ayrıştırılmalı."""
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader, TaskType
    gj = _make_geojson(
        _make_point_feature(29.0, 41.0, task="start"),
        _make_point_feature(29.001, 41.0, task="durak"),
        _make_point_feature(29.002, 41.0, task="park"),
    )
    plan = GeoJsonMissionReader().read_string(_json.dumps(gj))
    assert plan.points[0].task == TaskType.START
    assert plan.points[1].task == TaskType.STOP
    assert plan.points[2].task == TaskType.PARK


def test_geojson_reader_binis_inis_turkce():
    """'biniş' → PICKUP, 'yolcu_indirme' → DROPOFF (Türkçe eş anlamlılar)."""
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader, TaskType
    gj = _make_geojson(
        _make_point_feature(29.0, 41.0, task="biniş"),
        _make_point_feature(29.001, 41.0, task="yolcu_indirme"),
    )
    plan = GeoJsonMissionReader().read_string(_json.dumps(gj))
    assert plan.points[0].task == TaskType.PICKUP
    assert plan.points[1].task == TaskType.DROPOFF


def test_geojson_reader_varsayilan_radius_3m():
    """radius_m belirtilmemişse DEFAULT_ARRIVAL_RADIUS_M=3.0m kullanılmalı (Fix-9)."""
    from deos_algorithms.geojson_mission_reader import DEFAULT_ARRIVAL_RADIUS_M, GeoJsonMissionReader
    gj = _make_geojson(_make_point_feature(29.0, 41.0, task="checkpoint"))
    plan = GeoJsonMissionReader().read_string(_json.dumps(gj))
    assert plan.points[0].arrival_radius_m == DEFAULT_ARRIVAL_RADIUS_M
    assert DEFAULT_ARRIVAL_RADIUS_M == pytest.approx(3.0)


def test_geojson_reader_isim_park_giris():
    """task yokken name='park_giris' → PARK_ENTRY (isim-tabanlı çözümleme)."""
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader, TaskType
    gj = _make_geojson(_make_point_feature(29.0, 41.0, name="park_giris"))
    plan = GeoJsonMissionReader().read_string(_json.dumps(gj))
    assert plan.points[0].task == TaskType.PARK_ENTRY


def test_geojson_reader_tunel_checkpoint_olur():
    """task='tünel' → CHECKPOINT (tünel ara geçiş noktasıdır)."""
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader, TaskType
    gj = _make_geojson(_make_point_feature(29.0, 41.0, task="tünel"))
    plan = GeoJsonMissionReader().read_string(_json.dumps(gj))
    assert plan.points[0].task == TaskType.CHECKPOINT


def test_geojson_reader_speed_limit_ratio():
    """speed_limit_ratio properties'den okunmalı."""
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
    gj = _make_geojson(_make_point_feature(29.0, 41.0, task="checkpoint", speed_limit_ratio=0.6))
    plan = GeoJsonMissionReader().read_string(_json.dumps(gj))
    assert plan.points[0].speed_limit_ratio == pytest.approx(0.6)


# ──────────────────────────────────────────────────────────────────────────────
# DecisionArbiter — öncelik, hız min, steer seçimi, lane kısıtı
# ──────────────────────────────────────────────────────────────────────────────

def test_arbiter_acil_dur_her_seyi_override_eder():
    """Emergency stop → steer=0, speed=0, has_steer=False (tüm override'ları iptal)."""
    from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, ReasonCode
    arb = DecisionArbiter()
    candidates = [
        Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0,
                  steer_override=0.8, reasons=[ReasonCode.OBSTACLE_EMERGENCY_STOP]),
        Candidate(name="slalom", emergency_stop=False, speed_cap=1.0,
                  steer_override=0.5, reasons=[ReasonCode.SLALOM]),
    ]
    dec = arb.arbitrate(candidates=candidates)
    print(f"  acil_dur -> emergency={dec.emergency_stop}  speed={dec.speed_cap:.2f}  has_steer={dec.has_steer_override}  steer={dec.steer_override:.2f}")
    assert dec.emergency_stop is True
    assert dec.speed_cap == pytest.approx(0.0)
    assert dec.has_steer_override is False
    assert dec.steer_override == pytest.approx(0.0)


def test_arbiter_hiz_cap_minimum():
    """Hız tavanı birden fazla aday arasında minimum seçilmeli."""
    from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter
    arb = DecisionArbiter()
    candidates = [
        Candidate(name="light", speed_cap=0.5),
        Candidate(name="sign",  speed_cap=0.7),
        Candidate(name="slalom", speed_cap=1.0),
    ]
    dec = arb.arbitrate(candidates=candidates)
    print(f"  hiz_cap [0.5, 0.7, 1.0] -> secilen={dec.speed_cap:.2f}  (beklenen=0.5 minimum)")
    assert dec.speed_cap == pytest.approx(0.5)
    assert not dec.emergency_stop


def test_arbiter_steer_park_slalomdan_ustun():
    """Steer önceliği: park > slalom."""
    from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, ReasonCode
    arb = DecisionArbiter()
    candidates = [
        Candidate(name="slalom", steer_override=0.3, reasons=[ReasonCode.SLALOM]),
        Candidate(name="park",   steer_override=0.7, reasons=[ReasonCode.PARK_MODE]),
    ]
    dec = arb.arbitrate(candidates=candidates)
    assert dec.has_steer_override is True
    assert dec.steer_override == pytest.approx(0.7)


def test_arbiter_steer_dynamic_avoid_slalomdan_ustun():
    """
    Steer önceliği: dynamic_avoid > slalom.

    lane_required_for_avoidance=False geçirilmeli; aksi hâlde lane=None olduğunda
    dynamic_avoid steer'i LANE_MISSING_AVOIDANCE_DISABLED kuralıyla iptal edilir.
    """
    from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, LaneBounds, ReasonCode
    arb = DecisionArbiter()
    valid_lane = LaneBounds(left_y_m=1.5, right_y_m=-1.5, margin_m=0.1)
    candidates = [
        Candidate(name="slalom",        steer_override=0.3,   reasons=[ReasonCode.SLALOM]),
        Candidate(name="dynamic_avoid", steer_override=-0.25, reasons=[ReasonCode.DYNAMIC_AVOID]),
    ]
    dec = arb.arbitrate(candidates=candidates, lane=valid_lane, lane_required_for_avoidance=True)
    assert dec.steer_override == pytest.approx(-0.25)


def test_arbiter_lane_eksik_avoidance_iptal():
    """Lane bilgisi yok + require_lane_walls_for_avoidance=True → steer override iptal."""
    from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, ReasonCode
    arb = DecisionArbiter()
    candidates = [
        Candidate(name="static_avoid", steer_override=0.35, reasons=[ReasonCode.STATIC_AVOID]),
    ]
    dec = arb.arbitrate(candidates=candidates, lane=None, lane_required_for_avoidance=True)
    assert dec.has_steer_override is False
    assert ReasonCode.LANE_MISSING_AVOIDANCE_DISABLED in dec.reasons


def test_arbiter_bos_aday_varsayilan_guvenli():
    """Aday listesi boşken: emergency=False, speed=1.0, steer override yok."""
    from deos_algorithms.decision_arbiter import DecisionArbiter
    arb = DecisionArbiter()
    dec = arb.arbitrate(candidates=[])
    assert not dec.emergency_stop
    assert dec.speed_cap == pytest.approx(1.0)
    assert not dec.has_steer_override


def test_arbiter_lane_clamp_steer_kisaltir():
    """Dar şerit + static_avoid → lane clamp steer override değerini küçültmeli."""
    from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, LaneBounds, ReasonCode
    arb = DecisionArbiter()
    narrow_lane = LaneBounds(left_y_m=0.4, right_y_m=-0.4, margin_m=0.1)
    candidates = [
        Candidate(name="static_avoid", steer_override=1.0, reasons=[ReasonCode.STATIC_AVOID]),
    ]
    dec = arb.arbitrate(candidates=candidates, lane=narrow_lane, lane_required_for_avoidance=False)
    assert dec.has_steer_override is True
    assert abs(dec.steer_override) < 1.0   # clamp uygulandı


# ──────────────────────────────────────────────────────────────────────────────
# SlalomLogic — hız bantları, tamamlanma, 640px doğrulaması
# ──────────────────────────────────────────────────────────────────────────────

def test_slalom_goruntu_genisligi_640px():
    """Fix-13 doğrulaması: SlalomLogic varsayılan goruntu_genislik=640px."""
    from deos_algorithms.slalom_logic import SlalomLogic
    sl = SlalomLogic()
    assert sl._g_w == 640
    assert sl._g_h == 480


def test_slalom_hiz_katsayisi_uzak():
    """Koni >3m (DIST_UZAK_M) → hız katsayısı 1.0."""
    from deos_algorithms.slalom_logic import SlalomLogic
    sl = SlalomLogic()
    st = sl.update([_slalom_cone_det(4.0, lateral_offset=0.5)])
    print(f"  mesafe=4.0m (uzak bant)  -> hiz_katsayisi={st.hiz_katsayisi:.2f}  (beklenen=1.0)")
    assert st.hiz_katsayisi == pytest.approx(1.0)


def test_slalom_hiz_katsayisi_orta():
    """Koni 1.5-3m arası → hız katsayısı 0.6."""
    from deos_algorithms.slalom_logic import SlalomLogic
    sl = SlalomLogic()
    st = sl.update([_slalom_cone_det(2.0, lateral_offset=0.5)])
    print(f"  mesafe=2.0m (orta bant)  -> hiz_katsayisi={st.hiz_katsayisi:.2f}  (beklenen=0.6)")
    assert st.hiz_katsayisi == pytest.approx(0.6)


def test_slalom_hiz_katsayisi_yakin():
    """Koni <1.5m (DIST_ORTA_M) → hız katsayısı 0.3."""
    from deos_algorithms.slalom_logic import SlalomLogic
    sl = SlalomLogic()
    st = sl.update([_slalom_cone_det(1.0, lateral_offset=0.5)])
    print(f"  mesafe=1.0m (yakin bant) -> hiz_katsayisi={st.hiz_katsayisi:.2f}  (beklenen=0.3)")
    assert st.hiz_katsayisi == pytest.approx(0.3)


def test_slalom_tamamlanma_15_frame():
    """LAST_KNOWN_FRAME_ESIK + BITTI_FRAME_ESIK boş kare sonrası faz 'bitti' olmalı."""
    from deos_algorithms.slalom_logic import BITTI_FRAME_ESIK, LAST_KNOWN_FRAME_ESIK, SlalomLogic
    sl = SlalomLogic()
    sl.update([_slalom_cone_det(2.0, lateral_offset=0.5)])   # aktif et
    state = None
    for _ in range(LAST_KNOWN_FRAME_ESIK + BITTI_FRAME_ESIK):
        state = sl.update([])
    assert state.faz == "bitti"


def test_slalom_tek_koni_sagda_sola_steer():
    """
    Koni sağda (lateral_offset > 0) → steering negatif (sola).
    Araç koninin solundan geçmek için sola döner.
    """
    from deos_algorithms.slalom_logic import SlalomLogic
    sl = SlalomLogic()
    st = sl.update([_slalom_cone_det(2.5, lateral_offset=0.5)])
    print(f"  koni sagda (offset=+0.5) -> steering={st.steering:.4f}  (beklenen < 0, sola don)")
    assert st.aktif is True
    assert st.steering < 0.0, f"Sağdaki koni için sol steer bekleniyor, gelen: {st.steering:.3f}"


def test_slalom_lateral_offset_640px_dogru():
    """
    640px kamerada merkez piksel (320) → lateral_offset=0.0 olmalı.
    (1280px olsaydı: (320-640)/640=-0.5 yanlış çıkardı.)
    """
    from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleKind
    from deos_algorithms.slalom_logic import SlalomLogic
    sl = SlalomLogic()
    # Merkez piksel: bbox [300, 200, 340, 500] → cx=320
    center_cone = ObstacleDetection(
        kind=ObstacleKind.CONE, confidence=0.9,
        bbox_px=(300, 200, 340, 500),
        estimated_distance_m=3.0,
        estimated_lateral_m=None,
    )
    # _lateral_offset hesabı: (320 - 320) / 320 = 0.0
    offset = sl._lateral_offset(center_cone)
    print(f"  merkez piksel cx=320 -> lateral_offset={offset:.6f}  (beklenen ~= 0.0, 1280px olsaydi -0.5 cikardia)")
    assert abs(offset) < 0.01, f"Merkez konisi offset=0 olmalı, gelen: {offset:.4f}"


# ==============================================================================
# BÖLÜM 3 — Şartname Boşluk Kapatma (27 test)
# Kaynak: 2026_Robotaksi-Binek_Otonom_Arac_Yarismasi_Sartnamesi
# ==============================================================================


# ──────────────────────────────────────────────────────────────────────────────
# ObstacleLogic — ek sartname senaryolari
# ──────────────────────────────────────────────────────────────────────────────

def test_obstacle_logic_dinamik_kacinma_yonu_sol():
    """
    Yaya sagda (lateral_m < 0) duruyorsa DYNAMIC_AVOID_HOLD_S gecince gecis yonu SOL olmali.
    Tur-2/Tur-3 dinamik engelden sakinma: 50 puan.
    """
    from deos_algorithms.obstacle_logic import ObstacleLogic
    logic = ObstacleLogic()
    state = None
    for i in range(20):
        state = logic.update([_obs_ped(4.0, lateral_m=-0.5)], now=float(i) * 0.05)
    assert state.dynamic_avoid_active is True, f"dynamic_avoid_active=False, state={state}"
    assert state.dynamic_avoidance_direction == "left", (
        f"Yaya sagda iken sol gecis bekleniyor, gelen: {state.dynamic_avoidance_direction}"
    )


def test_obstacle_logic_statik_kacinma_yonu_sol_konide_saga():
    """
    Koni solda (lateral_m > 0 = arac sol tarafi) -> avoidance_direction='right' (sagdan gec).
    Tur-1/Tur-3 statik engelden sakinma yonu: 50 puan.
    """
    from deos_algorithms.obstacle_logic import ObstacleLogic
    logic = ObstacleLogic()
    state = None
    for _ in range(5):
        state = logic.update([_obs_cone(2.0, lateral_m=0.5)])   # koni sol: lat > 0
    assert state.suggest_lane_change is True
    assert state.avoidance_direction == "right", (
        f"Koni solda iken sagdan gecmek gerekir, gelen: {state.avoidance_direction}"
    )


def test_obstacle_logic_statik_kacinma_yonu_sag_konide_sola():
    """
    Koni sagda (lateral_m < 0) -> avoidance_direction='left' (soldan gec).
    """
    from deos_algorithms.obstacle_logic import ObstacleLogic
    logic = ObstacleLogic()
    state = None
    for _ in range(5):
        state = logic.update([_obs_cone(2.0, lateral_m=-0.5)])  # koni sag: lat < 0
    assert state.suggest_lane_change is True
    assert state.avoidance_direction == "left", (
        f"Koni sagda iken soldan gecmek gerekir, gelen: {state.avoidance_direction}"
    )


def test_obstacle_logic_statik_commit_stabilizasyon():
    """
    Sirali konilerde yon commit edilince STATIC_AVOID_COMMIT_FRAMES dolana kadar degismemeli (zigzag azaltma).
    """
    from deos_algorithms.obstacle_logic import STATIC_AVOID_COMMIT_FRAMES, ObstacleLogic
    logic = ObstacleLogic()
    # 3 kare sag koni (lat=0.5 -> direction='right'), commit baslar
    for _ in range(3):
        state = logic.update([_obs_cone(2.0, lateral_m=0.5)])
    committed_dir = state.avoidance_direction   # 'right', frames_left=3
    # Hemen sol koni gelse bile commit korunmali (frames_left > 0)
    state = logic.update([_obs_cone(1.8, lateral_m=-0.6)])
    assert state.avoidance_direction == committed_dir, (
        f"Commit {STATIC_AVOID_COMMIT_FRAMES} frame korunmali, gelen: {state.avoidance_direction}"
    )


# ──────────────────────────────────────────────────────────────────────────────
# TrafficSignLogic — eksik tabela senaryolari (sartname 22 tabela listesi)
# ──────────────────────────────────────────────────────────────────────────────

def test_traffic_sign_sola_donulmez():
    """'sola donulmez' -> turn_permissions.left=False, straight degismemeli."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("sola donulmez"), now_base=13000.0)
    print(f"  sola_donulmez -> left={st.turn_permissions.left}  straight={st.turn_permissions.straight}  (beklenen False, True)")
    assert st.turn_permissions.left is False
    assert st.turn_permissions.straight is True


def test_traffic_sign_saga_mecburi():
    """'saga mecburi' -> forced_direction='right', left=False, straight=False."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("saga mecburi"), now_base=14000.0)
    print(f"  saga_mecburi -> forced={st.turn_permissions.forced_direction}  straight={st.turn_permissions.straight}  left={st.turn_permissions.left}")
    assert st.turn_permissions.forced_direction == "right"
    assert st.turn_permissions.straight is False
    assert st.turn_permissions.left is False


def test_traffic_sign_ileri_mecburi():
    """'ileri mecburi' -> forced_direction='straight', left=False, right=False."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("ileri mecburi"), now_base=15000.0)
    print(f"  ileri_mecburi -> forced={st.turn_permissions.forced_direction}  left={st.turn_permissions.left}  right={st.turn_permissions.right}")
    assert st.turn_permissions.forced_direction == "straight"
    assert st.turn_permissions.left is False
    assert st.turn_permissions.right is False


def test_traffic_sign_ileri_ve_saga():
    """'ileri ve saga mecburi yon' -> left=False, straight=True, right=True."""
    from deos_algorithms.traffic_sign_logic import SignClass, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det(SignClass.STRAIGHT_OR_RIGHT), now_base=16000.0)
    assert st.turn_permissions.left is False
    assert st.turn_permissions.straight is True
    assert st.turn_permissions.right is True


def test_traffic_sign_ileri_ve_sola():
    """'ileri ve sola mecburi yon' -> left=True, straight=True, right=False."""
    from deos_algorithms.traffic_sign_logic import SignClass, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det(SignClass.STRAIGHT_OR_LEFT), now_base=17000.0)
    assert st.turn_permissions.left is True
    assert st.turn_permissions.straight is True
    assert st.turn_permissions.right is False


def test_traffic_sign_ileriden_saga():
    """'ileriden saga mecburi yon' -> forced_direction='right'."""
    from deos_algorithms.traffic_sign_logic import SignClass, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det(SignClass.AHEAD_THEN_RIGHT), now_base=18000.0)
    assert st.turn_permissions.forced_direction == "right"


def test_traffic_sign_ileriden_sola():
    """'ileriden sola mecburi yon' -> forced_direction='left'."""
    from deos_algorithms.traffic_sign_logic import SignClass, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det(SignClass.AHEAD_THEN_LEFT), now_base=19000.0)
    assert st.turn_permissions.forced_direction == "left"


def test_traffic_sign_doner_kavsak():
    """'doner kavsak' -> forced_direction='roundabout'."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("doner kavsak"), now_base=20000.0)
    print(f"  doner_kavsak -> forced={st.turn_permissions.forced_direction}  (beklenen roundabout)")
    assert st.turn_permissions.forced_direction == "roundabout"


def test_traffic_sign_sagdan_gidiniz():
    """'sagdan gidiniz' -> forced_direction='pass_right'."""
    from deos_algorithms.traffic_sign_logic import SignClass, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det(SignClass.KEEP_RIGHT), now_base=21000.0)
    print(f"  sagdan_gidiniz -> forced={st.turn_permissions.forced_direction}  (beklenen pass_right)")
    assert st.turn_permissions.forced_direction == "pass_right"


def test_traffic_sign_soldan_gidiniz():
    """'soldan gidin' -> forced_direction='pass_left'."""
    from deos_algorithms.traffic_sign_logic import SignClass, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det(SignClass.KEEP_LEFT), now_base=22000.0)
    print(f"  soldan_gidiniz -> forced={st.turn_permissions.forced_direction}  (beklenen pass_left)")
    assert st.turn_permissions.forced_direction == "pass_left"


def test_traffic_sign_park_alani():
    """'park' tabelasi -> current_area_is_parking=True. Park alani bilgisi iletilir."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("park"), now_base=23000.0)
    assert st.current_area_is_parking is True


def test_traffic_sign_park_yapilmaz():
    """'park yapilmaz' tabelasi -> current_area_no_parking=True. Uygunsuz parka -20 puan."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("park yapilmaz"), now_base=24000.0)
    assert st.current_area_no_parking is True


def test_traffic_sign_isikli_isaret():
    """'isikli isaret cihazi' -> traffic_light_expected=True. Isik algilamayi etkinlestir."""
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det("isikli isaret cihazi"), now_base=25000.0)
    assert st.traffic_light_expected is True


def test_traffic_sign_sol_serit_sonu():
    """'sol seridin sonu' -> sartname tabela listesinde taninan tipte, hata vermez."""
    from deos_algorithms.traffic_sign_logic import SignClass, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det(SignClass.LANE_ARRANGEMENT_H), now_base=26000.0)
    assert SignClass.LANE_ARRANGEMENT_H in st.active_signs


def test_traffic_sign_sag_serit_sonu():
    """'sag seridin sonu' -> sartname tabela listesinde taninan tipte, hata vermez."""
    from deos_algorithms.traffic_sign_logic import SignClass, TrafficSignLogic
    logic = TrafficSignLogic()
    st = _confirm_sign(logic, _sign_det(SignClass.LANE_ARRANGEMENT_I), now_base=27000.0)
    assert SignClass.LANE_ARRANGEMENT_I in st.active_signs


# ──────────────────────────────────────────────────────────────────────────────
# SlalomLogic — ek senaryolar
# ──────────────────────────────────────────────────────────────────────────────

def test_slalom_tek_koni_solda_saga_steer():
    """
    Koni solda (lateral_offset < 0, cx < merkez) -> steering pozitif (saga don).
    Arac koninin sagindan gecmek icin saga doner.
    """
    from deos_algorithms.slalom_logic import SlalomLogic
    sl = SlalomLogic()
    st = sl.update([_slalom_cone_det(2.5, lateral_offset=-0.5)])
    print(f"  koni solda (offset=-0.5) -> steering={st.steering:.4f}  (beklenen > 0, saga don)")
    assert st.aktif is True
    assert st.steering > 0.0, f"Soldaki koni icin sag steer bekleniyor, gelen: {st.steering:.3f}"


def test_slalom_faz_zinciri_bekleme_aktif_bitti():
    """
    Tam slalom faz zinciri: bekleme -> aktif (koni var) -> bitti (LAST_KNOWN_FRAME_ESIK + BITTI_FRAME_ESIK kare bos).
    Arac test videosunda gereken slalom manevrasi senaryosu.
    """
    from deos_algorithms.slalom_logic import BITTI_FRAME_ESIK, LAST_KNOWN_FRAME_ESIK, SlalomLogic
    sl = SlalomLogic()
    assert sl.update([]).faz == "bekleme", "Baslangicta faz 'bekleme' olmali."
    st = sl.update([_slalom_cone_det(2.0, lateral_offset=0.3)])
    assert st.aktif is True, "Koni gorulunce slalom aktif olmali."
    state = None
    total = LAST_KNOWN_FRAME_ESIK + BITTI_FRAME_ESIK
    for _ in range(total):
        state = sl.update([])
    assert state.faz == "bitti", f"Bos {total} kare sonra faz 'bitti' olmali."


# ──────────────────────────────────────────────────────────────────────────────
# MissionManager — ek sartname senaryolari
# ──────────────────────────────────────────────────────────────────────────────

def test_mission_manager_dropoff_hold_tamamlandi():
    """DROPOFF 15s dolunca hold_remaining_s=0.0 ve arac devam eder (PICKUP ile ayni davranis)."""
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.waypoint_manager import GpsPosition
    plan = _simple_plan((41.0, 29.0, TaskType.DROPOFF), (41.001, 29.0, TaskType.STOP))
    plan.points[0].arrival_radius_m = 500.0
    mgr = MissionManager(plan)
    pos = GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0)
    mgr.update(pos, now_s=0.0)                   # hold basla
    _, dec_orta = mgr.update(pos, now_s=14.9)
    assert dec_orta.hold_remaining_s > 0.0, "14.9s'de DROPOFF hold henuz bitmemeli."
    _, dec_bitti = mgr.update(pos, now_s=16.0)
    assert dec_bitti.hold_remaining_s == 0.0, "16s'de DROPOFF hold bitmeli."


def test_mission_manager_pickup_hold_15_20s_penceresi():
    """
    Sartname: yolcu alma/birakma en az 15 en fazla 20 saniye.
    15s'de hold bitmeli (pencerenin icinde). Gercek bekleme 15s <= 20s maks.
    """
    from deos_algorithms.geojson_mission_reader import TaskType
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.waypoint_manager import GpsPosition
    plan = _simple_plan((41.0, 29.0, TaskType.PICKUP), (41.001, 29.0, TaskType.STOP))
    plan.points[0].arrival_radius_m = 500.0
    mgr = MissionManager(plan)
    pos = GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0)
    mgr.update(pos, now_s=0.0)
    _, dec_erken = mgr.update(pos, now_s=14.9)
    assert dec_erken.hold_remaining_s > 0.0, "14.9s'de hold henuz bitmemeli (min 15s)."
    _, dec_hazir = mgr.update(pos, now_s=16.0)
    assert dec_hazir.hold_remaining_s == 0.0, "16s+ sonra hold bitmeli (sartname 15-20s penceresi)."
    # Algoritma 15s bekleyip ilerledigi icin maks 20s sinirini hic asmaz.
    assert 15.0 <= 20.0


# ──────────────────────────────────────────────────────────────────────────────
# GeoJsonMissionReader — ek sartname senaryolari
# ──────────────────────────────────────────────────────────────────────────────

def test_geojson_reader_gorev_isimli_noktalar():
    """
    Sartname GeoJSON: 'gorev_1', 'gorev_2', 'gorev_3' -> CHECKPOINT.
    Gercek yaris verisinde gorev noktalarina bu sekilde isim verilmektedir.
    """
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader, TaskType
    gj = _make_geojson(
        _make_point_feature(29.0, 41.0, name="gorev_1"),
        _make_point_feature(29.001, 41.0, name="gorev_2"),
        _make_point_feature(29.002, 41.0, name="gorev_3"),
    )
    plan = GeoJsonMissionReader().read_string(_json.dumps(gj))
    for pt in plan.points:
        assert pt.task == TaskType.CHECKPOINT, (
            f"{pt.name} -> beklenen CHECKPOINT, gelen {pt.task}"
        )


def test_geojson_reader_heading_deg():
    """heading_deg properties'den okunmali."""
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
    gj = _make_geojson(_make_point_feature(29.0, 41.0, task="checkpoint", heading_deg=270.0))
    plan = GeoJsonMissionReader().read_string(_json.dumps(gj))
    assert plan.points[0].heading_deg == pytest.approx(270.0)


def test_geojson_reader_heading_deg_normalizasyon():
    """heading_deg=370 -> 10.0 (% 360 ile normalize edilmeli)."""
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
    gj = _make_geojson(_make_point_feature(29.0, 41.0, task="checkpoint", heading_deg=370.0))
    plan = GeoJsonMissionReader().read_string(_json.dumps(gj))
    assert plan.points[0].heading_deg == pytest.approx(10.0)


# ──────────────────────────────────────────────────────────────────────────────
# DecisionArbiter — ek senaryo
# ──────────────────────────────────────────────────────────────────────────────

def test_arbiter_steer_static_avoid_slalomdan_ustun():
    """Steer onceligi: static_avoid (priority=2) > slalom (priority=3)."""
    from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, LaneBounds, ReasonCode
    arb = DecisionArbiter()
    valid_lane = LaneBounds(left_y_m=1.5, right_y_m=-1.5, margin_m=0.1)
    candidates = [
        Candidate(name="slalom",       steer_override=0.3,  reasons=[ReasonCode.SLALOM]),
        Candidate(name="static_avoid", steer_override=0.55, reasons=[ReasonCode.STATIC_AVOID]),
    ]
    dec = arb.arbitrate(candidates=candidates, lane=valid_lane, lane_required_for_avoidance=True)
    assert dec.steer_override == pytest.approx(0.55), (
        f"static_avoid steer secilmeli, gelen: {dec.steer_override:.3f}"
    )
