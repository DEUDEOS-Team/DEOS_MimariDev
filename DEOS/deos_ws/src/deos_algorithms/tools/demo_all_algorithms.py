#!/usr/bin/env python3
"""
Terminal demo / mini doğrulama (mimariyi bozmadan):
- `deos_algorithms` içindeki modüller için küçük senaryolar çalıştırır
- her senaryoda **beklenen** ve **çıkan** alanları kısa bir özetle yazar

Bu dosya `pytest` değildir; hızlı “gözle kontrol” içindir.
Birim testler için: `python -m pytest -vv`

Çalıştırma:
  cd DEOS/deos_ws/src/deos_algorithms
  python tools/demo_all_algorithms.py
"""

from __future__ import annotations

import json
import sys
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable

# Allow running without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader, TaskType  # noqa: E402
from deos_algorithms.mission_manager import MissionManager  # noqa: E402
from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleLogic  # noqa: E402
from deos_algorithms.parking_logic import ParkingDetection, ParkingLogic, ParkPhase  # noqa: E402
from deos_algorithms.perception_fusion import fuse  # noqa: E402
from deos_algorithms.safety_logic import Detection, SafetyLogic, ThreatLevel  # noqa: E402
from deos_algorithms.sensors.types import ImuSample, LidarObstacle, StereoBbox  # noqa: E402
from deos_algorithms.slalom_logic import SlalomLogic  # noqa: E402
from deos_algorithms.traffic_light_logic import (  # noqa: E402
    LightColor,
    LightDetection,
    TrafficLightLogic,
    TrafficLightState,
)
from deos_algorithms.traffic_sign_logic import SignClass, SignDetection, TrafficSignLogic  # noqa: E402
from deos_algorithms.waypoint_manager import GpsPosition, WaypointManager  # noqa: E402


def _ensure_utf8_stdio() -> None:
    """Windows konsolda Türkçe karakterlerin bozulmaması için (algoritma davranışını değiştirmez)."""
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if stream is not None and hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass


def _normalize(obj: Any) -> Any:
    if isinstance(obj, Enum):
        return obj.name
    if is_dataclass(obj):
        return {k: _normalize(v) for k, v in asdict(obj).items()}
    if isinstance(obj, dict):
        return {k: _normalize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_normalize(v) for v in obj]
    return obj


def _to_dict(obj: Any) -> Any:
    return _normalize(obj)


def _pick(d: dict[str, Any], keys: list[str]) -> dict[str, Any]:
    return {k: d.get(k) for k in keys}


def _cmp(expected: dict[str, Any], actual: dict[str, Any]) -> tuple[bool, dict[str, tuple[Any, Any]]]:
    mism: dict[str, tuple[Any, Any]] = {}
    ok = True
    for k, ev in expected.items():
        av = actual.get(k)
        if callable(ev):
            try:
                passed = bool(ev(av))
            except Exception:
                passed = False
            if not passed:
                ok = False
                mism[k] = ("<kosul>", av)
        else:
            if av != ev:
                ok = False
                mism[k] = (ev, av)
    return ok, mism


def case(
    *,
    algo: str,
    name: str,
    desc: str,
    run: Callable[[], Any],
    expect: dict[str, Any],
    show_keys: list[str],
) -> bool:
    out = run()
    out_d = _to_dict(out)
    if not isinstance(out_d, dict):
        out_d = {"value": out_d}

    actual = _pick(out_d, show_keys)
    ok, mism = _cmp(expect, actual)

    status = "GEÇTİ" if ok else "KALDI"
    print(f"\n[{status}] {algo} :: {name}")
    print(f"  senaryo  : {desc}")
    printable_expect = {k: ("<kosul>" if callable(v) else v) for k, v in expect.items()}
    print("  beklenen :", json.dumps(printable_expect, ensure_ascii=False, default=str))
    print("  cikan    :", json.dumps(actual, ensure_ascii=False, default=str))
    if mism:
        print("  farklar  :")
        for k, (ev, av) in mism.items():
            print(f"   - {k}: beklenen={ev} cikan={av}")
    return ok


def print_glossary() -> None:
    print("== Parametre sözlüğü (çıktı alanları) ==")
    print("\nSafetyLogic / ObstacleLogic")
    print(" - threat_level: tehdit seviyesi (NONE/SOFT_SLOW/HARD_SLOW/EMERGENCY)")
    print(" - emergency_stop: True ise acil dur (kontrolcü /cmd_vel'i sıfırlar)")
    print(" - speed_cap_ratio: 0..1 hız tavanı (1=serbest, 0=dur)")
    print(" - closest_obstacle_m: en yakın tehdit mesafesi (m), yoksa None")
    print(" - behavior_mode (ObstacleLogic): clear / dynamic_slow / dynamic_wait / static_avoid / emergency_stop")
    print(" - waiting_for_dynamic_obstacle: yaya vb. dinamik engel için 'bekle' modu")
    print(" - suggest_lane_change: statik engel için şerit değiştirme önerisi")
    print(" - avoidance_direction: 'left'/'right' önerisi (engelin lateral'ına göre)")
    print(" - active_kinds: koridorda görülen engel sınıfları listesi")
    print(" - reason / reasons: kararın kısa açıklaması")

    print("\nSlalomLogic")
    print(" - aktif: slalom manevrası aktif mi")
    print(" - faz: bekleme / aktif / bitti")
    print(" - steering: -1..+1 normalize direksiyon referansı (işaret yönü implementasyona bağlı)")
    print(" - hiz_katsayisi: 0..1 hız katsayısı")
    print(" - hedef_taraf: koniye göre hedef geçiş tarafı (sol/sag/kapi)")
    print(" - gecilen_koni: geçildiği varsayılan koni sayacı (yakınlık değişimine göre)")

    print("\nParkingLogic")
    print(" - phase: bekleme / yaklasma / konumlanma / manevra / park_edildi")
    print(" - speed_ratio: 0..1 hız oranı (park manevrası için düşük)")
    print(" - steering: -1..+1 hizalanma direksiyonu")
    print(" - reverse: geri vites ihtiyacı")
    print(" - distance_m: park alanına tahmini mesafe (bbox tabanlı)")
    print(" - lateral_m: park alanına yatay sapma (m)")
    print(" - complete: park tamamlandı mı")
    print(" - no_eligible_spot: izinli (park) tabelası olan aday yok")

    print("\nTrafficSignLogic")
    print(" - must_stop_soon: STOP/NO_ENTRY gibi işaretlerden doğan 'dur' zorunluluğu")
    print(" - speed_cap_ratio: tabela kaynaklı hız tavanı (örn yaya geçidi)")
    print(" - active_signs: doğrulanmış (confirmed) aktif tabelalar")
    print(" - turn_permissions: dönüş izinleri (left/straight/right + forced_direction)")

    print("\nTrafficLightLogic")
    print(" - must_stop: kırmızı -> dur")
    print(" - prepare_to_stop: sarı (yeşilden sonra / ilk sarı) -> yavaşlama")
    print(" - prepare_to_move: sarı (kırmızıdan sonra) -> harekete hazırlık")
    print(" - can_go: yeşil -> geç")
    print(" - active_color: red/yellow/green")
    print(" - last_distance_m: ışığa tahmini mesafe")

    print("\nWaypoint/Mission")
    print(" - steering_ref: waypoint'e yönelim + cross-track error'dan üretilen -1..+1 referans")
    print(" - speed_limit_ratio: görev/waypoint hız limiti (0..1)")
    print(" - arrived: waypoint varış eşiği içinde mi")
    print(" - mission_complete: tüm waypointler bitti mi")
    print(" - hold_reason / hold_remaining_s: pickup/dropoff duraklaması")

    print("\nPerception fusion")
    print(" - sign/light/obs listeleri: stereo + lidar sınıflandırması sonucu ayrıştırılmış tespit listeleri")


def main() -> None:
    total = 0
    passed = 0

    _ensure_utf8_stdio()
    print("== deos_algorithms: demo_all (Türkçe özet + mini doğrulama) ==")
    print("Not: Bazı senaryolar aynı modül örneğini ardışık çağırır; iç durum bir önceki senaryodan devam edebilir.")
    print_glossary()

    # --- SafetyLogic ---
    safety = SafetyLogic()
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

    def run_safety_frame(n: int):
        # aynı det'i n kez ver, son karar dönsün
        last = None
        for _ in range(n):
            last = safety.analyze([det]).decision
        return last

    total += 1
    passed += case(
        algo="Güvenlik (SafetyLogic)",
        name="TR: 2 kare — henüz onay yok (clear)",
        desc="Aynı yaya kutusu 2 kare üst üste geliyor; CONFIRM_FRAMES=3 olduğu için henüz 'confirmed' değil, tehdit NONE ve acil_dur=False beklenir.",
        run=lambda: run_safety_frame(2),
        expect={"threat_level": ThreatLevel.NONE.name, "emergency_stop": False},
        show_keys=["threat_level", "emergency_stop", "speed_cap_ratio", "reason"],
    )

    total += 1
    passed += case(
        algo="Güvenlik (SafetyLogic)",
        name="TR: 3 kare — onay sonrası sert yavaşlama (HARD_SLOW)",
        desc="3. kare ile tespit confirmed olur; yaya için caution katsayısı ile etkili mesafe düşer ve HARD_SLOW tetiklenir (önceki senaryo 2 kare yaptı, burada +1 kare = toplam 3).",
        run=lambda: run_safety_frame(1),  # önceki çağrı 2 frame yaptı, 1 frame daha = 3
        expect={"threat_level": ThreatLevel.HARD_SLOW.name, "speed_cap_ratio": 0.5},
        show_keys=["threat_level", "speed_cap_ratio", "reason"],
    )

    # --- ObstacleLogic ---
    obs = ObstacleLogic()

    pedestrian = ObstacleDetection(
        kind="pedestrian",
        confidence=0.95,
        bbox_px=(610, 250, 700, 700),
        estimated_distance_m=4.0,
        estimated_lateral_m=0.0,
    )

    def run_obstacle_frames(n: int, dets: list[ObstacleDetection]):
        st = None
        for _ in range(n):
            st = obs.update(dets)
        return st

    total += 1
    passed += case(
        algo="Engel (ObstacleLogic)",
        name="TR: 1 kare yaya — henüz onay yok (clear)",
        desc="ObstacleLogic, SafetyLogic doğrulama (confirm) kullandığı için tek karelik yaya tespiti davranışı değiştirmez; behavior_mode='clear' beklenir.",
        run=lambda: run_obstacle_frames(1, [pedestrian]),
        expect={"behavior_mode": "clear", "emergency_stop": False},
        show_keys=["behavior_mode", "emergency_stop", "speed_cap_ratio", "reason", "active_kinds"],
    )

    total += 1
    passed += case(
        algo="Engel (ObstacleLogic)",
        name="TR: 3 kare yaya (4m) — dinamik yavaşlama (hız tavanı düşer)",
        desc="Yaya 3 kare confirmed olduktan sonra mesafe eşiklerine göre hız tavanı düşer veya bekleme moduna girilir; burada +2 kare ile toplam 3 kare yapılır.",
        run=lambda: run_obstacle_frames(2, [pedestrian]),  # toplam 3 frame
        expect={"speed_cap_ratio": (lambda v: isinstance(v, float) and v <= 0.4)},
        show_keys=["behavior_mode", "speed_cap_ratio", "reason", "waiting_for_dynamic_obstacle"],
    )

    close_ped = ObstacleDetection(
        kind="pedestrian",
        confidence=0.95,
        bbox_px=(610, 250, 700, 700),
        estimated_distance_m=4.0,
        estimated_lateral_m=0.0,
    )
    # update threshold: DYNAMIC_STOP_DISTANCE_M=5.0 => 4m -> WAIT (speed 0)
    total += 1
    passed += case(
        algo="Engel (ObstacleLogic)",
        name="TR: Yakın yaya (<=5m) — onay sonrası bekle (dynamic_wait)",
        desc="DYNAMIC_STOP_DISTANCE_M=5.0 iken 4m yaya confirmed olunca 'dynamic_wait' ve speed_cap_ratio=0 beklenir (+3 kare ile onay tamamlanır).",
        run=lambda: run_obstacle_frames(3, [close_ped]),
        expect={"waiting_for_dynamic_obstacle": True, "speed_cap_ratio": 0.0},
        show_keys=["behavior_mode", "waiting_for_dynamic_obstacle", "speed_cap_ratio", "reason"],
    )

    # --- SlalomLogic ---
    slalom = SlalomLogic()
    cone_left = ObstacleDetection(
        kind="cone",
        confidence=0.9,
        bbox_px=(900, 300, 980, 650),  # sağda (offset > 0) => hedef taraf "sol"
        estimated_distance_m=2.0,
        estimated_lateral_m=0.8,
    )
    cone_right = ObstacleDetection(
        kind="cone",
        confidence=0.9,
        bbox_px=(200, 300, 280, 650),  # solda (offset < 0) => hedef taraf "sag"
        estimated_distance_m=2.0,
        estimated_lateral_m=-0.8,
    )

    total += 1
    passed += case(
        algo="Slalom (SlalomLogic)",
        name="TR: Koni yok — bekleme fazı",
        desc="Koni yoksa slalom başlamaz; faz='bekleme' ve aktif=False beklenir.",
        run=lambda: slalom.update([]),
        expect={"faz": "bekleme", "aktif": False},
        show_keys=["faz", "aktif", "sebep"],
    )

    total += 1
    passed += case(
        algo="Slalom (SlalomLogic)",
        name="TR: İki koni kapı — aktif ve hedef_taraf=kapi",
        desc="Zıt tarafta iki koni aynı yakınlıkta ise 'başlangıç kapısı' algılanır; faz='aktif' ve hedef_taraf='kapi' beklenir.",
        run=lambda: slalom.update([cone_left, cone_right]),
        expect={"faz": "aktif", "hedef_taraf": "kapi", "aktif": True},
        show_keys=["faz", "aktif", "hedef_taraf", "steering", "hiz_katsayisi", "sebep"],
    )

    # --- ParkingLogic ---
    park = ParkingLogic()
    # distance ~6m: dy=1.2*700/6=140 => y2=500
    det_far = ParkingDetection(bbox_px=(500, 100, 780, 500), confidence=0.95, parking_allowed=True)
    # distance ~2.5m: dy=1.2*700/2.5=336 => y2=696
    det_mid = ParkingDetection(bbox_px=(500, 100, 780, 696), confidence=0.95, parking_allowed=True)
    # distance ~1.0m: dy=840 => y2=1200
    det_close = ParkingDetection(bbox_px=(500, 100, 780, 1200), confidence=0.95, parking_allowed=True)

    total += 1
    passed += case(
        algo="Park (ParkingLogic)",
        name="TR: Uzak park alanı — yaklaşma fazı",
        desc="Park bbox'u mesafe ~6m iken WAITING->APPROACHING ve düşük hızla yaklaşma beklenir.",
        run=lambda: park.update([det_far]),
        expect={"phase": ParkPhase.APPROACHING},
        show_keys=["phase", "speed_ratio", "reverse", "distance_m", "reason"],
    )

    total += 1
    passed += case(
        algo="Park (ParkingLogic)",
        name="TR: Daha yakın park alanı — hizalanma veya manevra",
        desc="Mesafe küçülünce hizalanma veya doğrudan manevra fazına geçebilir (lateral toleransa bağlı).",
        run=lambda: park.update([det_mid]),
        expect={"phase": (lambda v: v in {ParkPhase.ALIGNING, ParkPhase.MANEUVERING})},
        show_keys=["phase", "speed_ratio", "steering", "distance_m", "lateral_m", "reason"],
    )

    # Force maneuver completion with repeated close frames
    def run_park_to_complete():
        st = None
        for _ in range(25):
            st = park.update([det_close])
        return st

    total += 1
    passed += case(
        algo="Park (ParkingLogic)",
        name="TR: Çok yakın park alanı (tekrarlı) — tamamlandı",
        desc="Mesafe ~1m iken yeterli sayıda kare (PARK_CONFIRM_FRAMES) görülünce complete=True beklenir.",
        run=run_park_to_complete,
        expect={"complete": True, "phase": (lambda v: v in {ParkPhase.MANEUVERING, ParkPhase.PARKED})},
        show_keys=["phase", "complete", "reverse", "distance_m", "reason"],
    )

    # --- TrafficSignLogic ---
    def run_sign_confirmed(
        sign_class: str,
        *,
        distance_m: float | None = 8.0,
        confirm_frames: int = 3,
        after_s: float = 0.0,
    ) -> TrafficSignState:
        logic = TrafficSignLogic()
        det = SignDetection(
            class_name=sign_class,
            confidence=0.9,
            bbox_px=(0, 0, 10, 10),
            estimated_distance_m=distance_m,
        )
        now0 = 1000.0
        st: TrafficSignState | None = None
        for i in range(confirm_frames):
            st = logic.update([det], now=now0 + i * 0.1)
        # Confirm sonrası state'in “etkisini” görmek için bir güncelleme daha.
        st = logic.update([det], now=now0 + confirm_frames * 0.1 + after_s)
        assert st is not None
        return st

    total += 1
    passed += case(
        algo="Trafik levhası (TrafficSignLogic)",
        name="TR: STOP — onay sonrası 'dur' zorunluluğu (hold)",
        desc="STOP 3 kare doğrulanınca STOP_HOLD_SECONDS boyunca must_stop_soon=True döner.",
        run=lambda: run_sign_confirmed(SignClass.STOP, distance_m=8.0),
        expect={"must_stop_soon": True},
        show_keys=["must_stop_soon", "speed_cap_ratio", "active_signs", "reasons"],
    )

    total += 1
    passed += case(
        algo="Trafik levhası (TrafficSignLogic)",
        name="TR: NO_ENTRY — must_stop_soon",
        desc="Girilmez levhası doğrulanınca must_stop_soon=True beklenir.",
        run=lambda: run_sign_confirmed(SignClass.NO_ENTRY, distance_m=8.0),
        expect={"must_stop_soon": True},
        show_keys=["must_stop_soon", "active_signs", "reasons"],
    )

    total += 1
    passed += case(
        algo="Trafik levhası (TrafficSignLogic)",
        name="TR: PEDESTRIAN_CROSSING — hız tavanı düşer",
        desc="Yaya geçidi levhası doğrulanınca speed_cap_ratio en fazla CROSSWALK_SPEED_RATIO olur.",
        run=lambda: run_sign_confirmed(SignClass.PEDESTRIAN_CROSSING, distance_m=8.0),
        expect={"speed_cap_ratio": (lambda v: isinstance(v, float) and v <= 0.5)},
        show_keys=["speed_cap_ratio", "active_signs", "reasons"],
    )

    total += 1
    passed += case(
        algo="Trafik levhası (TrafficSignLogic)",
        name="TR: BUS_STOP — durum alanı var, kısıt yok",
        desc="Durak levhası doğrulanır; varsayılan kısıtlar değişmeden kalır (must_stop_soon=False, speed_cap_ratio=1.0).",
        run=lambda: run_sign_confirmed(SignClass.BUS_STOP, distance_m=8.0),
        expect={"must_stop_soon": False, "speed_cap_ratio": 1.0},
        show_keys=["must_stop_soon", "speed_cap_ratio", "active_signs", "reasons"],
    )

    total += 1
    passed += case(
        algo="Trafik levhası (TrafficSignLogic)",
        name="TR: PARKING_AREA — park bölgesi işaretlenir",
        desc="Park levhası doğrulanınca current_area_is_parking=True beklenir.",
        run=lambda: run_sign_confirmed(SignClass.PARKING_AREA, distance_m=8.0),
        expect={"current_area_is_parking": True},
        show_keys=["current_area_is_parking", "current_area_no_parking", "active_signs"],
    )

    total += 1
    passed += case(
        algo="Trafik levhası (TrafficSignLogic)",
        name="TR: NO_PARKING — park yasağı işaretlenir",
        desc="Park yapılmaz levhası doğrulanınca current_area_no_parking=True beklenir.",
        run=lambda: run_sign_confirmed(SignClass.NO_PARKING, distance_m=8.0),
        expect={"current_area_no_parking": True},
        show_keys=["current_area_is_parking", "current_area_no_parking", "active_signs"],
    )

    total += 1
    passed += case(
        algo="Trafik levhası (TrafficSignLogic)",
        name="TR: TRAFFIC_LIGHT_AHEAD — ışık bekleniyor bayrağı",
        desc="Işıklı işaret cihazı levhası doğrulanınca traffic_light_expected=True beklenir.",
        run=lambda: run_sign_confirmed(SignClass.TRAFFIC_LIGHT_AHEAD, distance_m=8.0),
        expect={"traffic_light_expected": True},
        show_keys=["traffic_light_expected", "active_signs", "reasons"],
    )

    total += 1
    passed += case(
        algo="Trafik levhası (TrafficSignLogic)",
        name="TR: TUNNEL — hız tavanı düşer",
        desc="Tünel levhası doğrulanınca speed_cap_ratio en fazla TUNNEL_SPEED_RATIO olur.",
        run=lambda: run_sign_confirmed(SignClass.TUNNEL, distance_m=8.0),
        expect={"approaching_tunnel": True, "speed_cap_ratio": (lambda v: isinstance(v, float) and v <= 0.7)},
        show_keys=["approaching_tunnel", "speed_cap_ratio", "active_signs", "reasons"],
    )

    # Dönüş / yön kısıtları: her tabela için ayrı senaryo
    turn_cases: list[tuple[str, str, dict[str, Any]]] = [
        (SignClass.NO_LEFT_TURN, "TR: NO_LEFT_TURN — sola yasak", {"turn_permissions": (lambda v: isinstance(v, dict) and v.get("left") is False)}),
        (SignClass.NO_RIGHT_TURN, "TR: NO_RIGHT_TURN — sağa yasak", {"turn_permissions": (lambda v: isinstance(v, dict) and v.get("right") is False)}),
        (SignClass.MUST_LEFT, "TR: MUST_LEFT — zorunlu sol", {"turn_permissions": {"left": True, "straight": False, "right": False, "forced_direction": "left"}}),
        (SignClass.MUST_RIGHT, "TR: MUST_RIGHT — zorunlu sağ", {"turn_permissions": {"left": False, "straight": False, "right": True, "forced_direction": "right"}}),
        (SignClass.MUST_STRAIGHT, "TR: MUST_STRAIGHT — zorunlu düz", {"turn_permissions": {"left": False, "straight": True, "right": False, "forced_direction": "straight"}}),
        (SignClass.STRAIGHT_OR_LEFT, "TR: STRAIGHT_OR_LEFT — düz veya sol", {"turn_permissions": {"left": True, "straight": True, "right": False, "forced_direction": None}}),
        (SignClass.STRAIGHT_OR_RIGHT, "TR: STRAIGHT_OR_RIGHT — düz veya sağ", {"turn_permissions": {"left": False, "straight": True, "right": True, "forced_direction": None}}),
        (SignClass.AHEAD_THEN_LEFT, "TR: AHEAD_THEN_LEFT — ileriden zorunlu sol", {"turn_permissions": {"left": True, "straight": False, "right": False, "forced_direction": "left"}}),
        (SignClass.AHEAD_THEN_RIGHT, "TR: AHEAD_THEN_RIGHT — ileriden zorunlu sağ", {"turn_permissions": {"left": False, "straight": False, "right": True, "forced_direction": "right"}}),
        (SignClass.KEEP_LEFT, "TR: KEEP_LEFT — soldan geç", {"turn_permissions": (lambda v: isinstance(v, dict) and v.get("forced_direction") == "pass_left")}),
        (SignClass.KEEP_RIGHT, "TR: KEEP_RIGHT — sağdan geç", {"turn_permissions": (lambda v: isinstance(v, dict) and v.get("forced_direction") == "pass_right")}),
        (SignClass.ROUNDABOUT, "TR: ROUNDABOUT — döner kavşak", {"turn_permissions": (lambda v: isinstance(v, dict) and v.get("forced_direction") == "roundabout")}),
        (SignClass.LANE_ARRANGEMENT_H, "TR: LANE_ARRANGEMENT_H — sol şerit bitiyor", {"turn_permissions": (lambda v: isinstance(v, dict) and v.get("forced_direction") is None)}),
        (SignClass.LANE_ARRANGEMENT_I, "TR: LANE_ARRANGEMENT_I — sağ şerit bitiyor", {"turn_permissions": (lambda v: isinstance(v, dict) and v.get("forced_direction") is None)}),
        (SignClass.TWO_WAY, "TR: TWO_WAY — iki yönlü yol", {"turn_permissions": (lambda v: isinstance(v, dict) and v.get("forced_direction") is None)}),
    ]

    for cls, title, expect in turn_cases:
        total += 1
        passed += case(
            algo="Trafik levhası (TrafficSignLogic)",
            name=title,
            desc=f"{cls} 3 kare doğrulanınca turn_permissions alanında ilgili kısıtlar beklenir.",
            run=lambda cls=cls: run_sign_confirmed(cls, distance_m=8.0),
            expect=expect,
            show_keys=["turn_permissions", "active_signs", "reasons"],
        )

    # --- TrafficLightLogic ---
    def run_light_confirmed(
        color: str,
        *,
        distance_m: float | None = 15.0,
        vehicle_speed_mps: float | None = None,
    ) -> TrafficLightState:
        logic = TrafficLightLogic()
        det = LightDetection(color=color, confidence=0.9, bbox_px=(0, 0, 10, 10), estimated_distance_m=distance_m)
        now0 = 2000.0
        st = logic.update([det], now=now0, vehicle_speed_mps=vehicle_speed_mps)
        st = logic.update([det], now=now0 + 0.1, vehicle_speed_mps=vehicle_speed_mps)  # CONFIRM_FRAMES=2
        return st

    total += 1
    passed += case(
        algo="Trafik ışığı (TrafficLightLogic)",
        name="TR: KIRMIZI — dur (must_stop)",
        desc="Kırmızı ışık 2 kare doğrulanınca must_stop=True ve speed_cap_ratio=0.0 beklenir.",
        run=lambda: run_light_confirmed(LightColor.RED, distance_m=15.0),
        expect={"must_stop": True, "speed_cap_ratio": 0.0, "active_color": LightColor.RED},
        show_keys=["must_stop", "speed_cap_ratio", "active_color", "reason"],
    )

    total += 1
    passed += case(
        algo="Trafik ışığı (TrafficLightLogic)",
        name="TR: SARI (ilk / yeşilden önce) — yavaşlama",
        desc="İlk görünen sarı veya yeşilden sonra sarı: prepare_to_stop=True ve YELLOW_SPEED_RATIO.",
        run=lambda: run_light_confirmed(LightColor.YELLOW, distance_m=20.0),
        expect={"prepare_to_stop": True, "prepare_to_move": False, "speed_cap_ratio": 0.4, "active_color": LightColor.YELLOW},
        show_keys=["prepare_to_stop", "prepare_to_move", "can_go", "speed_cap_ratio", "active_color", "reason"],
    )

    total += 1
    passed += case(
        algo="Trafik ışığı (TrafficLightLogic)",
        name="TR: SARI (yakın) — yine yavaşlama (commit yok)",
        desc="Sarı yakında da tam hız verilmez; yavaşlama tavanı korunur.",
        run=lambda: run_light_confirmed(LightColor.YELLOW, distance_m=8.0),
        expect={"prepare_to_stop": True, "can_go": False, "speed_cap_ratio": 0.4, "active_color": LightColor.YELLOW},
        show_keys=["prepare_to_stop", "prepare_to_move", "can_go", "speed_cap_ratio", "active_color", "reason"],
    )

    def run_red_then_yellow():
        logic = TrafficLightLogic()
        r = LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 10, 10), estimated_distance_m=15.0)
        y = LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 10, 10), estimated_distance_m=8.0)
        t0 = 2000.0
        logic.update([r], now=t0)
        logic.update([r], now=t0 + 0.1)
        logic.update([y], now=t0 + 0.5)
        return logic.update([y], now=t0 + 0.6)

    total += 1
    passed += case(
        algo="Trafik ışığı (TrafficLightLogic)",
        name="TR: KIRMIZI → SARI — harekete hazırlık",
        desc="Önce kırmızı onaylandıysa sarıda prepare_to_move ve düşük hız tavanı.",
        run=run_red_then_yellow,
        expect={"prepare_to_move": True, "prepare_to_stop": False, "speed_cap_ratio": 0.25, "active_color": LightColor.YELLOW},
        show_keys=["prepare_to_move", "prepare_to_stop", "can_go", "speed_cap_ratio", "active_color", "reason"],
    )

    def run_green_then_yellow():
        logic = TrafficLightLogic()
        g = LightDetection(color=LightColor.GREEN, confidence=0.9, bbox_px=(0, 0, 10, 10), estimated_distance_m=15.0)
        y = LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 10, 10), estimated_distance_m=10.0)
        t0 = 3000.0
        logic.update([g], now=t0)
        logic.update([g], now=t0 + 0.1)
        logic.update([y], now=t0 + 0.5)
        return logic.update([y], now=t0 + 0.6)

    total += 1
    passed += case(
        algo="Trafik ışığı (TrafficLightLogic)",
        name="TR: YEŞİL → SARI — yavaşlama",
        desc="Yeşilden sonra sarı: prepare_to_stop (kırmızı sonrası hazırlık değil).",
        run=run_green_then_yellow,
        expect={"prepare_to_stop": True, "prepare_to_move": False, "speed_cap_ratio": 0.4, "active_color": LightColor.YELLOW},
        show_keys=["prepare_to_move", "prepare_to_stop", "can_go", "speed_cap_ratio", "active_color", "reason"],
    )

    total += 1
    passed += case(
        algo="Trafik ışığı (TrafficLightLogic)",
        name="TR: SARI + hız≈0 — durmaya devam",
        desc="vehicle_speed_mps verilirse ve ~0 ise sarıda da hız tavanı 0 kalır.",
        run=lambda: run_light_confirmed(LightColor.YELLOW, distance_m=10.0, vehicle_speed_mps=0.0),
        expect={"speed_cap_ratio": 0.0, "prepare_to_stop": True},
        show_keys=["prepare_to_stop", "prepare_to_move", "speed_cap_ratio", "reason"],
    )

    total += 1
    passed += case(
        algo="Trafik ışığı (TrafficLightLogic)",
        name="TR: YEŞİL — geç",
        desc="Yeşil ışık doğrulanınca can_go=True ve speed_cap_ratio=1.0 beklenir.",
        run=lambda: run_light_confirmed(LightColor.GREEN, distance_m=15.0),
        expect={"can_go": True, "speed_cap_ratio": 1.0, "active_color": LightColor.GREEN},
        show_keys=["can_go", "speed_cap_ratio", "active_color", "reason"],
    )

    # --- GeoJsonMissionReader ---
    reader = GeoJsonMissionReader()
    geojson = json.dumps(
        {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "geometry": {"type": "Point", "coordinates": [29.0, 41.0]},
                    "properties": {"task": "start", "name": "S"},
                },
                {
                    "type": "Feature",
                    "geometry": {"type": "Point", "coordinates": [29.00001, 41.00001]},
                    "properties": {"task": "pickup", "name": "P", "radius_m": 2.0},
                },
            ],
        },
        ensure_ascii=False,
    )

    total += 1
    passed += case(
        algo="Görev okuyucu (GeoJsonMissionReader)",
        name="TR: GeoJSON string — 2 nokta",
        desc="GeoJSON FeatureCollection içinden 2 waypoint parse edilir; source_file='<string>' beklenir.",
        run=lambda: reader.read_string(geojson),
        expect={"source_file": "<string>"},
        show_keys=["source_file", "raw_crs", "points"],
    )

    # --- WaypointManager ---
    plan = reader.read_string(geojson)
    wpm = WaypointManager(plan, auto_advance=False)
    pos = GpsPosition(lat=41.0, lon=29.0, heading_deg=0.0)

    total += 1
    passed += case(
        algo="Waypoint (WaypointManager)",
        name="TR: Start noktasında — arrived=True",
        desc="Araç start noktasındaysa mesafe 0'a yakın; arrived=True ve steering_ref ~0 beklenir.",
        run=lambda: wpm.update(pos),
        expect={"arrived": True, "wp_index": 0, "current_task": TaskType.START},
        show_keys=["arrived", "wp_index", "current_task", "distance_to_wp_m", "steering_ref", "reason"],
    )

    # --- MissionManager (pickup hold) ---
    mm = MissionManager(plan)
    # move to pickup wp
    mm.wp.advance()
    pickup_pos = GpsPosition(lat=41.00001, lon=29.00001, heading_deg=0.0)

    def run_mission_hold():
        # arrived at pickup; must hold at least 15s => speed cap 0
        st, dec = mm.update(pickup_pos, now_s=3000.0)
        return {"arrived": st.arrived, "task": st.current_task, "speed_cap_ratio": dec.speed_cap_ratio, "hold_reason": dec.hold_reason}

    total += 1
    passed += case(
        algo="Görev yöneticisi (MissionManager)",
        name="TR: Pickup varış — minimum bekleme (hız tavanı 0)",
        desc="PICKUP/DROPOFF görevlerinde arrived=True olunca minimum süre bekleme uygulanır (speed_cap_ratio=0).",
        run=run_mission_hold,
        expect={"arrived": True, "task": TaskType.PICKUP, "speed_cap_ratio": 0.0},
        show_keys=["arrived", "task", "speed_cap_ratio", "hold_reason"],
    )

    # --- Perception fusion ---
    stereo = [
        StereoBbox(class_name="red", confidence=0.9, bbox_px=(0, 0, 1, 1), distance_m=20.0),
        StereoBbox(class_name="koni", confidence=0.9, bbox_px=(10, 10, 20, 20), distance_m=5.0, lateral_m=0.2),
        StereoBbox(class_name=SignClass.STOP, confidence=0.9, bbox_px=(5, 5, 6, 6), distance_m=8.0),
    ]
    lidar = [LidarObstacle(kind="barrier", confidence=0.8, distance_m=6.0, lateral_m=-0.5)]
    imu = ImuSample(heading_deg=10.0)

    def run_fuse():
        frame = fuse(stereo=stereo, lidar=lidar, imu=imu)
        return {"sign": len(frame.sign_dets), "light": len(frame.light_dets), "obs": len(frame.obstacle_dets), "imu": frame.imu.heading_deg if frame.imu else None}

    total += 1
    passed += case(
        algo="Algı birleştirme (perception_fusion.fuse)",
        name="TR: Stereo + LiDAR — listelere ayrıştırma",
        desc="Stereo bbox sınıfları (red/koni/STOP) + LiDAR engeli birleştirilir; imu heading korunur.",
        run=run_fuse,
        expect={"sign": 1, "light": 1, "obs": 2, "imu": 10.0},
        show_keys=["sign", "light", "obs", "imu"],
    )

    print("\n== Özet ==")
    print(f"gecen {passed}/{total}")
    if passed != total:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

