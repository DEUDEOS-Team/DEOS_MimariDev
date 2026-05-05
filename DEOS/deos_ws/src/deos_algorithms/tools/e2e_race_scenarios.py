#!/usr/bin/env python3
"""
E2E yarış senaryoları (ROS'suz, okunur çıktı)
--------------------------------------------
Amaç: Yarışma turları **sırayla yaşanıyormuş gibi** çalıştırmak ve başka insanların da
anlayabileceği şekilde “beklenen vs çıkan” özetleri almak.

Kapsam (bu repodaki mevcut mimari):
- Global planlama: GeoJSON -> MissionManager/WaypointManager
- Lokal kararlar: TrafficLight/TrafficSign/Obstacle/Parking
- Karar birleştirme: DecisionArbiter (öncelik + şerit kısıtı)

Çalıştırma:
  cd DEOS/deos_ws/src/deos_algorithms
  python tools/e2e_race_scenarios.py
"""

import json
import sys
import time
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any

# Allow running without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, LaneBounds, ReasonCode  # noqa: E402
from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader  # noqa: E402
from deos_algorithms.mission_manager import MissionManager  # noqa: E402
from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleLogic  # noqa: E402
from deos_algorithms.parking_logic import ParkingDetection, ParkingLogic  # noqa: E402
from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic  # noqa: E402
from deos_algorithms.traffic_sign_logic import SignClass, SignDetection, TrafficSignLogic  # noqa: E402
from deos_algorithms.waypoint_manager import GpsPosition  # noqa: E402


def _ensure_utf8_stdio() -> None:
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


def _pp(title: str, d: dict[str, Any]) -> None:
    print(title, json.dumps(_normalize(d), ensure_ascii=False, default=str))


def _case(*, name: str, desc: str, expected: dict[str, Any], actual: dict[str, Any]) -> bool:
    ok = True
    mism: dict[str, tuple[Any, Any]] = {}
    for k, ev in expected.items():
        av = actual.get(k)
        if callable(ev):
            try:
                passed = bool(ev(av))
            except Exception:
                passed = False
            if not passed:
                ok = False
                mism[k] = ("<koşul>", av)
        else:
            if av != ev:
                ok = False
                mism[k] = (ev, av)

    status = "GEÇTİ" if ok else "KALDI"
    print(f"\n[{status}] {name}")
    print(f"  senaryo  : {desc}")
    print("  beklenen :", json.dumps({k: ("<koşul>" if callable(v) else v) for k, v in expected.items()}, ensure_ascii=False, default=str))
    print("  çıkan    :", json.dumps(_normalize(actual), ensure_ascii=False, default=str))
    if mism:
        print("  farklar  :")
        for k, (ev, av) in mism.items():
            print(f"   - {k}: beklenen={ev} çıkan={av}")
    return ok


def _arbiter_from_logic(
    *,
    light: TrafficLightLogic,
    sign: TrafficSignLogic,
    obstacle: ObstacleLogic,
    park: ParkingLogic,
    lane: LaneBounds | None,
    park_mode: bool,
    light_dets: list[LightDetection],
    sign_dets: list[SignDetection],
    obs_dets: list[ObstacleDetection],
    park_dets: list[ParkingDetection],
) -> tuple[Any, list[Candidate]]:
    sign_state = sign.update(sign_dets, now=100.0)
    light_state = light.update(light_dets, now=200.0)
    obs_state = obstacle.update(obs_dets)
    park_state = park.update(park_dets)

    candidates: list[Candidate] = []

    if light_state.must_stop:
        candidates.append(Candidate(name="light", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.LIGHT_MUST_STOP]))
    elif float(light_state.speed_cap_ratio) < 1.0:
        candidates.append(Candidate(name="light", speed_cap=float(light_state.speed_cap_ratio), reasons=[ReasonCode.LIGHT_YELLOW_SLOW]))

    if sign_state.must_stop_soon:
        candidates.append(Candidate(name="sign", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.SIGN_MUST_STOP]))
    elif float(sign_state.speed_cap_ratio) < 1.0:
        candidates.append(Candidate(name="sign", speed_cap=float(sign_state.speed_cap_ratio), reasons=[ReasonCode.SIGN_SPEED_CAP]))

    if obs_state.emergency_stop:
        candidates.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.OBSTACLE_EMERGENCY_STOP]))
    if obs_state.road_blocked:
        candidates.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.ROAD_BLOCKED]))
    if float(obs_state.speed_cap_ratio) < 1.0:
        candidates.append(Candidate(name="obstacle", speed_cap=float(obs_state.speed_cap_ratio), reasons=[]))

    if obs_state.suggest_lane_change and not obs_state.road_blocked:
        steer_bias = -0.35 if obs_state.avoidance_direction == "left" else 0.35
        candidates.append(Candidate(name="static_avoid", speed_cap=0.35, steer_override=float(steer_bias), reasons=[ReasonCode.STATIC_AVOID]))

    if park_mode and not park_state.complete:
        reasons = [ReasonCode.PARK_MODE]
        if park_state.no_eligible_spot:
            reasons.append(ReasonCode.PARK_NO_ELIGIBLE)
        candidates.append(Candidate(name="park", speed_cap=float(park_state.speed_ratio), steer_override=float(park_state.steering), reasons=reasons))

    arb = DecisionArbiter()
    decision = arb.arbitrate(candidates=candidates, lane=lane, lane_required_for_avoidance=True)
    return decision, candidates


def _mk_geojson(*, n_tasks: int, include_park: bool) -> str:
    feats: list[dict[str, Any]] = []
    feats.append({"type": "Feature", "geometry": {"type": "Point", "coordinates": [29.0, 41.0]}, "properties": {"name": "start"}})
    for i in range(1, n_tasks + 1):
        feats.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [29.0 + i * 0.00001, 41.0 + i * 0.00001]},
                "properties": {"name": f"gorev_{i}"},
            }
        )
    if include_park:
        feats.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [29.0 + (n_tasks + 1) * 0.00001, 41.0 + (n_tasks + 1) * 0.00001]},
                "properties": {"name": "park_giris"},
            }
        )
    return json.dumps({"type": "FeatureCollection", "features": feats}, ensure_ascii=False)


def _pos_for_feature(i: int) -> GpsPosition:
    # i=0 start, i=1 gorev_1, ...
    return GpsPosition(lat=41.0 + i * 0.00001, lon=29.0 + i * 0.00001, heading_deg=0.0)


def main() -> None:
    _ensure_utf8_stdio()
    print("== E2E: 2026 Robotaksi tur senaryoları (okunur çıktı) ==")
    lane = LaneBounds(left_y_m=1.5, right_y_m=-1.5, margin_m=0.25)

    total = 0
    passed = 0

    def run_tour(*, tour_name: str, n_tasks: int, with_lights_signs: bool, with_dynamic_obstacle: bool, with_static_obstacle: bool):
        nonlocal total, passed
        print(f"\n====================\nTUR: {tour_name}\n====================")

        plan = GeoJsonMissionReader().read_string(_mk_geojson(n_tasks=n_tasks, include_park=True))
        mission = MissionManager(plan)

        light = TrafficLightLogic()
        sign = TrafficSignLogic()
        obstacle = ObstacleLogic()
        park = ParkingLogic()

        now0 = time.monotonic()

        # 0) Start
        st, dec = mission.update(_pos_for_feature(0), now_s=now0)
        total += 1
        expected_task_after_start = "checkpoint" if n_tasks > 0 else "park_entry"
        passed += _case(
            name=f"{tour_name}: start -> ilerle",
            desc="Araç start noktasına gelince bir sonraki waypoint'e geçmeli (auto advance).",
            expected={"current_task": expected_task_after_start},
            actual={"current_task": st.current_task},
        )

        # 1..n gorev
        for i in range(1, n_tasks + 1):
            st, dec = mission.update(_pos_for_feature(i), now_s=now0 + i)
            total += 1
            passed += _case(
                name=f"{tour_name}: gorev_{i} -> sonraki",
                desc="Araç checkpoint'e gelince MissionManager waypoint'i ilerletir; dönen state bir SONRAKİ waypoint'e aittir.",
                expected={"wp_index": i + 1},
                actual={"wp_index": st.wp_index},
            )

        # Park girişine varış (park_mode)
        park_i = n_tasks + 1
        st, dec = mission.update(_pos_for_feature(park_i), now_s=now0 + 50.0)
        total += 1
        passed += _case(
            name=f"{tour_name}: park_giris -> park_mode",
            desc="Park girişine gelince park_mode aktif olmalı ve 3dk sayaç başlamalı.",
            expected={"park_mode": True, "park_remaining_s": (lambda v: isinstance(v, float) and 0.0 < v <= 180.0)},
            actual={"park_mode": dec.park_mode, "park_remaining_s": dec.park_remaining_s},
        )

        # Park alanında kararlar
        # Işık/tabela senaryoları (tur 2/3)
        light_dets: list[LightDetection] = []
        sign_dets: list[SignDetection] = []
        if with_lights_signs:
            # Kırmızı: dur
            light_dets = [LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=10.0)] * 2
            decision, _ = _arbiter_from_logic(
                light=TrafficLightLogic(),
                sign=TrafficSignLogic(),
                obstacle=ObstacleLogic(),
                park=ParkingLogic(),
                lane=lane,
                park_mode=False,
                light_dets=light_dets,
                sign_dets=[],
                obs_dets=[],
                park_dets=[],
            )
            total += 1
            passed += _case(
                name=f"{tour_name}: kırmızı ışık -> dur",
                desc="Kırmızı ışıkta must_stop baskın olmalı.",
                expected={"emergency_stop": True, "speed_cap": 0.0},
                actual={"emergency_stop": decision.emergency_stop, "speed_cap": decision.speed_cap},
            )

            # Dönüş kısıtı örneği: MUST_LEFT (zorunlu sol)
            sign_dets = [SignDetection(class_name=SignClass.MUST_LEFT, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=8.0)] * 3
            sst = TrafficSignLogic().update(sign_dets, now=1000.0)
            total += 1
            passed += _case(
                name=f"{tour_name}: tabela onayı -> forced_direction",
                desc="Zorunlu yön tabelası turn_permissions.forced_direction üretmeli.",
                expected={"forced_direction": "left"},
                actual={"forced_direction": sst.turn_permissions.forced_direction},
            )

        # Engeller (tur 2/3)
        obs_dets: list[ObstacleDetection] = []
        if with_dynamic_obstacle:
            ped = ObstacleDetection(kind="pedestrian", confidence=0.95, bbox_px=(0, 0, 1, 1), estimated_distance_m=4.0, estimated_lateral_m=0.0)
            # confirm frames
            for _ in range(4):
                obstacle.update([ped])
            obs_dets = [ped]
            decision, _ = _arbiter_from_logic(
                light=TrafficLightLogic(),
                sign=TrafficSignLogic(),
                obstacle=obstacle,
                park=ParkingLogic(),
                lane=lane,
                park_mode=False,
                light_dets=[],
                sign_dets=[],
                obs_dets=obs_dets,
                park_dets=[],
            )
            total += 1
            passed += _case(
                name=f"{tour_name}: dinamik engel -> dur/bekle",
                desc="Yaya çok yakınsa hız tavanı 0 olmalı (bekle).",
                expected={"speed_cap": 0.0},
                actual={"speed_cap": decision.speed_cap},
            )

        if with_static_obstacle:
            bar = ObstacleDetection(kind="barrier", confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=2.0, estimated_lateral_m=0.6)
            for _ in range(3):
                obstacle.update([bar])
            decision, _ = _arbiter_from_logic(
                light=TrafficLightLogic(),
                sign=TrafficSignLogic(),
                obstacle=obstacle,
                park=ParkingLogic(),
                lane=lane,
                park_mode=False,
                light_dets=[],
                sign_dets=[],
                obs_dets=[bar],
                park_dets=[],
            )
            total += 1
            passed += _case(
                name=f"{tour_name}: statik engel -> kaçınma override",
                desc="Statik engelde şerit varsa steer override devreye girebilir.",
                expected={"has_steer_override": True},
                actual={"has_steer_override": decision.has_steer_override},
            )

        # Park manevrası (park_mode=True)
        park_det = ParkingDetection(bbox_px=(500, 100, 780, 696), confidence=0.95, parking_allowed=True)
        decision, _ = _arbiter_from_logic(
            light=TrafficLightLogic(),
            sign=TrafficSignLogic(),
            obstacle=ObstacleLogic(),
            park=park,
            lane=lane,
            park_mode=True,
            light_dets=[],
            sign_dets=[SignDetection(class_name=SignClass.PARKING_AREA, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=5.0)] * 3,
            obs_dets=[],
            park_dets=[park_det],
        )
        total += 1
        passed += _case(
            name=f"{tour_name}: park_mode -> park override",
            desc="Park modunda arbiter park candidate'ını seçip steer override vermeli.",
            expected={"has_steer_override": True, "speed_cap": (lambda v: isinstance(v, float) and v <= 0.4)},
            actual={"has_steer_override": decision.has_steer_override, "speed_cap": decision.speed_cap},
        )

    # Şartname: 1. tur (statik, ışık/işaret yok) – start + 3 görev + park
    run_tour(tour_name="1. TUR", n_tasks=3, with_lights_signs=False, with_dynamic_obstacle=False, with_static_obstacle=True)
    # Şartname: 2. tur (dinamik, ışık+işaret aktif, dinamik engel) – start + park
    run_tour(tour_name="2. TUR", n_tasks=0, with_lights_signs=True, with_dynamic_obstacle=True, with_static_obstacle=False)
    # Şartname: 3. tur (dinamik, ışık+işaret + dinamik + statik) – start + 2 görev + park
    run_tour(tour_name="3. TUR", n_tasks=2, with_lights_signs=True, with_dynamic_obstacle=True, with_static_obstacle=True)

    print("\n== Özet ==")
    print(f"gecen {passed}/{total}")
    if passed != total:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

