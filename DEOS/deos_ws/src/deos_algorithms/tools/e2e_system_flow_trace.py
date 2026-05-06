#!/usr/bin/env python3
"""
ROS'suz "tüm sistem akışı" trace testi.

Amaç:
- ROS2 topic'leri çalışmıyorken bile, mimarideki veri akışını uçtan uca görmek:
  Planning (MissionManager) -> Perception logic'ler -> DecisionArbiter -> Controller mix -> /cmd_vel.

Çıktı:
- Her adımda "publish edilen topic" benzeri alanları tek JSON satırında basar.
- Başka ekip üyeleri için okunabilir bir "flow" dokümantasyonu gibi çalışır.

Çalıştırma:
  cd DEOS/deos_ws/src/deos_algorithms
  python tools/e2e_system_flow_trace.py
"""

from __future__ import annotations

import json
import sys
import time
import argparse
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


def _jprint(d: dict[str, Any]) -> None:
    print(json.dumps(_normalize(d), ensure_ascii=False, default=str))


def _short_summary(ev: dict[str, Any]) -> str:
    pub = ev.get("publish", {})
    task = pub.get("/planning/current_task")
    park = pub.get("/planning/park_mode")
    estop = pub.get("/perception/emergency_stop")
    sc = pub.get("/perception/speed_cap")
    hs = pub.get("/perception/has_steering_override")
    cmd = pub.get("/cmd_vel", {})
    return (
        f"task={task} park_mode={park} estop={estop} "
        f"speed_cap={sc} steer_ovr={hs} cmd(v={cmd.get('linear_x')}, w={cmd.get('angular_z')})"
    )

def _candidates_from_states(
    *,
    light_state,
    sign_state,
    obs_state,
    park_state,
    park_mode: bool,
) -> list[Candidate]:
    cands: list[Candidate] = []

    # light
    if light_state.must_stop:
        cands.append(Candidate(name="light", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.LIGHT_MUST_STOP]))
    elif float(light_state.speed_cap_ratio) < 1.0:
        cands.append(Candidate(name="light", emergency_stop=False, speed_cap=float(light_state.speed_cap_ratio), reasons=[ReasonCode.LIGHT_YELLOW_SLOW]))

    # sign
    if sign_state.must_stop_soon:
        cands.append(Candidate(name="sign", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.SIGN_MUST_STOP]))
    elif float(sign_state.speed_cap_ratio) < 1.0:
        cands.append(Candidate(name="sign", emergency_stop=False, speed_cap=float(sign_state.speed_cap_ratio), reasons=[ReasonCode.SIGN_SPEED_CAP]))

    # obstacle
    if obs_state.emergency_stop:
        cands.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.OBSTACLE_EMERGENCY_STOP]))
    if obs_state.road_blocked:
        cands.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.ROAD_BLOCKED]))
    if float(obs_state.speed_cap_ratio) < 1.0:
        cands.append(Candidate(name="obstacle", emergency_stop=False, speed_cap=float(obs_state.speed_cap_ratio), reasons=[]))
    if obs_state.suggest_lane_change and not obs_state.road_blocked:
        steer_bias = -0.35 if obs_state.avoidance_direction == "left" else 0.35
        cands.append(Candidate(name="static_avoid", emergency_stop=False, speed_cap=0.35, steer_override=float(steer_bias), reasons=[ReasonCode.STATIC_AVOID]))

    # park
    if park_mode and not park_state.complete:
        rs = [ReasonCode.PARK_MODE]
        if park_state.no_eligible_spot:
            rs.append(ReasonCode.PARK_NO_ELIGIBLE)
        cands.append(Candidate(name="park", emergency_stop=False, speed_cap=float(park_state.speed_ratio), steer_override=float(park_state.steering), reasons=rs))

    return cands


def _controller_mix(
    *,
    plan_steer: float,
    plan_speed: float,
    perception_speed_cap: float,
    perception_has_steer: bool,
    perception_steer: float,
    max_speed_mps: float = 3.0,
    max_steer_rads: float = 1.0,
) -> dict[str, Any]:
    speed_ratio = float(max(0.0, min(1.0, min(perception_speed_cap, plan_speed))))
    steer_ratio = float(perception_steer if perception_has_steer else plan_steer)
    steer_ratio = float(max(-1.0, min(1.0, steer_ratio)))
    return {
        "/cmd_vel": {"linear_x": speed_ratio * max_speed_mps, "angular_z": steer_ratio * max_steer_rads},
        "mix": {
            "plan": {"steer": plan_steer, "speed": plan_speed},
            "perception": {"speed_cap": perception_speed_cap, "has_steer": perception_has_steer, "steer": perception_steer},
            "result_ratio": {"speed": speed_ratio, "steer": steer_ratio},
        },
    }


def main() -> int:
    _ensure_utf8_stdio()
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="", help="NDJSON output file path (optional)")
    ap.add_argument("--pretty", action="store_true", help="Print human-readable headings + summaries")
    args = ap.parse_args()

    out_f = None
    if args.out:
        out_path = Path(args.out).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_f = out_path.open("w", encoding="utf-8")

    print("== ROS'suz Full Flow Trace (Planning→Perception→Arbiter→Controller) ==")
    if out_f is not None:
        print(f"ndjson_out = {str(Path(args.out).resolve())}")

    geojson = json.dumps(
        {
            "type": "FeatureCollection",
            "features": [
                {"type": "Feature", "geometry": {"type": "Point", "coordinates": [29.0, 41.0]}, "properties": {"name": "start"}},
                {"type": "Feature", "geometry": {"type": "Point", "coordinates": [29.00001, 41.00001]}, "properties": {"name": "gorev_1"}},
                {"type": "Feature", "geometry": {"type": "Point", "coordinates": [29.00002, 41.00002]}, "properties": {"name": "park_giris"}},
            ],
        },
        ensure_ascii=False,
    )
    plan = GeoJsonMissionReader().read_string(geojson)
    mission = MissionManager(plan)

    light = TrafficLightLogic()
    sign = TrafficSignLogic()
    obstacle = ObstacleLogic()
    park = ParkingLogic()
    arb = DecisionArbiter()

    lane = LaneBounds(left_y_m=1.5, right_y_m=-1.5, margin_m=0.25)

    # Steps simulate competition-like signals
    steps: list[dict[str, Any]] = [
        {
            "name": "S1 start",
            "gps": (41.0, 29.0),
            "light": [],
            "sign": [],
            "obs": [],
            "park": [],
        },
        {
            "name": "S2 sarı ışık",
            "gps": (41.000005, 29.000005),
            "light": [LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=20.0)] * 2,
            "sign": [],
            "obs": [],
            "park": [],
        },
        {
            "name": "S3 sağ-sol dönüşümlü bariyer (kaçınma commit örnek)",
            "gps": (41.000008, 29.000008),
            "light": [],
            "sign": [],
            "obs": [ObstacleDetection(kind="barrier", confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=2.5, estimated_lateral_m=0.6)],
            "park": [],
        },
        {
            "name": "S4 park girişine varış -> park_mode",
            "gps": (41.00002, 29.00002),
            "light": [],
            "sign": [SignDetection(class_name=SignClass.PARKING_AREA, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=5.0)] * 3,
            "obs": [],
            "park": [ParkingDetection(bbox_px=(500, 100, 780, 696), confidence=0.95, parking_allowed=True)],
        },
        {
            "name": "S5 park tabelası yok (robust search)",
            "gps": (41.0000205, 29.0000205),
            "light": [],
            "sign": [],  # no sign -> no eligible
            "obs": [],
            "park": [],  # no slot -> search
        },
    ]

    t0 = time.monotonic()
    for i, s in enumerate(steps):
        now = t0 + i * 0.5
        lat, lon = s["gps"]
        pos = GpsPosition(lat=float(lat), lon=float(lon), heading_deg=0.0)

        wp_state, mission_dec = mission.update(pos, now_s=now)
        plan_topics = {
            "/planning/steering_ref": float(wp_state.steering_ref),
            "/planning/speed_limit": float(wp_state.speed_limit_ratio) * float(mission_dec.speed_cap_ratio),
            "/planning/current_task": str(wp_state.current_task),
            "/planning/arrived": bool(wp_state.arrived),
            "/planning/park_mode": bool(mission_dec.park_mode),
            "/planning/park_remaining_s": float(mission_dec.park_remaining_s),
        }

        sign_state = sign.update(s["sign"], now=now)
        light_state = light.update(s["light"], now=now)
        obs_state = obstacle.update(s["obs"])

        # Park: only active in park_mode
        park_state = park.update(s["park"])

        candidates = _candidates_from_states(
            light_state=light_state,
            sign_state=sign_state,
            obs_state=obs_state,
            park_state=park_state,
            park_mode=bool(mission_dec.park_mode),
        )
        decision = arb.arbitrate(candidates=candidates, lane=lane, lane_required_for_avoidance=True)

        perception_topics = {
            "/perception/emergency_stop": bool(decision.emergency_stop),
            "/perception/speed_cap": float(decision.speed_cap),
            "/perception/has_steering_override": bool(decision.has_steer_override),
            "/perception/steering_override": float(decision.steer_override),
            "/perception/park_complete": bool(park_state.complete),
            "/perception/decision_debug": {
                "candidates": [
                    {"name": c.name, "speed_cap": c.speed_cap, "steer": c.steer_override, "reasons": [r.value for r in c.reasons]}
                    for c in candidates
                ],
                "final": {"reasons": [r.value for r in decision.reasons]},
            },
        }

        ctrl = _controller_mix(
            plan_steer=float(plan_topics["/planning/steering_ref"]),
            plan_speed=float(plan_topics["/planning/speed_limit"]),
            perception_speed_cap=float(perception_topics["/perception/speed_cap"]),
            perception_has_steer=bool(perception_topics["/perception/has_steering_override"]),
            perception_steer=float(perception_topics["/perception/steering_override"]),
        )

        ev = {
            "step": s["name"],
            "t_monotonic": float(now),
            "inputs": {
                "/gps/fix": {"lat": float(lat), "lon": float(lon)},
                "detections": {
                    "light": [getattr(d, "color", None) for d in s["light"]],
                    "sign": [getattr(d, "class_name", None) for d in s["sign"]],
                    "obs": [getattr(d, "kind", None) for d in s["obs"]],
                    "park": [
                        bool(getattr(d, "parking_allowed", None))
                        if getattr(d, "parking_allowed", None) is not None
                        else None
                        for d in s["park"]
                    ],
                },
            },
            "publish": {
                **plan_topics,
                **perception_topics,
                **ctrl,
            },
        }

        if bool(args.pretty):
            print("\n--------------------")
            print(f"ADIM: {s['name']}")
            print("Özet:", _short_summary(ev))
            print("Planning:", json.dumps(_normalize(plan_topics), ensure_ascii=False, default=str))
            print(
                "Perception:",
                json.dumps(
                    _normalize(
                        {
                            "/perception/emergency_stop": perception_topics["/perception/emergency_stop"],
                            "/perception/speed_cap": perception_topics["/perception/speed_cap"],
                            "/perception/has_steering_override": perception_topics["/perception/has_steering_override"],
                            "/perception/steering_override": perception_topics["/perception/steering_override"],
                            "/perception/decision_debug": perception_topics["/perception/decision_debug"],
                        }
                    ),
                    ensure_ascii=False,
                    default=str,
                ),
            )
            print("Controller:", json.dumps(_normalize(ctrl), ensure_ascii=False, default=str))

        _jprint(ev)
        if out_f is not None:
            out_f.write(json.dumps(_normalize(ev), ensure_ascii=False, default=str) + "\n")

    if out_f is not None:
        out_f.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

