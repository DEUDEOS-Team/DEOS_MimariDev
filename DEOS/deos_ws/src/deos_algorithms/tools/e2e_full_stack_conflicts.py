#!/usr/bin/env python3
"""
ROS'suz uçtan uca "çatışma" senaryoları (okunur çıktı + PASS/FAIL)
---------------------------------------------------------------
Amaç:
- Algoritmalar birlikte çalışırken (ışık + engel aynı anda) DecisionArbiter'ın
  deterministik şekilde doğru karar vermesini doğrulamak.

Kapsam:
- Planning: MissionManager (basit plan)
- Perception: TrafficLightLogic / ObstacleLogic (ve opsiyonel lane constraint)
- Arbiter: DecisionArbiter
- Controller mix: plan hız/direksiyon + perception speed_cap/steer_override -> /cmd_vel

Çalıştırma:
  cd DEOS/deos_ws/src/deos_algorithms
  python tools/e2e_full_stack_conflicts.py
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Optional

# Allow running without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, LaneBounds, ReasonCode  # noqa: E402
from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader  # noqa: E402
from deos_algorithms.mission_manager import MissionManager  # noqa: E402
from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleLogic  # noqa: E402
from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic  # noqa: E402
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
        return obj.value
    if is_dataclass(obj):
        return {k: _normalize(v) for k, v in asdict(obj).items()}
    if isinstance(obj, dict):
        return {k: _normalize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_normalize(v) for v in obj]
    return obj


def _print_output_key_legend() -> None:
    """
    Çıktı alanları (expected/actual) neyi doğrular?
    - emergency_stop: True => araç mutlak durmalı (arbiter hard-stop).
    - speed_cap: [0..1] hız üst limiti oranı. Adaylar arasında minimum alınır.
    - has_steer_override: True => perception kaynaklı steer override uygulanıyor.
    - steer_override: [-1..1] normalize steer (lane clamp/disable uygulayabilir).
    - reasons: Kararın gerekçe kodları (ReasonCode listesi).
    - controller./cmd_vel: Testte plan+perception karışımından üretilen kontrol çıktısı.
    """

    print("\n-- ÇIKTI ALANLARI SÖZLÜĞÜ --")
    print(" decision.emergency_stop     : Acil dur kararı (hız 0)")
    print(" decision.speed_cap          : Hız üst limiti oranı [0..1] (min kuralı)")
    print(" decision.has_steer_override : Steer override aktif mi")
    print(" decision.steer_override     : Normalize steer [-1..1] (lane clamp/disable olabilir)")
    print(" decision.reasons            : Karar gerekçeleri (ReasonCode)")
    print(" controller./cmd_vel         : Nihai /cmd_vel örnek çıktısı (simülasyon)")


def _case(
    *,
    name: str,
    desc: str,
    expected: dict[str, Any],
    actual: dict[str, Any],
    actions_if_pass: list[str] | None = None,
) -> bool:
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
    print(
        "  beklenen :",
        json.dumps({k: ("<koşul>" if callable(v) else v) for k, v in expected.items()}, ensure_ascii=False, default=str),
    )
    print("  çıkan    :", json.dumps(_normalize(actual), ensure_ascii=False, default=str))
    if ok and actions_if_pass:
        print("  eylemler :")
        for a in actions_if_pass:
            print(f"   - {a}")
    if mism:
        print("  farklar  :")
        for k, (ev, av) in mism.items():
            print(f"   - {k}: beklenen={ev} çıkan={av}")
    return ok


def _contains_reason(code: ReasonCode) -> Callable[[Any], bool]:
    want = str(code.value)

    def fn(v: Any) -> bool:
        if v is None:
            return False
        if isinstance(v, list):
            return want in [str(x) for x in v]
        return False

    return fn


def _run_step(
    *,
    lane: LaneBounds | None,
    light_dets: list[LightDetection],
    obs_dets: list[ObstacleDetection],
    now: float,
) -> dict[str, Any]:
    # Simple plan: start -> gorev_1 (so we always have a non-trivial planning output)
    geojson = json.dumps(
        {
            "type": "FeatureCollection",
            "features": [
                {"type": "Feature", "geometry": {"type": "Point", "coordinates": [29.0, 41.0]}, "properties": {"name": "start"}},
                {"type": "Feature", "geometry": {"type": "Point", "coordinates": [29.00001, 41.00001]}, "properties": {"name": "gorev_1"}},
            ],
        },
        ensure_ascii=False,
    )
    plan = GeoJsonMissionReader().read_string(geojson)
    mission = MissionManager(plan)
    # Put vehicle near gorev_1 (planning still outputs steer/speed)
    pos = GpsPosition(lat=41.000005, lon=29.000005, heading_deg=0.0)
    wp_state, mission_dec = mission.update(pos, now_s=now)

    light = TrafficLightLogic()
    obstacle = ObstacleLogic()
    arb = DecisionArbiter()

    # TrafficLightLogic: CONFIRM_FRAMES=2 can be satisfied by providing two detections in one call.
    light_state = light.update(light_dets, now=now)

    # ObstacleLogic/SafetyLogic: CONFIRM_FRAMES=3 requires temporal confirmation across updates.
    obs_state = None
    for _ in range(3):
        obs_state = obstacle.update(obs_dets)
    assert obs_state is not None

    cands: list[Candidate] = []
    if light_state.must_stop:
        cands.append(Candidate(name="light", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.LIGHT_MUST_STOP]))
    elif float(light_state.speed_cap_ratio) < 1.0:
        cands.append(Candidate(name="light", emergency_stop=False, speed_cap=float(light_state.speed_cap_ratio), reasons=[ReasonCode.LIGHT_YELLOW_SLOW]))

    if obs_state.emergency_stop:
        cands.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.OBSTACLE_EMERGENCY_STOP]))
    if obs_state.road_blocked:
        cands.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.ROAD_BLOCKED]))
    if float(obs_state.speed_cap_ratio) < 1.0:
        cands.append(Candidate(name="obstacle", emergency_stop=False, speed_cap=float(obs_state.speed_cap_ratio), reasons=[]))
    if obs_state.suggest_lane_change and not obs_state.road_blocked:
        steer_bias = -0.35 if obs_state.avoidance_direction == "left" else 0.35
        cands.append(Candidate(name="static_avoid", emergency_stop=False, speed_cap=0.35, steer_override=float(steer_bias), reasons=[ReasonCode.STATIC_AVOID]))

    decision = arb.arbitrate(candidates=cands, lane=lane, lane_required_for_avoidance=True)

    plan_steer = float(wp_state.steering_ref)
    plan_speed = float(wp_state.speed_limit_ratio) * float(mission_dec.speed_cap_ratio)

    speed_ratio = float(max(0.0, min(1.0, min(float(decision.speed_cap), plan_speed))))
    steer_ratio = float(float(decision.steer_override) if bool(decision.has_steer_override) else plan_steer)
    steer_ratio = float(max(-1.0, min(1.0, steer_ratio)))

    return {
        "planning": {"steer": plan_steer, "speed": plan_speed, "task": wp_state.current_task},
        "perception": {
            "candidates": [
                {"name": c.name, "emergency_stop": c.emergency_stop, "speed_cap": c.speed_cap, "steer": c.steer_override, "reasons": [r.value for r in c.reasons]}
                for c in cands
            ],
        },
        "decision": {
            "emergency_stop": bool(decision.emergency_stop),
            "speed_cap": float(decision.speed_cap),
            "has_steer_override": bool(decision.has_steer_override),
            "steer_override": float(decision.steer_override),
            "reasons": [r.value for r in decision.reasons],
        },
        "controller": {
            "speed_ratio": speed_ratio,
            "steer_ratio": steer_ratio,
            "/cmd_vel": {"linear_x": speed_ratio * 3.0, "angular_z": steer_ratio * 1.0},
        },
    }


def main() -> int:
    _ensure_utf8_stdio()
    print("== E2E: Full-stack conflict scenarios (Light + Obstacle) ==")
    _print_output_key_legend()

    total = 0
    passed = 0

    lane_ok = LaneBounds(left_y_m=1.5, right_y_m=-1.5, margin_m=0.25)

    # Static obstacle that should trigger lane change (<=3m) but not emergency stop (>1.5m).
    static_barrier = [
        ObstacleDetection(
            kind="barrier",
            confidence=0.9,
            bbox_px=(0, 0, 1, 1),
            estimated_distance_m=2.5,
            estimated_lateral_m=0.6,  # obstacle on right -> avoid to right (per _avoidance_direction)
        )
    ]

    # 1) RED + static avoid => emergency stop dominates, steer override disabled
    out = _run_step(
        lane=lane_ok,
        # CONFIRM_FRAMES=2 -> same light must be seen at least twice
        light_dets=[LightDetection(color=LightColor.RED, confidence=0.95, bbox_px=(0, 0, 1, 1), estimated_distance_m=15.0)] * 2,
        obs_dets=static_barrier,
        now=time.monotonic(),
    )
    total += 1
    passed += _case(
        name="RED ışık + statik engel aynı anda",
        desc="Kırmızı ışık varken engel kaçınma önerse bile emergency_stop baskın olmalı; direksiyon override kapatılmalı.",
        expected={
            "emergency_stop": True,
            "speed_cap": 0.0,
            "has_steer_override": False,
            "reasons": lambda rs: _contains_reason(ReasonCode.LIGHT_MUST_STOP)(rs)
            and _contains_reason(ReasonCode.STATIC_AVOID)(rs),
        },
        actual=out["decision"],
    )

    # 2) YELLOW + static avoid + lane ok => slow + steer override
    out = _run_step(
        lane=lane_ok,
        light_dets=[LightDetection(color=LightColor.YELLOW, confidence=0.95, bbox_px=(0, 0, 1, 1), estimated_distance_m=20.0)] * 2,
        obs_dets=static_barrier,
        now=time.monotonic(),
    )
    total += 1
    passed += _case(
        name="SARI ışık + statik engel aynı anda (lane var)",
        desc="Sarı ışık yavaşlatır; statik engel kaçınma direksiyon override verebilir; hız tavanı min olmalı.",
        expected={
            "emergency_stop": False,
            "speed_cap": lambda v: 0.0 < float(v) <= 0.35 + 1e-6,
            "has_steer_override": True,
            "steer_override": lambda v: abs(float(v)) > 0.05,
            "reasons": _contains_reason(ReasonCode.LIGHT_YELLOW_SLOW),
        },
        actual=out["decision"],
    )

    # 3) YELLOW + static avoid + lane missing => steer override disabled (safety)
    out = _run_step(
        lane=None,
        light_dets=[LightDetection(color=LightColor.YELLOW, confidence=0.95, bbox_px=(0, 0, 1, 1), estimated_distance_m=20.0)] * 2,
        obs_dets=static_barrier,
        now=time.monotonic(),
    )
    total += 1
    passed += _case(
        name="SARI ışık + statik engel aynı anda (lane yok)",
        desc="Şerit sınırı yokken statik kaçınma direksiyon override'ı iptal edilmeli (lane_required_for_avoidance).",
        expected={
            "emergency_stop": False,
            "has_steer_override": False,
            "reasons": lambda rs: _contains_reason(ReasonCode.LANE_MISSING_AVOIDANCE_DISABLED)(rs)
            and _contains_reason(ReasonCode.STATIC_AVOID)(rs),
        },
        actual=out["decision"],
    )

    # 4) GREEN (no cap) + road blocked (2 barriers) => emergency stop (ROAD_BLOCKED)
    blocked = static_barrier + [
        ObstacleDetection(
            kind="barrier",
            confidence=0.9,
            bbox_px=(0, 0, 1, 1),
            estimated_distance_m=2.7,
            estimated_lateral_m=-0.6,
        )
    ]
    out = _run_step(
        lane=lane_ok,
        light_dets=[LightDetection(color=LightColor.GREEN, confidence=0.95, bbox_px=(0, 0, 1, 1), estimated_distance_m=18.0)] * 2,
        obs_dets=blocked,
        now=time.monotonic(),
    )
    total += 1
    passed += _case(
        name="YEŞİL ışık + yol kapalı (2 bariyer)",
        desc="Yol kapalı tespitinde emergency_stop ve ROAD_BLOCKED gerekçesi gelmeli.",
        expected={
            "emergency_stop": True,
            "speed_cap": 0.0,
            "reasons": _contains_reason(ReasonCode.ROAD_BLOCKED),
        },
        actual=out["decision"],
    )

    print(f"\n== SONUÇ: {passed}/{total} geçti ==")
    return 0 if passed == total else 2


if __name__ == "__main__":
    raise SystemExit(main())

