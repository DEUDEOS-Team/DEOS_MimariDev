#!/usr/bin/env python3
"""
ROS'suz uçtan uca 3'lü çatışma senaryoları (Işık + Engel + Levha).

Amaç:
- Aynı anda ışık+engel+levha varken DecisionArbiter'ın öncelik tablosuna göre
  deterministik karar vermesini PASS/FAIL ile doğrulamak.

Çalıştırma:
  cd DEOS/deos_ws/src/deos_algorithms
  python tools/e2e_full_stack_conflicts_3way.py
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable

_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, LaneBounds, ReasonCode  # noqa: E402
from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleLogic  # noqa: E402
from deos_algorithms.traffic_light_logic import LightColor, LightDetection, TrafficLightLogic  # noqa: E402
from deos_algorithms.traffic_sign_logic import SignClass, SignDetection, TrafficSignLogic  # noqa: E402


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
    """

    print("\n-- ÇIKTI ALANLARI SÖZLÜĞÜ --")
    print(" emergency_stop     : Acil dur kararı (hız 0)")
    print(" speed_cap          : Hız üst limiti oranı [0..1] (min kuralı)")
    print(" has_steer_override : Steer override aktif mi")
    print(" steer_override     : Normalize steer [-1..1] (lane clamp/disable olabilir)")
    print(" reasons            : Karar gerekçeleri (ReasonCode)")


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
    print("  beklenen :", json.dumps({k: ("<koşul>" if callable(v) else v) for k, v in expected.items()}, ensure_ascii=False))
    print("  çıkan    :", json.dumps(_normalize(actual), ensure_ascii=False))
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
        if not isinstance(v, list):
            return False
        return want in [str(x) for x in v]

    return fn


def _arbitrate(
    *,
    lane: LaneBounds | None,
    light_dets: list[LightDetection],
    sign_dets: list[SignDetection],
    obs_dets: list[ObstacleDetection],
    now: float,
) -> dict[str, Any]:
    light = TrafficLightLogic()
    sign = TrafficSignLogic()
    obstacle = ObstacleLogic()
    arb = DecisionArbiter()

    # confirm windows
    light_state = light.update(light_dets, now=now)
    sign_state = sign.update(sign_dets, now=now)
    obs_state = None
    for _ in range(3):  # safety CONFIRM_FRAMES=3
        obs_state = obstacle.update(obs_dets)
    assert obs_state is not None

    cands: list[Candidate] = []

    if light_state.must_stop:
        cands.append(Candidate(name="light", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.LIGHT_MUST_STOP]))
    elif float(light_state.speed_cap_ratio) < 1.0:
        cands.append(Candidate(name="light", emergency_stop=False, speed_cap=float(light_state.speed_cap_ratio), reasons=[ReasonCode.LIGHT_YELLOW_SLOW]))

    if sign_state.must_stop_soon:
        cands.append(Candidate(name="sign", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.SIGN_MUST_STOP]))
    elif float(sign_state.speed_cap_ratio) < 1.0:
        cands.append(Candidate(name="sign", emergency_stop=False, speed_cap=float(sign_state.speed_cap_ratio), reasons=[ReasonCode.SIGN_SPEED_CAP]))

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
    return {
        "decision": {
            "emergency_stop": bool(decision.emergency_stop),
            "speed_cap": float(decision.speed_cap),
            "has_steer_override": bool(decision.has_steer_override),
            "steer_override": float(decision.steer_override),
            "reasons": [r.value for r in decision.reasons],
        },
        "candidates": [{"name": c.name, "emergency": c.emergency_stop, "speed_cap": c.speed_cap, "steer": c.steer_override, "reasons": [r.value for r in c.reasons]} for c in cands],
    }


def main() -> int:
    _ensure_utf8_stdio()
    print("== E2E: 3-way conflict scenarios (Light + Obstacle + Sign) ==")
    _print_output_key_legend()

    total = 0
    passed = 0
    lane_ok = LaneBounds(left_y_m=1.5, right_y_m=-1.5, margin_m=0.25)

    static_barrier = [
        ObstacleDetection(kind="barrier", confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=2.5, estimated_lateral_m=0.6)
    ]

    # 1) RED + STOP sign + static avoid => emergency stop (red dominates, but reasons include multiple)
    out = _arbitrate(
        lane=lane_ok,
        light_dets=[LightDetection(color=LightColor.RED, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=12.0)] * 2,
        sign_dets=[SignDetection(class_name=SignClass.STOP, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=6.0)] * 3,
        obs_dets=static_barrier,
        now=time.monotonic(),
    )
    total += 1
    passed += _case(
        name="KIRMIZI + DUR + statik engel",
        desc="Kırmızı ışık varken her şey olsa bile emergency_stop ve speed_cap=0 olmalı.",
        expected={
            "emergency_stop": True,
            "speed_cap": 0.0,
            "reasons": lambda rs: _contains_reason(ReasonCode.LIGHT_MUST_STOP)(rs) and _contains_reason(ReasonCode.SIGN_MUST_STOP)(rs),
        },
        actual=out["decision"],
    )

    # 2) YELLOW + STOP sign + static avoid => emergency stop (STOP must_stop_soon dominates)
    out = _arbitrate(
        lane=lane_ok,
        light_dets=[LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=18.0)] * 2,
        sign_dets=[SignDetection(class_name=SignClass.STOP, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=5.0)] * 3,
        obs_dets=static_barrier,
        now=time.monotonic(),
    )
    total += 1
    passed += _case(
        name="SARI + DUR + statik engel",
        desc="Dur tabelası must_stop_soon ürettiğinde emergency_stop True olmalı (arbiter).",
        expected={
            "emergency_stop": True,
            "speed_cap": 0.0,
            "reasons": _contains_reason(ReasonCode.SIGN_MUST_STOP),
        },
        actual=out["decision"],
    )

    # 3) YELLOW + speed-cap sign (crosswalk) + static avoid => speed cap is min, steer override allowed
    out = _arbitrate(
        lane=lane_ok,
        light_dets=[LightDetection(color=LightColor.YELLOW, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=25.0)] * 2,
        sign_dets=[SignDetection(class_name=SignClass.PEDESTRIAN_CROSSING, confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=8.0)] * 3,
        obs_dets=static_barrier,
        now=time.monotonic(),
    )
    total += 1
    passed += _case(
        name="SARI + yaya_gecidi + statik engel",
        desc="Sarı yavaşlatır; yaya geçidi de hız kısıtı getirir; statik kaçınma steer override verebilir.",
        expected={
            "emergency_stop": False,
            "speed_cap": lambda v: 0.0 < float(v) <= 0.35 + 1e-6,
            "has_steer_override": True,
            "reasons": lambda rs: _contains_reason(ReasonCode.LIGHT_YELLOW_SLOW)(rs) and _contains_reason(ReasonCode.STATIC_AVOID)(rs),
        },
        actual=out["decision"],
    )

    # 4) Conf vs distance separation: near low-conf vs far high-conf should pick near after MIN_CONFIDENCE filtering.
    # Here both pass MIN_CONFIDENCE=0.5; near should dominate (distance-based).
    out = _arbitrate(
        lane=lane_ok,
        light_dets=[LightDetection(color=LightColor.RED, confidence=0.55, bbox_px=(0, 0, 1, 1), estimated_distance_m=6.0)] * 2
        + [LightDetection(color=LightColor.GREEN, confidence=0.95, bbox_px=(0, 0, 1, 1), estimated_distance_m=25.0)] * 2,
        sign_dets=[],
        obs_dets=[],
        now=time.monotonic(),
    )
    total += 1
    passed += _case(
        name="Işık seçiminde yakınlık > doğruluk (ikisi de eşik üstü)",
        desc="Yakın kırmızı (conf düşük ama eşik üstü) varken uzak yeşil olsa bile kırmızı baskın olmalı.",
        expected={
            "emergency_stop": True,
            "speed_cap": 0.0,
            "reasons": _contains_reason(ReasonCode.LIGHT_MUST_STOP),
        },
        actual=out["decision"],
    )

    print(f"\n== SONUÇ: {passed}/{total} geçti ==")
    return 0 if passed == total else 2


if __name__ == "__main__":
    raise SystemExit(main())

