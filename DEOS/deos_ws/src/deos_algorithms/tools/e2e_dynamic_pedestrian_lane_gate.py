"""
ROS-less E2E: Dynamic pedestrian behavior lane gating

Goal:
- Pedestrian seen -> vehicle must stop/wait (speed_cap=0) regardless of lane data.
- Dynamic avoidance steer override only if lane bounds are available (lane_fresh).
"""

from __future__ import annotations

import json
import sys
import time

from pathlib import Path

_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, LaneBounds, ReasonCode
from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleLogic


def _contains_reason(code: ReasonCode):
    def _pred(rs):
        return any(str(r) == code.value for r in rs)

    return _pred


def _case(*, name: str, expected: dict, actual: dict) -> bool:
    ok = True
    for k, v in expected.items():
        if callable(v):
            ok = ok and bool(v(actual.get(k)))
        else:
            ok = ok and (actual.get(k) == v)
    status = "GEÇTİ" if ok else "KALDI"
    exp_print = {k: ("<koşul>" if callable(v) else v) for k, v in expected.items()}
    print(f"\n[{status}] {name}")
    print(f"  beklenen : {json.dumps(exp_print, ensure_ascii=False)}")
    print(f"  çıkan    : {json.dumps(actual, ensure_ascii=False)}")
    return ok


def _run(ped_lat_m: float, lane: LaneBounds | None) -> dict:
    obs = ObstacleLogic()
    arb = DecisionArbiter()

    dets = [
        ObstacleDetection(
            kind="pedestrian",
            confidence=0.95,
            bbox_px=(0, 0, 10, 10),
            estimated_distance_m=4.0,  # within dynamic stop distance
            estimated_lateral_m=float(ped_lat_m),
        )
    ]

    # Confirm + wait + pass hold ticks to allow dynamic_avoid_active
    st = None
    for _ in range(25):
        st = obs.update(dets)
    assert st is not None

    cands: list[Candidate] = []
    # Pedestrian must force stop/wait (speed_cap=0 via obstacle speed cap)
    if float(st.speed_cap_ratio) < 1.0:
        cands.append(Candidate(name="obstacle", emergency_stop=False, speed_cap=float(st.speed_cap_ratio)))

    # Only add dynamic_avoid candidate if lane exists/valid (mirrors perception_fusion_node rule)
    if lane is not None and lane.is_valid and bool(st.dynamic_avoid_active) and st.dynamic_avoidance_direction in {"left", "right"}:
        steer_bias = -0.25 if st.dynamic_avoidance_direction == "left" else 0.25
        cands.append(
            Candidate(
                name="dynamic_avoid",
                emergency_stop=False,
                speed_cap=0.2,
                steer_override=float(steer_bias),
                reasons=[ReasonCode.DYNAMIC_AVOID],
            )
        )

    dec = arb.arbitrate(candidates=cands, lane=lane, lane_required_for_avoidance=True)
    return {
        "decision": {
            "emergency_stop": bool(dec.emergency_stop),
            "speed_cap": float(dec.speed_cap),
            "has_steer_override": bool(dec.has_steer_override),
            "steer_override": float(dec.steer_override),
            "reasons": [r.value for r in dec.reasons],
        }
    }


def main() -> int:
    print("== E2E: dynamic pedestrian lane gate ==")

    total = 0
    passed = 0

    # Case A: lane missing -> stop/wait, NO steer override
    out = _run(ped_lat_m=0.6, lane=None)
    total += 1
    passed += _case(
        name="Lane yok -> yaya görür görmez dur/bekle, steer yok",
        expected={"emergency_stop": False, "speed_cap": 0.0, "has_steer_override": False},
        actual=out["decision"],
    )

    # Case B: lane available -> stop/wait + dynamic avoid steer override allowed (after hold)
    lane_ok = LaneBounds(left_y_m=1.2, right_y_m=-1.2, margin_m=0.25)
    out = _run(ped_lat_m=0.6, lane=lane_ok)
    total += 1
    passed += _case(
        name="Lane var -> dur/bekle + steer override (dinamik kaçınma)",
        expected={
            "emergency_stop": False,
            "speed_cap": 0.0,  # stop dominates (min speed cap)
            "has_steer_override": True,
            "reasons": _contains_reason(ReasonCode.DYNAMIC_AVOID),
        },
        actual=out["decision"],
    )

    print(f"\n== SONUÇ: {passed}/{total} geçti ==")
    return 0 if passed == total else 2


if __name__ == "__main__":
    raise SystemExit(main())

