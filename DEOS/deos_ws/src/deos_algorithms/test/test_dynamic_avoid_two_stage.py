import sys
from pathlib import Path


_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_dynamic_obstacle_two_stage_stop_then_avoid():
    from deos_algorithms.obstacle_logic import (
        DYNAMIC_AVOID_HOLD_S,
        ObstacleDetection,
        ObstacleLogic,
    )

    logic = ObstacleLogic()
    dets = [
        ObstacleDetection(
            kind="pedestrian",
            confidence=0.9,
            bbox_px=(0, 0, 10, 10),
            estimated_distance_m=4.0,
            estimated_lateral_m=0.6,
        )
    ]

    # Phase 1: confirm + start hold (synthetic time, tight ticks)
    t0 = 1000.0
    st = None
    for i in range(10):
        st = logic.update(dets, now=t0 + i * 0.01)
    assert st is not None
    assert st.waiting_for_dynamic_obstacle is True
    assert st.speed_cap_ratio == 0.0

    # Phase 2: advance time past DYNAMIC_AVOID_HOLD_S → dynamic avoid should activate
    t1 = t0 + 9 * 0.01 + DYNAMIC_AVOID_HOLD_S + 0.1
    for i in range(10):
        st = logic.update(dets, now=t1 + i * 0.01)
    assert st is not None
    assert st.dynamic_avoid_active is True
    assert st.dynamic_avoidance_direction in {"left", "right"}

