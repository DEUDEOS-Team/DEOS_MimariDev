import sys
from pathlib import Path


_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_dynamic_obstacle_two_stage_stop_then_avoid():
    from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleLogic

    logic = ObstacleLogic()
    dets = [
        ObstacleDetection(
            kind="pedestrian",
            confidence=0.9,
            bbox_px=(0, 0, 10, 10),
            estimated_distance_m=4.0,
            estimated_lateral_m=0.6,  # obstacle on left (pos=left in safety_logic)
        )
    ]

    # Need confirmation; run multiple updates
    st = None
    for _ in range(10):
        st = logic.update(dets)
    assert st is not None
    assert st.waiting_for_dynamic_obstacle is True
    assert st.speed_cap_ratio == 0.0

    # After stop-hold ticks, dynamic avoid should become active
    for _ in range(10):
        st = logic.update(dets)
    assert st is not None
    assert st.dynamic_avoid_active is True
    assert st.dynamic_avoidance_direction in {"left", "right"}

