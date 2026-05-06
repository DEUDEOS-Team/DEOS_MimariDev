import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_wheels_outside_lane_basic():
    from deos_algorithms.decision_arbiter import LaneBounds
    from deos_algorithms.lane_violation import wheels_outside_lane

    lane = LaneBounds(left_y_m=1.5, right_y_m=-1.5, margin_m=0.25)
    assert wheels_outside_lane(lane=lane, half_width_m=0.7) is False
    assert wheels_outside_lane(lane=lane, half_width_m=2.0) is True


def test_lane_violation_tracker_buckets():
    from deos_algorithms.lane_violation import LaneViolationTracker

    tr = LaneViolationTracker(bucket_s=10.0)
    tr.update(now_s=0.0, outside=True, speed_mps=None)
    assert tr.violation_count == 0
    tr.update(now_s=9.9, outside=True, speed_mps=None)
    assert tr.violation_count == 0
    tr.update(now_s=10.1, outside=True, speed_mps=None)
    assert tr.violation_count == 1
    tr.update(now_s=20.2, outside=True, speed_mps=None)
    assert tr.violation_count >= 2

