import sys
from pathlib import Path


_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_lane_contains_lateral_with_margin():
    from deos_algorithms.decision_arbiter import LaneBounds, lane_contains_lateral

    lane = LaneBounds(left_y_m=1.0, right_y_m=-1.0, margin_m=0.25)
    assert lane_contains_lateral(lane, 0.0) is True
    assert lane_contains_lateral(lane, 0.74) is True
    assert lane_contains_lateral(lane, 0.76) is False  # outside after margin
    assert lane_contains_lateral(lane, -0.74) is True
    assert lane_contains_lateral(lane, -0.76) is False

