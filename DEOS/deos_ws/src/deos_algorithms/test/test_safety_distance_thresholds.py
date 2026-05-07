import sys
from pathlib import Path


_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_safety_logic_distance_thresholds_barrier():
    from deos_algorithms.safety_logic import (
        DIST_EMERGENCY_STOP,
        DIST_HARD_SLOWDOWN,
        DIST_SOFT_SLOWDOWN,
        Detection,
        SafetyLogic,
        ThreatLevel,
    )

    s = SafetyLogic()

    # Barrier caution=1.0 so effective_distance == distance.
    def det_at(dist_m: float) -> Detection:
        return Detection(
            x1=0,
            y1=0,
            x2=10,
            y2=10,
            class_name="barrier",
            confidence=0.9,
            estimated_distance_m=float(dist_m),
            estimated_lateral_m=0.0,
        )

    # Confirm frames required
    for _ in range(3):
        d = s.decide([det_at(DIST_SOFT_SLOWDOWN - 0.1)])
    assert d.threat_level == ThreatLevel.SOFT_SLOW
    assert d.emergency_stop is False
    assert d.speed_cap_ratio == 0.8

    for _ in range(3):
        d = s.decide([det_at(DIST_HARD_SLOWDOWN - 0.1)])
    assert d.threat_level == ThreatLevel.HARD_SLOW
    assert d.emergency_stop is False
    assert d.speed_cap_ratio == 0.5

    for _ in range(3):
        d = s.decide([det_at(DIST_EMERGENCY_STOP - 0.1)])
    assert d.threat_level == ThreatLevel.EMERGENCY
    assert d.emergency_stop is True
    assert d.speed_cap_ratio == 0.0

