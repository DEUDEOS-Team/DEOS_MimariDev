import sys
from pathlib import Path


_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_dynamic_avoid_not_disabled_when_lane_missing():
    from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, ReasonCode

    arb = DecisionArbiter()
    out = arb.arbitrate(
        candidates=[
            Candidate(
                name="dynamic_avoid",
                emergency_stop=False,
                speed_cap=0.2,
                steer_override=0.25,
                reasons=[ReasonCode.DYNAMIC_AVOID],
            )
        ],
        lane=None,
        lane_required_for_avoidance=True,
    )
    assert out.emergency_stop is False
    assert out.speed_cap == 0.2
    # Lane yoksa kaçınma yönü (steer override) kapatılmalı; dinamikte "dur/bekle" davranışı
    # zaten obstacle speed_cap ile uygulanır.
    assert out.has_steer_override is False
    assert out.steer_override == 0.0
    assert ReasonCode.DYNAMIC_AVOID in out.reasons

