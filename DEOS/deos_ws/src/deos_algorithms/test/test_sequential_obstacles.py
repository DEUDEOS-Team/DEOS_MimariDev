import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_obstacle_logic_sequential_static_hysteresis():
    """
    Sağ-sol dönüşümlü, aralıklı statik engellerde (bariyer/koni) kaçınma yönü
    her frame'de flip etmemeli (commit/hysteresis).
    """
    from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleLogic

    logic = ObstacleLogic()

    # Engel-1: sağda (lateral +) -> kaçınma "right" (mevcut implementasyonda)
    e1 = ObstacleDetection(kind="barrier", confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=2.5, estimated_lateral_m=0.6)
    # Engel-2: solda (lateral -) -> kaçınma "left"
    e2 = ObstacleDetection(kind="barrier", confidence=0.9, bbox_px=(0, 0, 1, 1), estimated_distance_m=2.4, estimated_lateral_m=-0.6)

    dirs = []
    for i in range(10):
        det = e1 if i % 2 == 0 else e2  # sağ-sol dönüşümlü
        st = logic.update([det])
        if st.suggest_lane_change:
            dirs.append(st.avoidance_direction)

    # commit sayesinde tüm frame'lerde flip beklemiyoruz:
    # en azından ilk birkaç kararın aynı kalması yeterli.
    assert len(dirs) >= 6
    assert len(set(dirs[:6])) == 1, f"İlk commit penceresinde yön sabit kalmalı, görülen: {dirs[:6]}"

