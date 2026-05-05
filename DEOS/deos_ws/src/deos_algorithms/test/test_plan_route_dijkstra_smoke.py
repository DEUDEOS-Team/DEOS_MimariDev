import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_plan_route_dijkstra_import_smoke():
    from tools.plan_route_dijkstra import main

    assert callable(main)

