import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_e2e_system_flow_trace_import_smoke():
    from tools.e2e_system_flow_trace import main

    assert callable(main)

