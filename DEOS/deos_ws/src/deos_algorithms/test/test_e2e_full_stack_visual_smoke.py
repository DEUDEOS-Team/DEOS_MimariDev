import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_e2e_full_stack_visual_import_smoke():
    from tools.e2e_full_stack_visual_test import main

    assert callable(main)

