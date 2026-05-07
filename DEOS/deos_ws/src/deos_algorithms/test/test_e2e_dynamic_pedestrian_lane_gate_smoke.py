import subprocess
import sys
from pathlib import Path


def test_e2e_dynamic_pedestrian_lane_gate_smoke():
    root = Path(__file__).resolve().parents[1]
    script = root / "tools" / "e2e_dynamic_pedestrian_lane_gate.py"
    p = subprocess.run(
        [sys.executable, str(script)],
        cwd=str(root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    assert p.returncode == 0, p.stdout + "\n" + p.stderr

