import subprocess
import sys
from pathlib import Path


def test_e2e_rule_constrained_routing_block_smoke():
    root = Path(__file__).resolve().parents[1]
    script = root / "tools" / "e2e_rule_constrained_routing_block.py"
    p = subprocess.run(
        [sys.executable, str(script)],
        cwd=str(root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    assert p.returncode == 0, p.stdout + "\n" + p.stderr

