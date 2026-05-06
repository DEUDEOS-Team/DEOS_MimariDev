import subprocess
import sys
from pathlib import Path


def test_e2e_full_stack_conflicts_smoke():
    # Run the script as a subprocess to validate it exits successfully.
    repo_root = Path(__file__).resolve().parents[1]
    script = repo_root / "tools" / "e2e_full_stack_conflicts.py"
    assert script.exists()

    p = subprocess.run(
        [sys.executable, str(script)],
        cwd=str(repo_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    assert p.returncode == 0, f"stdout:\n{p.stdout}\n\nstderr:\n{p.stderr}"

