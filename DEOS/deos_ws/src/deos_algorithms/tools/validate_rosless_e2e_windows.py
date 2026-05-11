#!/usr/bin/env python3
"""
ROS'suz uçtan uca doğrulama (Windows dostu).

Çalıştırır:
- pytest (deos_algorithms)
- e2e_race_scenarios.py (PASS/FAIL)
- e2e_full_stack_visual_test.py (overlay.png + report.json)

Amaç: Windows'ta tek komutla "uçtan uca" doğrulama.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def _run(cmd: list[str], *, cwd: Path) -> int:
    print("\n$", " ".join(cmd))
    p = subprocess.run(cmd, cwd=str(cwd), env=os.environ.copy())
    return int(p.returncode)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="", help="Output directory for artifacts (optional)")
    ap.add_argument("--skip-pytest", action="store_true", help="Skip pytest")
    ap.add_argument("--skip-visual", action="store_true", help="Skip visual overlay test")
    args = ap.parse_args(argv)

    pkg_root = Path(__file__).resolve().parents[1]  # .../src/deos_algorithms
    tools_dir = pkg_root / "tools"

    out_dir = Path(args.out).resolve() if args.out else (pkg_root / "tools" / "_rosless_validate_out").resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    rc = 0

    if not bool(args.skip_pytest):
        rc |= _run([sys.executable, "-m", "pytest", "-q"], cwd=pkg_root)

    rc |= _run([sys.executable, str(tools_dir / "e2e_race_scenarios.py")], cwd=pkg_root)

    if not bool(args.skip_visual):
        rc |= _run(
            [
                sys.executable,
                str(tools_dir / "e2e_full_stack_visual_test.py"),
                "--out",
                str(out_dir),
            ],
            cwd=pkg_root,
        )

    print("\n== Özet ==")
    print(f"artifacts_dir: {out_dir}")
    print("status:", "PASS" if rc == 0 else "FAIL")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())

