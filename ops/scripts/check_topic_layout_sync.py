#!/usr/bin/env python3
"""
Python `ros_topic_layout.py` ile C++ `deos_cpp_common/topic_layout.hpp` arasındaki
topic anahtarlarının ve yol şablonlarının bire bir eşleştiğini doğrular.

Kullanım:
    python3 ops/scripts/check_topic_layout_sync.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PY_LAYOUT_DIR = REPO_ROOT / "DEOS" / "deos_ws" / "src" / "deos_algorithms"
CPP_HEADER = (
    REPO_ROOT
    / "DEOS"
    / "deos_ws"
    / "src"
    / "deos_cpp_common"
    / "include"
    / "deos_cpp_common"
    / "topic_layout.hpp"
)

sys.path.insert(0, str(PY_LAYOUT_DIR))
from deos_algorithms.ros_topic_layout import build_deos_topics  # noqa: E402


def parse_cpp_topic_layout(header_text: str, root: str = "/deos") -> dict[str, str]:
    r = root.strip().rstrip("/")
    out: dict[str, str] = {}

    # .field = j({"seg1", "seg2", ...}) eşleşmeleri
    join_pat = re.compile(r"\.([a-zA-Z0-9_]+)\s*=\s*j\(\{([^}]*)\}\)")
    for m in join_pat.finditer(header_text):
        key = m.group(1)
        segs = re.findall(r'"([^"]+)"', m.group(2))
        out[key] = "/".join([r, *segs])

    # .field = "/literal/path" eşleşmeleri (ör. localization_odom_icp)
    lit_pat = re.compile(r'\.([a-zA-Z0-9_]+)\s*=\s*"([^"]+)"')
    for m in lit_pat.finditer(header_text):
        out[m.group(1)] = m.group(2)

    # as_map() içindeki anahtar kümesini de kontrol et
    map_keys = set(re.findall(r'\{"([a-zA-Z0-9_]+)",\s*[a-zA-Z0-9_]+\}', header_text))
    if map_keys != set(out.keys()):
        missing_in_map = set(out.keys()) - map_keys
        extra_in_map = map_keys - set(out.keys())
        raise AssertionError(
            f"C++ TopicLayout::as_map() uyuşmazlığı: eksik={missing_in_map}, fazla={extra_in_map}"
        )

    return out


def verify_sync() -> None:
    if not CPP_HEADER.is_file():
        raise FileNotFoundError(f"C++ başlık dosyası bulunamadı: {CPP_HEADER}")

    header_text = CPP_HEADER.read_text(encoding="utf-8")
    for test_root in ("/deos", "/arac1/", " /test_root/// "):
        py_topics = build_deos_topics(test_root)
        cpp_topics = parse_cpp_topic_layout(header_text, test_root)

        only_py = sorted(set(py_topics) - set(cpp_topics))
        only_cpp = sorted(set(cpp_topics) - set(py_topics))
        diff_vals = {
            k: (py_topics[k], cpp_topics[k])
            for k in sorted(set(py_topics) & set(cpp_topics))
            if py_topics[k] != cpp_topics[k]
        }

        if only_py or only_cpp or diff_vals:
            lines = [f"Topic şeması uyumsuz (root={test_root!r}):"]
            if only_py:
                lines.append(f"  Sadece Python'da var : {only_py}")
            if only_cpp:
                lines.append(f"  Sadece C++'ta var    : {only_cpp}")
            for k, (pv, cv) in diff_vals.items():
                lines.append(f"  Değer farkı [{k}]: py={pv!r} != cpp={cv!r}")
            raise AssertionError("\n".join(lines))


def main() -> int:
    try:
        verify_sync()
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1
    count = len(build_deos_topics("/deos"))
    print(f"[OK] Python <-> C++ topic şeması tam uyumlu ({count} topic).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
