from __future__ import annotations

import re
from pathlib import Path

from deos_algorithms.ros_topic_layout import build_deos_topics


def _parse_cpp_topic_layout(header_text: str, root: str) -> dict[str, str]:
    r = root.strip().rstrip("/")
    out: dict[str, str] = {}

    for m in re.finditer(r"\.([a-zA-Z0-9_]+)\s*=\s*j\(\{([^}]*)\}\)", header_text):
        segs = re.findall(r'"([^"]+)"', m.group(2))
        out[m.group(1)] = "/".join([r, *segs])

    for m in re.finditer(r'\.([a-zA-Z0-9_]+)\s*=\s*"([^"]+)"', header_text):
        out[m.group(1)] = m.group(2)

    map_keys = set(re.findall(r'\{"([a-zA-Z0-9_]+)",\s*[a-zA-Z0-9_]+\}', header_text))
    assert map_keys == set(out.keys()), (
        f"TopicLayout::as_map() anahtar uyumsuzluğu: {set(out.keys()) ^ map_keys}"
    )
    return out


def test_python_and_cpp_topic_layouts_are_in_sync() -> None:
    src_dir = Path(__file__).resolve().parents[2]
    cpp_header = (
        src_dir / "deos_cpp_common" / "include" / "deos_cpp_common" / "topic_layout.hpp"
    )
    assert cpp_header.is_file(), f"C++ topic_layout.hpp bulunamadı: {cpp_header}"

    header_text = cpp_header.read_text(encoding="utf-8")
    for root in ("/deos", "/arac1/", " /test_root/// "):
        assert _parse_cpp_topic_layout(header_text, root) == build_deos_topics(root)
