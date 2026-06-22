"""
conftest.py -- terminal cikti formatlama
-----------------------------------------
Her test calistirilmadan once:
  - Modul bolum basligi (bolum degistiginde)
  - Test fonksiyonunun docstring ilk satiri
terminale yazdirilir.

Calistirma: pytest -v -s   (ya da pytest.ini addopts ile otomatik)
"""
from __future__ import annotations

import sys
import pytest

_SECTION_RULES: list[tuple[str, str]] = [
    ("test_imports",      "[ Kurulum ] Paket Ice Aktarma"),
    ("safety_logic",      "[ SafetyLogic ] Guvenlik & Mesafe Bantlari"),
    ("obstacle_logic",    "[ ObstacleLogic ] Dinamik / Statik Engel"),
    ("traffic_sign",      "[ TrafficSignLogic ] Tabela Kurallari"),
    ("traffic_light",     "[ TrafficLightLogic ] Trafik Isigi"),
    ("waypoint",          "[ WaypointManager ] GPS Navigasyon"),
    ("mission_manager",   "[ MissionManager ] Gorev Yonetimi"),
    ("route",             "[ RouteGraph ] Rota & Dijkstra"),
    ("geojson",           "[ GeoJsonMissionReader ] Harita Ayristirma"),
    ("arbiter",           "[ DecisionArbiter ] Karar Birlestirici"),
    ("slalom",            "[ SlalomLogic ] Slalom Manevrasi"),
    ("parking",           "[ ParkingLogic ] Park"),
    ("perception",        "[ PerceptionFusion ] Algi Birlestirme"),
]

_last_section: list[str | None] = [None]


def _safe(text: str) -> str:
    """Terminale yazdirilmadan once encode edilemeyen karakterleri '?' ile degistir."""
    enc = getattr(sys.stdout, "encoding", None) or "utf-8"
    return text.encode(enc, errors="replace").decode(enc)


def _section_for(test_name: str) -> str:
    name = test_name.lower()
    for key, label in _SECTION_RULES:
        if key in name:
            return label
    return "[ Genel ]"


def pytest_runtest_setup(item: pytest.Item) -> None:
    section = _section_for(item.name)
    if section != _last_section[0]:
        _last_section[0] = section
        print(f"\n{'=' * 66}")
        print(f"  {section}")
        print(f"{'=' * 66}")

    doc = (getattr(item.function, "__doc__", None) or "").strip()
    if doc:
        first_line = next((ln.strip() for ln in doc.splitlines() if ln.strip()), "")
        if first_line:
            print(f"  > {_safe(first_line)}")
