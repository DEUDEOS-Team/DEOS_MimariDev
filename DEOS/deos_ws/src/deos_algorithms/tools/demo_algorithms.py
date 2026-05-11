#!/usr/bin/env python3
"""
Terminal demo (mimariyi bozmadan):
- `deos_algorithms` içindeki modülleri örnek girdilerle çalıştırır
- çıktıları mümkün olduğunca **Türkçe ve sade** anlatımla yazdırır

Bu dosya birim test değildir; hızlı “gözle kontrol” içindir.
Birim testler için: `python -m pytest -vv`

Çalıştırma:
  cd DEOS/deos_ws/src/deos_algorithms
  python tools/demo_algorithms.py
"""

from __future__ import annotations

import sys
from pathlib import Path
from dataclasses import asdict

# Allow running without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.obstacle_logic import ObstacleDetection, ObstacleLogic
from deos_algorithms.safety_logic import Detection, SafetyLogic


def _ensure_utf8_stdio() -> None:
    """Windows konsolda Türkçe karakterlerin bozulmaması için (algoritma davranışını değiştirmez)."""
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if stream is not None and hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass


def demo_obstacle_logic() -> None:
    logic = ObstacleLogic()

    scenarios: list[tuple[str, str, list[ObstacleDetection]]] = [
        (
            "TR-1: Engel yok",
            "engel_yok",
            [],
        ),
        (
            "TR-2: Uzak koni (tek kare)",
            "uzak_koni_tek_kare",
            [
                ObstacleDetection(
                    kind="cone",
                    confidence=0.9,
                    bbox_px=(600, 300, 650, 600),
                    estimated_distance_m=8.0,
                    estimated_lateral_m=0.2,
                )
            ],
        ),
        (
            "TR-3: Yakın yaya (tek kare) — onay kareleri yoksa sonuç 'clear' kalabilir",
            "yakin_yaya_tek_kare",
            [
                ObstacleDetection(
                    kind="pedestrian",
                    confidence=0.95,
                    bbox_px=(610, 250, 700, 700),
                    estimated_distance_m=4.0,
                    estimated_lateral_m=0.0,
                )
            ],
        ),
    ]

    print("== ObstacleLogic demo (Türkçe senaryolar) ==")
    print("Not: Bu demo aynı ObstacleLogic örneğini ardışık senaryolarda kullanır; önceki senaryonun iç durumu sonrakini etkileyebilir.")
    for title, slug, dets in scenarios:
        state = logic.update(dets)
        print(f"\n-- {title} --")
        print(f"(kisa_kod={slug})")
        for k, v in asdict(state).items():
            print(f"{k}: {v}")


def demo_safety_tracker_confirmation() -> None:
    logic = SafetyLogic()

    print("\n== SafetyLogic demo: 'birkaç kare üst üste görme' (confirmation) ==\n")
    print("Aynı yaya kutusu art arda veriliyor. Beklenti: ilk karelerde 'clear', yeterli kareden sonra yavaşlama.")
    # Aynı tespiti art arda verip CONFIRM_FRAMES davranışını gösterir
    det = Detection(
        x1=600,
        y1=300,
        x2=680,
        y2=700,
        class_name="pedestrian",
        confidence=0.95,
        estimated_distance_m=6.0,
        estimated_lateral_m=0.0,
    )
    for frame in range(1, 6):
        analysis = logic.analyze([det])
        d = analysis.decision
        print(
            f"kare={frame} "
            f"acil_dur={d.emergency_stop} "
            f"hiz_tavan={d.speed_cap_ratio:.2f} "
            f"tehdit={d.threat_level.name} "
            f"aciklama={d.reason}"
        )


def main() -> None:
    _ensure_utf8_stdio()
    demo_obstacle_logic()
    demo_safety_tracker_confirmation()


if __name__ == "__main__":
    main()

