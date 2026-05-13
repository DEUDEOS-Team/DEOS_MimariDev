#!/usr/bin/env python3
"""
ROS-less E2E: Rule constrained routing via blocked edges

Amaç:
- Trafik kuralı/levha kaynaklı edge-block uygulandığında alternatif yol bulunuyor mu?
- Edge-block yüzünden rota yoksa (boş plan), üst katmanın "son iyi rota"yı koruması gerektiği mesajını netleştir.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.route_graph import Edge, Node, RouteGraph, dijkstra  # noqa: E402


def _print_output_key_legend() -> None:
    print("\n-- ÇIKTI ALANLARI SÖZLÜĞÜ --")
    print(" path_nodes   : bulunan node id yolu (boş ise rota yok)")
    print(" blocked_edges: uygulanan yönlü edge blokları")


def _case(*, name: str, desc: str, expected: dict[str, Any], actual: dict[str, Any], actions_if_pass: list[str] | None = None) -> bool:
    ok = True
    mism: dict[str, tuple[Any, Any]] = {}
    for k, ev in expected.items():
        av = actual.get(k)
        if callable(ev):
            try:
                passed = bool(ev(av))
            except Exception:
                passed = False
            if not passed:
                ok = False
                mism[k] = ("<koşul>", av)
        else:
            if av != ev:
                ok = False
                mism[k] = (ev, av)
    status = "GEÇTİ" if ok else "KALDI"
    print(f"\n[{status}] {name}")
    print(f"  senaryo  : {desc}")
    print("  beklenen :", json.dumps({k: ("<koşul>" if callable(v) else v) for k, v in expected.items()}, ensure_ascii=False))
    print("  çıkan    :", json.dumps(actual, ensure_ascii=False))
    if ok and actions_if_pass:
        print("  eylemler :")
        for a in actions_if_pass:
            print(f"   - {a}")
    if mism:
        print("  farklar  :")
        for k, (ev, av) in mism.items():
            print(f"   - {k}: beklenen={ev} çıkan={av}")
    return ok


def main() -> int:
    print("== E2E: rule constrained routing (blocked edges) ==")
    _print_output_key_legend()

    # Graph:
    # 0 -> 1 -> 3 is default
    # 0 -> 2 -> 3 is alternative
    nodes = [Node(id=i, lat=0.0, lon=float(i)) for i in range(4)]
    adj = {
        0: [Edge(u=0, v=1, cost=1.0, props={}), Edge(u=0, v=2, cost=2.0, props={})],
        1: [Edge(u=1, v=3, cost=1.0, props={})],
        2: [Edge(u=2, v=3, cost=1.0, props={})],
        3: [],
    }
    g = RouteGraph(nodes=nodes, adj=adj)

    total = 0
    passed = 0

    # 1) No blocks -> shortest path 0-1-3
    p = dijkstra(g, start=0, goal=3, blocked_edges=set(), edge_cost_multiplier=None)
    total += 1
    passed += _case(
        name="Blok yok -> default en kısa yol",
        desc="Kısıt yokken klasik en kısa yol seçilir.",
        expected={"path_nodes": [0, 1, 3]},
        actual={"path_nodes": p, "blocked_edges": []},
        actions_if_pass=["Normal şartta en kısa rota seçilir."],
    )

    # 2) Block 0->1 -> route should use alternative 0-2-3
    blocks = {(0, 1)}
    p = dijkstra(g, start=0, goal=3, blocked_edges=blocks, edge_cost_multiplier=None)
    total += 1
    passed += _case(
        name="Kural blok (0->1) -> alternatif rota",
        desc="Levha/kural 0->1 dönüşünü yasakladıysa planner alternatif bacağı bulmalı.",
        expected={"path_nodes": [0, 2, 3]},
        actual={"path_nodes": p, "blocked_edges": sorted(list(blocks))},
        actions_if_pass=["Kural/kısıt geçerli: rota alternatif yola replanned edilir."],
    )

    # 3) Block both -> no route (empty), upper layer should keep last plan
    blocks = {(0, 1), (0, 2)}
    p = dijkstra(g, start=0, goal=3, blocked_edges=blocks, edge_cost_multiplier=None)
    total += 1
    passed += _case(
        name="Tüm çıkışlar blok -> rota yok",
        desc="Kısıtlar yüzünden rota bulunamazsa boş yol döner.",
        expected={"path_nodes": []},
        actual={"path_nodes": p, "blocked_edges": sorted(list(blocks))},
        actions_if_pass=["Üst katman politika: 'son iyi rota' korunmalı (gevşetme yok)."],
    )

    print(f"\n== SONUÇ: {passed}/{total} geçti ==")
    return 0 if passed == total else 2


if __name__ == "__main__":
    raise SystemExit(main())

