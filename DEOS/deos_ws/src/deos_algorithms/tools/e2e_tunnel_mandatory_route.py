#!/usr/bin/env python3
"""
ROS-less E2E: Mandatory tunnel routing (Ek Mimari / yarış kuralı)

Amaç:
- Centerlines graph üzerinde tunnel:true kenarı varsa, rota en az bir tünel kenarından geçmeli.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.route_graph import Edge, Node, RouteGraph, dijkstra_mandatory_tunnel, edge_is_tunnel  # noqa: E402


def _print_output_key_legend() -> None:
    print("\n-- ÇIKTI ALANLARI SÖZLÜĞÜ --")
    print(" path_nodes : bulunan node id yolu")
    print(" used_tunnel: yol üzerinde en az bir tunnel:true edge var mı")


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
    print("== E2E: mandatory tunnel routing ==")
    _print_output_key_legend()

    # Build a tiny graph:
    # start(0) -> mid(1) -> goal(3) is shortest but non-tunnel
    # start(0) -> tA(2) -> goal(3) includes tunnel edge (0->2)
    nodes = [
        Node(id=0, lat=0.0, lon=0.0),
        Node(id=1, lat=0.0, lon=0.0001),
        Node(id=2, lat=0.0, lon=0.0002),
        Node(id=3, lat=0.0, lon=0.0003),
    ]
    adj = {
        0: [Edge(u=0, v=1, cost=1.0, props={}), Edge(u=0, v=2, cost=5.0, props={"tunnel": True})],
        1: [Edge(u=1, v=3, cost=1.0, props={})],
        2: [Edge(u=2, v=3, cost=1.0, props={})],
        3: [],
    }
    g = RouteGraph(nodes=nodes, adj=adj)

    path = dijkstra_mandatory_tunnel(g, start=0, goal=3, blocked_edges=set(), edge_cost_multiplier=None)

    used_tunnel = False
    for u, v in zip(path, path[1:], strict=False):
        for e in g.adj.get(int(u), []):
            if int(e.v) == int(v) and edge_is_tunnel(e):
                used_tunnel = True
                break
        if used_tunnel:
            break

    actual = {"path_nodes": path, "used_tunnel": used_tunnel}
    total = 1
    passed = 0
    passed += _case(
        name="Tünel zorunlu rota tünel edge içerir",
        desc="Non-tunnel daha kısa olsa bile, tunnel:true edge varken rota tünelden geçmeli.",
        expected={"used_tunnel": True},
        actual=actual,
        actions_if_pass=["Global route planner, centerlines'taki tünel segmentinden geçişi garanti eder."],
    )

    print(f"\n== SONUÇ: {passed}/{total} geçti ==")
    return 0 if passed == total else 2


if __name__ == "__main__":
    raise SystemExit(main())

