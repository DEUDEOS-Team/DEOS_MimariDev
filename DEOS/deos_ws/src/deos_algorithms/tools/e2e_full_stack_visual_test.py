#!/usr/bin/env python3
"""
Tek dosyada “mimariyi görselleştir + raporla” E2E test (ROS'suz).

Bu script şunları yapar:
- maps/map.png ve maps/graph_pixels.json (PNG'den otomatik çıkarılmış pixel-graph) kullanarak
  görev noktaları seçer (start/gorev_*/park_giris benzeri),
  Dijkstra ile rota üretir ve harita üstüne çizer.
- Aynı anda, bu repodaki mimarinin (lokalizasyon + global planlama + lokal planlama)
  detay özetini ve bu koşumun metriklerini JSON olarak çıktı verir.

Not:
- Bu test “görsel doğrulama” içindir. Gerçek yarışta önerilen yöntem:
  QGIS ile `maps/lane_centerlines.geojson` üretmek + centerlines_file ile routing çalıştırmak.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

import cv2
import numpy as np


def _ensure_utf8_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if stream is not None and hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass


@dataclass(frozen=True)
class PixNode:
    id: int
    x: int
    y: int


@dataclass(frozen=True)
class PixEdge:
    u: int
    v: int
    poly: list[tuple[int, int]]
    cost: float


@dataclass
class PixGraph:
    nodes: dict[int, PixNode]
    adj: dict[int, list[PixEdge]]


def _euclid(a: tuple[int, int], b: tuple[int, int]) -> float:
    return float(math.hypot(float(a[0] - b[0]), float(a[1] - b[1])))


def load_pixel_graph(path: str | Path) -> PixGraph:
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    nodes = {int(n["id"]): PixNode(id=int(n["id"]), x=int(n["x"]), y=int(n["y"])) for n in d.get("nodes", [])}
    adj: dict[int, list[PixEdge]] = {nid: [] for nid in nodes}

    for e in d.get("edges", []):
        u = int(e["u"])
        v = int(e["v"])
        poly = [(int(p["x"]), int(p["y"])) for p in e.get("polyline", [])]
        if not poly:
            continue
        cost = 0.0
        for p0, p1 in zip(poly, poly[1:], strict=False):
            cost += _euclid(p0, p1)
        edge_uv = PixEdge(u=u, v=v, poly=poly, cost=float(cost))
        edge_vu = PixEdge(u=v, v=u, poly=list(reversed(poly)), cost=float(cost))
        if u in adj:
            adj[u].append(edge_uv)
        if v in adj:
            adj[v].append(edge_vu)

    return PixGraph(nodes=nodes, adj=adj)


def _find_maps_file(rel: str) -> str:
    """
    tools/ içinden çalıştırıldığında workspace üst dizinlerinde `maps/` klasörünü bulmaya çalışır.
    """
    here = Path(__file__).resolve()
    for p in [here.parent, *here.parents]:
        cand = (p / rel).resolve()
        if cand.exists():
            return str(cand)
    # Son çare: CWD göreli dene
    cand2 = (Path.cwd() / rel).resolve()
    return str(cand2)


def nearest_node(g: PixGraph, *, x: int, y: int) -> int:
    best = -1
    best_d = float("inf")
    for nid, n in g.nodes.items():
        d = _euclid((x, y), (n.x, n.y))
        if d < best_d:
            best_d = d
            best = nid
    if best < 0:
        raise ValueError("graph boş")
    return best


def dijkstra_pix(g: PixGraph, *, start: int, goal: int) -> list[int]:
    if start == goal:
        return [start]
    import heapq

    dist: dict[int, float] = {start: 0.0}
    prev: dict[int, int] = {}
    pq: list[tuple[float, int]] = [(0.0, start)]
    seen: set[int] = set()

    while pq:
        d, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        if u == goal:
            break
        for e in g.adj.get(u, []):
            nd = d + float(e.cost)
            if nd < dist.get(e.v, float("inf")):
                dist[e.v] = nd
                prev[e.v] = u
                heapq.heappush(pq, (nd, e.v))

    if goal not in dist:
        return []

    path: list[int] = [goal]
    cur = goal
    while cur != start:
        cur = prev[cur]
        path.append(cur)
    path.reverse()
    return path


def polyline_for_node_path(g: PixGraph, node_path: Iterable[int]) -> list[tuple[int, int]]:
    ids = list(node_path)
    if len(ids) < 2:
        if ids and ids[0] in g.nodes:
            n = g.nodes[ids[0]]
            return [(n.x, n.y)]
        return []

    out: list[tuple[int, int]] = []
    for u, v in zip(ids, ids[1:], strict=False):
        best: Optional[PixEdge] = None
        for e in g.adj.get(int(u), []):
            if int(e.v) == int(v):
                best = e
                break
        if best is None:
            # fallback: straight segment between node coords
            nu = g.nodes[int(u)]
            nv = g.nodes[int(v)]
            seg = [(nu.x, nu.y), (nv.x, nv.y)]
        else:
            seg = list(best.poly)
        if not out:
            out.extend(seg)
        else:
            out.extend(seg[1:])
    return out


def draw_overlay(
    *,
    img_bgr: np.ndarray,
    g: PixGraph,
    route_poly: list[tuple[int, int]],
    mission_pts: list[dict[str, Any]],
    draw_all_edges: bool,
) -> np.ndarray:
    vis = img_bgr.copy()

    if draw_all_edges:
        for u, edges in g.adj.items():
            _ = u
            for e in edges:
                # draw each undirected twice; cheap and ok for debug
                pts = np.array(e.poly, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(vis, [pts], isClosed=False, color=(0, 180, 255), thickness=1)

    if route_poly:
        pts = np.array(route_poly, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(vis, [pts], isClosed=False, color=(0, 0, 255), thickness=3)

    for mp in mission_pts:
        x, y = int(mp["x"]), int(mp["y"])
        name = str(mp["name"])
        color = (0, 255, 0) if mp["task"] == "start" else ((255, 0, 0) if "park" in mp["task"] else (0, 255, 255))
        cv2.circle(vis, (x, y), 6, color, -1)
        cv2.putText(vis, name, (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, name, (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    return vis


def _architecture_report() -> dict[str, Any]:
    return {
        "localization": {
            "summary": "pcl_localization_ros2, LiDAR map matching (NDT/GICP) ile map→odom TF/pose üretir.",
            "inputs": ["/scan_fullframe veya /cloud_unstructured", "/imu/data (opsiyonel)", "map (PCD/pointcloud)"],
            "outputs": ["TF map→odom", "pose/odometry topic (pakete göre)"],
            "notes": [
                "Lokalizasyon haritası (pointcloud) yol graph değildir; 'neredeyim?' için kullanılır.",
                "Rota graph'ı ayrı bir topoloji verisidir (lane centerlines).",
            ],
        },
        "global_planning": {
            "summary": "mission_planning_node: mission_file (GeoJSON) + (opsiyonel) centerlines_file -> route (Dijkstra) -> WaypointManager/MissionManager.",
            "inputs": ["/gps/fix", "/imu/data", "mission_file", "centerlines_file (opsiyonel)", "/perception/turn_permissions", "/perception/park_complete"],
            "outputs": ["/planning/steering_ref", "/planning/speed_limit", "/planning/current_task", "/planning/arrived", "/planning/park_mode", "/planning/park_remaining_s"],
        },
        "local_planning": {
            "summary": "perception_fusion_node: stereo/lidar/lane_walls -> logic modülleri -> DecisionArbiter ile tek karar.",
            "inputs": ["/perception/stereo_detections", "/perception/lidar_obstacles", "/lane_walls", "/planning/park_mode"],
            "outputs": ["/perception/emergency_stop", "/perception/speed_cap", "/perception/steering_override", "/perception/decision_debug", "/perception/park_complete", "/perception/turn_permissions"],
            "notes": [
                "Şerit dışına çıkmama: lane_walls'tan LaneBounds çıkarılıp static_avoid steer clamp edilir ya da lane yoksa avoidance kapatılır.",
            ],
        },
        "control": {
            "summary": "vehicle_controller_node: planning referansı + perception override/estop + motion_enable -> /cmd_vel.",
            "inputs": ["/planning/*", "/perception/*", "/hardware/motion_enable (opsiyonel)"],
            "outputs": ["/cmd_vel", "/safety/emergency_stop"],
        },
    }


def main() -> int:
    _ensure_utf8_stdio()
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=_find_maps_file("maps/map.png"), help="Input map PNG")
    ap.add_argument("--graph", default=_find_maps_file("maps/graph_pixels.json"), help="Pixel graph JSON")
    ap.add_argument("--out", default=str(Path.cwd() / "full_stack_visual"), help="Output directory")
    ap.add_argument("--draw-all-edges", action="store_true", help="Draw all graph edges (can be busy)")
    ap.add_argument("--show", action="store_true", help="Open the overlay window")
    args = ap.parse_args()

    map_path = Path(args.map)
    graph_path = Path(args.graph)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    img = cv2.imread(str(map_path), cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit(f"map okunamadı: {map_path}")
    g = load_pixel_graph(graph_path)

    # Varsayılan "tur benzeri" 3 nokta seçimi (pixel): start, gorev_1, park_giris
    # Not: İsterseniz bu noktaları arg ile parametrik yapabiliriz.
    picks = [
        {"name": "start", "task": "start", "xy": (110, 95)},
        {"name": "gorev_1", "task": "checkpoint", "xy": (145, 360)},
        {"name": "park_giris", "task": "park_entry", "xy": (640, 170)},
    ]
    mission_pts: list[dict[str, Any]] = []
    snapped_ids: list[int] = []
    for p in picks:
        x, y = int(p["xy"][0]), int(p["xy"][1])
        nid = nearest_node(g, x=x, y=y)
        n = g.nodes[nid]
        mission_pts.append({"name": p["name"], "task": p["task"], "x": n.x, "y": n.y, "node_id": nid})
        snapped_ids.append(nid)

    legs: list[dict[str, Any]] = []
    full: list[int] = []
    ok = True
    for a, b in zip(snapped_ids, snapped_ids[1:], strict=False):
        p = dijkstra_pix(g, start=int(a), goal=int(b))
        legs.append({"from": int(a), "to": int(b), "node_path": p})
        if not p:
            ok = False
            break
        if not full:
            full.extend(p)
        else:
            full.extend(p[1:])

    route_poly = polyline_for_node_path(g, full) if ok else []
    overlay = draw_overlay(img_bgr=img, g=g, route_poly=route_poly, mission_pts=mission_pts, draw_all_edges=bool(args.draw_all_edges))

    overlay_path = out_dir / "overlay.png"
    cv2.imwrite(str(overlay_path), overlay)

    report = {
        "title": "E2E Full Stack Visual Test (ROS'suz)",
        "inputs": {"map_png": str(map_path), "graph_pixels_json": str(graph_path)},
        "graph_stats": {"n_nodes": len(g.nodes), "n_edges": sum(len(v) for v in g.adj.values()) // 2},
        "mission_pixels": mission_pts,
        "routing": {"ok": ok, "legs": legs, "route_node_count": len(full), "route_polyline_points": len(route_poly)},
        "outputs": {"overlay_png": str(overlay_path)},
        "architecture": _architecture_report(),
    }

    (out_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print("== E2E Full Stack Visual Test ==")
    print(json.dumps({"overlay": str(overlay_path), "report": str(out_dir / "report.json")}, ensure_ascii=False))

    if args.show:
        cv2.imshow("full_stack_overlay", overlay)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

