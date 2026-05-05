#!/usr/bin/env python3
"""
Global rota üretimi (Dijkstra) — ROS'suz yardımcı araç.

Girdi:
- lane centerlines: QGIS ile çizilmiş LineString GeoJSON (`maps/lane_centerlines.geojson`)
- görev dosyası: şartname formatındaki Point GeoJSON (start / gorev_* / park_giris)

Çıktı:
- Rota: graph node koordinatları (lat/lon) + segment bazlı özet
- İsterseniz `--out` ile dosyaya yazdırır.

Örnek:
  cd DEOS/deos_ws/src/deos_algorithms
  python tools/plan_route_dijkstra.py --centerlines ../../../../maps/lane_centerlines.geojson --mission <mission>.geojson
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

# Allow running without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader  # noqa: E402
from deos_algorithms.route_graph import (  # noqa: E402
    build_graph_from_centerlines_geojson,
    dijkstra,
    load_centerlines_geojson,
    nearest_node_id,
    path_coords,
)


def _ensure_utf8_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if stream is not None and hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass


def _load_mission_points(path: str) -> list[dict[str, Any]]:
    plan = GeoJsonMissionReader().read_file(path)
    out: list[dict[str, Any]] = []
    for p in plan.points:
        out.append(
            {
                "name": p.name,
                "task": p.task,
                "lat": p.lat,
                "lon": p.lon,
                "heading_deg": p.heading_deg,
            }
        )
    return out


def main(argv: list[str] | None = None) -> int:
    _ensure_utf8_stdio()

    ap = argparse.ArgumentParser()
    ap.add_argument("--centerlines", required=True, help="LineString GeoJSON (lane centerlines) path")
    ap.add_argument("--mission", required=True, help="Mission GeoJSON path (start/gorev_*/park_giris)")
    ap.add_argument("--round", type=int, default=7, help="Node coordinate rounding decimals (default: 7)")
    ap.add_argument("--out", default="", help="Output JSON path (optional). Empty -> stdout only")
    args = ap.parse_args(argv)

    if not Path(args.centerlines).exists():
        raise SystemExit(f"centerlines bulunamadı: {args.centerlines}")
    if not Path(args.mission).exists():
        raise SystemExit(f"mission bulunamadı: {args.mission}")

    center_geo = load_centerlines_geojson(args.centerlines)
    g = build_graph_from_centerlines_geojson(center_geo, coord_round_decimals=int(args.round))

    mission_pts = _load_mission_points(args.mission)
    if not mission_pts:
        raise SystemExit("mission boş")

    # Snap each mission point to nearest graph node
    snapped: list[dict[str, Any]] = []
    for mp in mission_pts:
        nid = nearest_node_id(g, lat=float(mp["lat"]), lon=float(mp["lon"]))
        snapped.append({**mp, "node_id": int(nid)})

    # Route as sequential legs (mission order)
    legs: list[dict[str, Any]] = []
    full_node_path: list[int] = []
    for a, b in zip(snapped, snapped[1:], strict=False):
        p = dijkstra(g, start=int(a["node_id"]), goal=int(b["node_id"]))
        legs.append(
            {
                "from": {"name": a["name"], "task": a["task"], "node_id": a["node_id"]},
                "to": {"name": b["name"], "task": b["task"], "node_id": b["node_id"]},
                "node_path": p,
            }
        )
        if p:
            if not full_node_path:
                full_node_path.extend(p)
            else:
                # avoid duplicating join node
                full_node_path.extend(p[1:])

    coords = path_coords(g, full_node_path)

    out = {
        "centerlines": str(Path(args.centerlines).as_posix()),
        "mission": str(Path(args.mission).as_posix()),
        "graph": {"n_nodes": len(g.nodes), "n_edges": sum(len(v) for v in g.adj.values())},
        "mission_snapped": snapped,
        "route": {
            "node_path": full_node_path,
            "coords_latlon": coords,
            "legs": legs,
        },
        "geojson_linestring": {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {"name": "dijkstra_route"},
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [[lon, lat] for (lat, lon) in coords],  # GeoJSON uses [lon,lat]
                    },
                }
            ],
        },
    }

    txt = json.dumps(out, ensure_ascii=False)
    print(txt)

    if args.out:
        Path(args.out).write_text(txt, encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

