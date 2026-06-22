#!/usr/bin/env python3
"""
Rota görselleştirme aracı — tarayıcıda Leaflet haritası açar.

Argüman verilmezse dahili demo verisiyle çalışır.
Gerçek dosyalarla:
  python tools/visualize_route.py --centerlines maps/centerlines.geojson --mission maps/mission.geojson

Demo (argümansız):
  python tools/visualize_route.py
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
import webbrowser
from pathlib import Path

_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from deos_algorithms.route_graph import (  # noqa: E402
    build_graph_from_centerlines_geojson,
    haversine_m,
    load_centerlines_geojson,
    path_coords,
)
from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader  # noqa: E402
from deos_algorithms.route_planner import route_mission_plan_via_graph  # noqa: E402


# ---------------------------------------------------------------------------
# Demo verisi — GPS koordinatlı küçük bir test pisti (Istanbul civarı)
# ---------------------------------------------------------------------------
# Izgara: 4 sütun × 3 satır düğüm, ~40 m aralıklı
# Ek tünel segmenti: alt ortadan üst ortaya kısayol
_LAT0, _LON0 = 41.0000, 29.0000
_DLAT, _DLON = 0.00040, 0.00050  # ~44 m / ~41 m

def _ll(row: int, col: int) -> list[float]:
    return [_LON0 + col * _DLON, _LAT0 + row * _DLAT]

DEMO_CENTERLINES: dict = {
    "type": "FeatureCollection",
    "features": [
        # --- yatay yollar ---
        *[
            {"type": "Feature", "properties": {"speed_limit_mps": 3.0},
             "geometry": {"type": "LineString",
                          "coordinates": [_ll(r, c) for c in range(4)]}}
            for r in range(3)
        ],
        # --- dikey yollar ---
        *[
            {"type": "Feature", "properties": {"speed_limit_mps": 3.0},
             "geometry": {"type": "LineString",
                          "coordinates": [_ll(r, c) for r in range(3)]}}
            for c in range(4)
        ],
        # --- tünel segmenti (alt-orta → tünel via → üst-orta) ---
        {"type": "Feature",
         "properties": {"speed_limit_mps": 5.0, "tunnel": True},
         "geometry": {"type": "LineString", "coordinates": [
             _ll(0, 1),
             [_LON0 + 1.5 * _DLON, _LAT0 + 1.0 * _DLAT],  # tünel eğimi
             _ll(2, 2),
         ]}},
    ],
}

DEMO_MISSION: dict = {
    "type": "FeatureCollection",
    "features": [
        {"type": "Feature",
         "properties": {"name": "baslangic", "task": "start"},
         "geometry": {"type": "Point", "coordinates": _ll(0, 0)}},
        {"type": "Feature",
         "properties": {"name": "gorev_1", "task": "checkpoint"},
         "geometry": {"type": "Point", "coordinates": _ll(2, 3)}},
        {"type": "Feature",
         "properties": {"name": "gorev_2", "task": "checkpoint"},
         "geometry": {"type": "Point", "coordinates": _ll(0, 3)}},
        {"type": "Feature",
         "properties": {"name": "park_giris", "task": "park_entry"},
         "geometry": {"type": "Point", "coordinates": _ll(2, 0)}},
    ],
}


# ---------------------------------------------------------------------------
# HTML şablonu (Leaflet)
# ---------------------------------------------------------------------------
HTML_TEMPLATE = """\
<!DOCTYPE html>
<html lang="tr">
<head>
<meta charset="utf-8"/>
<title>DEOS Rota Görselleştirici</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<style>
  body {{ margin: 0; font-family: sans-serif; }}
  #map {{ height: 100vh; }}
  #legend {{
    position: absolute; top: 12px; right: 12px; z-index: 1000;
    background: rgba(255,255,255,0.92); padding: 12px 16px;
    border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.2);
    font-size: 13px; min-width: 200px;
  }}
  #legend h3 {{ margin: 0 0 8px; font-size: 14px; }}
  .leg-row {{ display: flex; align-items: center; gap: 8px; margin: 4px 0; }}
  .swatch {{ width: 28px; height: 4px; border-radius: 2px; flex-shrink: 0; }}
  .dot {{ width: 12px; height: 12px; border-radius: 50%; flex-shrink: 0; }}
  #stats {{ margin-top: 10px; font-size: 12px; color: #555; border-top: 1px solid #ddd; padding-top: 8px; }}
</style>
</head>
<body>
<div id="map"></div>
<div id="legend">
  <h3>DEOS Rota Planı</h3>
  <div class="leg-row"><div class="swatch" style="background:#888;height:2px;"></div>Centerlines</div>
  <div class="leg-row"><div class="swatch" style="background:#2196F3;height:3px;"></div>Tünel segmenti</div>
  <div class="leg-row"><div class="swatch" style="background:#4CAF50;height:4px;"></div>Dijkstra rotası</div>
  <div class="leg-row"><div class="dot" style="background:#e53935;"></div>Görev noktaları</div>
  <div class="leg-row"><div class="dot" style="background:#1565C0;border:2px solid #fff;"></div>Rota waypoint'leri</div>
  <div id="stats">{STATS}</div>
</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const centerlines = {CENTERLINES_JSON};
const routeCoords = {ROUTE_COORDS_JSON};
const missionPoints = {MISSION_JSON};
const waypointCoords = {WAYPOINTS_JSON};

const map = L.map('map');
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
  attribution: '© OpenStreetMap contributors', maxZoom: 21
}}).addTo(map);

// Centerlines
centerlines.features.forEach(f => {{
  const coords = f.geometry.coordinates.map(c => [c[1], c[0]]);
  const isTunnel = f.properties && f.properties.tunnel;
  L.polyline(coords, {{
    color: isTunnel ? '#2196F3' : '#888',
    weight: isTunnel ? 3 : 2,
    opacity: 0.7,
    dashArray: isTunnel ? '6 4' : null,
  }}).addTo(map).bindPopup(isTunnel ? 'Tünel segmenti' : 'Centerline');
}});

// Dijkstra rotası
if (routeCoords.length > 1) {{
  L.polyline(routeCoords, {{color:'#4CAF50', weight:5, opacity:0.9}}).addTo(map);
  // Yön oku için ara noktalar
  for (let i = 0; i < routeCoords.length - 1; i += Math.max(1, Math.floor(routeCoords.length / 10))) {{
    const a = routeCoords[i], b = routeCoords[i+1];
    const mid = [(a[0]+b[0])/2, (a[1]+b[1])/2];
    const angle = Math.atan2(b[1]-a[1], b[0]-a[0]) * 180 / Math.PI;
    L.marker(mid, {{
      icon: L.divIcon({{
        html: `<div style="transform:rotate(${{angle-90}}deg);color:#4CAF50;font-size:16px;">▲</div>`,
        className: '', iconSize: [16,16], iconAnchor: [8,8]
      }})
    }}).addTo(map);
  }}
}}

// Rota waypoint'leri (küçük mavi noktalar)
waypointCoords.forEach((c, i) => {{
  L.circleMarker(c, {{radius:4, color:'#1565C0', fillColor:'#42A5F5', fillOpacity:0.9, weight:1}})
    .addTo(map).bindPopup(`Waypoint #${{i}}`);
}});

// Görev noktaları
const taskColors = {{
  'start': '#43A047', 'checkpoint': '#FB8C00', 'park_entry': '#8E24AA',
  'stop': '#E53935', 'park': '#00838F', 'pickup': '#1E88E5', 'dropoff': '#6D4C41'
}};
missionPoints.forEach(mp => {{
  const color = taskColors[mp.task] || '#E53935';
  L.circleMarker([mp.lat, mp.lon], {{
    radius: 10, color: '#fff', weight: 2,
    fillColor: color, fillOpacity: 1
  }}).addTo(map).bindPopup(
    `<b>${{mp.name}}</b><br>Görev: ${{mp.task}}<br>Lat: ${{mp.lat.toFixed(6)}}<br>Lon: ${{mp.lon.toFixed(6)}}`
  );
  L.tooltip({{permanent: true, direction: 'top', offset: [0, -12], className: 'mp-label'}})
    .setContent(mp.name)
    .setLatLng([mp.lat, mp.lon])
    .addTo(map);
}});

// Haritayı rotaya sığdır
if (routeCoords.length > 0) {{
  map.fitBounds(L.latLngBounds(routeCoords).pad(0.15));
}} else if (missionPoints.length > 0) {{
  map.fitBounds(L.latLngBounds(missionPoints.map(m => [m.lat, m.lon])).pad(0.2));
}}
</script>
</body>
</html>
"""


def _total_distance_m(coords: list[tuple[float, float]]) -> float:
    total = 0.0
    for (la1, lo1), (la2, lo2) in zip(coords, coords[1:]):
        total += haversine_m(la1, lo1, la2, lo2)
    return total


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Rota görselleştirici — tarayıcıda Leaflet haritası açar.")
    ap.add_argument("--centerlines", default="", help="LineString centerlines GeoJSON (boşsa demo verisi kullanılır)")
    ap.add_argument("--mission", default="", help="Mission GeoJSON (boşsa demo verisi kullanılır)")
    ap.add_argument("--no-tunnel-mandatory", action="store_true", help="Tünel zorunluluğunu devre dışı bırak")
    ap.add_argument("--out", default="", help="Çıktı HTML dosyası (boşsa geçici dosya kullanılır)")
    args = ap.parse_args(argv)

    # --- Veri yükle ---
    if args.centerlines and Path(args.centerlines).exists():
        center_geo = load_centerlines_geojson(args.centerlines)
        print(f"Centerlines: {args.centerlines}")
    else:
        center_geo = DEMO_CENTERLINES
        print("Centerlines: demo verisi kullanılıyor")

    if args.mission and Path(args.mission).exists():
        reader = GeoJsonMissionReader()
        plan = reader.read_file(args.mission)
        print(f"Mission: {args.mission}")
    else:
        reader = GeoJsonMissionReader()
        plan = reader.read_string(json.dumps(DEMO_MISSION))
        print("Mission: demo verisi kullanılıyor")

    # --- Graf inşa et ---
    g = build_graph_from_centerlines_geojson(center_geo)
    n_nodes = len(g.nodes)
    n_edges = sum(len(v) for v in g.adj.values())
    print(f"Graf: {n_nodes} node, {n_edges} kenar")

    # --- Rota üret ---
    tunnel_mandatory = not bool(args.no_tunnel_mandatory)
    routed = route_mission_plan_via_graph(plan, g, tunnel_mandatory=tunnel_mandatory)
    print(f"Rota: {len(plan)} gorev noktasi -> {len(routed.points)} waypoint")

    route_latlon = [(float(p.lat), float(p.lon)) for p in routed.points]
    total_m = _total_distance_m(route_latlon)
    print(f"Toplam rota mesafesi: {total_m:.1f} m")

    # --- Görev noktası bilgisi ---
    mission_json = [
        {"name": p.name, "task": p.task, "lat": float(p.lat), "lon": float(p.lon)}
        for p in plan.points
    ]

    # --- HTML oluştur ---
    stats_html = (
        f"Graf: {n_nodes} node / {n_edges} kenar<br>"
        f"Görev: {len(plan)} nokta → {len(routed.points)} waypoint<br>"
        f"Mesafe: {total_m:.1f} m<br>"
        f"Tünel zorunlu: {'evet' if tunnel_mandatory else 'hayır'}"
    )

    html = HTML_TEMPLATE.format(
        CENTERLINES_JSON=json.dumps(center_geo, ensure_ascii=False),
        ROUTE_COORDS_JSON=json.dumps(route_latlon, ensure_ascii=False),
        MISSION_JSON=json.dumps(mission_json, ensure_ascii=False),
        WAYPOINTS_JSON=json.dumps(route_latlon, ensure_ascii=False),
        STATS=stats_html,
    )

    # --- Dosyaya yaz ve aç ---
    if args.out:
        out_path = Path(args.out)
    else:
        tmp = tempfile.NamedTemporaryFile(suffix=".html", delete=False, mode="w", encoding="utf-8")
        tmp.write(html)
        tmp.close()
        out_path = Path(tmp.name)

    if args.out:
        out_path.write_text(html, encoding="utf-8")

    print(f"\nHarita dosyası: {out_path}")
    print("Tarayıcıda açılıyor...")
    webbrowser.open(out_path.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
