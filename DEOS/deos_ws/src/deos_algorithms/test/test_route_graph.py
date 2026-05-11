import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_route_graph_build_and_dijkstra():
    from deos_algorithms.route_graph import build_graph_from_centerlines_geojson, dijkstra, nearest_node_id, path_coords

    # Basit T şekli (3 edge)
    geo = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {},
                "geometry": {"type": "LineString", "coordinates": [[29.0, 41.0], [29.0, 41.00001]]},
            },
            {
                "type": "Feature",
                "properties": {},
                "geometry": {"type": "LineString", "coordinates": [[29.0, 41.00001], [29.00001, 41.00001]]},
            },
        ],
    }

    g = build_graph_from_centerlines_geojson(geo)
    assert len(g.nodes) >= 3

    s = nearest_node_id(g, lat=41.0, lon=29.0)
    t = nearest_node_id(g, lat=41.00001, lon=29.00001)
    p = dijkstra(g, start=s, goal=t)
    assert p, "Dijkstra path boş olmamalı"

    coords = path_coords(g, p)
    assert coords[0] != coords[-1]


def test_route_graph_dijkstra_blocked_edge():
    from deos_algorithms.route_graph import build_graph_from_centerlines_geojson, dijkstra

    # Line: A-B-C
    geo = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {},
                "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [0.0, 1.0], [0.0, 2.0]]},
            }
        ],
    }
    g = build_graph_from_centerlines_geojson(geo)
    # nodes are 0,1,2 in build order
    p_ok = dijkstra(g, start=0, goal=2)
    assert p_ok == [0, 1, 2]
    p_blk = dijkstra(g, start=0, goal=2, blocked_edges={(1, 2)})
    assert p_blk == [], "1->2 bloke ise 0'dan 2'ye path olmamalı"


def test_route_graph_blocked_property_skips_edge():
    from deos_algorithms.route_graph import build_graph_from_centerlines_geojson, dijkstra

    geo = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"blocked": True},
                "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [0.0, 1.0]]},
            }
        ],
    }
    g = build_graph_from_centerlines_geojson(geo)
    assert len(g.nodes) == 0 or sum(len(v) for v in g.adj.values()) == 0


def test_route_graph_speed_limit_changes_cost_preference():
    from deos_algorithms.route_graph import build_graph_from_centerlines_geojson, dijkstra

    # Two routes from A to C:
    # - Direct A-C is longer distance but faster speed_limit_mps -> lower time cost
    # - A-B-C is shorter distance but very slow speed_limit_mps -> higher time cost
    geo = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "properties": {"speed_limit_mps": 0.5}, "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [0.0, 0.5], [0.0, 1.0]]}},
            {"type": "Feature", "properties": {"speed_limit_mps": 5.0}, "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [0.1, 1.0]]}},
        ],
    }
    g = build_graph_from_centerlines_geojson(geo, coord_round_decimals=7)
    # node ids depend on insertion order but should exist
    # pick start as first node (0.0,0.0) and goal as (1.0,0.1) or (1.0,0.0) depending
    # Here goal should be (lat=1.0, lon=0.1) which is second line end.
    # We can find ids by nearest_node_id but keep test local:
    from deos_algorithms.route_graph import nearest_node_id

    s = nearest_node_id(g, lat=0.0, lon=0.0)
    c_fast = nearest_node_id(g, lat=1.0, lon=0.1)
    p = dijkstra(g, start=s, goal=c_fast)
    assert len(p) == 2, f"Hızlı direct edge bekleniyor, path={p}"


def test_route_graph_tunnel_edge_multiplier_prefers_tunnel():
    from deos_algorithms.route_graph import Edge, build_graph_from_centerlines_geojson, dijkstra, nearest_node_id

    # Düz yol: (0,0)-(0,0.5)-(0,1.0) — kısa
    # Tünel işaretli detur: (0,0)-(0.2,0.5)-(0,1.0) — gerçekte daha uzun; çarpan ile ucuzlar
    geo = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "properties": {}, "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [0.0, 0.5], [0.0, 1.0]]}},
            {"type": "Feature", "properties": {"tunnel": True}, "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [0.2, 0.5], [0.0, 1.0]]}},
        ],
    }
    g = build_graph_from_centerlines_geojson(geo, coord_round_decimals=7)
    s = nearest_node_id(g, lat=0.0, lon=0.0)
    t = nearest_node_id(g, lat=1.0, lon=0.0)

    p_short = dijkstra(g, start=s, goal=t)
    assert len(p_short) == 3, "Çarpansız en kısa düz hat olmalı"

    def mult(e: Edge) -> float:
        if e.props.get("tunnel") is True:
            return 0.05
        return 1.0

    p_tunnel = dijkstra(g, start=s, goal=t, edge_cost_multiplier=mult)
    assert len(p_tunnel) == 3
    # Orta düğüm düz hatta (0,0.5); tünel hattında (0.2,0.5)
    mid_id = p_tunnel[1]
    n = g.nodes[mid_id]
    assert abs(n.lon - 0.2) < 1e-5 and abs(n.lat - 0.5) < 1e-5


def test_dijkstra_mandatory_tunnel_skips_shorter_non_tunnel_path():
    from deos_algorithms.route_graph import (
        build_graph_from_centerlines_geojson,
        dijkstra,
        dijkstra_mandatory_tunnel,
        nearest_node_id,
    )

    geo = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "properties": {}, "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [0.0, 0.5], [0.0, 1.0]]}},
            {"type": "Feature", "properties": {"tunnel": True}, "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [0.2, 0.5], [0.0, 1.0]]}},
        ],
    }
    g = build_graph_from_centerlines_geojson(geo, coord_round_decimals=7)
    s = nearest_node_id(g, lat=0.0, lon=0.0)
    t = nearest_node_id(g, lat=1.0, lon=0.0)
    p_free = dijkstra(g, start=s, goal=t)
    p_must = dijkstra_mandatory_tunnel(g, start=s, goal=t)
    assert len(p_free) == 3
    assert len(p_must) == 3
    mid_must = g.nodes[p_must[1]]
    assert abs(mid_must.lon - 0.2) < 1e-5 and abs(mid_must.lat - 0.5) < 1e-5

