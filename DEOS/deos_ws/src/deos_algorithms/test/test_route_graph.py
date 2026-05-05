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

