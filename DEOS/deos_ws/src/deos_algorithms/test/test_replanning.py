import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_route_remaining_mission_via_graph_uses_current_pos_and_blocked():
    from deos_algorithms.geojson_mission_reader import MissionPlan, MissionPoint, TaskType
    from deos_algorithms.route_graph import build_graph_from_centerlines_geojson
    from deos_algorithms.route_planner import route_remaining_mission_via_graph

    # Square: 0-1-2-3-0 (coordinates as lon,lat)
    geo = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "properties": {}, "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [1.0, 0.0]]}},
            {"type": "Feature", "properties": {}, "geometry": {"type": "LineString", "coordinates": [[1.0, 0.0], [1.0, 1.0]]}},
            {"type": "Feature", "properties": {}, "geometry": {"type": "LineString", "coordinates": [[1.0, 1.0], [0.0, 1.0]]}},
            {"type": "Feature", "properties": {}, "geometry": {"type": "LineString", "coordinates": [[0.0, 1.0], [0.0, 0.0]]}},
        ],
    }
    g = build_graph_from_centerlines_geojson(geo, coord_round_decimals=7)

    plan = MissionPlan(
        points=[
            MissionPoint(index=0, point_id=None, name="start", lat=0.0, lon=0.0, task=TaskType.START, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=0.1),
            MissionPoint(index=1, point_id=None, name="park_giris", lat=1.0, lon=1.0, task=TaskType.PARK_ENTRY, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=0.1),
        ]
    )

    # Current pos near (0,0) but block the direct route to (1,1) by blocking one edge.
    routed1 = route_remaining_mission_via_graph(plan, g, current_lat=0.0, current_lon=0.0, start_index=0)
    assert routed1.points

    routed2 = route_remaining_mission_via_graph(
        plan,
        g,
        current_lat=0.0,
        current_lon=0.0,
        start_index=0,
        blocked_edges={(0, 1)},
    )
    assert routed2.points, "Square graph'da 0->1 bloke olsa da alternatif yol olmalı"

