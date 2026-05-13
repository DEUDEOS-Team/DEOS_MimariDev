import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_route_mission_plan_via_graph_labels_tasks():
    from deos_algorithms.geojson_mission_reader import MissionPlan, MissionPoint, TaskType
    from deos_algorithms.route_graph import build_graph_from_centerlines_geojson
    from deos_algorithms.route_planner import route_mission_plan_via_graph

    # Graph: A(0,0) - B(0,1) - C(0,2)
    geo = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "properties": {}, "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [0.0, 1.0], [0.0, 2.0]]}},
        ],
    }
    g = build_graph_from_centerlines_geojson(geo, coord_round_decimals=7)

    plan = MissionPlan(
        points=[
            MissionPoint(index=0, point_id=None, name="start", lat=0.0, lon=0.0, task=TaskType.START, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=1.0),
            MissionPoint(index=1, point_id=None, name="gorev_1", lat=1.0, lon=0.0, task=TaskType.CHECKPOINT, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=1.0),
            MissionPoint(index=2, point_id=None, name="park_giris", lat=2.0, lon=0.0, task=TaskType.PARK_ENTRY, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=1.0),
        ],
        source_file="<test>",
    )

    routed = route_mission_plan_via_graph(plan, g)
    assert len(routed.points) >= 3
    assert routed.points[0].task == TaskType.START
    assert any(p.task == TaskType.PARK_ENTRY for p in routed.points), "park entry label should be preserved"


def test_route_mission_plan_without_graph_orders_tasks_and_keeps_park_last():
    from deos_algorithms.geojson_mission_reader import MissionPlan, MissionPoint, TaskType
    from deos_algorithms.route_planner import route_mission_plan_without_graph

    plan = MissionPlan(
        points=[
            MissionPoint(index=0, point_id=None, name="start", lat=0.0, lon=0.0, task=TaskType.START, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=1.0),
            MissionPoint(index=1, point_id=None, name="gorev_uzak", lat=0.0, lon=3.0, task=TaskType.CHECKPOINT, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=1.0),
            MissionPoint(index=2, point_id=None, name="park_giris", lat=0.0, lon=0.5, task=TaskType.PARK_ENTRY, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=1.0),
            MissionPoint(index=3, point_id=None, name="gorev_yakin", lat=0.0, lon=1.0, task=TaskType.CHECKPOINT, heading_deg=None, speed_limit_ratio=1.0, arrival_radius_m=1.0),
        ],
        source_file="<test>",
    )

    routed = route_mission_plan_without_graph(plan)
    assert [p.name for p in routed.points] == ["start", "gorev_yakin", "gorev_uzak", "park_giris"]
    assert routed.points[0].task == TaskType.START
    assert routed.points[-1].task == TaskType.PARK_ENTRY
    assert [p.index for p in routed.points] == [0, 1, 2, 3]
    assert any(p.task == TaskType.PARK_ENTRY for p in routed.points), "Park giriş etiketi rotada korunmalı"

