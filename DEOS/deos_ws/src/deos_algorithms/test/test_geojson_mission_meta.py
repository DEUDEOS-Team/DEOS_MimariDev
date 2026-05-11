import json
import sys
from pathlib import Path

_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_mission_meta_deos_mission_meta_roundtrip():
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader, TaskType

    gj = {
        "type": "FeatureCollection",
        "deos_mission_meta": {"note": "opsiyonel; tünel zorunluluğu artık sadece centerlines tunnel:true ile"},
        "features": [
            {"type": "Feature", "properties": {"name": "start", "task": "baslangic"}, "geometry": {"type": "Point", "coordinates": [29.0, 41.0]}},
            {"type": "Feature", "properties": {"name": "durak_1", "task": "durak"}, "geometry": {"type": "Point", "coordinates": [29.0001, 41.0001]}},
            {"type": "Feature", "properties": {"name": "gorev_1", "task": "gorev_yeri"}, "geometry": {"type": "Point", "coordinates": [29.0002, 41.0002]}},
            {"type": "Feature", "properties": {"name": "tunel_orta", "task": "via"}, "geometry": {"type": "Point", "coordinates": [29.00015, 41.00015]}},
            {"type": "Feature", "properties": {"name": "park_giris", "task": "park_giris"}, "geometry": {"type": "Point", "coordinates": [29.0003, 41.0003]}},
            {"type": "Feature", "properties": {"name": "park_yeri_1", "task": "park_yeri"}, "geometry": {"type": "Point", "coordinates": [29.00035, 41.00035]}},
        ],
    }
    plan = GeoJsonMissionReader().read_string(json.dumps(gj, ensure_ascii=False))
    assert plan.meta.get("note")
    assert plan.points[0].task == TaskType.START
    assert plan.points[1].task == TaskType.STOP
    assert plan.points[2].task == TaskType.CHECKPOINT
    assert plan.points[3].task == TaskType.CHECKPOINT
    assert plan.points[4].task == TaskType.PARK_ENTRY
    assert plan.points[5].task == TaskType.PARK
