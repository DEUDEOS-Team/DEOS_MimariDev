import sys
from pathlib import Path


# Allow running tests without installing the package (colcon/pip).
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))


def test_imports():
    from deos_algorithms.safety_logic import SafetyLogic
    from deos_algorithms.obstacle_logic import ObstacleLogic, ObstacleDetection
    from deos_algorithms.slalom_logic import SlalomLogic
    from deos_algorithms.parking_logic import ParkingLogic, ParkingDetection
    from deos_algorithms.traffic_sign_logic import TrafficSignLogic, SignDetection
    from deos_algorithms.traffic_light_logic import TrafficLightLogic, LightDetection
    from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader, MissionPlan
    from deos_algorithms.waypoint_manager import WaypointManager, GpsPosition
    from deos_algorithms.mission_manager import MissionManager
    from deos_algorithms.perception_fusion import fuse
    from deos_algorithms.sensors.types import StereoBbox, LidarObstacle, ImuSample


def test_obstacle_logic_empty():
    from deos_algorithms.obstacle_logic import ObstacleLogic

    logic = ObstacleLogic()
    state = logic.update([])
    assert state.speed_cap_ratio == 1.0
    assert not state.emergency_stop


def test_slalom_no_cones():
    from deos_algorithms.slalom_logic import SlalomLogic

    logic = SlalomLogic()
    state = logic.update([])
    assert state.faz == "bekleme"
    assert not state.aktif

