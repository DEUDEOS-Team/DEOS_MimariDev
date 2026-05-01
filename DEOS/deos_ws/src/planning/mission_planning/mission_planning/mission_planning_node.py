import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float32, String

from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
from deos_algorithms.mission_manager import MissionManager
from deos_algorithms.waypoint_manager import GpsPosition


GPS_TIMEOUT_SLOW_S = 2.0
GPS_TIMEOUT_STOP_S = 5.0
SPEED_DECAY_PER_SEC = 0.2


class MissionPlanningNode(Node):
    def __init__(self):
        super().__init__("mission_planning_node")

        self.declare_parameter("mission_file", "")
        self.declare_parameter("heading_offset_deg", 0.0)

        mission_file = str(self.get_parameter("mission_file").value)
        self._heading_offset = float(self.get_parameter("heading_offset_deg").value)

        self._manager: MissionManager | None = None
        if mission_file:
            reader = GeoJsonMissionReader()
            plan = reader.read_file(mission_file)
            self._manager = MissionManager(plan)
            self.get_logger().info(f"Görev yüklendi: {len(plan)} waypoint — {mission_file}")
        else:
            self.get_logger().warn("mission_file parametresi boş — waypoint takibi pasif")

        self._heading_deg = 0.0
        self._gps_stamp: float = 0.0

        self._last_steering = 0.0
        self._last_speed = 0.0
        self._last_task = ""

        self.create_subscription(NavSatFix, "/gps/fix", self._gps_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)

        self._pub_steer = self.create_publisher(Float32, "/planning/steering_ref", 10)
        self._pub_speed = self.create_publisher(Float32, "/planning/speed_limit", 10)
        self._pub_task = self.create_publisher(String, "/planning/current_task", 10)
        self._pub_arrived = self.create_publisher(Bool, "/planning/arrived", 10)

        self.create_timer(0.2, self._tick_timeout)  # 5 Hz

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_deg = math.degrees(math.atan2(siny, cosy)) % 360.0
        self._heading_deg = (raw_deg + self._heading_offset) % 360.0

    def _gps_cb(self, msg: NavSatFix) -> None:
        self._gps_stamp = time.monotonic()

        if self._manager is None:
            self._publish(0.0, 0.0, "no_mission", False)
            return

        pos = GpsPosition(lat=float(msg.latitude), lon=float(msg.longitude), heading_deg=self._heading_deg)
        wp_state, mission_dec = self._manager.update(pos, now_s=time.monotonic())

        steer = float(wp_state.steering_ref)
        base_speed = float(wp_state.speed_limit_ratio) * float(mission_dec.speed_cap_ratio)
        speed = self._apply_gps_timeout(base_speed)

        self._last_steering = steer
        self._last_speed = speed
        self._last_task = str(wp_state.current_task or "")

        self._publish(steer, speed, self._last_task, bool(wp_state.arrived))

    def _gps_age_s(self) -> float:
        if self._gps_stamp == 0.0:
            return float("inf")
        return time.monotonic() - self._gps_stamp

    def _apply_gps_timeout(self, base_speed: float) -> float:
        age = self._gps_age_s()
        if age < GPS_TIMEOUT_SLOW_S:
            return base_speed
        if age < GPS_TIMEOUT_STOP_S:
            return base_speed * 0.5
        excess = age - GPS_TIMEOUT_STOP_S
        return max(0.0, base_speed - excess * SPEED_DECAY_PER_SEC)

    def _tick_timeout(self) -> None:
        # GPS yoksa da son değerleri yayınlayalım (controller tarafında stabil kalır)
        if self._manager is None:
            return
        self._publish(self._last_steering, self._last_speed, self._last_task, False)

    def _publish(self, steer: float, speed: float, task: str, arrived: bool) -> None:
        self._pub_steer.publish(Float32(data=float(steer)))
        self._pub_speed.publish(Float32(data=float(speed)))
        self._pub_task.publish(String(data=str(task)))
        self._pub_arrived.publish(Bool(data=bool(arrived)))


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

