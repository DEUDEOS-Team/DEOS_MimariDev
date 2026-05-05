import math
import time
import json

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
TURN_RULE_APPLY_DISTANCE_M = 8.0


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
        self._turn_perm: dict | None = None

        self.create_subscription(NavSatFix, "/gps/fix", self._gps_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)
        self.create_subscription(String, "/perception/turn_permissions", self._turn_perm_cb, 10)

        self._pub_steer = self.create_publisher(Float32, "/planning/steering_ref", 10)
        self._pub_speed = self.create_publisher(Float32, "/planning/speed_limit", 10)
        self._pub_task = self.create_publisher(String, "/planning/current_task", 10)
        self._pub_arrived = self.create_publisher(Bool, "/planning/arrived", 10)
        self._pub_park_mode = self.create_publisher(Bool, "/planning/park_mode", 10)
        self._pub_park_remaining = self.create_publisher(Float32, "/planning/park_remaining_s", 10)

        # Park tamamlandı sinyali (perception) — park girişinden sonra 3dk içinde park etmek için
        self.create_subscription(Bool, "/perception/park_complete", self._park_complete_cb, 10)

        self.create_timer(0.2, self._tick_timeout)  # 5 Hz

    def _park_complete_cb(self, msg: Bool) -> None:
        if self._manager is None:
            return
        if bool(msg.data):
            self._manager.notify_park_completed()

    def _turn_perm_cb(self, msg: String) -> None:
        try:
            self._turn_perm = json.loads(msg.data) if msg.data else None
        except Exception:
            self._turn_perm = None

    def _apply_turn_permissions(self, steer: float, dist_to_wp_m: float) -> tuple[float, float]:
        """
        Tabela bazlı dönüş kısıtlarını waypoint'e yaklaşırken steer/speed üzerinde uygula.
        Çıktı: (steer, speed_multiplier)
        """
        if self._turn_perm is None:
            return steer, 1.0
        if dist_to_wp_m > TURN_RULE_APPLY_DISTANCE_M:
            return steer, 1.0

        forced = self._turn_perm.get("forced_direction")
        left_ok = bool(self._turn_perm.get("left", True))
        straight_ok = bool(self._turn_perm.get("straight", True))
        right_ok = bool(self._turn_perm.get("right", True))

        # forced_direction: left/right/straight/pass_left/pass_right/roundabout
        if forced == "left":
            return min(steer, -0.6), 0.7
        if forced == "right":
            return max(steer, 0.6), 0.7
        if forced == "straight":
            return 0.0, 0.7
        if forced == "pass_left":
            return min(steer, -0.35), 0.8
        if forced == "pass_right":
            return max(steer, 0.35), 0.8
        if forced == "roundabout":
            # Basit yaklaşım: hız düşür, steer'i sınırlama (roundabout için özel planner gerekebilir)
            return steer, 0.6

        # Yasak dönüşleri “yumuşak” şekilde engelle (tam replanning yok; güvenli yavaşlama)
        if not left_ok and steer < -0.2:
            return 0.0, 0.6
        if not right_ok and steer > 0.2:
            return 0.0, 0.6
        if not straight_ok and abs(steer) < 0.2:
            return (0.35 if right_ok else (-0.35 if left_ok else 0.0)), 0.6

        return steer, 1.0

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
        steer, turn_speed_mul = self._apply_turn_permissions(steer, float(wp_state.distance_to_wp_m))
        base_speed *= float(turn_speed_mul)
        speed = self._apply_gps_timeout(base_speed)

        self._last_steering = steer
        self._last_speed = speed
        self._last_task = str(wp_state.current_task or "")

        self._publish(steer, speed, self._last_task, bool(wp_state.arrived), mission_dec.park_mode, mission_dec.park_remaining_s)

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
        self._publish(self._last_steering, self._last_speed, self._last_task, False, False, 0.0)

    def _publish(self, steer: float, speed: float, task: str, arrived: bool, park_mode: bool, park_remaining_s: float) -> None:
        self._pub_steer.publish(Float32(data=float(steer)))
        self._pub_speed.publish(Float32(data=float(speed)))
        self._pub_task.publish(String(data=str(task)))
        self._pub_arrived.publish(Bool(data=bool(arrived)))
        self._pub_park_mode.publish(Bool(data=bool(park_mode)))
        self._pub_park_remaining.publish(Float32(data=float(park_remaining_s)))


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

