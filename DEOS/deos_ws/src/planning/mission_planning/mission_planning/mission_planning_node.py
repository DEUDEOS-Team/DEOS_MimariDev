import math
import time
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float32, String

from deos_algorithms.geojson_mission_reader import GeoJsonMissionReader
from deos_algorithms.mission_manager import MissionManager
from deos_algorithms.route_graph import build_graph_from_centerlines_geojson, load_centerlines_geojson, nearest_node_id
from deos_algorithms.route_planner import (
    advance_mission_index_by_position,
    route_remaining_mission_via_graph,
    route_mission_plan_via_graph,
)
from deos_algorithms.waypoint_manager import GpsPosition


GPS_TIMEOUT_SLOW_S = 2.0
GPS_TIMEOUT_STOP_S = 5.0
SPEED_DECAY_PER_SEC = 0.2
TURN_RULE_APPLY_DISTANCE_M = 8.0


def _param_as_bool(v) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("true", "1", "yes", "on")
    return bool(v)


class MissionPlanningNode(Node):
    def __init__(self):
        super().__init__("mission_planning_node")

        self.declare_parameter("mission_file", "")
        self.declare_parameter("centerlines_file", "")
        self.declare_parameter("centerlines_round_decimals", 7)
        # Centerlines'ta tunnel: true varsa rota en az bir tünel kenarından geçer (görev GeoJSON gerekmez).
        self.declare_parameter("tunnel_mandatory", True)
        # UMS-2 Go: araç göreve başlamadan önce onay bekle
        self.declare_parameter("require_go_signal", True)
        self.declare_parameter("go_topic", "/hardware/motion_enable")
        self.declare_parameter("heading_offset_deg", 0.0)

        mission_file = str(self.get_parameter("mission_file").value)
        centerlines_file = str(self.get_parameter("centerlines_file").value)
        centerlines_round = int(self.get_parameter("centerlines_round_decimals").value)
        self._tunnel_mandatory = _param_as_bool(self.get_parameter("tunnel_mandatory").value)
        self._heading_offset = float(self.get_parameter("heading_offset_deg").value)
        self._require_go = _param_as_bool(self.get_parameter("require_go_signal").value)
        self._go_ok: bool = False
        self._go_stamp: float = 0.0

        self._manager: MissionManager | None = None
        self._base_plan = None
        if mission_file:
            reader = GeoJsonMissionReader()
            self._base_plan = reader.read_file(mission_file)
        else:
            self.get_logger().warn("mission_file parametresi boş — waypoint takibi pasif")

        # Yol graph'ı (lane centerlines) — node/edge'ler açılışta otomatik çıkarılır.
        self._route_graph = None
        if centerlines_file:
            try:
                gj = load_centerlines_geojson(centerlines_file)
                self._route_graph = build_graph_from_centerlines_geojson(gj, coord_round_decimals=centerlines_round)
                n_nodes = len(self._route_graph.nodes)
                n_edges = sum(len(v) for v in self._route_graph.adj.values())
                self.get_logger().info(
                    f"Centerlines yüklendi: nodes={n_nodes} edges={n_edges} — {centerlines_file}"
                )
            except Exception as e:
                self.get_logger().error(f"Centerlines yüklenemedi: {centerlines_file} — {e}")

        # Routing: Mission noktalarını graph'a snap edip Dijkstra ile bir route waypoint listesi üret.
        self._blocked_edges: set[tuple[int, int]] = set()
        self._road_blocked: bool = False
        self._mission_idx: int = 0  # base_plan hedef indeksi (replan için)

        if self._base_plan is not None:
            plan = self._base_plan
            if self._route_graph is not None:
                try:
                    plan = route_mission_plan_via_graph(
                        self._base_plan,
                        self._route_graph,
                        tunnel_mandatory=self._tunnel_mandatory,
                    )
                    self.get_logger().info(
                        f"Routing aktif: {len(self._base_plan)} hedef -> {len(plan)} route waypoint"
                    )
                except Exception as e:
                    self.get_logger().error(f"Routing başarısız, mission plan kullanılacak — {e}")
                    plan = self._base_plan
            self._manager = MissionManager(plan)
            self.get_logger().info(f"Görev yüklendi: {len(plan)} waypoint — {mission_file}")

        self._heading_deg = 0.0
        self._gps_stamp: float = 0.0

        self._last_steering = 0.0
        self._last_speed = 0.0
        self._last_task = ""
        self._turn_perm: dict | None = None

        self.create_subscription(NavSatFix, "/gps/fix", self._gps_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)
        self.create_subscription(String, "/perception/turn_permissions", self._turn_perm_cb, 10)
        self.create_subscription(String, "/perception/decision_debug", self._decision_debug_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("go_topic").value), self._go_cb, 10)

        self._pub_steer = self.create_publisher(Float32, "/planning/steering_ref", 10)
        self._pub_speed = self.create_publisher(Float32, "/planning/speed_limit", 10)
        self._pub_task = self.create_publisher(String, "/planning/current_task", 10)
        self._pub_arrived = self.create_publisher(Bool, "/planning/arrived", 10)
        self._pub_park_mode = self.create_publisher(Bool, "/planning/park_mode", 10)
        self._pub_park_remaining = self.create_publisher(Float32, "/planning/park_remaining_s", 10)

        # Park tamamlandı sinyali (perception) — park girişinden sonra 3dk içinde park etmek için
        self.create_subscription(Bool, "/perception/park_complete", self._park_complete_cb, 10)

        self.create_timer(0.2, self._tick_timeout)  # 5 Hz

    def _go_cb(self, msg: Bool) -> None:
        self._go_ok = bool(msg.data)
        self._go_stamp = time.monotonic()

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

    def _decision_debug_cb(self, msg: String) -> None:
        # decision_debug JSON: {final: {...}, candidates: [...], reasons:[...]} (perception_fusion_node)
        try:
            d = json.loads(msg.data) if msg.data else {}
            reasons = d.get("final", {}).get("reasons") or d.get("reasons") or []
            # ReasonCode values are strings like "road_blocked"
            self._road_blocked = any(str(r) == "road_blocked" for r in reasons)
        except Exception:
            self._road_blocked = False

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

        if self._require_go and not self._go_ok:
            # Go gelmeden görev başlamasın: steering ref yayınla ama speed=0 ver
            pos0 = GpsPosition(lat=float(msg.latitude), lon=float(msg.longitude), heading_deg=self._heading_deg)
            wp_state, mission_dec = self._manager.update(pos0, now_s=time.monotonic())
            self._last_steering = float(wp_state.steering_ref)
            self._last_speed = 0.0
            self._last_task = "waiting_go"
            self._publish(self._last_steering, 0.0, self._last_task, False, mission_dec.park_mode, mission_dec.park_remaining_s)
            return

        pos = GpsPosition(lat=float(msg.latitude), lon=float(msg.longitude), heading_deg=self._heading_deg)
        # Base mission ilerleme (routing resetlerini güvenli yapmak için)
        if self._base_plan is not None:
            self._mission_idx = advance_mission_index_by_position(
                self._base_plan, start_index=self._mission_idx, lat=pos.lat, lon=pos.lon
            )

        wp_state, mission_dec = self._manager.update(pos, now_s=time.monotonic())

        # Replanning: yol kapalı sinyali geldiyse, park modunda değilken graph üzerinden alternatif rota üret.
        if (
            self._road_blocked
            and self._route_graph is not None
            and self._base_plan is not None
            and not bool(mission_dec.park_mode)
        ):
            try:
                cur_node = int(nearest_node_id(self._route_graph, lat=pos.lat, lon=pos.lon))
                # "Önümüzdeki" waypoint'i edge olarak bloke et (directed)
                if wp_state.current_wp is not None:
                    nxt_node = int(nearest_node_id(self._route_graph, lat=float(wp_state.current_wp.lat), lon=float(wp_state.current_wp.lon)))
                    if cur_node != nxt_node:
                        self._blocked_edges.add((cur_node, nxt_node))

                new_plan = route_remaining_mission_via_graph(
                    self._base_plan,
                    self._route_graph,
                    current_lat=pos.lat,
                    current_lon=pos.lon,
                    start_index=self._mission_idx,
                    blocked_edges=self._blocked_edges,
                    tunnel_mandatory=self._tunnel_mandatory,
                )
                self._manager = MissionManager(new_plan)
                # reset flag once we replanned
                self._road_blocked = False
                wp_state, mission_dec = self._manager.update(pos, now_s=time.monotonic())
                self.get_logger().warn(
                    f"REPLAN: road_blocked -> new_route_waypoints={len(new_plan.points)} blocked_edges={len(self._blocked_edges)}"
                )
            except Exception as e:
                self.get_logger().error(f"REPLAN başarısız: {e}")

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

