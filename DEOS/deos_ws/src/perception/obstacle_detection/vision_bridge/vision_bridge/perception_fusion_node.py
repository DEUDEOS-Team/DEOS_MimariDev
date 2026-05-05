import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String

from deos_algorithms.obstacle_logic import ObstacleLogic
from deos_algorithms.parking_logic import ParkingLogic
from deos_algorithms.perception_fusion import fuse, parking_detections_from_signs
from deos_algorithms.decision_arbiter import Candidate, DecisionArbiter, LaneBounds, ReasonCode
from deos_algorithms.sensors.types import ImuSample, LidarObstacle, StereoBbox
from deos_algorithms.slalom_logic import SlalomLogic
from deos_algorithms.traffic_light_logic import TrafficLightLogic
from deos_algorithms.traffic_sign_logic import TrafficSignLogic


class PerceptionFusionNode(Node):
    STEREO_TIMEOUT_S = 0.5
    LIDAR_TIMEOUT_S = 0.5
    LANE_WALLS_TIMEOUT_S = 0.5

    def __init__(self):
        super().__init__("perception_fusion_node")

        self.declare_parameter("hardware_motion_enable_topic", "/hardware/motion_enable")
        self.declare_parameter("hardware_motion_enable_timeout_s", 0.5)
        self.declare_parameter("hardware_motion_enable_fail_safe_stop", True)
        # Sensör fail-safe (lokal planlama): veri yoksa hız düşür / dur
        self.declare_parameter("fail_safe_stop_on_all_sensors_lost", True)
        self.declare_parameter("lidar_missing_speed_cap", 0.20)   # engel için kritik
        self.declare_parameter("stereo_missing_speed_cap", 0.30)  # ışık/tabela için kritik
        # Şerit dışına çıkmamak: engel kaçınma override'ı sadece şerit algısı tazeyken aktif olsun
        self.declare_parameter("lane_walls_topic", "/lane_walls")
        self.declare_parameter("require_lane_walls_for_avoidance", True)
        # Lane bounds çıkarımı (şerit içinde kalma kısıtı için)
        self.declare_parameter("lane_bounds_min_points", 20)
        self.declare_parameter("lane_bounds_x_min_m", 1.0)
        self.declare_parameter("lane_bounds_x_max_m", 4.0)
        self.declare_parameter("lane_bounds_z_max_m", 0.3)
        self.declare_parameter("lane_bounds_margin_m", 0.25)
        self.declare_parameter("publish_decision_debug", True)

        self._sign = TrafficSignLogic()
        self._light = TrafficLightLogic()
        self._obstacle = ObstacleLogic()
        self._slalom = SlalomLogic()
        self._parking = ParkingLogic()
        self._arbiter = DecisionArbiter()

        self._stereo: list[StereoBbox] = []
        self._lidar: list[LidarObstacle] = []
        self._imu: ImuSample | None = None
        self._stereo_stamp: float = 0.0
        self._lidar_stamp: float = 0.0
        self._lane_walls_stamp: float = 0.0
        self._lane_bounds: LaneBounds | None = None

        # STM32 -> Pi: tek topic, olay bazlı (sadece değişimde publish edilir)
        # std_msgs/Bool: false = DUR (algoritmalar durur), true = DEVAM
        # STM32 ilk "DEVAM" mesajını gönderene kadar güvenli tarafta kal
        self._motion_enable: bool = False
        self._motion_enable_stamp: float = 0.0

        # Planning -> Perception: park arama/manevra modu
        self._park_mode: bool = False

        self.create_subscription(String, "/perception/stereo_detections", self._stereo_cb, 10)
        self.create_subscription(String, "/perception/lidar_obstacles", self._lidar_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)
        self.create_subscription(Bool, "/planning/park_mode", self._park_mode_cb, 10)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("lane_walls_topic").value),
            self._lane_walls_cb,
            1,
        )

        motion_topic = str(self.get_parameter("hardware_motion_enable_topic").value)
        self.create_subscription(Bool, motion_topic, self._motion_enable_cb, 10)

        self._pub_estop = self.create_publisher(Bool, "/perception/emergency_stop", 10)
        self._pub_speed = self.create_publisher(Float32, "/perception/speed_cap", 10)
        self._pub_steer = self.create_publisher(Float32, "/perception/steering_override", 10)
        self._pub_has_steer = self.create_publisher(Bool, "/perception/has_steering_override", 10)
        self._pub_park_complete = self.create_publisher(Bool, "/perception/park_complete", 10)
        self._pub_turn_permissions = self.create_publisher(String, "/perception/turn_permissions", 10)
        self._pub_decision_debug = self.create_publisher(String, "/perception/decision_debug", 10)

        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info(
            "perception_fusion_node ready — "
            f"STM32 motion topic={motion_topic} (Bool: false=STOP algorithms, true=RUN)"
        )

    def _motion_enable_cb(self, msg: Bool) -> None:
        self._motion_enable = bool(msg.data)
        self._motion_enable_stamp = time.monotonic()

    def _park_mode_cb(self, msg: Bool) -> None:
        self._park_mode = bool(msg.data)

    def _stereo_cb(self, msg: String) -> None:
        try:
            raw: list[dict] = json.loads(msg.data)
            self._stereo = [
                StereoBbox(
                    class_name=d["class_name"],
                    confidence=float(d["confidence"]),
                    bbox_px=tuple(float(v) for v in d["bbox_px"]),
                    distance_m=d.get("distance_m"),
                    lateral_m=d.get("lateral_m"),
                )
                for d in raw
            ]
            self._stereo_stamp = time.monotonic()
        except Exception as e:
            self.get_logger().error(f"stereo parse: {e}")

    def _lidar_cb(self, msg: String) -> None:
        try:
            raw: list[dict] = json.loads(msg.data)
            self._lidar = [
                LidarObstacle(
                    kind=d["kind"],
                    confidence=float(d["confidence"]),
                    distance_m=float(d["distance_m"]),
                    lateral_m=float(d["lateral_m"]),
                    bbox_px=tuple(d["bbox_px"]) if d.get("bbox_px") else None,
                )
                for d in raw
            ]
            self._lidar_stamp = time.monotonic()
        except Exception as e:
            self.get_logger().error(f"lidar parse: {e}")

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        heading_deg = math.degrees(math.atan2(siny, cosy)) % 360.0
        yaw_rate_dps = math.degrees(msg.angular_velocity.z)
        self._imu = ImuSample(heading_deg=heading_deg, yaw_rate_dps=yaw_rate_dps)

    def _lane_walls_cb(self, _msg: PointCloud2) -> None:
        # Şerit algısı tazeliği için stamp tutuyoruz + basit sol/sağ sınır çıkarımı.
        self._lane_walls_stamp = time.monotonic()
        self._try_extract_lane_bounds(_msg)

    def _try_extract_lane_bounds(self, msg: PointCloud2) -> None:
        """
        /lane_walls pointcloud'undan basit sol/sağ sınır çıkarımı.
        Varsayım: pointcloud base_link frame'de, x ileri, y sol.
        """
        try:
            import struct

            x_min = float(self.get_parameter("lane_bounds_x_min_m").value)
            x_max = float(self.get_parameter("lane_bounds_x_max_m").value)
            z_max = float(self.get_parameter("lane_bounds_z_max_m").value)
            min_pts = int(self.get_parameter("lane_bounds_min_points").value)
            margin = float(self.get_parameter("lane_bounds_margin_m").value)

            if msg.point_step < 12:
                return

            ys: list[float] = []
            for i in range(0, len(msg.data), msg.point_step):
                x, y, z = struct.unpack_from("fff", msg.data, i)
                if x < x_min or x > x_max:
                    continue
                if z < -0.1 or z > z_max:
                    continue
                ys.append(float(y))

            if len(ys) < min_pts:
                return

            lb = LaneBounds(left_y_m=float(max(ys)), right_y_m=float(min(ys)), margin_m=float(margin))
            if lb.is_valid:
                self._lane_bounds = lb
        except Exception:
            return

    def _tick(self) -> None:
        now = time.monotonic()
        timeout_s = float(self.get_parameter("hardware_motion_enable_timeout_s").value)
        fail_safe = bool(self.get_parameter("hardware_motion_enable_fail_safe_stop").value)

        motion_ok = (now - self._motion_enable_stamp) <= timeout_s
        if not motion_ok and fail_safe:
            # STM32 komut akışı kesildi: güvenli tarafta kal (algoritmaları durdur)
            self._pub_estop.publish(Bool(data=True))
            self._pub_speed.publish(Float32(data=0.0))
            self._pub_steer.publish(Float32(data=0.0))
            self._pub_has_steer.publish(Bool(data=False))
            return

        if not self._motion_enable:
            # STM32: DUR — algoritma/model çalıştırma, güvenli kısıtları yayınla
            self._pub_estop.publish(Bool(data=True))
            self._pub_speed.publish(Float32(data=0.0))
            self._pub_steer.publish(Float32(data=0.0))
            self._pub_has_steer.publish(Bool(data=False))
            return

        stereo_fresh = (now - self._stereo_stamp) < self.STEREO_TIMEOUT_S
        lidar_fresh = (now - self._lidar_stamp) < self.LIDAR_TIMEOUT_S
        lane_fresh = (now - self._lane_walls_stamp) < self.LANE_WALLS_TIMEOUT_S
        stereo = self._stereo if stereo_fresh else []
        lidar = self._lidar if lidar_fresh else []

        frame = fuse(stereo=stereo, lidar=lidar, imu=self._imu)

        sign_state = self._sign.update(frame.sign_dets)
        light_state = self._light.update(frame.light_dets)
        # Sensör önceliği:
        # - Engeller: LiDAR öncelikli; stereo sadece "pedestrian" ile destek (sınıflandırma).
        # - Slalom: stereo "cone" (LiDAR'dan koni sınıfı gelmiyor varsayımı).
        lidar_obs = frame.lidar_obstacle_dets
        stereo_obs = frame.stereo_obstacle_dets
        obs_input = list(lidar_obs) + [d for d in stereo_obs if d.kind == "pedestrian"]
        slalom_input = [d for d in stereo_obs if d.kind == "cone"]

        obs_state = self._obstacle.update(obs_input)
        slalom_state = self._slalom.update(slalom_input)
        park_dets = parking_detections_from_signs(frame.sign_dets)
        park_state = self._parking.update(park_dets)
        self._pub_turn_permissions.publish(
            String(
                data=json.dumps(
                    {
                        "left": bool(sign_state.turn_permissions.left),
                        "straight": bool(sign_state.turn_permissions.straight),
                        "right": bool(sign_state.turn_permissions.right),
                        "forced_direction": sign_state.turn_permissions.forced_direction,
                    },
                    ensure_ascii=False,
                )
            )
        )

        # --- Decision Arbiter: adayları topla ve tek karar üret ---
        candidates: list[Candidate] = []

        # Işık / levha (kural)
        if light_state.must_stop:
            candidates.append(Candidate(name="light", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.LIGHT_MUST_STOP]))
        elif float(light_state.speed_cap_ratio) < 1.0:
            candidates.append(Candidate(name="light", emergency_stop=False, speed_cap=float(light_state.speed_cap_ratio), reasons=[ReasonCode.LIGHT_YELLOW_SLOW]))

        if sign_state.must_stop_soon:
            candidates.append(Candidate(name="sign", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.SIGN_MUST_STOP]))
        elif float(sign_state.speed_cap_ratio) < 1.0:
            candidates.append(Candidate(name="sign", emergency_stop=False, speed_cap=float(sign_state.speed_cap_ratio), reasons=[ReasonCode.SIGN_SPEED_CAP]))

        # Engel
        if obs_state.emergency_stop:
            candidates.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.OBSTACLE_EMERGENCY_STOP]))
        if obs_state.road_blocked:
            candidates.append(Candidate(name="obstacle", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.ROAD_BLOCKED]))
        if float(obs_state.speed_cap_ratio) < 1.0:
            reasons = [ReasonCode.STATIC_AVOID] if obs_state.suggest_lane_change else []
            candidates.append(Candidate(name="obstacle", emergency_stop=False, speed_cap=float(obs_state.speed_cap_ratio), reasons=reasons))

        # Park
        if self._park_mode and not park_state.complete:
            reasons = [ReasonCode.PARK_MODE]
            if bool(park_state.no_eligible_spot):
                reasons.append(ReasonCode.PARK_NO_ELIGIBLE)
            candidates.append(
                Candidate(
                    name="park",
                    emergency_stop=False,
                    speed_cap=float(park_state.speed_ratio),
                    steer_override=float(park_state.steering),
                    reasons=reasons,
                )
            )

        # Slalom (düşük öncelik)
        if slalom_state.aktif:
            candidates.append(
                Candidate(
                    name="slalom",
                    emergency_stop=False,
                    speed_cap=1.0,
                    steer_override=float(slalom_state.steering),
                    reasons=[ReasonCode.SLALOM],
                )
            )

        # Statik kaçınma override (lane ile clamp/disable)
        if obs_state.suggest_lane_change and not obs_state.road_blocked:
            steer_bias = -0.35 if obs_state.avoidance_direction == "left" else 0.35
            candidates.append(
                Candidate(
                    name="static_avoid",
                    emergency_stop=False,
                    speed_cap=0.35,
                    steer_override=float(steer_bias),
                    reasons=[ReasonCode.STATIC_AVOID],
                )
            )

        # Sensör fail-safe
        if not stereo_fresh and not lidar_fresh and bool(self.get_parameter("fail_safe_stop_on_all_sensors_lost").value):
            candidates.append(Candidate(name="failsafe", emergency_stop=True, speed_cap=0.0, reasons=[ReasonCode.ALL_SENSORS_LOST]))
        else:
            if not lidar_fresh:
                candidates.append(Candidate(name="failsafe", emergency_stop=False, speed_cap=float(self.get_parameter("lidar_missing_speed_cap").value)))
            if not stereo_fresh:
                candidates.append(Candidate(name="failsafe", emergency_stop=False, speed_cap=float(self.get_parameter("stereo_missing_speed_cap").value)))

        lane = self._lane_bounds if lane_fresh else None
        decision = self._arbiter.arbitrate(
            candidates=candidates,
            lane=lane,
            lane_required_for_avoidance=bool(self.get_parameter("require_lane_walls_for_avoidance").value),
        )

        if bool(self.get_parameter("publish_decision_debug").value):
            try:
                self._pub_decision_debug.publish(
                    String(
                        data=json.dumps(
                            {
                                "ts_monotonic": now,
                                "fresh": {
                                    "stereo": stereo_fresh,
                                    "lidar": lidar_fresh,
                                    "lane_walls": lane_fresh,
                                },
                                "lane_bounds": (
                                    None
                                    if lane is None
                                    else {
                                        "left_y_m": lane.left_y_m,
                                        "right_y_m": lane.right_y_m,
                                        "margin_m": lane.margin_m,
                                    }
                                ),
                                "candidates": [
                                    {
                                        "name": c.name,
                                        "emergency_stop": c.emergency_stop,
                                        "speed_cap": c.speed_cap,
                                        "steer_override": c.steer_override,
                                        "reasons": [r.value for r in c.reasons],
                                    }
                                    for c in candidates
                                ],
                                "final": {
                                    "emergency_stop": decision.emergency_stop,
                                    "speed_cap": decision.speed_cap,
                                    "has_steer_override": decision.has_steer_override,
                                    "steer_override": decision.steer_override,
                                    "reasons": [r.value for r in decision.reasons],
                                },
                            },
                            ensure_ascii=False,
                        )
                    )
                )
            except Exception:
                # Debug yayınları kritik değil; ana akışı bozmasın.
                pass

        self._pub_park_complete.publish(Bool(data=bool(park_state.complete)))

        self._pub_estop.publish(Bool(data=bool(decision.emergency_stop)))
        self._pub_speed.publish(Float32(data=float(decision.speed_cap)))
        self._pub_steer.publish(Float32(data=float(decision.steer_override)))
        self._pub_has_steer.publish(Bool(data=bool(decision.has_steer_override)))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

