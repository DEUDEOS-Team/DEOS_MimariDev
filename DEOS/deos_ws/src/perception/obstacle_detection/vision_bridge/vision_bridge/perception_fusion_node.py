import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String

from deos_algorithms.obstacle_logic import ObstacleLogic
from deos_algorithms.parking_logic import ParkingLogic
from deos_algorithms.perception_fusion import fuse
from deos_algorithms.sensors.types import ImuSample, LidarObstacle, StereoBbox
from deos_algorithms.slalom_logic import SlalomLogic
from deos_algorithms.traffic_light_logic import TrafficLightLogic
from deos_algorithms.traffic_sign_logic import TrafficSignLogic


class PerceptionFusionNode(Node):
    STEREO_TIMEOUT_S = 0.5
    LIDAR_TIMEOUT_S = 0.5

    def __init__(self):
        super().__init__("perception_fusion_node")

        self.declare_parameter("hardware_motion_enable_topic", "/hardware/motion_enable")
        self.declare_parameter("hardware_motion_enable_timeout_s", 0.5)
        self.declare_parameter("hardware_motion_enable_fail_safe_stop", True)

        self._sign = TrafficSignLogic()
        self._light = TrafficLightLogic()
        self._obstacle = ObstacleLogic()
        self._slalom = SlalomLogic()
        self._parking = ParkingLogic()

        self._stereo: list[StereoBbox] = []
        self._lidar: list[LidarObstacle] = []
        self._imu: ImuSample | None = None
        self._stereo_stamp: float = 0.0
        self._lidar_stamp: float = 0.0

        # STM32 -> Pi: tek topic, olay bazlı (sadece değişimde publish edilir)
        # std_msgs/Bool: false = DUR (algoritmalar durur), true = DEVAM
        # STM32 ilk "DEVAM" mesajını gönderene kadar güvenli tarafta kal
        self._motion_enable: bool = False
        self._motion_enable_stamp: float = 0.0

        self.create_subscription(String, "/perception/stereo_detections", self._stereo_cb, 10)
        self.create_subscription(String, "/perception/lidar_obstacles", self._lidar_cb, 10)
        self.create_subscription(Imu, "/imu/data", self._imu_cb, 10)

        motion_topic = str(self.get_parameter("hardware_motion_enable_topic").value)
        self.create_subscription(Bool, motion_topic, self._motion_enable_cb, 10)

        self._pub_estop = self.create_publisher(Bool, "/perception/emergency_stop", 10)
        self._pub_speed = self.create_publisher(Float32, "/perception/speed_cap", 10)
        self._pub_steer = self.create_publisher(Float32, "/perception/steering_override", 10)
        self._pub_has_steer = self.create_publisher(Bool, "/perception/has_steering_override", 10)

        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info(
            "perception_fusion_node ready — "
            f"STM32 motion topic={motion_topic} (Bool: false=STOP algorithms, true=RUN)"
        )

    def _motion_enable_cb(self, msg: Bool) -> None:
        self._motion_enable = bool(msg.data)
        self._motion_enable_stamp = time.monotonic()

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

        stereo = self._stereo if (now - self._stereo_stamp) < self.STEREO_TIMEOUT_S else []
        lidar = self._lidar if (now - self._lidar_stamp) < self.LIDAR_TIMEOUT_S else []

        frame = fuse(stereo=stereo, lidar=lidar, imu=self._imu)

        sign_state = self._sign.update(frame.sign_dets)
        light_state = self._light.update(frame.light_dets)
        obs_state = self._obstacle.update(frame.obstacle_dets)
        slalom_state = self._slalom.update(frame.obstacle_dets)
        _park_state = self._parking.update([])  # YOLO stub: park tespiti yok

        emergency = bool(obs_state.emergency_stop or light_state.must_stop or sign_state.must_stop_soon)
        speed_cap = float(min(obs_state.speed_cap_ratio, sign_state.speed_cap_ratio, light_state.speed_cap_ratio))

        has_steer = bool(slalom_state.aktif)
        steer = float(slalom_state.steering if slalom_state.aktif else 0.0)

        self._pub_estop.publish(Bool(data=emergency))
        self._pub_speed.publish(Float32(data=speed_cap))
        self._pub_steer.publish(Float32(data=steer))
        self._pub_has_steer.publish(Bool(data=has_steer))


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

