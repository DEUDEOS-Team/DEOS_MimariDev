from __future__ import annotations

import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int8


class LaneControlNode(Node):
    """
    /perception/center_pts -> şerit bazlı steer/speed referansı üretir.

    Mimari uyumu:
    - Bu node DOĞRUDAN /cmd_vel üretmez.
    - /lane/steering_ref ve /lane/speed_limit yayınlar.
    - vehicle_controller_node parametre ile bu lane referanslarını tercih edebilir (tazelik kontrolüyle).
    """

    def __init__(self):
        super().__init__("lane_control_node")

        self.declare_parameter("center_pts_topic", "/perception/center_pts")
        self.declare_parameter("lane_steer_topic", "/lane/steering_ref")
        self.declare_parameter("lane_speed_topic", "/lane/speed_limit")
        self.declare_parameter("camera_w", 1280)
        self.declare_parameter("camera_h", 960)
        self.declare_parameter("steer_cmd_mul", 0.65)
        self.declare_parameter("lane_lost_patience_s", 1.0)

        # intent (opsiyonel): 1=sol, 2=sağ, 0=reset
        self.declare_parameter("intent_topic", "/control/intent")
        self.declare_parameter("use_intent", False)

        self._cam_w = float(self.get_parameter("camera_w").value)
        self._cam_h = float(self.get_parameter("camera_h").value)

        self._pub_steer = self.create_publisher(Float32, str(self.get_parameter("lane_steer_topic").value), 10)
        self._pub_speed = self.create_publisher(Float32, str(self.get_parameter("lane_speed_topic").value), 10)

        self._center_pts_list: list[list[tuple[float, float]]] = []
        self._last_valid_steer = 0.0
        self._last_valid_throttle = 0.0
        self._last_lane_t = time.perf_counter()

        self._intent_queue = deque()
        if bool(self.get_parameter("use_intent").value):
            self.create_subscription(Int8, str(self.get_parameter("intent_topic").value), self._intent_cb, 10)

        self.create_subscription(Float32MultiArray, str(self.get_parameter("center_pts_topic").value), self._pts_cb, 10)

    def _intent_cb(self, msg: Int8) -> None:
        v = int(msg.data)
        if v in (1, 2):
            self._intent_queue.append(v)
        elif v == 0:
            self._intent_queue.clear()

    def _calculate_commands(self) -> tuple[float, float]:
        now = time.perf_counter()
        steer, throttle = 0.0, 0.0

        if self._center_pts_list and self._center_pts_list[0]:
            pts = self._center_pts_list[0]
            # target point: mid-ish; if intent exists, bias to near field (turn prep)
            if self._intent_queue:
                target_idx = min(2, len(pts) - 1)
            else:
                target_idx = min(len(pts) // 2 + 2, len(pts) - 1)

            target_x, _ = pts[target_idx]
            error = (float(target_x) - (self._cam_w / 2.0)) / (self._cam_w / 2.0)
            steer = float(np.clip(error, -1.0, 1.0))

            # simple speed policy
            throttle = 0.5 if abs(steer) < 0.2 else 0.35

            steer = float(np.clip(steer * float(self.get_parameter("steer_cmd_mul").value), -1.0, 1.0))
            self._last_valid_steer = steer
            self._last_valid_throttle = throttle
            self._last_lane_t = now
            return steer, throttle

        # lane lost fallback
        patience = float(self.get_parameter("lane_lost_patience_s").value)
        if (now - self._last_lane_t) < patience:
            return self._last_valid_steer, self._last_valid_throttle * 0.8
        return 0.0, 0.0

    def _pts_cb(self, msg: Float32MultiArray) -> None:
        pts_data = list(msg.data)
        if pts_data:
            self._center_pts_list = [[(pts_data[i], pts_data[i + 1]) for i in range(0, len(pts_data), 2)]]
        else:
            self._center_pts_list = []

        steer, throttle = self._calculate_commands()

        self._pub_steer.publish(Float32(data=float(steer)))       # -1..1
        self._pub_speed.publish(Float32(data=float(throttle)))    # 0..1 (ratio)


def main(args=None):
    rclpy.init(args=args)
    node = LaneControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

