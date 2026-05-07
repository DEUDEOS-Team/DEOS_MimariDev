import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(x)))


class Stm32BridgeNode(Node):
    """
    Bridge:
      /cmd_vel (Twist) -> /stm32/speed_delta_mps (Float32) + /stm32/steering_deg (Float32)

    Expectations (from user):
      - speed command: delta m/s (+/-)
      - steering command: absolute degrees in [-540, +540]
    """

    def __init__(self):
        super().__init__("stm32_bridge_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motion_enable_topic", "/hardware/motion_enable")
        self.declare_parameter("require_motion_enable", True)
        self.declare_parameter("autonomy_enable_topic", "/hardware/autonomy_enable")
        self.declare_parameter("require_autonomy_enable", True)
        self.declare_parameter("speed_delta_topic", "/stm32/speed_delta_mps")
        self.declare_parameter("speed_target_topic", "/stm32/speed_target_mps")
        self.declare_parameter("publish_speed_delta", True)
        self.declare_parameter("publish_speed_target", False)
        self.declare_parameter("steering_deg_topic", "/stm32/steering_deg")
        self.declare_parameter("max_steer_rads", 1.0)  # should match vehicle_controller_node
        self.declare_parameter("steer_deg_limit", 540.0)
        self.declare_parameter("round_decimals", 2)

        self._last_speed_target: float | None = None
        self._motion_enable: bool = False
        self._motion_stamp: float = 0.0
        self._autonomy_enable: bool = True
        self._autonomy_stamp: float = 0.0

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        motion_topic = str(self.get_parameter("motion_enable_topic").value)
        autonomy_topic = str(self.get_parameter("autonomy_enable_topic").value)

        self._pub_speed_delta = self.create_publisher(Float32, str(self.get_parameter("speed_delta_topic").value), 10)
        self._pub_speed_target = self.create_publisher(Float32, str(self.get_parameter("speed_target_topic").value), 10)
        self._pub_steer_deg = self.create_publisher(
            Float32, str(self.get_parameter("steering_deg_topic").value), 10
        )

        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)
        self.create_subscription(Bool, motion_topic, self._motion_cb, 10)
        self.create_subscription(Bool, autonomy_topic, self._autonomy_cb, 10)

        self.get_logger().info(
            "stm32_bridge_node ready — "
            f"cmd_vel={cmd_topic} -> "
            f"speed_delta={str(self.get_parameter('speed_delta_topic').value)}, "
            f"steer_deg={str(self.get_parameter('steering_deg_topic').value)} "
            f"(motion_enable={motion_topic}, autonomy_enable={autonomy_topic})"
        )

    def _motion_cb(self, msg: Bool) -> None:
        self._motion_enable = bool(msg.data)
        self._motion_stamp = time.monotonic()

    def _cmd_cb(self, msg: Twist) -> None:
        require_motion = bool(self.get_parameter("require_motion_enable").value)
        require_auto = bool(self.get_parameter("require_autonomy_enable").value)
        if (require_motion and not self._motion_enable) or (require_auto and not self._autonomy_enable):
            # If not enabled, publish neutral commands
            if bool(self.get_parameter("publish_speed_delta").value):
                self._pub_speed_delta.publish(Float32(data=0.0))
            if bool(self.get_parameter("publish_speed_target").value):
                self._pub_speed_target.publish(Float32(data=0.0))
            self._pub_steer_deg.publish(Float32(data=0.0))
            self._last_speed_target = float(msg.linear.x)
            return

        speed_target = float(msg.linear.x)
        if self._last_speed_target is None:
            speed_delta = 0.0
        else:
            speed_delta = speed_target - float(self._last_speed_target)
        self._last_speed_target = speed_target

        max_steer_rads = float(self.get_parameter("max_steer_rads").value)
        steer_deg_limit = float(self.get_parameter("steer_deg_limit").value)
        if abs(max_steer_rads) < 1e-6:
            steer_ratio = 0.0
        else:
            steer_ratio = float(msg.angular.z) / max_steer_rads
        steer_deg = _clamp(steer_ratio * steer_deg_limit, -steer_deg_limit, steer_deg_limit)

        dec = int(self.get_parameter("round_decimals").value)
        try:
            speed_delta = round(speed_delta, dec)
            speed_target = round(speed_target, dec)
            steer_deg = round(steer_deg, dec)
        except Exception:
            pass

        if bool(self.get_parameter("publish_speed_delta").value):
            self._pub_speed_delta.publish(Float32(data=float(speed_delta)))
        if bool(self.get_parameter("publish_speed_target").value):
            self._pub_speed_target.publish(Float32(data=float(speed_target)))
        self._pub_steer_deg.publish(Float32(data=float(steer_deg)))

    def _autonomy_cb(self, msg: Bool) -> None:
        self._autonomy_enable = bool(msg.data)
        self._autonomy_stamp = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Stm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

