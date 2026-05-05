import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class VehicleControllerNode(Node):
    PERCEPTION_TIMEOUT_S = 0.2

    def __init__(self):
        super().__init__("vehicle_controller_node")

        self.declare_parameter("max_speed_mps", 3.0)
        self.declare_parameter("max_steer_rads", 1.0)
        self.declare_parameter("safety_emergency_stop_pulse_count", 3)
        self.declare_parameter("publish_cmd_vel_in_emergency", False)
        self.declare_parameter("publish_safety_emergency_stop_false_on_clear", True)
        self._max_speed = float(self.get_parameter("max_speed_mps").value)
        self._max_steer = float(self.get_parameter("max_steer_rads").value)

        self._emergency_stop: bool = False
        self._speed_cap: float = 1.0
        self._steer_override: float = 0.0
        self._has_steer_override: bool = False
        self._perception_stamp: float = 0.0

        self._plan_steer: float = 0.0
        self._plan_speed: float = 0.0

        self._prev_emergency_active: bool = False
        self._safety_estop_pulse_remaining: int = 0
        self._prev_raw_estop: bool = False

        self.create_subscription(Bool, "/perception/emergency_stop", self._estop_cb, 10)
        self.create_subscription(Float32, "/perception/speed_cap", self._speed_cap_cb, 10)
        self.create_subscription(Float32, "/perception/steering_override", self._steer_ovr_cb, 10)
        self.create_subscription(Bool, "/perception/has_steering_override", self._has_steer_cb, 10)

        self.create_subscription(Float32, "/planning/steering_ref", self._plan_steer_cb, 10)
        self.create_subscription(Float32, "/planning/speed_limit", self._plan_speed_cb, 10)

        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_estop = self.create_publisher(Bool, "/safety/emergency_stop", 10)

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "vehicle_controller_node ready — "
            f"max_speed={self._max_speed} m/s, max_steer={self._max_steer} rad/s, "
            f"estop_pulse_count={int(self.get_parameter('safety_emergency_stop_pulse_count').value)}, "
            f"publish_cmd_vel_in_emergency={bool(self.get_parameter('publish_cmd_vel_in_emergency').value)}"
        )

    def _touch_perception(self) -> None:
        self._perception_stamp = time.monotonic()

    def _estop_cb(self, msg: Bool) -> None:
        new_val = bool(msg.data)
        # Pulse sadece gerçek estop talebinin yükselen kenarında tetiklensin (timeout vb. durumlarda spam olmasın)
        if new_val and not self._prev_raw_estop:
            pulse_n = int(self.get_parameter("safety_emergency_stop_pulse_count").value)
            pulse_n = max(1, min(10, pulse_n))
            self._safety_estop_pulse_remaining = pulse_n

        self._prev_raw_estop = new_val
        self._emergency_stop = new_val
        self._touch_perception()

    def _speed_cap_cb(self, msg: Float32) -> None:
        self._speed_cap = float(msg.data)
        self._touch_perception()

    def _steer_ovr_cb(self, msg: Float32) -> None:
        self._steer_override = float(msg.data)

    def _has_steer_cb(self, msg: Bool) -> None:
        self._has_steer_override = bool(msg.data)

    def _plan_steer_cb(self, msg: Float32) -> None:
        self._plan_steer = float(msg.data)

    def _plan_speed_cb(self, msg: Float32) -> None:
        self._plan_speed = float(msg.data)

    def _tick(self) -> None:
        cmd = Twist()
        perception_age = time.monotonic() - self._perception_stamp

        motion_hold = bool(self._emergency_stop or perception_age > self.PERCEPTION_TIMEOUT_S)

        if motion_hold:
            # /safety/emergency_stop: sınırlı sayıda True pulse (estop_cb yükselen kenarda doldurulur)
            if self._safety_estop_pulse_remaining > 0:
                self._pub_estop.publish(Bool(data=True))
                self._safety_estop_pulse_remaining -= 1

            if bool(self.get_parameter("publish_cmd_vel_in_emergency").value):
                self._pub_cmd.publish(cmd)

            self._prev_emergency_active = True
            return

        # motion_hold=false iken bekleyen pulse varsa iptal et (estop kalktıktan sonra True spam olmasın)
        self._safety_estop_pulse_remaining = 0

        # Acil durumdan çıkış: istenirse tek seferlik false publish (son durumu netleştirmek için)
        if self._prev_emergency_active and bool(
            self.get_parameter("publish_safety_emergency_stop_false_on_clear").value
        ):
            self._pub_estop.publish(Bool(data=False))

        self._prev_emergency_active = False

        speed_ratio = min(self._speed_cap, self._plan_speed)
        speed_ratio = max(0.0, min(1.0, speed_ratio))

        steer_ratio = self._steer_override if self._has_steer_override else self._plan_steer
        steer_ratio = max(-1.0, min(1.0, steer_ratio))

        cmd.linear.x = speed_ratio * self._max_speed
        cmd.angular.z = steer_ratio * self._max_steer

        self._pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

