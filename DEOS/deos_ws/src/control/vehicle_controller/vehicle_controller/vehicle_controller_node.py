import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from deos_algorithms.ros_topic_layout import build_deos_topics

from deos_logging.logger import DeosLogger


class VehicleControllerNode(Node):
    PERCEPTION_TIMEOUT_S = 0.2
    LANE_TIMEOUT_S = 0.2

    def __init__(self):
        super().__init__("vehicle_controller_node")
        self.logger = DeosLogger(self.get_logger(), "vehicle_controller_node")

        self.declare_parameter("max_speed_mps", 3.0)
        self.declare_parameter("max_steer_rads", 1.0)
        self.declare_parameter("safety_emergency_stop_pulse_count", 3)
        # True: acil dur sırasında sıfır hız Twist yayınla (STM32 bridge negatif delta üretir).
        # Birincil stop yolu /safety/emergency_stop via microROS; bu yazılım katmanı ikincil güvencedir.
        self.declare_parameter("publish_cmd_vel_in_emergency", True)
        self.declare_parameter("publish_safety_emergency_stop_false_on_clear", True)
        self.declare_parameter("deos_root", "/deos")
        _T = build_deos_topics(str(self.get_parameter("deos_root").value))
        # Opsiyonel: lane_control referanslarını kullan (GPS planning yerine)
        self.declare_parameter("use_lane_control", True)
        self.declare_parameter("lane_steering_topic", _T["lane_steering_ref"])
        self.declare_parameter("lane_speed_topic", _T["lane_speed_limit"])
        self.declare_parameter("lane_timeout_s", 0.2)
        # Opsiyonel çift kilit: STM32 komutunu doğrudan dinle (perception ile aynı semantik)
        self.declare_parameter("subscribe_hardware_motion_enable", True)
        self.declare_parameter("hardware_motion_enable_topic", _T["hardware_motion_enable"])
        self.declare_parameter("hardware_motion_enable_timeout_s", 0.5)
        self.declare_parameter("hardware_motion_enable_fail_safe_stop", True)
        # Manuel/otonom ayrımı: otonom kapalıysa /cmd_vel publish etme (STM32 manual sürüşe geçebilir)
        self.declare_parameter("subscribe_autonomy_enable", True)
        self.declare_parameter("autonomy_enable_topic", _T["hardware_autonomy_enable"])
        self.declare_parameter("subscribe_failsafe", True)
        self.declare_parameter("green_elapsed_s_topic", _T["perception_fusion_green_elapsed_s"])
        self.declare_parameter("perception_emergency_stop_topic", _T["perception_fusion_emergency_stop"])
        self.declare_parameter("perception_speed_cap_topic", _T["perception_fusion_speed_cap"])
        self.declare_parameter("perception_steering_override_topic", _T["perception_fusion_steering_override"])
        self.declare_parameter("perception_has_steering_override_topic", _T["perception_fusion_has_steering_override"])
        self.declare_parameter("planning_steering_ref_topic", _T["planning_steering_ref"])
        self.declare_parameter("planning_speed_limit_topic", _T["planning_speed_limit"])
        self.declare_parameter("cmd_vel_topic", _T["control_cmd_vel"])
        self.declare_parameter("safety_emergency_stop_topic", _T["safety_emergency_stop"])
        self._max_speed = float(self.get_parameter("max_speed_mps").value)
        self._max_steer = float(self.get_parameter("max_steer_rads").value)

        self._emergency_stop: bool = False
        self._speed_cap: float = 1.0
        self._steer_override: float = 0.0
        self._has_steer_override: bool = False
        self._perception_stamp: float = 0.0

        self._plan_steer: float = 0.0
        self._plan_speed: float = 0.0
        self._lane_steer: float = 0.0
        self._lane_speed: float = 0.0
        self._lane_stamp: float = 0.0

        self._prev_emergency_active: bool = False
        self._safety_estop_pulse_remaining: int = 0
        self._prev_raw_estop: bool = False

        # STM32 -> Pi (std_msgs/Bool): false=DUR, true=DEVAM; olay bazlı; perception ile aynı timeout fail-safe
        self._hw_motion_enable: bool = False
        self._hw_motion_stamp: float = 0.0
        self._autonomy_enable: bool = True
        self._autonomy_stamp: float = 0.0

        self._failsafe_estop: bool = False
        self._failsafe_speed_cap: float = 1.0
        # Yeşil ışık geçen süre: -1.0 = yeşil ışık yok (şartname tepki süresi takibi)
        self._green_elapsed_s: float = -1.0

        self.create_subscription(
            Bool, str(self.get_parameter("perception_emergency_stop_topic").value), self._estop_cb, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("perception_speed_cap_topic").value), self._speed_cap_cb, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("perception_steering_override_topic").value), self._steer_ovr_cb, 10
        )
        self.create_subscription(
            Bool, str(self.get_parameter("perception_has_steering_override_topic").value), self._has_steer_cb, 10
        )

        self.create_subscription(
            Float32, str(self.get_parameter("planning_steering_ref_topic").value), self._plan_steer_cb, 10
        )
        self.create_subscription(
            Float32, str(self.get_parameter("planning_speed_limit_topic").value), self._plan_speed_cb, 10
        )

        if bool(self.get_parameter("use_lane_control").value):
            self.create_subscription(Float32, str(self.get_parameter("lane_steering_topic").value), self._lane_steer_cb, 10)
            self.create_subscription(Float32, str(self.get_parameter("lane_speed_topic").value), self._lane_speed_cb, 10)
            self.logger.info(
                "lane_control enabled — "
                f"steer={str(self.get_parameter('lane_steering_topic').value)}, "
                f"speed={str(self.get_parameter('lane_speed_topic').value)}"
            )

        if bool(self.get_parameter("subscribe_hardware_motion_enable").value):
            hw_topic = str(self.get_parameter("hardware_motion_enable_topic").value)
            self.create_subscription(Bool, hw_topic, self._hw_motion_enable_cb, 10)
            self.logger.info(f"hardware motion_enable subscription enabled on {hw_topic}")

        if bool(self.get_parameter("subscribe_autonomy_enable").value):
            at = str(self.get_parameter("autonomy_enable_topic").value)
            self.create_subscription(Bool, at, self._autonomy_enable_cb, 10)
            self.logger.info(f"autonomy_enable subscription enabled on {at}")

        if bool(self.get_parameter("subscribe_failsafe").value):
            fet = _T["failsafe_out_emergency_stop"]
            fst = _T["failsafe_out_speed_cap"]
            self.create_subscription(Bool, fet, self._failsafe_estop_cb, 10)
            self.create_subscription(Float32, fst, self._failsafe_speed_cap_cb, 10)
            self.logger.info(f"failsafe topics: {fet}, {fst}")

        self.create_subscription(
            Float32,
            str(self.get_parameter("green_elapsed_s_topic").value),
            self._green_elapsed_cb,
            10,
        )

        cmd_out = str(self.get_parameter("cmd_vel_topic").value)
        self._pub_cmd = self.create_publisher(Twist, cmd_out, 10)
        self._pub_estop = self.create_publisher(
            Bool, str(self.get_parameter("safety_emergency_stop_topic").value), 10
        )

        self.create_timer(0.05, self._tick)
        self.logger.info(
            "vehicle_controller_node ready — "
            f"max_speed={self._max_speed} m/s, max_steer={self._max_steer} rad/s, "
            f"estop_pulse_count={int(self.get_parameter('safety_emergency_stop_pulse_count').value)}, "
            f"publish_cmd_vel_in_emergency={bool(self.get_parameter('publish_cmd_vel_in_emergency').value)}"
        )

    def _hardware_motion_allows_actuation(self) -> bool:
        """STM32 canlı ve DEVAM (true) ise True; aksi halde False (DUR / mesaj yok / timeout)."""
        now = time.monotonic()
        timeout_s = float(self.get_parameter("hardware_motion_enable_timeout_s").value)
        fail_safe = bool(self.get_parameter("hardware_motion_enable_fail_safe_stop").value)
        motion_ok = (now - self._hw_motion_stamp) <= timeout_s
        if not motion_ok and fail_safe:
            return False
        return bool(self._hw_motion_enable)

    def _hw_motion_enable_cb(self, msg: Bool) -> None:
        prev = bool(self._hw_motion_enable)
        self._hw_motion_enable = bool(msg.data)
        self._hw_motion_stamp = time.monotonic()
        # False'a düşüşte STM32'ye kısa pulse (perception path'i çökse bile)
        if prev and not self._hw_motion_enable:
            pulse_n = int(self.get_parameter("safety_emergency_stop_pulse_count").value)
            pulse_n = max(1, min(10, pulse_n))
            self._safety_estop_pulse_remaining = max(self._safety_estop_pulse_remaining, pulse_n)

    def _autonomy_enable_cb(self, msg: Bool) -> None:
        self._autonomy_enable = bool(msg.data)
        self._autonomy_stamp = time.monotonic()

    def _green_elapsed_cb(self, msg: Float32) -> None:
        self._green_elapsed_s = float(msg.data)

    def _failsafe_estop_cb(self, msg: Bool) -> None:
        self._failsafe_estop = bool(msg.data)

    def _failsafe_speed_cap_cb(self, msg: Float32) -> None:
        v = float(msg.data)
        self._failsafe_speed_cap = max(0.0, min(1.0, v))

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

    def _lane_steer_cb(self, msg: Float32) -> None:
        self._lane_steer = float(msg.data)
        self._lane_stamp = time.monotonic()

    def _lane_speed_cb(self, msg: Float32) -> None:
        self._lane_speed = float(msg.data)
        self._lane_stamp = time.monotonic()

    def _tick(self) -> None:
        cmd = Twist()
        perception_age = time.monotonic() - self._perception_stamp

        hw_hold = not self._hardware_motion_allows_actuation() if bool(
            self.get_parameter("subscribe_hardware_motion_enable").value
        ) else False

        motion_hold = bool(
            self._emergency_stop
            or self._failsafe_estop
            or perception_age > self.PERCEPTION_TIMEOUT_S
            or hw_hold
        )

        autonomy_hold = False
        if bool(self.get_parameter("subscribe_autonomy_enable").value):
            autonomy_hold = not bool(self._autonomy_enable)

        if autonomy_hold:
            # Manual mode: do not publish /cmd_vel (avoid fighting manual driver),
            # and do not pulse /safety/emergency_stop.
            if self._prev_emergency_active and bool(
                self.get_parameter("publish_safety_emergency_stop_false_on_clear").value
            ):
                self._pub_estop.publish(Bool(data=False))
            self._prev_emergency_active = False
            self._safety_estop_pulse_remaining = 0
            return

        if motion_hold:
            if self._failsafe_estop:
                pulse_n = int(self.get_parameter("safety_emergency_stop_pulse_count").value)
                pulse_n = max(1, min(10, pulse_n))
                self._safety_estop_pulse_remaining = max(self._safety_estop_pulse_remaining, pulse_n)
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

        lane_ok = False
        if bool(self.get_parameter("use_lane_control").value):
            lane_timeout_s = float(self.get_parameter("lane_timeout_s").value)
            lane_ok = (time.monotonic() - self._lane_stamp) <= lane_timeout_s

        base_speed = self._plan_speed
        base_steer = self._plan_steer
        if lane_ok:
            base_speed = min(base_speed, self._lane_speed)
            base_steer = self._lane_steer

        fs_cap = (
            float(self._failsafe_speed_cap)
            if bool(self.get_parameter("subscribe_failsafe").value)
            else 1.0
        )
        speed_ratio = min(self._speed_cap, fs_cap, base_speed)
        speed_ratio = max(0.0, min(1.0, speed_ratio))

        steer_ratio = self._steer_override if self._has_steer_override else base_steer
        steer_ratio = max(-1.0, min(1.0, steer_ratio))

        cmd.linear.x = speed_ratio * self._max_speed
        cmd.angular.z = steer_ratio * self._max_steer

        # Şartname tepki süresi diagnostiği: yeşil ışıkta araç duruyorsa log üret
        if self._green_elapsed_s >= 0.0:
            elapsed = self._green_elapsed_s
            if elapsed > 30.0 and speed_ratio < 0.05:
                self.logger.warning(
                    f"YESIL_ISIK_GECIKME: {elapsed:.1f}s gecti, arac duruyor — sartname -20p riski"
                )
            elif elapsed > 5.0 and speed_ratio < 0.05:
                self.logger.debug(
                    f"YESIL_ISIK_GECIKME: {elapsed:.1f}s gecti, arac duruyor — sartname +20p bandi"
                )

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

