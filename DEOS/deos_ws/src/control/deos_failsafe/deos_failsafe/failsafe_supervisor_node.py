import json
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import Bool, Float32, String

from deos_failsafe.control_evaluator import evaluate_control, twist_to_command_dict
from deos_failsafe.decision_engine_core import FailSafeDecisionCore
from deos_failsafe.planning_validator import PlanningValidator
from deos_failsafe.safety_types import FailSafeCommand, Health, PlanStatus


class FailsafeSupervisorNode(Node):
    """
    ROS 2 supervisor: sensör / algı / plan / kontrol sağlığı -> FailSafeDecisionCore.

    Topic hiyerarşisi (failsafe_root varsayılan /deos/failsafe):
      {root}/out/emergency_stop  (std_msgs/Bool)
      {root}/out/speed_cap       (std_msgs/Float32, 0..1)
      {root}/out/diagnostics    (std_msgs/String, JSON)
      {root}/in/fsm_reset       (std_msgs/Bool, true = acil durum kilidini kaldır)
    """

    def __init__(self) -> None:
        super().__init__("failsafe_supervisor_node")

        self.declare_parameter("failsafe_root", "/deos/failsafe")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("lidar_topic", "/cloud_unstructured_fullframe")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("perception_stereo_topic", "/perception/stereo_detections")
        self.declare_parameter("planning_speed_topic", "/planning/speed_limit")
        self.declare_parameter("planning_steer_topic", "/planning/steering_ref")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("sensor_timeout_s", 0.5)
        self.declare_parameter("perception_timeout_s", 0.6)
        self.declare_parameter("planning_timeout_s", 0.5)
        self.declare_parameter("max_vehicle_speed_mps", 3.0)
        self.declare_parameter("max_vehicle_steer_rad", 1.0)
        self.declare_parameter("planning_max_speed_mps", 4.0)
        self.declare_parameter("planning_max_curvature", 1.2)
        self.declare_parameter("planning_min_sensor_health", 0.45)
        self.declare_parameter("allow_planning_if_perception_degraded", True)
        self.declare_parameter("control_max_steering_ratio", 1.05)
        self.declare_parameter("control_max_throttle", 1.0)
        self.declare_parameter("control_max_brake", 1.0)
        self.declare_parameter("control_allowed_error", 1.5)
        self.declare_parameter("startup_grace_s", 4.0)

        self._node_start = time.monotonic()
        self._core = FailSafeDecisionCore()
        self._validator = PlanningValidator(
            {
                "max_speed": float(self.get_parameter("planning_max_speed_mps").value),
                "max_curvature": float(self.get_parameter("planning_max_curvature").value),
                "min_sensor_health": float(self.get_parameter("planning_min_sensor_health").value),
                "allow_planning_if_perception_degraded": bool(
                    self.get_parameter("allow_planning_if_perception_degraded").value
                ),
            }
        )

        self._cam_t: float | None = None
        self._lidar_t: float | None = None
        self._imu_t: float | None = None
        self._stereo_t: float | None = None
        self._plan_speed_t: float | None = None
        self._plan_steer_t: float | None = None
        self._last_cmd_vel: Twist | None = None
        self._last_cmd_t: float | None = None
        self._ctrl_track_err: float = 0.0
        self._plan_speed_val: float = 0.0
        self._plan_steer_val: float = 0.0

        root = str(self.get_parameter("failsafe_root").value).strip().rstrip("/")
        t_out_estop = f"{root}/out/emergency_stop"
        t_out_cap = f"{root}/out/speed_cap"
        t_out_diag = f"{root}/out/diagnostics"
        t_in_reset = f"{root}/in/fsm_reset"

        self.create_subscription(
            Image, str(self.get_parameter("camera_topic").value), self._on_cam, 10
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter("lidar_topic").value), self._on_lidar, 5
        )
        self.create_subscription(Imu, str(self.get_parameter("imu_topic").value), self._on_imu, 20)
        self.create_subscription(
            String,
            str(self.get_parameter("perception_stereo_topic").value),
            self._on_stereo,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("planning_speed_topic").value),
            self._on_plan_speed,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("planning_steer_topic").value),
            self._on_plan_steer,
            10,
        )
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self._on_cmd_vel, 10
        )
        self.create_subscription(Bool, t_in_reset, self._on_manual_reset, 10)

        self._pub_estop = self.create_publisher(Bool, t_out_estop, 10)
        self._pub_cap = self.create_publisher(Float32, t_out_cap, 10)
        self._pub_status = self.create_publisher(String, t_out_diag, 10)

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"failsafe_supervisor_node ready — root={root!r} "
            f"publishes {t_out_estop}, {t_out_cap}, {t_out_diag}; subscribes {t_in_reset}"
        )

    def _touch(self, slot: str) -> None:
        t = time.monotonic()
        if slot == "cam":
            self._cam_t = t
        elif slot == "lidar":
            self._lidar_t = t
        elif slot == "imu":
            self._imu_t = t
        elif slot == "stereo":
            self._stereo_t = t

    def _on_cam(self, _msg: Image) -> None:
        self._touch("cam")

    def _on_lidar(self, _msg: PointCloud2) -> None:
        self._touch("lidar")

    def _on_imu(self, _msg: Imu) -> None:
        self._touch("imu")

    def _on_stereo(self, _msg: String) -> None:
        self._touch("stereo")

    def _on_plan_speed(self, msg: Float32) -> None:
        self._plan_speed_val = float(msg.data)
        self._plan_speed_t = time.monotonic()

    def _on_plan_steer(self, msg: Float32) -> None:
        self._plan_steer_val = float(msg.data)
        self._plan_steer_t = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg
        self._last_cmd_t = time.monotonic()

    def _on_manual_reset(self, msg: Bool) -> None:
        if bool(msg.data):
            self._core.fsm.manual_reset()
            self.get_logger().warn("Fail-safe FSM manuel reset")

    def _sensor_health(self) -> Health:
        now = time.monotonic()
        grace = float(self.get_parameter("startup_grace_s").value)
        if (now - self._node_start) < grace:
            return Health(
                module_name="sensors",
                overall_score=1.0,
                is_healthy=True,
                status="OK",
                reason="startup grace",
            )

        timeout = float(self.get_parameter("sensor_timeout_s").value)
        status = "OK"
        confidence = 1.0
        reasons: list[str] = []

        def stale(last: float | None) -> bool:
            if last is None:
                return True
            return (now - last) > timeout

        if stale(self._cam_t):
            status = "CRITICAL"
            confidence -= 0.3
            reasons.append("Kamera Verisi Yok / Timeout")
        if stale(self._lidar_t):
            status = "CRITICAL"
            confidence -= 0.3
            reasons.append("Lidar Verisi Yok / Timeout")
        if stale(self._imu_t):
            status = "CRITICAL"
            confidence -= 0.3
            reasons.append("IMU Verisi Yok / Timeout")

        confidence = max(0.0, min(1.0, confidence))
        is_healthy = status != "CRITICAL"
        return Health(
            module_name="sensors",
            overall_score=confidence,
            is_healthy=is_healthy,
            status=status,
            reason=" | ".join(reasons) if reasons else "Tum sensorler stabil",
        )

    def _perception_health(self) -> Health:
        now = time.monotonic()
        grace = float(self.get_parameter("startup_grace_s").value)
        if (now - self._node_start) < grace:
            return Health(
                module_name="perception",
                overall_score=1.0,
                is_healthy=True,
                status="OK",
                reason="startup grace",
            )

        timeout = float(self.get_parameter("perception_timeout_s").value)
        if self._stereo_t is None or (now - self._stereo_t) > timeout:
            return Health(
                module_name="perception",
                overall_score=0.2,
                is_healthy=False,
                status="CRITICAL",
                reason="Stereo / algı verisi timeout",
            )
        return Health(
            module_name="perception",
            overall_score=1.0,
            is_healthy=True,
            status="OK",
            reason="Stereo akisi taze",
        )

    def _build_plan_dict(self, plan_fresh: bool, speed_ratio: float, steer_ratio: float) -> dict[str, Any]:
        max_v = float(self.get_parameter("max_vehicle_speed_mps").value)
        target_speed = abs(speed_ratio) * max_v if plan_fresh else 0.0
        trajectory = [(0.0, 0.0), (1.0, 0.0)] if plan_fresh else []
        return {
            "target_speed": target_speed,
            "max_curvature": abs(steer_ratio),
            "trajectory": trajectory,
            "collision_risk": False,
        }

    def _planning_status(self, sensor: Health, perception: Health) -> PlanStatus:
        now = time.monotonic()
        grace = float(self.get_parameter("startup_grace_s").value)
        if (now - self._node_start) < grace:
            return PlanStatus(is_valid=True, risk_level="SAFE", reason="startup grace")

        timeout = float(self.get_parameter("planning_timeout_s").value)
        plan_fresh = (
            self._plan_speed_t is not None
            and self._plan_steer_t is not None
            and (now - self._plan_speed_t) <= timeout
            and (now - self._plan_steer_t) <= timeout
        )
        if not plan_fresh:
            return PlanStatus(is_valid=True, risk_level="SAFE", reason="planning verisi bekleniyor")

        speed_ratio = self._plan_speed_val
        steer_ratio = self._plan_steer_val
        plan = self._build_plan_dict(True, speed_ratio, steer_ratio)
        return self._validator.validate(plan, sensor, perception)

    def _control_status(self) -> ControlStatus | None:
        if self._last_cmd_vel is None:
            return None
        max_v = float(self.get_parameter("max_vehicle_speed_mps").value)
        max_st = float(self.get_parameter("max_vehicle_steer_rad").value)
        cmd = twist_to_command_dict(self._last_cmd_vel, max_v, max_st)
        cfg: dict[str, Any] = {
            "max_steering_angle": float(self.get_parameter("control_max_steering_ratio").value),
            "max_throttle": float(self.get_parameter("control_max_throttle").value),
            "max_brake": float(self.get_parameter("control_max_brake").value),
            "max_control_latency_ms": 500.0,
            "allowed_error_threshold": float(self.get_parameter("control_allowed_error").value),
        }
        st, new_err = evaluate_control(cmd, {"tracking_error": self._ctrl_track_err}, cfg, self._ctrl_track_err)
        self._ctrl_track_err = new_err
        return st

    def _tick(self) -> None:
        sensor_h = self._sensor_health()
        perc_h = self._perception_health()
        plan_s = self._planning_status(sensor_h, perc_h)
        ctrl_s = self._control_status()

        cmd = self._core.evaluate(sensor_h, perc_h, plan_s, ctrl_s)

        estop = cmd.action == "EMERGENCY_STOP"
        cap = self._speed_cap_from_command(cmd)

        self._pub_estop.publish(Bool(data=estop))
        self._pub_cap.publish(Float32(data=float(cap)))
        self._pub_status.publish(
            String(
                data=json.dumps(
                    {
                        "fsm": self._core.fsm.get_state().name,
                        "action": cmd.action,
                        "speed_cap": cap,
                        "request_emergency_stop": estop,
                        "reason": cmd.reason,
                    },
                    ensure_ascii=False,
                )
            )
        )

    @staticmethod
    def _speed_cap_from_command(cmd: FailSafeCommand) -> float:
        if cmd.action == "EMERGENCY_STOP":
            return 0.0
        if cmd.action == "SLOW_DOWN":
            return 0.2
        if cmd.action == "CAREFUL":
            return 0.5
        return 1.0


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = FailsafeSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
