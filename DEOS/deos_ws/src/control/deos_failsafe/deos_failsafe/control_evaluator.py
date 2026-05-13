from __future__ import annotations

from typing import Any

from geometry_msgs.msg import Twist

from deos_failsafe.safety_types import ControlStatus


def twist_to_command_dict(twist: Twist, max_speed_mps: float, max_steer_rad: float) -> dict[str, float]:
    """Map /cmd_vel style command to throttle/brake/steering proxy for monitoring."""
    if max_speed_mps <= 0.0:
        max_speed_mps = 1.0
    if max_steer_rad <= 0.0:
        max_steer_rad = 1.0
    lin = float(twist.linear.x)
    ang = float(twist.angular.z)
    throttle = max(0.0, min(1.0, lin / max_speed_mps)) if lin >= 0 else 0.0
    brake = max(0.0, min(1.0, (-lin) / max_speed_mps)) if lin < 0 else 0.0
    steering = max(-1.0, min(1.0, ang / max_steer_rad))
    return {"throttle": throttle, "brake": brake, "steering": steering}


def evaluate_control(
    control_command: dict[str, float],
    feedback: dict[str, float],
    cfg: dict[str, Any],
    last_error: float,
) -> tuple[ControlStatus, float]:
    """
    Ported from Fail-Safe/monitor/control_monitor.py evaluate().
    Returns (status, new_last_error).
    """
    reasons: list[str] = []
    risk = "SAFE"
    control_enabled = True

    steering = abs(float(control_command.get("steering", 0.0)))
    if steering > float(cfg["max_steering_angle"]):
        risk = "CRITICAL"
        control_enabled = False
        reasons.append(f"Steering saturation: {steering:.3f}")

    throttle = float(control_command.get("throttle", 0.0))
    brake = float(control_command.get("brake", 0.0))

    if throttle > float(cfg["max_throttle"]):
        risk = _upgrade(risk, "WARNING")
        reasons.append("Throttle too high")

    if brake > float(cfg["max_brake"]):
        risk = "CRITICAL"
        control_enabled = False
        reasons.append("Brake saturation")

    error = float(feedback.get("tracking_error", 0.0))
    if abs(error) > float(cfg["allowed_error_threshold"]):
        risk = _upgrade(risk, "WARNING")
        reasons.append(f"High tracking error: {error:.3f}")

    error_delta = abs(error - last_error)
    if error_delta > 2.5:
        risk = "CRITICAL"
        control_enabled = False
        reasons.append("PID instability (error jump)")

    is_healthy = control_enabled
    return (
        ControlStatus(
            is_healthy=is_healthy,
            risk_level=risk,
            reason=" | ".join(reasons) if reasons else "OK",
            control_enabled=control_enabled,
        ),
        error,
    )


def _upgrade(current: str, new: str) -> str:
    order = ["SAFE", "WARNING", "CRITICAL"]
    return new if order.index(new) > order.index(current) else current
