from __future__ import annotations

from typing import Any

from deos_failsafe.safety_types import Health, PlanStatus


class PlanningValidator:
    """Ported from Fail-Safe/monitor/planning_validator.py (validate-only)."""

    def __init__(self, thresholds: dict[str, Any]) -> None:
        self.thresholds = thresholds

    def validate(
        self,
        plan: dict[str, Any],
        sensor_health: Health,
        perception_health: Health,
    ) -> PlanStatus:
        reasons: list[str] = []
        risk_level = "SAFE"
        is_valid = True

        if sensor_health.overall_score < self.thresholds["min_sensor_health"]:
            risk_level = "CRITICAL"
            is_valid = False
            reasons.append(f"Sensor sagligi cok dusuk: {sensor_health.overall_score:.2f}")

        if perception_health.overall_score < 0.6:
            risk_level = self._upgrade_risk(risk_level, "WARNING")
            reasons.append(f"Algilama zayif: {perception_health.overall_score:.2f}")
            if not self.thresholds.get("allow_planning_if_perception_degraded", False):
                is_valid = False

        speed = float(plan.get("target_speed", 0.0))
        if speed > float(self.thresholds["max_speed"]):
            risk_level = "CRITICAL"
            is_valid = False
            reasons.append(f"Hiz limiti ihlali: {speed}")

        curvature = float(plan.get("max_curvature", 0.0))
        if curvature > float(self.thresholds["max_curvature"]):
            risk_level = self._upgrade_risk(risk_level, "WARNING")
            reasons.append(f"Yuksek kavis riski: {curvature}")

        path = plan.get("trajectory", [])
        if not path or len(path) < 2:
            risk_level = "CRITICAL"
            is_valid = False
            reasons.append("Gecersiz rota")

        if plan.get("collision_risk", False):
            risk_level = "CRITICAL"
            is_valid = False
            reasons.append("Planner carpisma riski")

        return PlanStatus(
            is_valid=is_valid,
            risk_level=risk_level,
            reason=" | ".join(reasons) if reasons else "OK",
        )

    @staticmethod
    def _upgrade_risk(current: str, new: str) -> str:
        order = ["SAFE", "WARNING", "CRITICAL"]
        return new if order.index(new) > order.index(current) else current
