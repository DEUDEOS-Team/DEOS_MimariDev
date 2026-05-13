from __future__ import annotations

from deos_failsafe.fsm import AutonomousFSM, SystemState
from deos_failsafe.safety_types import ControlStatus, FailSafeCommand, Health, PlanStatus


class FailSafeDecisionCore:
    """
    Ported from Fail-Safe/manager/decision_engine.py without in-process topics.
    Produces FailSafeCommand from latest health snapshots.
    """

    def __init__(self) -> None:
        self.fsm = AutonomousFSM()

    def evaluate(
        self,
        latest_sensor: Health | None,
        latest_perception: Health | None,
        latest_plan: PlanStatus | None,
        latest_control: ControlStatus | None,
    ) -> FailSafeCommand:
        critical_errors: list[str] = []
        warnings: list[str] = []

        if latest_sensor:
            if latest_sensor.status == "CRITICAL" or not latest_sensor.is_healthy:
                critical_errors.append(f"Sensor: {latest_sensor.reason}")
            elif latest_sensor.status == "WARNING":
                warnings.append(f"Sensor: {latest_sensor.reason}")

        if latest_perception:
            if latest_perception.status == "CRITICAL" or not latest_perception.is_healthy:
                critical_errors.append(f"Algilama: {latest_perception.reason}")
            elif latest_perception.status == "WARNING":
                warnings.append(f"Algilama: {latest_perception.reason}")

        if latest_plan:
            if latest_plan.risk_level == "CRITICAL" or not latest_plan.is_valid:
                critical_errors.append(f"Planlama: {latest_plan.reason}")
            elif latest_plan.risk_level == "WARNING":
                warnings.append(f"Planlama: {latest_plan.reason}")

        if latest_control:
            if latest_control.risk_level == "CRITICAL" or not latest_control.is_healthy:
                critical_errors.append(f"Kontrol: {latest_control.reason}")
            elif latest_control.risk_level == "WARNING":
                warnings.append(f"Kontrol: {latest_control.reason}")

        target_state = SystemState.NORMAL
        reason = "Tum sistemler stabil"

        if len(critical_errors) > 0:
            target_state = SystemState.EMERGENCY_STOP
            reason = " | ".join(critical_errors)
        elif len(warnings) >= 2:
            target_state = SystemState.DEGRADED
            reason = "Coklu Uyari: " + " | ".join(warnings)
        elif len(warnings) == 1:
            target_state = SystemState.WARNING
            reason = warnings[0]

        self.fsm.request_transition(target_state, reason)
        return self._command_for_state()

    def _command_for_state(self) -> FailSafeCommand:
        current = self.fsm.get_state()
        cmd = FailSafeCommand(
            action="NORMAL",
            throttle_override=-1.0,
            brake_override=-1.0,
            reason=self.fsm.last_reason,
        )

        if current == SystemState.WARNING:
            cmd.action = "CAREFUL"
            cmd.throttle_override = 0.5
        elif current == SystemState.DEGRADED:
            cmd.action = "SLOW_DOWN"
            cmd.throttle_override = 0.2
            cmd.brake_override = 0.3
        elif current == SystemState.EMERGENCY_STOP:
            cmd.action = "EMERGENCY_STOP"
            cmd.throttle_override = 0.0
            cmd.brake_override = 1.0

        return cmd
