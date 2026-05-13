from dataclasses import dataclass


@dataclass
class Health:
    module_name: str
    overall_score: float
    is_healthy: bool
    status: str  # OK / WARNING / CRITICAL
    reason: str


@dataclass
class PlanStatus:
    is_valid: bool
    risk_level: str  # SAFE / WARNING / CRITICAL
    reason: str


@dataclass
class ControlStatus:
    is_healthy: bool
    risk_level: str
    reason: str
    control_enabled: bool


@dataclass
class FailSafeCommand:
    action: str  # NORMAL, CAREFUL, SLOW_DOWN, EMERGENCY_STOP
    throttle_override: float
    brake_override: float
    reason: str
