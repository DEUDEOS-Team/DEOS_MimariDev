import logging
from enum import Enum, auto

_logger = logging.getLogger(__name__)


class SystemState(Enum):
    NORMAL = auto()
    WARNING = auto()
    DEGRADED = auto()
    EMERGENCY_STOP = auto()


class AutonomousFSM:
    """
    Global system state machine (ported from Fail-Safe/manager/fsm.py).
    EMERGENCY_STOP latches until manual_reset().
    """

    def __init__(self) -> None:
        self.current_state = SystemState.NORMAL
        self.last_reason = "Sistem baslatildi"

    def request_transition(self, new_state: SystemState, reason: str) -> bool:
        if self.current_state == SystemState.EMERGENCY_STOP and new_state != SystemState.EMERGENCY_STOP:
            _logger.warning(
                "FSM: EMERGENCY_STOP kilitli, gecis reddedildi: %s | %s",
                new_state.name,
                reason,
            )
            return False

        if self.current_state == new_state:
            return True

        _logger.warning("FSM: %s -> %s | %s", self.current_state.name, new_state.name, reason)
        self.current_state = new_state
        self.last_reason = reason
        return True

    def manual_reset(self) -> None:
        _logger.info("FSM: manuel reset -> NORMAL")
        self.current_state = SystemState.NORMAL
        self.last_reason = "Manuel Reset"

    def get_state(self) -> SystemState:
        return self.current_state
