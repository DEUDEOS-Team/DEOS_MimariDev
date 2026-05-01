from __future__ import annotations

"""
mission_manager.py
------------------
Şartnameye göre görev-temelli davranışları (pickup/dropoff duraklama gibi)
WaypointManager üstüne ekleyen ince bir katman.
"""

from dataclasses import dataclass
from typing import Optional

from deos_algorithms.geojson_mission_reader import TaskType
from deos_algorithms.waypoint_manager import GpsPosition, MissionPlan, WaypointManager, WaypointState


@dataclass
class MissionDecision:
    speed_cap_ratio: float = 1.0
    hold_reason: str = ""
    hold_remaining_s: float = 0.0


class MissionManager:
    def __init__(self, plan: MissionPlan):
        self.wp = WaypointManager(plan, auto_advance=False)
        self._hold_started_at: Optional[float] = None
        self._holding_task: Optional[str] = None

    def update(self, pos: GpsPosition, *, now_s: float) -> tuple[WaypointState, MissionDecision]:
        state = self.wp.update(pos)
        dec = MissionDecision()

        task = state.current_task
        is_hold_task = task in {TaskType.PICKUP, TaskType.DROPOFF}

        if not state.arrived or not is_hold_task:
            self._hold_started_at = None
            self._holding_task = None
            return state, dec

        if self._hold_started_at is None or self._holding_task != task:
            self._hold_started_at = now_s
            self._holding_task = task

        elapsed = now_s - (self._hold_started_at if self._hold_started_at is not None else now_s)
        min_hold = 15.0
        target_hold = 20.0

        if elapsed < min_hold:
            dec.speed_cap_ratio = 0.0
            dec.hold_reason = f"{task} hold"
            dec.hold_remaining_s = max(0.0, min_hold - elapsed)
            return state, dec

        self.wp.advance()
        dec.speed_cap_ratio = 1.0
        dec.hold_reason = f"{task} completed ({min(elapsed, target_hold):.1f}s)"
        dec.hold_remaining_s = 0.0
        self._hold_started_at = None
        self._holding_task = None
        return state, dec

