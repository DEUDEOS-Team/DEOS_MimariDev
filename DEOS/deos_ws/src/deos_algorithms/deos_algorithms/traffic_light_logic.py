"""
traffic_light_logic.py
----------------------
Trafik ışığı (kırmızı/sarı/yeşil) tespitlerini alıp aracın davranışına
dönüşen KISITLAR üretir.
"""

from dataclasses import dataclass
from typing import Optional
import time


class LightColor:
    RED = "red"
    YELLOW = "yellow"
    GREEN = "green"


RED_ALIASES = {
    "red", "kirmizi", "kırmızı",
    "kirmizi isik", "kırmızı ışık", "kirmizi_isik",
    "red_light", "traffic_light_red", "tl_red",
}
YELLOW_ALIASES = {
    "yellow", "amber", "sari", "sarı",
    "sari isik", "sarı ışık", "sari_isik",
    "yellow_light", "traffic_light_yellow", "tl_yellow",
}
GREEN_ALIASES = {
    "green", "yesil", "yeşil",
    "yesil isik", "yeşil ışık", "yesil_isik",
    "green_light", "traffic_light_green", "tl_green",
}


def classify_color(class_name: str) -> Optional[str]:
    if not class_name:
        return None
    low = class_name.strip().lower()
    if low in RED_ALIASES:
        return LightColor.RED
    if low in YELLOW_ALIASES:
        return LightColor.YELLOW
    if low in GREEN_ALIASES:
        return LightColor.GREEN
    return None


def is_traffic_light_class(class_name: str) -> bool:
    return classify_color(class_name) is not None


MIN_CONFIDENCE = 0.45
CONFIRM_FRAMES = 2
LIGHT_VALIDITY_SECONDS = 1.5
YELLOW_COMMIT_DISTANCE_M = 12.0
YELLOW_SPEED_RATIO = 0.4


@dataclass
class LightDetection:
    color: str
    confidence: float
    bbox_px: tuple[float, float, float, float]
    estimated_distance_m: Optional[float] = None


@dataclass
class TrafficLightState:
    must_stop: bool = False
    prepare_to_stop: bool = False
    can_go: bool = False
    speed_cap_ratio: float = 1.0
    active_color: Optional[str] = None
    last_distance_m: Optional[float] = None
    reason: str = ""


@dataclass
class _LightMemory:
    color: str
    frames_seen: int
    last_seen_time: float
    last_distance_m: Optional[float]
    last_confidence: float
    confirmed: bool


class TrafficLightLogic:
    def __init__(self):
        self._memories: dict[str, _LightMemory] = {}

    def update(self, detections: list[LightDetection], now: Optional[float] = None) -> TrafficLightState:
        if now is None:
            now = time.time()
        detections = [d for d in detections if d.confidence >= MIN_CONFIDENCE]
        self._update_memory(detections, now)
        self._forget_expired(now)
        active = [m for m in self._memories.values() if m.confirmed]
        return self._decide(active)

    def reset(self) -> None:
        self._memories.clear()

    def _update_memory(self, detections: list[LightDetection], now: float) -> None:
        for det in detections:
            color = det.color
            if color not in (LightColor.RED, LightColor.YELLOW, LightColor.GREEN):
                continue
            mem = self._memories.get(color)
            if mem is None:
                self._memories[color] = _LightMemory(
                    color=color,
                    frames_seen=1,
                    last_seen_time=now,
                    last_distance_m=det.estimated_distance_m,
                    last_confidence=det.confidence,
                    confirmed=(CONFIRM_FRAMES <= 1),
                )
            else:
                mem.frames_seen += 1
                mem.last_seen_time = now
                mem.last_confidence = det.confidence
                if det.estimated_distance_m is not None:
                    mem.last_distance_m = det.estimated_distance_m
                if mem.frames_seen >= CONFIRM_FRAMES:
                    mem.confirmed = True

    def _forget_expired(self, now: float) -> None:
        expired = [
            color for color, mem in self._memories.items()
            if (now - mem.last_seen_time) > LIGHT_VALIDITY_SECONDS
        ]
        for color in expired:
            del self._memories[color]

    def _decide(self, active: list[_LightMemory]) -> TrafficLightState:
        state = TrafficLightState()
        if not active:
            return state

        by_color = {m.color: m for m in active}

        if LightColor.RED in by_color:
            mem = by_color[LightColor.RED]
            state.must_stop = True
            state.speed_cap_ratio = 0.0
            state.active_color = LightColor.RED
            state.last_distance_m = mem.last_distance_m
            dist_txt = f"{mem.last_distance_m:.1f}m" if mem.last_distance_m is not None else "?"
            state.reason = f"RED light at {dist_txt}"
            return state

        if LightColor.YELLOW in by_color:
            mem = by_color[LightColor.YELLOW]
            state.active_color = LightColor.YELLOW
            state.last_distance_m = mem.last_distance_m
            dist = mem.last_distance_m
            if dist is None or dist > YELLOW_COMMIT_DISTANCE_M:
                state.prepare_to_stop = True
                state.speed_cap_ratio = YELLOW_SPEED_RATIO
                dist_txt = f"{dist:.1f}m" if dist is not None else "?"
                state.reason = f"YELLOW, prepare to stop ({dist_txt})"
            else:
                state.can_go = True
                state.speed_cap_ratio = 1.0
                state.reason = f"YELLOW but too close ({dist:.1f}m), commit to cross"
            return state

        if LightColor.GREEN in by_color:
            mem = by_color[LightColor.GREEN]
            state.can_go = True
            state.active_color = LightColor.GREEN
            state.last_distance_m = mem.last_distance_m
            state.reason = "GREEN, go"
            return state

        return state

