"""
geojson_mission_reader.py
-------------------------
GeoJSON FeatureCollection dosyasından görev noktalarını okur.
"""

import json
from dataclasses import dataclass, field


class TaskType:
    START = "start"
    CHECKPOINT = "checkpoint"
    STOP = "stop"
    PARK = "park"
    PARK_ENTRY = "park_entry"
    PICKUP = "pickup"
    DROPOFF = "dropoff"


VALID_TASKS = {
    TaskType.START,
    TaskType.CHECKPOINT,
    TaskType.STOP,
    TaskType.PARK,
    TaskType.PARK_ENTRY,
    TaskType.PICKUP,
    TaskType.DROPOFF,
}

DEFAULT_ARRIVAL_RADIUS_M = 1.0
DEFAULT_SPEED_LIMIT_RATIO = 1.0


@dataclass
class MissionPoint:
    index: int
    point_id: int | None
    name: str
    lat: float
    lon: float
    task: str
    heading_deg: float | None
    speed_limit_ratio: float
    arrival_radius_m: float


@dataclass
class MissionPlan:
    points: list[MissionPoint] = field(default_factory=list)
    source_file: str = ""
    raw_crs: str | None = None

    @property
    def start(self) -> MissionPoint | None:
        return next((p for p in self.points if p.task == TaskType.START), None)

    @property
    def goal(self) -> MissionPoint | None:
        return next(
            (p for p in reversed(self.points) if p.task in {TaskType.STOP, TaskType.DROPOFF}),
            None,
        )

    def __len__(self) -> int:
        return len(self.points)

    def __iter__(self):
        return iter(self.points)


class GeoJsonMissionReader:
    def read_file(self, path: str) -> MissionPlan:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return self._parse(data, source_file=path)

    def read_string(self, geojson_str: str) -> MissionPlan:
        data = json.loads(geojson_str)
        return self._parse(data, source_file="<string>")

    def _parse(self, data: dict, source_file: str) -> MissionPlan:
        if data.get("type") != "FeatureCollection":
            raise ValueError(
                f"GeoJSON tipi 'FeatureCollection' olmalı, alındı: '{data.get('type')}'"
            )

        plan = MissionPlan(source_file=source_file)
        if "crs" in data:
            plan.raw_crs = str(data["crs"])

        for idx, feature in enumerate(data.get("features", [])):
            point = self._parse_feature(idx, feature)
            if point is not None:
                plan.points.append(point)

        return plan

    def _parse_feature(self, idx: int, feature: dict) -> MissionPoint | None:
        if feature.get("type") != "Feature":
            return None

        geometry = feature.get("geometry") or {}
        if geometry.get("type") != "Point":
            return None

        coords = geometry.get("coordinates", [])
        if len(coords) < 2:
            raise ValueError(
                f"Feature index={idx}: 'coordinates' en az [lon, lat] içermeli, alındı: {coords}"
            )

        lon = float(coords[0])
        lat = float(coords[1])

        props = feature.get("properties") or {}
        task = str(props.get("task", "")).strip().lower()
        name = str(props.get("name", f"Nokta-{idx}")).strip()

        # Şartname GeoJSON örneğinde `name` alanı start/gorev_*/park_giris gibi geliyor.
        # Repodaki algoritmalar ise `task` alanı üzerinden ilerliyor. Bu yüzden:
        # - task yoksa veya tanımsızsa name ile eşle.
        # - park giriş noktası: park_giris -> PARK_ENTRY
        if not task or task not in VALID_TASKS:
            low_name = name.strip().lower()
            if low_name == "start":
                task = TaskType.START
            elif low_name.startswith("park"):
                # park_giris / park_* isimleri: park moduna giriş tetikleyicisi
                task = TaskType.PARK_ENTRY if low_name in {"park_giris", "park giriş", "otopark giris", "otopark_giris"} else TaskType.PARK
            elif low_name.startswith(("gorev", "görev")):
                task = TaskType.CHECKPOINT
            else:
                task = TaskType.CHECKPOINT

        heading_deg: float | None = None
        if "heading_deg" in props and props["heading_deg"] is not None:
            heading_deg = float(props["heading_deg"]) % 360.0

        return MissionPoint(
            index=idx,
            point_id=int(props["id"]) if "id" in props else None,
            name=name,
            lat=lat,
            lon=lon,
            task=task,
            heading_deg=heading_deg,
            speed_limit_ratio=float(props.get("speed_limit_ratio", DEFAULT_SPEED_LIMIT_RATIO)),
            arrival_radius_m=float(props.get("radius_m", DEFAULT_ARRIVAL_RADIUS_M)),
        )

