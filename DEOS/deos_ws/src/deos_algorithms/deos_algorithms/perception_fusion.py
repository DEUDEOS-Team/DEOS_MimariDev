from __future__ import annotations

"""
perception_fusion.py
-------------------
Stereo kamera + LiDAR + IMU çıktısını mevcut logic modüllerinin beklediği
Detection listelerine dönüştüren ince bir adaptasyon katmanı.

Bu dosya ROS içermez; gerçek sensör sürücüleri dışarıda kalır.
"""

from dataclasses import dataclass, field
from typing import Optional

from deos_algorithms.obstacle_logic import ObstacleDetection, classify_obstacle
from deos_algorithms.sensors.types import ImuSample, LidarObstacle, StereoBbox
from deos_algorithms.traffic_light_logic import LightDetection, classify_color
from deos_algorithms.traffic_sign_logic import SignDetection


@dataclass
class PerceptionFrame:
    sign_dets: list[SignDetection] = field(default_factory=list)
    light_dets: list[LightDetection] = field(default_factory=list)
    obstacle_dets: list[ObstacleDetection] = field(default_factory=list)
    imu: Optional[ImuSample] = None


def fuse(
    *,
    stereo: list[StereoBbox] | None = None,
    lidar: list[LidarObstacle] | None = None,
    imu: ImuSample | None = None,
) -> PerceptionFrame:
    out = PerceptionFrame(imu=imu)

    stereo = stereo or []
    lidar = lidar or []

    for s in stereo:
        if classify_color(s.class_name) is not None:
            out.light_dets.append(
                LightDetection(
                    color=classify_color(s.class_name) or "red",
                    confidence=s.confidence,
                    bbox_px=s.bbox_px,
                    estimated_distance_m=s.distance_m,
                )
            )
            continue

        kind = classify_obstacle(s.class_name)
        if kind is not None:
            out.obstacle_dets.append(
                ObstacleDetection(
                    kind=kind,
                    confidence=s.confidence,
                    bbox_px=s.bbox_px,
                    estimated_distance_m=s.distance_m,
                    estimated_lateral_m=s.lateral_m,
                )
            )
            continue

        out.sign_dets.append(
            SignDetection(
                class_name=s.class_name,
                confidence=s.confidence,
                bbox_px=s.bbox_px,
                estimated_distance_m=s.distance_m,
            )
        )

    for o in lidar:
        out.obstacle_dets.append(
            ObstacleDetection(
                kind=o.kind,
                confidence=o.confidence,
                bbox_px=o.bbox_px or (0.0, 0.0, 1.0, 1.0),
                estimated_distance_m=o.distance_m,
                estimated_lateral_m=o.lateral_m,
            )
        )

    return out

