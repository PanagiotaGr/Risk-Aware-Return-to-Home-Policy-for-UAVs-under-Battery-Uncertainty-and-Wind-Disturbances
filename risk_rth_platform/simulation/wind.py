from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .geometry import Point2D


@dataclass(frozen=True)
class WindVector:
    speed_mps: float
    direction_rad: float

    @property
    def x(self) -> float:
        return self.speed_mps * math.cos(self.direction_rad)

    @property
    def y(self) -> float:
        return self.speed_mps * math.sin(self.direction_rad)


@dataclass
class WindField:
    """Spatially varying wind field with deterministic gust noise."""

    base_speed_mps: float
    base_direction_rad: float
    spatial_scale_m: float
    gust_std_mps: float
    seed: int = 0

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def sample(self, point: Point2D, step: int) -> WindVector:
        spatial_wave = math.sin((point.x + 0.5 * point.y) / max(1.0, self.spatial_scale_m))
        temporal_wave = math.sin(step / 8.0)
        gust = self._rng.gauss(0.0, self.gust_std_mps)
        speed = max(0.0, self.base_speed_mps + 1.2 * spatial_wave + 0.8 * temporal_wave + gust)
        direction = self.base_direction_rad + 0.25 * math.sin(point.y / max(1.0, self.spatial_scale_m))
        return WindVector(speed_mps=speed, direction_rad=direction)

    @classmethod
    def from_dict(cls, data: dict) -> "WindField":
        return cls(
            base_speed_mps=float(data.get("base_speed_mps", 5.0)),
            base_direction_rad=float(data.get("base_direction_rad", 0.0)),
            spatial_scale_m=float(data.get("spatial_scale_m", 500.0)),
            gust_std_mps=float(data.get("gust_std_mps", 1.0)),
            seed=int(data.get("seed", 0)),
        )
