from __future__ import annotations

from dataclasses import dataclass
from math import hypot


@dataclass(frozen=True)
class Point2D:
    """Simple 2D point used by the lightweight simulator."""

    x: float
    y: float

    def distance_to(self, other: "Point2D") -> float:
        return hypot(self.x - other.x, self.y - other.y)

    def move_towards(self, target: "Point2D", distance: float) -> "Point2D":
        total = self.distance_to(target)
        if total <= 1e-9 or distance >= total:
            return target
        ratio = distance / total
        return Point2D(
            x=self.x + (target.x - self.x) * ratio,
            y=self.y + (target.y - self.y) * ratio,
        )

    def clamp(self, width: float, height: float) -> "Point2D":
        return Point2D(
            x=min(width, max(0.0, self.x)),
            y=min(height, max(0.0, self.y)),
        )
