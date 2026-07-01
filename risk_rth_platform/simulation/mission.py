from __future__ import annotations

from dataclasses import dataclass
from typing import List

from .geometry import Point2D


@dataclass(frozen=True)
class MissionPlan:
    """Waypoint mission with deterministic progress tracking."""

    waypoints: List[Point2D]
    acceptance_radius_m: float = 20.0

    def current_target(self, waypoint_index: int) -> Point2D:
        if waypoint_index >= len(self.waypoints):
            return self.waypoints[-1]
        return self.waypoints[waypoint_index]

    def update_waypoint_index(self, position: Point2D, waypoint_index: int) -> int:
        if waypoint_index >= len(self.waypoints):
            return waypoint_index
        if position.distance_to(self.waypoints[waypoint_index]) <= self.acceptance_radius_m:
            return min(len(self.waypoints), waypoint_index + 1)
        return waypoint_index

    def progress(self, waypoint_index: int) -> float:
        if not self.waypoints:
            return 1.0
        return min(1.0, waypoint_index / len(self.waypoints))

    @classmethod
    def from_dict(cls, data: dict) -> "MissionPlan":
        return cls(
            waypoints=[Point2D(*coords) for coords in data["waypoints"]],
            acceptance_radius_m=float(data.get("acceptance_radius_m", 20.0)),
        )
