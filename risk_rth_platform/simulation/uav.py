from __future__ import annotations

from dataclasses import dataclass, replace

from .geometry import Point2D
from .wind import WindVector


@dataclass(frozen=True)
class UAVTelemetry:
    step: int
    position: Point2D
    target: Point2D
    battery_soc: float
    battery_health: float
    mission_progress: float
    distance_to_home_m: float
    wind_speed_mps: float
    energy_used: float
    inside_no_fly_zone: bool


@dataclass(frozen=True)
class UAVKinematics:
    """Lightweight point-mass UAV model for 2D mission animation."""

    position: Point2D
    cruise_speed_mps: float
    battery_soc: float
    battery_health: float
    payload_kg: float = 0.0

    def step_towards(
        self,
        target: Point2D,
        wind: WindVector,
        dt_s: float,
        world_width_m: float,
        world_height_m: float,
    ) -> tuple["UAVKinematics", float]:
        wind_drag_factor = 1.0 + 0.025 * wind.speed_mps
        payload_factor = 1.0 + 0.07 * self.payload_kg
        health_factor = 1.0 / max(0.45, self.battery_health)

        travel_distance = max(0.0, self.cruise_speed_mps * dt_s / wind_drag_factor)
        new_position = self.position.move_towards(target, travel_distance).clamp(world_width_m, world_height_m)
        distance_moved = self.position.distance_to(new_position)

        energy_used = (0.00018 * distance_moved + 0.0015) * wind_drag_factor * payload_factor * health_factor
        new_soc = max(0.0, self.battery_soc - energy_used)

        return replace(self, position=new_position, battery_soc=new_soc), energy_used
