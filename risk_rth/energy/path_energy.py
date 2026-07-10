"""Path-level energy prediction from waypoint kinematics and wind.

Research Prototype / Simulation-Only.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .multirotor_power import MultirotorPowerModel


@dataclass(frozen=True)
class PathEnergyEstimate:
    expected_energy_wh: float
    standard_deviation_wh: float
    travel_time_s: float
    distance_m: float
    mean_power_w: float


class PathEnergyModel:
    def __init__(self, power_model: MultirotorPowerModel) -> None:
        self.power_model = power_model

    def estimate(
        self,
        waypoints_xyz_m: Iterable[tuple[float, float, float]],
        commanded_ground_speed_m_s: float,
        wind_xy_m_s: tuple[float, float] = (0.0, 0.0),
        payload_multiplier: float = 1.0,
        safety_factor: float = 1.0,
    ) -> PathEnergyEstimate:
        points = list(waypoints_xyz_m)
        if len(points) < 2:
            raise ValueError("at least two waypoints are required")
        if commanded_ground_speed_m_s <= 0:
            raise ValueError("commanded_ground_speed_m_s must be positive")

        energy_wh = 0.0
        variance_wh2 = 0.0
        total_time_s = 0.0
        total_distance_m = 0.0
        wind_x, wind_y = wind_xy_m_s

        for start, end in zip(points[:-1], points[1:]):
            dx, dy, dz = (end[i] - start[i] for i in range(3))
            horizontal_distance = math.hypot(dx, dy)
            segment_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if segment_distance == 0:
                continue
            ux = dx / horizontal_distance if horizontal_distance else 0.0
            uy = dy / horizontal_distance if horizontal_distance else 0.0
            along_wind = wind_x * ux + wind_y * uy
            required_airspeed = max(0.1, commanded_ground_speed_m_s - along_wind)
            segment_time_s = horizontal_distance / commanded_ground_speed_m_s if horizontal_distance else abs(dz) / 1.0
            climb_rate = dz / max(segment_time_s, 1e-9)
            power = self.power_model.estimate(
                airspeed_m_s=required_airspeed,
                climb_rate_m_s=climb_rate,
                payload_multiplier=payload_multiplier,
                safety_factor=safety_factor,
            )
            segment_energy_wh = power.total_w * segment_time_s / 3600.0
            segment_std_wh = power.standard_deviation_w * segment_time_s / 3600.0
            energy_wh += segment_energy_wh
            variance_wh2 += segment_std_wh**2
            total_time_s += segment_time_s
            total_distance_m += segment_distance

        mean_power = energy_wh * 3600.0 / total_time_s if total_time_s else 0.0
        return PathEnergyEstimate(
            expected_energy_wh=energy_wh,
            standard_deviation_wh=math.sqrt(variance_wh2),
            travel_time_s=total_time_s,
            distance_m=total_distance_m,
            mean_power_w=mean_power,
        )
