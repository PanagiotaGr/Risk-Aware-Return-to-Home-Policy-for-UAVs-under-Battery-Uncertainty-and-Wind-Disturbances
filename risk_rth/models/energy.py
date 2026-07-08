"""Wind-aware return energy approximation.

This model is intended for reproducible controlled simulation. It is not a
platform-identified aerodynamic power model.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class EnergyModel:
    """Distance and wind adjusted energy-cost model."""

    base_wh_per_m: float = 0.012
    airspeed_mps: float = 8.0
    headwind_gain: float = 0.08
    crosswind_gain: float = 0.03
    safety_factor: float = 1.15

    def required_energy_wh(
        self,
        position: NDArray[np.float64],
        home: NDArray[np.float64],
        wind_xy_mps: NDArray[np.float64],
    ) -> float:
        delta = home - position
        distance = float(np.linalg.norm(delta))
        if distance < 1e-9:
            return 0.0
        direction_home = delta / distance
        wind_along_return = float(np.dot(wind_xy_mps, direction_home))
        headwind = max(0.0, -wind_along_return)
        crosswind = float(np.linalg.norm(wind_xy_mps - wind_along_return * direction_home))
        wind_multiplier = 1.0 + self.headwind_gain * headwind + self.crosswind_gain * crosswind
        return distance * self.base_wh_per_m * wind_multiplier * self.safety_factor

    def step_energy_wh(self, velocity_xy_mps: NDArray[np.float64], wind_xy_mps: NDArray[np.float64], dt_s: float) -> float:
        air_velocity = velocity_xy_mps - wind_xy_mps
        airspeed = float(np.linalg.norm(air_velocity))
        return self.base_wh_per_m * airspeed * dt_s * (1.0 + 0.01 * airspeed**2)
