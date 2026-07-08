from __future__ import annotations

import random
from dataclasses import dataclass
from math import cos, hypot, pi


@dataclass(frozen=True)
class UAVState:
    x_m: float
    y_m: float
    soc_wh: float
    speed_mps: float = 12.0
    mode: str = "MISSION"

    @property
    def distance_home_m(self) -> float:
        return hypot(self.x_m, self.y_m)


@dataclass(frozen=True)
class Uncertainty:
    soc_sigma_wh: float = 4.0
    wind_mean_mps: float = 0.0
    wind_sigma_mps: float = 2.0
    wind_dir_mean_rad: float = pi
    wind_dir_sigma_rad: float = 0.5


@dataclass(frozen=True)
class EnergyModel:
    base_power_w: float = 220.0
    wind_power_gain: float = 28.0
    reserve_wh: float = 8.0

    def return_energy_wh(self, distance_m: float, speed_mps: float, wind_mps: float, wind_dir_rad: float) -> float:
        # Positive headwind component increases power needed for the direct return-to-home leg.
        headwind = max(0.0, wind_mps * cos(wind_dir_rad))
        groundspeed = max(2.0, speed_mps - headwind)
        flight_time_h = distance_m / groundspeed / 3600.0
        power_w = self.base_power_w + self.wind_power_gain * headwind * headwind
        return power_w * flight_time_h + self.reserve_wh


def estimate_safe_return_probability(
    state: UAVState,
    uncertainty: Uncertainty,
    energy_model: EnergyModel | None = None,
    samples: int = 1000,
    seed: int | None = None,
) -> float:
    if samples <= 0:
        raise ValueError("samples must be positive")
    model = energy_model or EnergyModel()
    rng = random.Random(seed)
    feasible = 0
    for _ in range(samples):
        soc = max(0.0, rng.gauss(state.soc_wh, uncertainty.soc_sigma_wh))
        wind = max(0.0, rng.gauss(uncertainty.wind_mean_mps, uncertainty.wind_sigma_mps))
        direction = rng.gauss(uncertainty.wind_dir_mean_rad, uncertainty.wind_dir_sigma_rad)
        required = model.return_energy_wh(state.distance_home_m, state.speed_mps, wind, direction)
        feasible += soc >= required
    return feasible / samples


def should_trigger_rth(probability_safe_return: float, threshold: float = 0.7) -> bool:
    if not 0.0 <= probability_safe_return <= 1.0:
        raise ValueError("probability must be in [0, 1]")
    if not 0.0 < threshold < 1.0:
        raise ValueError("threshold must be in (0, 1)")
    return probability_safe_return < threshold
