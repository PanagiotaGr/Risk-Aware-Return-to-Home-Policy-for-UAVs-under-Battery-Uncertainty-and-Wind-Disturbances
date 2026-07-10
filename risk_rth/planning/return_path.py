"""Grid-free candidate return-path planning for controlled 2D studies."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

@dataclass(frozen=True)
class ReturnPlan:
    name: str
    waypoints: NDArray[np.float64]
    path_length_m: float
    expected_energy_wh: float
    energy_variance_wh2: float
    wind_exposure: float
    safe_return_probability: float
    reserve_at_arrival_wh: float
    feasible: bool

class ReturnPathPlanner:
    def __init__(self, energy_per_m_wh: float = 0.02, wind_penalty_per_m: float = 0.004) -> None:
        self.energy_per_m_wh = energy_per_m_wh
        self.wind_penalty_per_m = wind_penalty_per_m

    @staticmethod
    def _length(points: NDArray[np.float64]) -> float:
        return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())

    def evaluate(self, name: str, points: NDArray[np.float64], available_energy_wh: float, reserve_wh: float, wind_vectors: NDArray[np.float64], energy_std_fraction: float = 0.1) -> ReturnPlan:
        length = self._length(points)
        segments = np.diff(points, axis=0)
        unit = segments / np.maximum(np.linalg.norm(segments, axis=1, keepdims=True), 1e-9)
        wind = np.asarray(wind_vectors, dtype=float)
        if wind.ndim == 1:
            wind = np.repeat(wind[None, :], len(segments), axis=0)
        headwind = np.maximum(0.0, -(wind * unit).sum(axis=1))
        exposure = float((headwind * np.linalg.norm(segments, axis=1)).sum())
        energy = self.energy_per_m_wh * length + self.wind_penalty_per_m * exposure
        variance = (energy * energy_std_fraction) ** 2
        margin = available_energy_wh - reserve_wh - energy
        z = margin / max(np.sqrt(variance), 1e-9)
        probability = float(0.5 * (1.0 + np.math.erf(z / np.sqrt(2.0))))
        return ReturnPlan(name, points, length, energy, variance, exposure, probability, available_energy_wh - energy, margin >= 0.0)

    def candidates(self, start: tuple[float, float], home: tuple[float, float], available_energy_wh: float, reserve_wh: float, wind_xy_mps: tuple[float, float]) -> list[ReturnPlan]:
        s, h = np.asarray(start, float), np.asarray(home, float)
        direct = np.vstack([s, h])
        delta = h - s
        normal = np.array([-delta[1], delta[0]]) / max(np.linalg.norm(delta), 1e-9)
        offset = min(0.25 * np.linalg.norm(delta), 50.0)
        routes = [("direct_home", direct), ("wind_aware_left", np.vstack([s, (s + h) / 2 + normal * offset, h])), ("wind_aware_right", np.vstack([s, (s + h) / 2 - normal * offset, h]))]
        return [self.evaluate(name, route, available_energy_wh, reserve_wh, np.asarray(wind_xy_mps)) for name, route in routes]

    def select(self, *args, minimum_probability: float = 0.95, **kwargs) -> ReturnPlan | None:
        plans = self.candidates(*args, **kwargs)
        feasible = [p for p in plans if p.feasible and p.safe_return_probability >= minimum_probability]
        return min(feasible, key=lambda p: (p.expected_energy_wh, p.wind_exposure), default=None)
