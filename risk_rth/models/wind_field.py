"""Spatiotemporal wind fields for simulation-only experiments."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

@dataclass(frozen=True)
class WindDistribution:
    mean_xy_mps: NDArray[np.float64]
    covariance: NDArray[np.float64]

@dataclass
class SpatioTemporalWindField:
    base_xy_mps: tuple[float, float] = (0.0, 0.0)
    std_mps: float = 0.5
    gust_amplitude_mps: float = 0.0
    gust_period_s: float = 20.0
    spatial_scale_m: float = 100.0
    forecast_bias_xy_mps: tuple[float, float] = (0.0, 0.0)
    headwind_corridor: tuple[float, float, float, float] | None = None
    corridor_vector_xy_mps: tuple[float, float] = (0.0, 0.0)
    seed: int | None = None

    def __post_init__(self) -> None:
        self.rng = np.random.default_rng(self.seed)

    def wind(self, position: NDArray[np.float64] | tuple[float, float], time_s: float) -> NDArray[np.float64]:
        p = np.asarray(position, dtype=float)
        base = np.asarray(self.base_xy_mps, dtype=float)
        phase = 2.0 * np.pi * time_s / max(self.gust_period_s, 1e-9)
        spatial = np.array([np.sin(p[1] / max(self.spatial_scale_m, 1e-9)), np.cos(p[0] / max(self.spatial_scale_m, 1e-9))])
        value = base + self.gust_amplitude_mps * np.sin(phase) * spatial
        if self.headwind_corridor is not None:
            xmin, xmax, ymin, ymax = self.headwind_corridor
            if xmin <= p[0] <= xmax and ymin <= p[1] <= ymax:
                value = value + np.asarray(self.corridor_vector_xy_mps, dtype=float)
        return value

    def wind_distribution(self, position: NDArray[np.float64] | tuple[float, float], time_s: float, forecast: bool = False) -> WindDistribution:
        mean = self.wind(position, time_s)
        if forecast:
            mean = mean + np.asarray(self.forecast_bias_xy_mps, dtype=float)
        return WindDistribution(mean, np.eye(2) * self.std_mps**2)

    def sample(self, position: NDArray[np.float64] | tuple[float, float], time_s: float, n: int, forecast: bool = False) -> NDArray[np.float64]:
        d = self.wind_distribution(position, time_s, forecast=forecast)
        return self.rng.multivariate_normal(d.mean_xy_mps, d.covariance, size=n)

    def noisy_local_measurement(self, position: NDArray[np.float64] | tuple[float, float], time_s: float, noise_std_mps: float) -> NDArray[np.float64]:
        return self.wind(position, time_s) + self.rng.normal(0.0, noise_std_mps, size=2)
