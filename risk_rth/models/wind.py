"""Wind disturbance models for controlled UAV RTH simulations."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class WindModel:
    """Constant wind plus optional sinusoidal gust and Gaussian uncertainty."""

    mean_xy_mps: tuple[float, float] = (0.0, 0.0)
    std_mps: float = 0.5
    gust_amplitude_mps: float = 0.0
    gust_period_s: float = 20.0

    def deterministic(self, time_s: float = 0.0) -> NDArray[np.float64]:
        base = np.asarray(self.mean_xy_mps, dtype=float)
        if self.gust_amplitude_mps == 0.0:
            return base
        gust = self.gust_amplitude_mps * np.sin(2.0 * np.pi * time_s / self.gust_period_s)
        return base + np.array([gust, 0.5 * gust], dtype=float)

    def sample(self, n: int, rng: np.random.Generator, time_s: float = 0.0) -> NDArray[np.float64]:
        mean = self.deterministic(time_s)
        return rng.normal(mean, self.std_mps, size=(n, 2))
