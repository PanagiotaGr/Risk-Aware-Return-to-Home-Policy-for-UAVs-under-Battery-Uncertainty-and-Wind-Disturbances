"""Analytic wind-field models for standalone UAV simulations."""

from __future__ import annotations

import random
from dataclasses import dataclass
from math import cos, pi, sin

from risk_rth.wind.field import WindSample, WindVector


@dataclass(frozen=True)
class ConstantWindField:
    """A spatially and temporally constant wind vector."""

    u_mps: float
    v_mps: float

    def sample(self, x_m: float, y_m: float, t_s: float) -> WindSample:
        return WindSample(x_m=x_m, y_m=y_m, t_s=t_s, vector=WindVector(self.u_mps, self.v_mps))


@dataclass(frozen=True)
class GaussianWindField:
    """A reproducible stochastic wind model around a mean vector.

    The random generator is re-seeded from position and time so repeated calls at
    the same coordinate are deterministic for a fixed seed. This makes Monte
    Carlo experiments reproducible while still allowing different wind
    realizations across space and time.
    """

    mean_u_mps: float
    mean_v_mps: float
    std_mps: float
    seed: int = 7

    def sample(self, x_m: float, y_m: float, t_s: float) -> WindSample:
        local_seed = hash((self.seed, round(x_m, 2), round(y_m, 2), round(t_s, 2)))
        rng = random.Random(local_seed)
        vector = WindVector(
            u=rng.gauss(self.mean_u_mps, self.std_mps),
            v=rng.gauss(self.mean_v_mps, self.std_mps),
        )
        return WindSample(x_m=x_m, y_m=y_m, t_s=t_s, vector=vector)


@dataclass(frozen=True)
class SinusoidalGustWindField:
    """Wind model with periodic gust modulation.

    The gust component is useful for controlled experiments where wind intensity
    changes over time but remains reproducible.
    """

    base_u_mps: float
    base_v_mps: float
    gust_u_amplitude_mps: float = 0.0
    gust_v_amplitude_mps: float = 0.0
    period_s: float = 30.0
    phase_rad: float = 0.0

    def sample(self, x_m: float, y_m: float, t_s: float) -> WindSample:
        if self.period_s <= 0.0:
            raise ValueError("period_s must be positive")

        wave = sin(2.0 * pi * t_s / self.period_s + self.phase_rad)
        # Small spatial modulation avoids identical wind along long trajectories.
        spatial_wave = cos(0.002 * x_m + 0.002 * y_m)
        vector = WindVector(
            u=self.base_u_mps + self.gust_u_amplitude_mps * wave * spatial_wave,
            v=self.base_v_mps + self.gust_v_amplitude_mps * wave * spatial_wave,
        )
        return WindSample(x_m=x_m, y_m=y_m, t_s=t_s, vector=vector)
