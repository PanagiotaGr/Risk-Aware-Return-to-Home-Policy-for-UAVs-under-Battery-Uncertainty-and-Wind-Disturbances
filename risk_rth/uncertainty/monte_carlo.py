"""Monte Carlo safe-return probability estimator."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from risk_rth.models.battery import BatteryModel
from risk_rth.models.energy import EnergyModel
from risk_rth.models.wind import WindModel


@dataclass(frozen=True)
class RiskEstimate:
    """Safe-return probability estimate with a binomial confidence interval."""

    p_safe: float
    ci_low: float
    ci_high: float
    n_samples: int
    required_energy_samples_wh: NDArray[np.float64]


@dataclass(frozen=True)
class MonteCarloRiskEstimator:
    """Estimate P(E_required < E_available | x_t, SoC_t, w_t)."""

    n_samples: int = 512
    confidence_z: float = 1.96

    def estimate(
        self,
        position: NDArray[np.float64],
        home: NDArray[np.float64],
        estimated_soc: float,
        battery: BatteryModel,
        wind: WindModel,
        energy: EnergyModel,
        rng: np.random.Generator,
        time_s: float = 0.0,
    ) -> RiskEstimate:
        wind_samples = wind.sample(self.n_samples, rng, time_s=time_s)
        available = battery.sample_available_energy_wh(estimated_soc, self.n_samples, rng)
        required = np.array(
            [energy.required_energy_wh(position, home, w) for w in wind_samples], dtype=float
        )
        safe = required < available
        p_safe = float(np.mean(safe))
        half_width = self.confidence_z * np.sqrt(
            max(p_safe * (1.0 - p_safe), 1e-12) / self.n_samples
        )
        return RiskEstimate(
            p_safe=p_safe,
            ci_low=max(0.0, p_safe - half_width),
            ci_high=min(1.0, p_safe + half_width),
            n_samples=self.n_samples,
            required_energy_samples_wh=required,
        )
