"""Transparent Gaussian battery-state estimator.

Research prototype: this is a scalar Kalman-style filter for simulation studies,
not an electrochemical model and not validated for flight certification.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BatteryBelief:
    """Posterior belief over state of charge and usable energy."""

    mean_soc: float
    variance_soc: float
    health_mean: float
    health_variance: float
    capacity_wh_mean: float
    capacity_wh_variance: float

    @property
    def std_soc(self) -> float:
        return float(np.sqrt(max(self.variance_soc, 0.0)))

    @property
    def usable_energy_mean_wh(self) -> float:
        return max(0.0, self.mean_soc) * max(0.0, self.health_mean) * max(
            0.0, self.capacity_wh_mean
        )

    @property
    def usable_energy_variance_wh2(self) -> float:
        """First-order uncertainty propagation for E = SoC * health * capacity."""
        s, h, c = self.mean_soc, self.health_mean, self.capacity_wh_mean
        return max(
            0.0,
            (h * c) ** 2 * self.variance_soc
            + (s * c) ** 2 * self.health_variance
            + (s * h) ** 2 * self.capacity_wh_variance,
        )


class BayesianBatteryEstimator:
    """Scalar Gaussian filter with explicit prediction and measurement updates."""

    def __init__(
        self,
        initial_soc_mean: float,
        initial_soc_variance: float,
        capacity_wh_mean: float,
        capacity_wh_variance: float = 0.0,
        health_mean: float = 1.0,
        health_variance: float = 0.0,
        process_variance_per_s: float = 1e-6,
        measurement_variance: float = 4e-4,
        discharge_rate_variance: float = 0.0,
        degradation_rate_per_s: float = 0.0,
        seed: int | None = None,
    ) -> None:
        if initial_soc_variance < 0 or measurement_variance <= 0:
            raise ValueError("Variances must be non-negative and measurement variance positive")
        if capacity_wh_mean <= 0:
            raise ValueError("capacity_wh_mean must be positive")
        self._belief = BatteryBelief(
            mean_soc=float(np.clip(initial_soc_mean, 0.0, 1.0)),
            variance_soc=float(initial_soc_variance),
            health_mean=float(np.clip(health_mean, 0.0, 1.0)),
            health_variance=float(max(0.0, health_variance)),
            capacity_wh_mean=float(capacity_wh_mean),
            capacity_wh_variance=float(max(0.0, capacity_wh_variance)),
        )
        self.process_variance_per_s = max(0.0, process_variance_per_s)
        self.measurement_variance = measurement_variance
        self.discharge_rate_variance = max(0.0, discharge_rate_variance)
        self.degradation_rate_per_s = max(0.0, degradation_rate_per_s)
        self.rng = np.random.default_rng(seed)

    @property
    def belief(self) -> BatteryBelief:
        return self._belief

    def predict(self, energy_used_wh: float, dt_s: float) -> BatteryBelief:
        if energy_used_wh < 0 or dt_s < 0:
            raise ValueError("energy_used_wh and dt_s must be non-negative")
        effective_capacity = max(
            self._belief.capacity_wh_mean * self._belief.health_mean, 1e-9
        )
        predicted_soc = np.clip(
            self._belief.mean_soc - energy_used_wh / effective_capacity, 0.0, 1.0
        )
        predicted_variance = (
            self._belief.variance_soc
            + self.process_variance_per_s * dt_s
            + self.discharge_rate_variance * dt_s**2
        )
        predicted_health = np.clip(
            self._belief.health_mean - self.degradation_rate_per_s * dt_s,
            0.0,
            1.0,
        )
        self._belief = BatteryBelief(
            mean_soc=float(predicted_soc),
            variance_soc=float(max(0.0, predicted_variance)),
            health_mean=float(predicted_health),
            health_variance=self._belief.health_variance,
            capacity_wh_mean=self._belief.capacity_wh_mean,
            capacity_wh_variance=self._belief.capacity_wh_variance,
        )
        return self._belief

    def update(self, observed_soc: float, measurement_variance: float | None = None) -> BatteryBelief:
        variance = self.measurement_variance if measurement_variance is None else measurement_variance
        if variance <= 0:
            raise ValueError("measurement variance must be positive")
        prior_var = self._belief.variance_soc
        gain = prior_var / max(prior_var + variance, 1e-12)
        posterior_mean = self._belief.mean_soc + gain * (
            float(np.clip(observed_soc, 0.0, 1.0)) - self._belief.mean_soc
        )
        posterior_variance = (1.0 - gain) * prior_var
        self._belief = BatteryBelief(
            mean_soc=float(np.clip(posterior_mean, 0.0, 1.0)),
            variance_soc=float(max(0.0, posterior_variance)),
            health_mean=self._belief.health_mean,
            health_variance=self._belief.health_variance,
            capacity_wh_mean=self._belief.capacity_wh_mean,
            capacity_wh_variance=self._belief.capacity_wh_variance,
        )
        return self._belief

    def observe(self, true_soc: float) -> float:
        """Generate a reproducible noisy observation for synthetic experiments."""
        return float(
            np.clip(
                self.rng.normal(true_soc, np.sqrt(self.measurement_variance)),
                0.0,
                1.0,
            )
        )

    def log_record(self, true_soc: float, observed_soc: float) -> dict[str, float]:
        return {
            "true_soc": float(true_soc),
            "observed_soc": float(observed_soc),
            "estimated_soc": self._belief.mean_soc,
            "posterior_variance": self._belief.variance_soc,
            "estimated_usable_energy_wh": self._belief.usable_energy_mean_wh,
            "battery_health_estimate": self._belief.health_mean,
            "estimation_error": self._belief.mean_soc - float(true_soc),
        }
