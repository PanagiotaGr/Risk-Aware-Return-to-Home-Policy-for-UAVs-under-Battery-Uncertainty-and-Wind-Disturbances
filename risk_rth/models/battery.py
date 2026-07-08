"""Battery state-of-charge and uncertainty models.

The model is deliberately simple and documented as simulation-only. It is not a
validated electrochemical battery model.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BatteryModel:
    """Nominal SoC model with Gaussian estimator uncertainty."""

    capacity_wh: float = 100.0
    soc_noise_std: float = 0.03
    health: float = 1.0
    reserve_soc: float = 0.08

    def available_energy_wh(self, soc: float) -> float:
        """Return usable energy after preserving a safety reserve."""
        usable_soc = max(0.0, min(1.0, soc) - self.reserve_soc)
        return usable_soc * self.capacity_wh * max(0.0, min(1.0, self.health))

    def sample_available_energy_wh(
        self, estimated_soc: float, n: int, rng: np.random.Generator
    ) -> np.ndarray:
        """Sample available energy from uncertain SoC estimates."""
        soc_samples = rng.normal(estimated_soc, self.soc_noise_std, size=n)
        soc_samples = np.clip(soc_samples, 0.0, 1.0)
        return np.array([self.available_energy_wh(float(s)) for s in soc_samples])

    def drain_soc(self, soc: float, energy_used_wh: float) -> float:
        """Apply energy drain to SoC."""
        effective_capacity = self.capacity_wh * max(self.health, 1e-6)
        return max(0.0, soc - energy_used_wh / effective_capacity)
