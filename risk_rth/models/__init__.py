"""Core models for risk-aware UAV RTH.

This module exposes the current modular models and a small legacy-compatible API
used by earlier tests. The compatibility functions are intentionally simple and
are kept only to avoid breaking existing examples while the package is refactored
into clearer submodules.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from risk_rth.models.battery import BatteryModel
from risk_rth.models.energy import EnergyModel
from risk_rth.models.state import MissionState, SimulationResult
from risk_rth.models.wind import WindModel


@dataclass(frozen=True)
class UAVState:
    """Legacy-compatible 2D state in metres and watt-hours."""

    x_m: float
    y_m: float
    soc_wh: float


@dataclass(frozen=True)
class Uncertainty:
    """Legacy-compatible uncertainty container for simple policy tests."""

    wind_mean_mps: float = 0.0
    wind_sigma_mps: float = 0.5
    soc_sigma_wh: float = 1.0


def should_trigger_rth(p_safe: float, tau: float) -> bool:
    """Return True when safe-return probability falls below threshold tau."""
    return p_safe < tau


def estimate_safe_return_probability(
    state: UAVState,
    uncertainty: Uncertainty,
    samples: int = 512,
    seed: int = 7,
) -> float:
    """Estimate a simple safe-return probability for legacy tests.

    This helper is intentionally conservative and lightweight. It treats home as
    the origin, samples scalar headwind and available battery energy, and checks
    whether sampled required energy is below sampled available energy. New code
    should prefer `risk_rth.uncertainty.MonteCarloRiskEstimator`.
    """
    rng = np.random.default_rng(seed)
    distance_m = float(np.hypot(state.x_m, state.y_m))
    wind_samples = rng.normal(uncertainty.wind_mean_mps, uncertainty.wind_sigma_mps, samples)
    available_wh = rng.normal(state.soc_wh, uncertainty.soc_sigma_wh, samples)
    required_wh = distance_m * 0.012 * (1.0 + 0.08 * np.maximum(wind_samples, 0.0)) * 1.15
    return float(np.mean(required_wh < available_wh))


__all__ = [
    "BatteryModel",
    "EnergyModel",
    "MissionState",
    "SimulationResult",
    "UAVState",
    "Uncertainty",
    "WindModel",
    "estimate_safe_return_probability",
    "should_trigger_rth",
]
