"""Return-to-home decision policies and baselines."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from risk_rth.models.battery import BatteryModel
from risk_rth.models.energy import EnergyModel
from risk_rth.models.wind import WindModel
from risk_rth.uncertainty.monte_carlo import MonteCarloRiskEstimator, RiskEstimate


class RTHPolicy(Protocol):
    name: str

    def should_return(
        self,
        position: NDArray[np.float64],
        home: NDArray[np.float64],
        estimated_soc: float,
        battery: BatteryModel,
        wind: WindModel,
        energy: EnergyModel,
        rng: np.random.Generator,
        time_s: float,
    ) -> tuple[bool, RiskEstimate | None]:
        ...


@dataclass(frozen=True)
class FixedBatteryThresholdPolicy:
    """Baseline: trigger RTH when estimated SoC is below a fixed threshold."""

    threshold_soc: float = 0.25
    name: str = "fixed_battery_threshold"

    def should_return(self, position, home, estimated_soc, battery, wind, energy, rng, time_s):
        return estimated_soc <= self.threshold_soc, None


@dataclass(frozen=True)
class DistanceBasedPolicy:
    """Baseline: trigger when distance to home exceeds a configured radius."""

    max_distance_m: float = 350.0
    name: str = "distance_based_rth"

    def should_return(self, position, home, estimated_soc, battery, wind, energy, rng, time_s):
        return float(np.linalg.norm(position - home)) >= self.max_distance_m, None


@dataclass(frozen=True)
class DeterministicEnergyPolicy:
    """Baseline: compare nominal required energy with nominal available energy."""

    name: str = "deterministic_energy_estimate"

    def should_return(self, position, home, estimated_soc, battery, wind, energy, rng, time_s):
        required = energy.required_energy_wh(position, home, wind.deterministic(time_s))
        available = battery.available_energy_wh(estimated_soc)
        return available < required, None


@dataclass(frozen=True)
class RiskAwareMonteCarloPolicy:
    """Trigger RTH when estimated safe-return probability falls below tau."""

    tau: float = 0.95
    estimator: MonteCarloRiskEstimator = MonteCarloRiskEstimator()
    name: str = "risk_aware_mc"

    def should_return(self, position, home, estimated_soc, battery, wind, energy, rng, time_s):
        estimate = self.estimator.estimate(position, home, estimated_soc, battery, wind, energy, rng, time_s)
        return estimate.p_safe < self.tau, estimate


@dataclass(frozen=True)
class OraclePolicyScaffold:
    """Placeholder for an upper-bound policy using true future disturbances.

    Not implemented because this repository currently has no ground-truth future
    wind or battery trajectory oracle. It is retained as a research scaffold.
    """

    name: str = "oracle_policy_placeholder"

    def should_return(self, *args, **kwargs):
        raise NotImplementedError("Oracle policy is planned, not implemented.")
