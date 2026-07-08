"""Evaluation metrics for policy comparison."""

from __future__ import annotations

import math
from collections.abc import Sequence

from risk_rth.models.state import SimulationResult


def summarize_results(results: Sequence[SimulationResult]) -> dict[str, float]:
    """Aggregate simulation outcomes without claiming external validity."""
    n = len(results)
    if n == 0:
        raise ValueError("At least one simulation result is required.")
    return {
        "n_trials": float(n),
        "mission_success_rate": sum(r.success for r in results) / n,
        "unsafe_failure_rate": sum(r.unsafe_failure for r in results) / n,
        "early_return_rate": sum(r.early_return for r in results) / n,
        "mean_remaining_soc": sum(r.remaining_soc for r in results) / n,
        "mean_distance_completed_m": sum(r.distance_completed_m for r in results) / n,
    }


def binomial_ci(rate: float, n: int, z: float = 1.96) -> tuple[float, float]:
    """Normal-approximation confidence interval for quick diagnostics."""
    if n <= 0:
        raise ValueError("n must be positive")
    half_width = z * math.sqrt(max(rate * (1.0 - rate), 1e-12) / n)
    return max(0.0, rate - half_width), min(1.0, rate + half_width)
