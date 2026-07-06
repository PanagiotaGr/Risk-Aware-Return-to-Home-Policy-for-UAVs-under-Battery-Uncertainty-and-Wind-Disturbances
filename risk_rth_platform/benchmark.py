from __future__ import annotations

from typing import List, Tuple

from .models import SimulationResult
from .simulator import RiskAwareMissionSimulator
from .statistics import binomial_ci95


def run_trials(simulator: RiskAwareMissionSimulator, trials: int) -> List[SimulationResult]:
    return [simulator.run_trial(index) for index in range(trials)]


def safe_rate_with_ci(results: List[SimulationResult]) -> Tuple[float, float]:
    if not results:
        return 0.0, 0.0
    rate = sum(1.0 if result.safe else 0.0 for result in results) / len(results)
    return rate, binomial_ci95(rate, len(results))
