"""Reusable evaluation metrics for UAV return-to-home experiments.

The functions in this module are intentionally dependency-light so they can be
used by standalone experiments, notebooks, and ROS-adjacent tooling.
"""

from __future__ import annotations

from math import sqrt
from statistics import mean, pstdev
from typing import Iterable, Sequence


def rate(values: Iterable[bool]) -> float:
    """Return the fraction of true values.

    Raises:
        ValueError: if the iterable is empty.
    """

    items = list(values)
    if not items:
        raise ValueError("rate() requires at least one value")
    return mean(bool(item) for item in items)


def mean_or_zero(values: Iterable[float]) -> float:
    """Return the arithmetic mean, or zero for an empty sequence."""

    items = list(values)
    return mean(items) if items else 0.0


def population_std_or_zero(values: Iterable[float]) -> float:
    """Return population standard deviation, or zero if fewer than two samples exist."""

    items = list(values)
    return pstdev(items) if len(items) > 1 else 0.0


def binomial_ci95_half_width(successes: int, trials: int) -> float:
    """Approximate 95% confidence interval half-width for a binomial rate.

    The normal approximation is adequate for the large Monte Carlo experiment
    sizes used in this repository and keeps the implementation transparent.
    """

    if trials <= 0:
        raise ValueError("trials must be positive")
    if successes < 0 or successes > trials:
        raise ValueError("successes must be in the range [0, trials]")

    p_hat = successes / trials
    return 1.96 * sqrt(p_hat * (1.0 - p_hat) / trials)


def cvar_lower_tail(values: Sequence[float], alpha: float = 0.05) -> float:
    """Return lower-tail Conditional Value at Risk for energy margins.

    For safety analysis, lower energy margins are worse. This function therefore
    averages the worst ``alpha`` fraction of the provided values.
    """

    if not values:
        raise ValueError("cvar_lower_tail() requires at least one value")
    if not 0.0 < alpha <= 1.0:
        raise ValueError("alpha must be in the interval (0, 1]")

    sorted_values = sorted(float(value) for value in values)
    tail_count = max(1, int(len(sorted_values) * alpha))
    return mean(sorted_values[:tail_count])
