from __future__ import annotations

import math

import pytest

from risk_rth.metrics import (
    binomial_ci95_half_width,
    cvar_lower_tail,
    mean_or_zero,
    population_std_or_zero,
    rate,
)


def test_rate_returns_fraction_of_true_values() -> None:
    assert rate([True, False, True, True]) == 0.75


def test_rate_rejects_empty_iterable() -> None:
    with pytest.raises(ValueError):
        rate([])


def test_mean_and_std_helpers_handle_empty_or_singleton_inputs() -> None:
    assert mean_or_zero([]) == 0.0
    assert mean_or_zero([1.0, 3.0]) == 2.0
    assert population_std_or_zero([2.0]) == 0.0


def test_binomial_ci95_half_width_matches_normal_approximation() -> None:
    value = binomial_ci95_half_width(successes=50, trials=100)
    assert math.isclose(value, 0.098, rel_tol=1e-3)


def test_binomial_ci95_rejects_invalid_counts() -> None:
    with pytest.raises(ValueError):
        binomial_ci95_half_width(successes=3, trials=0)
    with pytest.raises(ValueError):
        binomial_ci95_half_width(successes=4, trials=3)


def test_cvar_lower_tail_uses_worst_energy_margins() -> None:
    margins = [-0.20, -0.10, 0.0, 0.10, 0.20]
    assert cvar_lower_tail(margins, alpha=0.4) == pytest.approx(-0.15)


def test_cvar_lower_tail_rejects_invalid_input() -> None:
    with pytest.raises(ValueError):
        cvar_lower_tail([])
    with pytest.raises(ValueError):
        cvar_lower_tail([1.0], alpha=0.0)
