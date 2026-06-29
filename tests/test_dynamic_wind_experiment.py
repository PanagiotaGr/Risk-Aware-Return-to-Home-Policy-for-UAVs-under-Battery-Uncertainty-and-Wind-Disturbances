from __future__ import annotations

import pytest

from experiments.run_dynamic_wind_exposure import straight_return_trajectory


def test_straight_return_trajectory_starts_away_and_ends_home() -> None:
    trajectory = straight_return_trajectory(distance_m=100.0, duration_s=10.0, samples=3)

    assert trajectory[0].x_m == 100.0
    assert trajectory[0].t_s == 0.0
    assert trajectory[-1].x_m == 0.0
    assert trajectory[-1].t_s == 10.0
    assert trajectory[1].x_m == 50.0


def test_straight_return_trajectory_requires_at_least_two_samples() -> None:
    with pytest.raises(ValueError):
        straight_return_trajectory(distance_m=100.0, duration_s=10.0, samples=1)
