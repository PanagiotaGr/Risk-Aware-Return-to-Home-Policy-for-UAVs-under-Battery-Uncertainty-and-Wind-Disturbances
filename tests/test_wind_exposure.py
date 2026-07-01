from __future__ import annotations

import pytest

from risk_rth.wind.exposure import TrajectoryPoint, sample_trajectory, summarize_wind_exposure
from risk_rth.wind.models import ConstantWindField


def test_sample_trajectory_returns_one_sample_per_point() -> None:
    field = ConstantWindField(u_mps=1.0, v_mps=0.0)
    trajectory = [TrajectoryPoint(0.0, 0.0, 0.0), TrajectoryPoint(10.0, 0.0, 1.0)]

    samples = sample_trajectory(field, trajectory)

    assert len(samples) == 2
    assert samples[1].x_m == 10.0


def test_wind_exposure_identifies_tailwind() -> None:
    field = ConstantWindField(u_mps=2.0, v_mps=0.0)
    trajectory = [TrajectoryPoint(0.0, 0.0, 0.0), TrajectoryPoint(10.0, 0.0, 1.0)]
    samples = sample_trajectory(field, trajectory)

    summary = summarize_wind_exposure(samples)

    assert summary.samples == 2
    assert summary.mean_speed_mps == pytest.approx(2.0)
    assert summary.mean_tailwind_mps == pytest.approx(2.0)
    assert summary.mean_headwind_mps == pytest.approx(0.0)
    assert summary.tailwind_ratio == 1.0


def test_wind_exposure_identifies_headwind() -> None:
    field = ConstantWindField(u_mps=-3.0, v_mps=0.0)
    trajectory = [TrajectoryPoint(0.0, 0.0, 0.0), TrajectoryPoint(10.0, 0.0, 1.0)]
    samples = sample_trajectory(field, trajectory)

    summary = summarize_wind_exposure(samples)

    assert summary.mean_headwind_mps == pytest.approx(3.0)
    assert summary.headwind_ratio == 1.0


def test_wind_exposure_rejects_empty_samples() -> None:
    with pytest.raises(ValueError):
        summarize_wind_exposure([])
