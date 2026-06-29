"""Trajectory-level wind exposure metrics."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from statistics import mean
from typing import Iterable

from risk_rth.wind.field import WindField, WindSample


@dataclass(frozen=True)
class TrajectoryPoint:
    """A point on a UAV trajectory."""

    x_m: float
    y_m: float
    t_s: float


@dataclass(frozen=True)
class WindExposureSummary:
    """Aggregated wind statistics along a trajectory."""

    samples: int
    mean_speed_mps: float
    max_speed_mps: float
    mean_headwind_mps: float
    mean_tailwind_mps: float
    headwind_ratio: float
    tailwind_ratio: float
    wind_risk_index: float


def sample_trajectory(field: WindField, trajectory: Iterable[TrajectoryPoint]) -> list[WindSample]:
    """Sample a wind field along a trajectory."""

    return [field.sample(point.x_m, point.y_m, point.t_s) for point in trajectory]


def summarize_wind_exposure(samples: list[WindSample]) -> WindExposureSummary:
    """Summarize wind exposure using consecutive sample geometry.

    Headwind is defined relative to the direction of motion between consecutive
    samples. Positive projection against the motion vector is headwind; positive
    projection along the motion vector is tailwind.
    """

    if not samples:
        raise ValueError("at least one wind sample is required")

    speeds = [sample.speed_mps for sample in samples]
    headwind_values: list[float] = []
    tailwind_values: list[float] = []

    for current, next_sample in zip(samples, samples[1:]):
        dx = next_sample.x_m - current.x_m
        dy = next_sample.y_m - current.y_m
        norm = hypot(dx, dy)
        if norm == 0.0:
            continue
        ux = dx / norm
        uy = dy / norm
        along_track = current.vector.dot(ux, uy)
        tailwind_values.append(max(0.0, along_track))
        headwind_values.append(max(0.0, -along_track))

    if not headwind_values:
        headwind_values = [0.0]
        tailwind_values = [0.0]

    mean_speed = mean(speeds)
    max_speed = max(speeds)
    mean_headwind = mean(headwind_values)
    mean_tailwind = mean(tailwind_values)
    moving_segments = len(headwind_values)
    headwind_ratio = sum(value > 0.0 for value in headwind_values) / moving_segments
    tailwind_ratio = sum(value > 0.0 for value in tailwind_values) / moving_segments

    # Dimensionless, monotonic risk indicator for quick comparisons.
    wind_risk_index = mean_speed + 0.5 * max_speed + mean_headwind - 0.25 * mean_tailwind

    return WindExposureSummary(
        samples=len(samples),
        mean_speed_mps=mean_speed,
        max_speed_mps=max_speed,
        mean_headwind_mps=mean_headwind,
        mean_tailwind_mps=mean_tailwind,
        headwind_ratio=headwind_ratio,
        tailwind_ratio=tailwind_ratio,
        wind_risk_index=wind_risk_index,
    )
