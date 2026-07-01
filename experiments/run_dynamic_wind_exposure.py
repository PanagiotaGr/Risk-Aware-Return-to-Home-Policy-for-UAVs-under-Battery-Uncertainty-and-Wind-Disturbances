#!/usr/bin/env python3
"""Run a reproducible dynamic wind exposure experiment.

This script demonstrates the new wind-field engine without replacing the
existing Monte Carlo RTH experiments. It evaluates a straight return trajectory
under three wind models:

1. constant tailwind,
2. sinusoidal gust headwind,
3. spatial grid wind field.

Outputs:
- results_dynamic_wind/wind_exposure_summary.csv
- results_dynamic_wind/wind_speed_profiles.csv
- results_dynamic_wind/wind_speed_profile.png, if matplotlib is installed
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from pathlib import Path
from typing import Iterable

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    plt = None

from risk_rth.wind.exposure import TrajectoryPoint, sample_trajectory, summarize_wind_exposure
from risk_rth.wind.models import ConstantWindField, SinusoidalGustWindField
from risk_rth.wind.spatial import GridWindField


def straight_return_trajectory(
    distance_m: float,
    duration_s: float,
    samples: int,
) -> list[TrajectoryPoint]:
    """Generate a straight-line return trajectory from distance to home."""

    if samples < 2:
        raise ValueError("samples must be at least 2")
    points: list[TrajectoryPoint] = []
    for index in range(samples):
        fraction = index / (samples - 1)
        points.append(
            TrajectoryPoint(
                x_m=distance_m * (1.0 - fraction),
                y_m=0.0,
                t_s=duration_s * fraction,
            )
        )
    return points


def default_wind_fields() -> dict[str, object]:
    """Return representative wind models for comparison."""

    return {
        "constant_tailwind": ConstantWindField(u_mps=-3.0, v_mps=0.0),
        "gust_headwind": SinusoidalGustWindField(
            base_u_mps=4.0,
            base_v_mps=0.0,
            gust_u_amplitude_mps=3.0,
            period_s=35.0,
        ),
        "spatial_grid_crosswind": GridWindField(
            u_grid=(
                (1.0, 2.0, 3.0, 4.0),
                (1.5, 2.5, 3.5, 4.5),
                (2.0, 3.0, 4.0, 5.0),
                (2.5, 3.5, 4.5, 5.5),
            ),
            v_grid=(
                (0.0, 1.0, 2.0, 3.0),
                (0.5, 1.5, 2.5, 3.5),
                (1.0, 2.0, 3.0, 4.0),
                (1.5, 2.5, 3.5, 4.5),
            ),
            resolution_m=300.0,
        ),
    }


def write_summary(rows: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_profiles(rows: Iterable[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def make_plot(profile_rows: list[dict[str, object]], output_dir: Path) -> None:
    if plt is None:
        print("matplotlib is not installed; skipping plot")
        return

    output_dir.mkdir(parents=True, exist_ok=True)
    scenario_names = sorted({str(row["scenario"]) for row in profile_rows})
    fig = plt.figure(figsize=(8, 5))
    for scenario in scenario_names:
        rows = [row for row in profile_rows if row["scenario"] == scenario]
        rows.sort(key=lambda row: float(row["t_s"]))
        plt.plot([float(row["t_s"]) for row in rows], [float(row["speed_mps"]) for row in rows], label=scenario)
    plt.xlabel("Time [s]")
    plt.ylabel("Wind speed [m/s]")
    plt.legend()
    plt.tight_layout()
    fig.savefig(output_dir / "wind_speed_profile.png", dpi=200)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run dynamic wind exposure experiment")
    parser.add_argument("--distance", type=float, default=900.0, help="Return distance in metres")
    parser.add_argument("--duration", type=float, default=120.0, help="Return duration in seconds")
    parser.add_argument("--samples", type=int, default=121, help="Trajectory samples")
    parser.add_argument("--output-dir", type=Path, default=Path("results_dynamic_wind"), help="Output directory")
    args = parser.parse_args()

    trajectory = straight_return_trajectory(args.distance, args.duration, args.samples)
    summary_rows: list[dict[str, object]] = []
    profile_rows: list[dict[str, object]] = []

    for name, field in default_wind_fields().items():
        wind_samples = sample_trajectory(field, trajectory)
        summary = summarize_wind_exposure(wind_samples)
        row = {"scenario": name, **asdict(summary)}
        summary_rows.append(row)

        for sample in wind_samples:
            profile_rows.append(
                {
                    "scenario": name,
                    "x_m": sample.x_m,
                    "y_m": sample.y_m,
                    "t_s": sample.t_s,
                    "u_mps": sample.vector.u,
                    "v_mps": sample.vector.v,
                    "speed_mps": sample.speed_mps,
                }
            )

    write_summary(summary_rows, args.output_dir / "wind_exposure_summary.csv")
    write_profiles(profile_rows, args.output_dir / "wind_speed_profiles.csv")
    make_plot(profile_rows, args.output_dir)

    print(f"Wrote wind exposure summary to {args.output_dir / 'wind_exposure_summary.csv'}")
    print(f"Wrote wind speed profiles to {args.output_dir / 'wind_speed_profiles.csv'}")


if __name__ == "__main__":
    main()
