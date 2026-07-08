"""Plotting utilities for generated simulation artifacts.

The functions in this module generate visual diagnostics from simulator output.
They do not encode experimental claims by themselves; claims must cite the
configuration and generated result files.
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt


def _apply_research_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 250,
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
            "axes.grid": True,
            "grid.alpha": 0.25,
        }
    )


def plot_trajectory(history: list[dict[str, float]], output_path: str | Path) -> None:
    """Save a 2D trajectory plot from simulator history."""
    _apply_research_style()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    xs = [row["x_m"] for row in history]
    ys = [row["y_m"] for row in history]
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    ax.plot(xs, ys, linewidth=2.2, label="UAV trajectory")
    ax.scatter([0.0], [0.0], marker="*", s=150, label="home")
    trigger = next((row for row in history if bool(row.get("returning_home", 0.0))), None)
    if trigger is not None:
        ax.scatter([trigger["x_m"]], [trigger["y_m"]], marker="D", s=70, label="RTH trigger")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Mission trajectory and RTH trigger")
    ax.axis("equal")
    ax.legend(frameon=True)
    fig.tight_layout()
    fig.savefig(output_path)
    fig.savefig(output_path.with_suffix(".pdf"))
    plt.close(fig)


def plot_timeseries(history: list[dict[str, float]], key: str, ylabel: str, output_path: str | Path) -> None:
    """Save a scalar time-series plot."""
    _apply_research_style()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    t = [row["time_s"] for row in history]
    y = [row[key] for row in history]
    fig, ax = plt.subplots(figsize=(6.2, 3.2))
    ax.plot(t, y, linewidth=2.2)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(ylabel)
    ax.set_title(ylabel)
    fig.tight_layout()
    fig.savefig(output_path)
    fig.savefig(output_path.with_suffix(".pdf"))
    plt.close(fig)


def plot_wind_timeseries(history: list[dict[str, float]], output_path: str | Path) -> None:
    """Save wind components over time."""
    _apply_research_style()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    t = [row["time_s"] for row in history]
    wx = [row["wind_x_mps"] for row in history]
    wy = [row["wind_y_mps"] for row in history]
    fig, ax = plt.subplots(figsize=(6.2, 3.2))
    ax.plot(t, wx, linewidth=2.0, label="wind x")
    ax.plot(t, wy, linewidth=2.0, label="wind y")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("wind [m/s]")
    ax.set_title("Wind disturbance over time")
    ax.legend(frameon=True)
    fig.tight_layout()
    fig.savefig(output_path)
    fig.savefig(output_path.with_suffix(".pdf"))
    plt.close(fig)


def plot_policy_comparison(metrics_csv_paths: list[str | Path], output_path: str | Path) -> None:
    """Create a simple comparison plot from generated metrics CSV files."""
    _apply_research_style()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    names: list[str] = []
    success: list[float] = []
    unsafe: list[float] = []
    early: list[float] = []
    for path_like in metrics_csv_paths:
        path = Path(path_like)
        with path.open("r", encoding="utf-8") as handle:
            row = next(csv.DictReader(handle))
        names.append(path.parent.name)
        success.append(float(row["mission_success_rate"]))
        unsafe.append(float(row["unsafe_failure_rate"]))
        early.append(float(row["early_return_rate"]))

    x = range(len(names))
    fig, ax = plt.subplots(figsize=(max(6.2, 0.55 * len(names)), 3.6))
    ax.plot(x, success, marker="o", linewidth=2.0, label="success")
    ax.plot(x, unsafe, marker="s", linewidth=2.0, label="unsafe failure")
    ax.plot(x, early, marker="^", linewidth=2.0, label="early return")
    ax.set_xticks(list(x), names, rotation=35, ha="right")
    ax.set_ylim(-0.02, 1.02)
    ax.set_ylabel("rate")
    ax.set_title("Policy/scenario comparison from generated metrics")
    ax.legend(frameon=True)
    fig.tight_layout()
    fig.savefig(output_path)
    fig.savefig(output_path.with_suffix(".pdf"))
    plt.close(fig)
