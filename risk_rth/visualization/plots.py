"""Plotting utilities for generated simulation artifacts."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt


def plot_trajectory(history: list[dict[str, float]], output_path: str | Path) -> None:
    """Save a 2D trajectory plot from simulator history."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    xs = [row["x_m"] for row in history]
    ys = [row["y_m"] for row in history]
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(xs, ys, linewidth=2)
    ax.scatter([0.0], [0.0], marker="*", s=120, label="home")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("UAV mission trajectory")
    ax.axis("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def plot_timeseries(history: list[dict[str, float]], key: str, ylabel: str, output_path: str | Path) -> None:
    """Save a scalar time-series plot."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    t = [row["time_s"] for row in history]
    y = [row[key] for row in history]
    fig, ax = plt.subplots(figsize=(6, 3))
    ax.plot(t, y, linewidth=2)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(ylabel)
    ax.set_title(ylabel)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
