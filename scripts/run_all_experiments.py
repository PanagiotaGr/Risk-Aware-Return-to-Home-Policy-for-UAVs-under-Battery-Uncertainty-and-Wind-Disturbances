"""Run the configured controlled experiment suite.

This script is intentionally lightweight: it executes existing YAML scenarios and
leaves all numerical claims in generated CSV files under `results/`.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from risk_rth.visualization.plots import plot_policy_comparison


DEFAULT_CONFIGS = [
    "configs/experiments/nominal.yaml",
    "configs/experiments/low_battery_uncertainty.yaml",
    "configs/experiments/high_battery_uncertainty.yaml",
    "configs/experiments/headwind.yaml",
    "configs/experiments/tailwind.yaml",
    "configs/experiments/crosswind.yaml",
    "configs/experiments/gust.yaml",
    "configs/experiments/high_wind_variance.yaml",
    "configs/experiments/long_distance.yaml",
    "configs/experiments/aggressive_tau.yaml",
    "configs/experiments/conservative_tau.yaml",
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="results/controlled")
    parser.add_argument("--python", default=sys.executable)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    generated_metrics: list[Path] = []
    for config in DEFAULT_CONFIGS:
        subprocess.run(
            [args.python, "scripts/run_experiment.py", "--config", config, "--output-dir", str(output_dir)],
            check=True,
        )

    for metrics in output_dir.glob("*/metrics.csv"):
        generated_metrics.append(metrics)

    if generated_metrics:
        plot_policy_comparison(generated_metrics, output_dir / "figures" / "scenario_comparison.png")


if __name__ == "__main__":
    main()
