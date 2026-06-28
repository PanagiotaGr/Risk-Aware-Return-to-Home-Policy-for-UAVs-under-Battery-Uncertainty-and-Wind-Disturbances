#!/usr/bin/env python3
"""Battery aging study for the UAV return-to-home experiments.

This script studies how battery health degradation affects deterministic,
risk-aware Monte Carlo, and adaptive risk-aware RTH policies.

Run from the repository root:

    python3 experiments/run_battery_aging_study.py --trials 200

Outputs are written to results_battery_aging/.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import replace
from pathlib import Path
from statistics import mean, pstdev

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

from run_monte_carlo_experiments import PolicyConfig, default_scenarios, run_trial


POLICIES = [
    PolicyConfig(name="deterministic_threshold", deterministic_soc_threshold=0.50),
    PolicyConfig(name="risk_aware_mc", risk_threshold=0.95, mc_samples=100),
    PolicyConfig(name="adaptive_risk_mc", risk_threshold=0.88, mc_samples=100),
]

BATTERY_HEALTH_LEVELS = [1.00, 0.95, 0.90, 0.80, 0.70]


def ci95(rate: float, n: int) -> float:
    return 1.96 * math.sqrt(max(rate * (1.0 - rate), 0.0) / n)


def apply_battery_health(scenario, battery_health: float):
    """Reduce true available energy while keeping the estimator imperfect.

    In this standalone experiment, battery aging is represented as reduced
    effective initial SoC. The SoC estimator is made mildly optimistic when the
    battery is degraded, mimicking a realistic calibration error where the UAV
    reports charge percentage but not full capacity loss.
    """
    degradation = 1.0 - battery_health
    return replace(
        scenario,
        name=f"{scenario.name}_health_{int(battery_health * 100)}pct",
        initial_soc=scenario.initial_soc * battery_health,
        soc_bias=scenario.soc_bias + 0.35 * degradation,
    )


def summarize(results, policy_name, battery_health):
    n = len(results)
    safe = mean(r.safe_return for r in results)
    fail = mean(r.failure for r in results)
    early = mean(r.early_rth for r in results)
    rth = mean(r.rth_triggered for r in results)
    battery = [r.battery_left for r in results]
    return {
        "battery_health": battery_health,
        "policy": policy_name,
        "trials": n,
        "rth_trigger_rate": rth,
        "safe_return_rate": safe,
        "safe_return_ci95": ci95(safe, n),
        "failure_rate": fail,
        "failure_ci95": ci95(fail, n),
        "early_rth_rate": early,
        "early_rth_ci95": ci95(early, n),
        "mean_battery_left": mean(battery),
        "std_battery_left": pstdev(battery) if n > 1 else 0.0,
    }


def write_csv(rows, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_metric(rows, metric, path, title):
    if plt is None:
        return
    policies = sorted(set(row["policy"] for row in rows))
    fig = plt.figure(figsize=(8, 5))
    for policy in policies:
        selected = sorted([row for row in rows if row["policy"] == policy], key=lambda r: r["battery_health"])
        x = [row["battery_health"] for row in selected]
        y = [row[metric] for row in selected]
        plt.plot(x, y, marker="o", label=policy)
    plt.xlabel("Battery health")
    plt.ylabel(metric)
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--output-dir", type=Path, default=Path("results_battery_aging"))
    args = parser.parse_args()

    scenarios = default_scenarios()
    rows = []

    for health_index, health in enumerate(BATTERY_HEALTH_LEVELS):
        aged_scenarios = [apply_battery_health(s, health) for s in scenarios]
        for policy_index, policy in enumerate(POLICIES):
            results = []
            for scenario_index, scenario in enumerate(aged_scenarios):
                for trial in range(args.trials):
                    seed = 900000 + health_index * 100000 + policy_index * 10000 + scenario_index * 100 + trial
                    results.append(run_trial(scenario, policy, trial, seed))
            rows.append(summarize(results, policy.name, health))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(rows, args.output_dir / "battery_aging_summary.csv")
    plot_metric(rows, "failure_rate", args.output_dir / "battery_health_vs_failure.png", "Failure rate vs battery health")
    plot_metric(rows, "safe_return_rate", args.output_dir / "battery_health_vs_safe_return.png", "Safe return rate vs battery health")
    plot_metric(rows, "mean_battery_left", args.output_dir / "battery_health_vs_remaining_energy.png", "Remaining energy vs battery health")
    print(f"Wrote battery aging study to {args.output_dir}")


if __name__ == "__main__":
    main()
