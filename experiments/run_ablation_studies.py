#!/usr/bin/env python3
"""Ablation studies for publication-level UAV RTH evaluation.

Runs:
- deterministic SoC threshold sweep
- risk threshold tau sweep
- Monte Carlo sample-count sweep

Outputs CSV summaries and simple plots in results_ablation/.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path
from statistics import mean, pstdev

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

from run_monte_carlo_experiments import PolicyConfig, default_scenarios, run_trial


def ci95(rate, n):
    return 1.96 * math.sqrt(max(rate * (1.0 - rate), 0.0) / n)


def summarize(results, label, extra=None):
    n = len(results)
    safe = mean(r.safe_return for r in results)
    fail = mean(r.failure for r in results)
    early = mean(r.early_rth for r in results)
    rth = mean(r.rth_triggered for r in results)
    battery = [r.battery_left for r in results]
    row = {
        "label": label,
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
    if extra:
        row.update(extra)
    return row


def write_csv(rows, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_policy(policy, trials, seed):
    results = []
    scenarios = default_scenarios()
    for si, scenario in enumerate(scenarios):
        for trial in range(trials):
            results.append(run_trial(scenario, policy, trial, seed + si * 10000 + trial))
    return results


def deterministic_sweep(trials, outdir):
    rows = []
    for i, threshold in enumerate([0.25, 0.30, 0.35, 0.40, 0.45, 0.50]):
        policy = PolicyConfig(name="deterministic_threshold", deterministic_soc_threshold=threshold)
        results = run_policy(policy, trials, 100000 * (i + 1))
        rows.append(summarize(results, f"soc_{int(threshold * 100)}pct", {"soc_threshold": threshold}))
    write_csv(rows, outdir / "deterministic_threshold_sweep.csv")
    return rows


def tau_sweep(trials, outdir):
    rows = []
    for i, tau in enumerate([0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98]):
        policy = PolicyConfig(name="risk_aware_mc", risk_threshold=tau, mc_samples=500)
        results = run_policy(policy, trials, 200000 * (i + 1))
        rows.append(summarize(results, f"tau_{tau:.2f}", {"tau": tau}))
    write_csv(rows, outdir / "risk_threshold_tau_sweep.csv")
    return rows


def sample_sweep(trials, outdir):
    rows = []
    for i, samples in enumerate([50, 100, 250, 500, 1000]):
        policy = PolicyConfig(name="risk_aware_mc", risk_threshold=0.90, mc_samples=samples)
        start = time.perf_counter()
        results = run_policy(policy, trials, 300000 * (i + 1))
        elapsed = time.perf_counter() - start
        rows.append(summarize(results, f"samples_{samples}", {
            "mc_samples": samples,
            "runtime_seconds": elapsed,
            "runtime_per_trial_ms": 1000.0 * elapsed / len(results),
        }))
    write_csv(rows, outdir / "mc_sample_sweep.csv")
    return rows


def plot_line(rows, xkey, ykey, path, title):
    if plt is None:
        return
    rows = sorted(rows, key=lambda r: r[xkey])
    x = [r[xkey] for r in rows]
    y = [r[ykey] for r in rows]
    fig = plt.figure(figsize=(8, 5))
    plt.plot(x, y, marker="o")
    plt.xlabel(xkey)
    plt.ylabel(ykey)
    plt.title(title)
    plt.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--output-dir", type=Path, default=Path("results_ablation"))
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    deterministic = deterministic_sweep(args.trials, args.output_dir)
    tau = tau_sweep(args.trials, args.output_dir)
    samples = sample_sweep(args.trials, args.output_dir)

    plot_line(deterministic, "soc_threshold", "failure_rate", args.output_dir / "deterministic_failure_vs_threshold.png", "Deterministic failure vs SoC threshold")
    plot_line(deterministic, "soc_threshold", "safe_return_rate", args.output_dir / "deterministic_safe_return_vs_threshold.png", "Deterministic safe return vs SoC threshold")
    plot_line(tau, "tau", "failure_rate", args.output_dir / "risk_failure_vs_tau.png", "Risk-aware failure vs tau")
    plot_line(tau, "tau", "safe_return_rate", args.output_dir / "risk_safe_return_vs_tau.png", "Risk-aware safe return vs tau")
    plot_line(samples, "mc_samples", "failure_rate", args.output_dir / "failure_vs_mc_samples.png", "Failure vs Monte Carlo samples")
    plot_line(samples, "mc_samples", "runtime_per_trial_ms", args.output_dir / "runtime_vs_mc_samples.png", "Runtime vs Monte Carlo samples")

    print(f"Wrote ablation results to {args.output_dir}")


if __name__ == "__main__":
    main()
