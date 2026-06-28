#!/usr/bin/env python3
"""Dynamic wind gust study for UAV return-to-home experiments.

This standalone experiment evaluates deterministic, risk-aware, adaptive, and
health-aware RTH policies under time-varying wind disturbances. The return energy
is computed by integrating over a generated wind profile instead of using a
single steady wind value.

Run from the repository root:

    python3 experiments/run_wind_gust_study.py --trials 200

Outputs are written to results_wind_gust/.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, pstdev

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

from run_monte_carlo_experiments import (
    PolicyConfig,
    Scenario,
    adaptive_threshold,
    return_energy,
)


@dataclass(frozen=True)
class GustScenario:
    name: str
    distance_m: float
    initial_soc: float
    soc_noise_std: float
    soc_bias: float
    wind_mean_mps: float
    wind_std_mps: float
    wind_direction: str
    gust_std_mps: float
    gust_probability: float
    gust_peak_mps: float
    extra_drain_probability: float
    extra_drain_soc: float
    battery_health: float = 1.0


POLICIES = [
    PolicyConfig(name="deterministic_threshold", deterministic_soc_threshold=0.50),
    PolicyConfig(name="risk_aware_mc", risk_threshold=0.95, mc_samples=100),
    PolicyConfig(name="adaptive_risk_mc", risk_threshold=0.88, mc_samples=100),
    PolicyConfig(name="health_aware_risk_mc", risk_threshold=0.95, mc_samples=100),
]


def ci95(rate: float, n: int) -> float:
    return 1.96 * math.sqrt(max(rate * (1.0 - rate), 0.0) / n)


def wind_profile(scenario: GustScenario, rng: random.Random, steps: int = 12) -> list[float]:
    values = []
    for _ in range(steps):
        wind = rng.gauss(scenario.wind_mean_mps, scenario.wind_std_mps)
        if rng.random() < scenario.gust_probability:
            wind += abs(rng.gauss(scenario.gust_peak_mps, scenario.gust_std_mps))
        values.append(max(0.0, wind))
    return values


def integrated_return_energy(scenario: GustScenario, profile: list[float], extra_drain: float) -> float:
    if not profile:
        profile = [scenario.wind_mean_mps]
    segment_distance = scenario.distance_m / len(profile)
    segment_energy = [return_energy(segment_distance, wind, scenario.wind_direction, 0.0) for wind in profile]
    # return_energy includes reserve per call, so remove repeated reserves and add one final reserve
    reserve = 0.08
    corrected = sum(e - reserve for e in segment_energy) + reserve + extra_drain
    return corrected


def estimate_probability(policy: PolicyConfig, scenario: GustScenario, estimated_soc: float, rng: random.Random) -> float:
    feasible = 0
    use_health = policy.name == "health_aware_risk_mc"
    for _ in range(policy.mc_samples):
        sampled_soc = max(0.0, min(1.0, rng.gauss(estimated_soc - scenario.soc_bias, scenario.soc_noise_std)))
        if use_health:
            sampled_soc *= scenario.battery_health
        profile = wind_profile(scenario, rng)
        extra = scenario.extra_drain_soc if rng.random() < scenario.extra_drain_probability else 0.0
        required = integrated_return_energy(scenario, profile, extra)
        if sampled_soc >= required:
            feasible += 1
    return feasible / policy.mc_samples


def trigger(policy: PolicyConfig, scenario: GustScenario, estimated_soc: float, rng: random.Random) -> tuple[bool, float, float]:
    if policy.name == "deterministic_threshold":
        return estimated_soc < policy.deterministic_soc_threshold, float("nan"), policy.deterministic_soc_threshold

    base = Scenario(
        name=scenario.name,
        distance_m=scenario.distance_m,
        initial_soc=scenario.initial_soc,
        soc_noise_std=scenario.soc_noise_std,
        soc_bias=scenario.soc_bias,
        wind_mean_mps=scenario.wind_mean_mps,
        wind_std_mps=scenario.wind_std_mps,
        wind_direction=scenario.wind_direction,
        extra_drain_probability=scenario.extra_drain_probability,
        extra_drain_soc=scenario.extra_drain_soc,
        battery_health=scenario.battery_health,
    )
    tau = adaptive_threshold(policy.risk_threshold, base) if policy.name == "adaptive_risk_mc" else policy.risk_threshold
    probability = estimate_probability(policy, scenario, estimated_soc, rng)
    return probability < tau, probability, tau


def run_trial(scenario: GustScenario, policy: PolicyConfig, trial: int, seed: int) -> dict[str, object]:
    rng = random.Random(seed)
    true_soc = max(0.0, min(1.0, rng.gauss(scenario.initial_soc, 0.015)))
    estimated_soc = max(0.0, min(1.0, true_soc + scenario.soc_bias + rng.gauss(0.0, scenario.soc_noise_std)))
    profile = wind_profile(scenario, rng)
    extra = scenario.extra_drain_soc if rng.random() < scenario.extra_drain_probability else 0.0
    required = integrated_return_energy(scenario, profile, extra)
    triggered, probability, threshold = trigger(policy, scenario, estimated_soc, rng)

    available = true_soc
    margin = available - required
    can_return = margin >= 0.0
    safe_return = triggered and can_return
    failure = (not triggered and not can_return) or (triggered and not can_return)
    battery_left = true_soc - required if triggered else true_soc - 0.55 * required
    completion = 0.55 if triggered and battery_left >= 0.0 else 0.40 if triggered else max(0.0, min(1.0, 0.75 + battery_left)) if failure else 1.0

    return {
        "scenario": scenario.name,
        "policy": policy.name,
        "trial": trial,
        "gust_std_mps": scenario.gust_std_mps,
        "gust_probability": scenario.gust_probability,
        "gust_peak_mps": scenario.gust_peak_mps,
        "mean_profile_wind_mps": mean(profile),
        "max_profile_wind_mps": max(profile),
        "rth_triggered": triggered,
        "safe_return": safe_return,
        "failure": failure,
        "mission_completion_ratio": completion,
        "available_energy": available,
        "required_return_energy": required,
        "energy_margin": margin,
        "negative_margin": margin < 0.0,
        "battery_left": battery_left,
        "estimated_safe_return_probability": probability,
        "threshold_used": threshold,
    }


def scenarios() -> list[GustScenario]:
    levels = [
        ("gust_low", 0.5, 0.10, 1.0),
        ("gust_medium", 1.0, 0.20, 2.0),
        ("gust_high", 1.5, 0.30, 3.0),
        ("gust_extreme", 2.0, 0.40, 4.5),
    ]
    out = []
    for name, gust_std, gust_prob, gust_peak in levels:
        out.append(
            GustScenario(
                name=name,
                distance_m=950.0,
                initial_soc=0.58,
                soc_noise_std=0.05,
                soc_bias=0.03,
                wind_mean_mps=5.0,
                wind_std_mps=0.75,
                wind_direction="headwind",
                gust_std_mps=gust_std,
                gust_probability=gust_prob,
                gust_peak_mps=gust_peak,
                extra_drain_probability=0.08,
                extra_drain_soc=0.04,
                battery_health=0.90,
            )
        )
    return out


def summarize(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    groups = {}
    for row in rows:
        key = (row["scenario"], row["policy"])
        groups.setdefault(key, []).append(row)

    summary = []
    for (scenario, policy), items in sorted(groups.items()):
        n = len(items)
        safe = mean(bool(i["safe_return"]) for i in items)
        fail = mean(bool(i["failure"]) for i in items)
        margins = [float(i["energy_margin"]) for i in items]
        summary.append({
            "scenario": scenario,
            "policy": policy,
            "trials": n,
            "rth_trigger_rate": mean(bool(i["rth_triggered"]) for i in items),
            "safe_return_rate": safe,
            "safe_return_ci95": ci95(safe, n),
            "failure_rate": fail,
            "failure_ci95": ci95(fail, n),
            "mission_completion_rate": mean(float(i["mission_completion_ratio"]) for i in items),
            "mean_energy_margin": mean(margins),
            "std_energy_margin": pstdev(margins) if n > 1 else 0.0,
            "negative_margin_rate": mean(bool(i["negative_margin"]) for i in items),
            "mean_profile_wind_mps": mean(float(i["mean_profile_wind_mps"]) for i in items),
            "mean_max_wind_mps": mean(float(i["max_profile_wind_mps"]) for i in items),
        })
    return summary


def write_csv(rows: list[dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_metric(summary: list[dict[str, object]], metric: str, path: Path, title: str) -> None:
    if plt is None:
        return
    policies = sorted(set(str(r["policy"]) for r in summary))
    scenarios_order = ["gust_low", "gust_medium", "gust_high", "gust_extreme"]
    fig = plt.figure(figsize=(8, 5))
    for policy in policies:
        selected = [r for r in summary if r["policy"] == policy]
        selected = sorted(selected, key=lambda r: scenarios_order.index(str(r["scenario"])))
        x = [scenarios_order.index(str(r["scenario"])) for r in selected]
        y = [float(r[metric]) for r in selected]
        plt.plot(x, y, marker="o", label=policy)
    plt.xticks(range(len(scenarios_order)), scenarios_order, rotation=20)
    plt.ylabel(metric)
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=200)
    parser.add_argument("--output-dir", type=Path, default=Path("results_wind_gust"))
    args = parser.parse_args()

    rows = []
    for scenario_index, scenario in enumerate(scenarios()):
        for policy_index, policy in enumerate(POLICIES):
            for trial in range(args.trials):
                seed = 700000 + scenario_index * 100000 + policy_index * 10000 + trial
                rows.append(run_trial(scenario, policy, trial, seed))

    summary = summarize(rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(rows, args.output_dir / "wind_gust_trials.csv")
    write_csv(summary, args.output_dir / "wind_gust_summary.csv")
    plot_metric(summary, "safe_return_rate", args.output_dir / "gust_safe_return_rate.png", "Safe return under dynamic wind gusts")
    plot_metric(summary, "failure_rate", args.output_dir / "gust_failure_rate.png", "Failure under dynamic wind gusts")
    plot_metric(summary, "mean_energy_margin", args.output_dir / "gust_energy_margin.png", "Energy margin under dynamic wind gusts")
    plot_metric(summary, "mission_completion_rate", args.output_dir / "gust_mission_completion.png", "Mission completion under dynamic wind gusts")
    print(f"Wrote dynamic wind gust study to {args.output_dir}")


if __name__ == "__main__":
    main()
