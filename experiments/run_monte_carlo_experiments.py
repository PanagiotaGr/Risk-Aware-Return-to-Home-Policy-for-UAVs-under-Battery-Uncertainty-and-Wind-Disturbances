#!/usr/bin/env python3
"""
Reproducible Monte Carlo experiments for risk-aware UAV return-to-home policies.

The script is intentionally standalone so that the experimental results can be
reproduced without launching the ROS 2 runtime. It compares four policies:

1. deterministic_threshold: triggers RTH when estimated SoC is below a fixed threshold
2. risk_aware_mc: estimates P(safe return) with Monte Carlo sampling
3. adaptive_risk_mc: adjusts the risk threshold using wind, distance, and SoC uncertainty
4. health_aware_risk_mc: estimates returnability using battery health degradation

Outputs:
- results/experiment_trials.csv: one row per trial
- results/summary_by_policy.csv: aggregated statistics
- results/summary_by_scenario.csv: aggregated statistics by scenario and policy
- results/*.png: publication-oriented plots
"""

from __future__ import annotations

import argparse
import csv
import random
from dataclasses import dataclass, asdict
from pathlib import Path
from statistics import mean, pstdev
from typing import Dict, Iterable, List, Tuple

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    plt = None


@dataclass(frozen=True)
class Scenario:
    name: str
    distance_m: float
    initial_soc: float
    soc_noise_std: float
    soc_bias: float
    wind_mean_mps: float
    wind_std_mps: float
    wind_direction: str
    extra_drain_probability: float
    extra_drain_soc: float
    battery_health: float = 1.0


@dataclass(frozen=True)
class PolicyConfig:
    name: str
    deterministic_soc_threshold: float = 0.30
    risk_threshold: float = 0.90
    mc_samples: int = 500


@dataclass
class TrialResult:
    scenario: str
    policy: str
    trial: int
    distance_m: float
    planned_distance: float
    distance_completed: float
    mission_completion_ratio: float
    initial_soc_true: float
    initial_soc_estimated: float
    battery_health: float
    available_energy: float
    required_return_energy: float
    energy_margin: float
    normalized_energy_margin: float
    wind_mps: float
    wind_direction: str
    return_energy_required: float
    rth_triggered: bool
    safe_return: bool
    failure: bool
    early_rth: bool
    battery_left: float
    estimated_safe_return_probability: float
    threshold_used: float


BASE_RETURN_ENERGY_PER_M = 0.00042
HEADWIND_PENALTY = 0.035
CROSSWIND_PENALTY = 0.014
TAILWIND_CREDIT = 0.018
SAFETY_RESERVE_SOC = 0.08
EARLY_RTH_MARGIN = 0.18


def wind_factor(wind_mps: float, direction: str) -> float:
    if direction == "headwind":
        return 1.0 + HEADWIND_PENALTY * wind_mps
    if direction == "crosswind":
        return 1.0 + CROSSWIND_PENALTY * wind_mps
    if direction == "tailwind":
        return max(0.65, 1.0 - TAILWIND_CREDIT * wind_mps)
    raise ValueError(f"Unknown wind direction: {direction}")


def return_energy(distance_m: float, wind_mps: float, direction: str, extra_drain: float = 0.0) -> float:
    nominal = BASE_RETURN_ENERGY_PER_M * distance_m
    return nominal * wind_factor(wind_mps, direction) + extra_drain + SAFETY_RESERVE_SOC


def estimate_safe_return_probability(
    estimated_soc: float,
    scenario: Scenario,
    rng: random.Random,
    mc_samples: int,
    use_battery_health: bool = False,
) -> float:
    feasible = 0
    for _ in range(mc_samples):
        sampled_soc = max(0.0, min(1.0, rng.gauss(estimated_soc - scenario.soc_bias, scenario.soc_noise_std)))
        if use_battery_health:
            sampled_soc *= scenario.battery_health
        sampled_wind = max(0.0, rng.gauss(scenario.wind_mean_mps, scenario.wind_std_mps))
        sampled_extra = scenario.extra_drain_soc if rng.random() < scenario.extra_drain_probability else 0.0
        required = return_energy(scenario.distance_m, sampled_wind, scenario.wind_direction, sampled_extra)
        if sampled_soc >= required:
            feasible += 1
    return feasible / mc_samples


def adaptive_threshold(base_tau: float, scenario: Scenario) -> float:
    wind_risk = min(0.08, 0.008 * scenario.wind_mean_mps)
    uncertainty_risk = min(0.08, 1.8 * scenario.soc_noise_std)
    distance_risk = min(0.06, scenario.distance_m / 15000.0)
    health_risk = min(0.10, 0.25 * (1.0 - scenario.battery_health))
    return max(0.75, min(0.99, base_tau + wind_risk + uncertainty_risk + distance_risk + health_risk))


def should_trigger_rth(policy: PolicyConfig, scenario: Scenario, estimated_soc: float, rng: random.Random) -> Tuple[bool, float, float]:
    if policy.name == "deterministic_threshold":
        return estimated_soc < policy.deterministic_soc_threshold, float("nan"), policy.deterministic_soc_threshold

    tau = policy.risk_threshold
    use_battery_health = policy.name == "health_aware_risk_mc"
    if policy.name == "adaptive_risk_mc":
        tau = adaptive_threshold(policy.risk_threshold, scenario)

    probability = estimate_safe_return_probability(
        estimated_soc,
        scenario,
        rng,
        policy.mc_samples,
        use_battery_health=use_battery_health,
    )
    return probability < tau, probability, tau


def mission_progress(triggered: bool, failure: bool, battery_left: float) -> Tuple[float, float]:
    """Return distance-completion fraction and normalized mission completion."""
    if triggered:
        ratio = 0.55 if battery_left >= 0.0 else 0.40
    elif failure:
        ratio = max(0.0, 0.75 + battery_left)
    else:
        ratio = 1.0
    ratio = max(0.0, min(1.0, ratio))
    return ratio, ratio


def run_trial(scenario: Scenario, policy: PolicyConfig, trial: int, seed: int) -> TrialResult:
    rng = random.Random(seed)
    true_soc = max(0.0, min(1.0, rng.gauss(scenario.initial_soc, 0.015)))
    estimated_soc = max(0.0, min(1.0, true_soc + scenario.soc_bias + rng.gauss(0.0, scenario.soc_noise_std)))
    actual_wind = max(0.0, rng.gauss(scenario.wind_mean_mps, scenario.wind_std_mps))
    actual_extra = scenario.extra_drain_soc if rng.random() < scenario.extra_drain_probability else 0.0
    required = return_energy(scenario.distance_m, actual_wind, scenario.wind_direction, actual_extra)

    triggered, probability, threshold = should_trigger_rth(policy, scenario, estimated_soc, rng)
    available_energy = true_soc
    energy_margin = available_energy - required
    normalized_energy_margin = energy_margin / max(required, 1e-9)
    can_return = energy_margin >= 0.0
    safe_return = triggered and can_return
    failure = (not triggered and not can_return) or (triggered and not can_return)
    battery_left = true_soc - required if triggered else true_soc - 0.55 * required
    early_rth = triggered and battery_left > EARLY_RTH_MARGIN
    completion_ratio, _ = mission_progress(triggered, failure, battery_left)
    planned_distance = 2.0 * scenario.distance_m
    distance_completed = planned_distance * completion_ratio

    return TrialResult(
        scenario=scenario.name,
        policy=policy.name,
        trial=trial,
        distance_m=scenario.distance_m,
        planned_distance=planned_distance,
        distance_completed=distance_completed,
        mission_completion_ratio=completion_ratio,
        initial_soc_true=true_soc,
        initial_soc_estimated=estimated_soc,
        battery_health=scenario.battery_health,
        available_energy=available_energy,
        required_return_energy=required,
        energy_margin=energy_margin,
        normalized_energy_margin=normalized_energy_margin,
        wind_mps=actual_wind,
        wind_direction=scenario.wind_direction,
        return_energy_required=required,
        rth_triggered=triggered,
        safe_return=safe_return,
        failure=failure,
        early_rth=early_rth,
        battery_left=battery_left,
        estimated_safe_return_probability=probability,
        threshold_used=threshold,
    )


def default_scenarios() -> List[Scenario]:
    scenarios: List[Scenario] = []
    for wind_direction in ["tailwind", "crosswind", "headwind"]:
        for wind in [0.0, 2.0, 5.0, 8.0, 10.0]:
            scenarios.append(
                Scenario(
                    name=f"wind_{wind_direction}_{wind:.0f}mps",
                    distance_m=900.0,
                    initial_soc=0.62,
                    soc_noise_std=0.04,
                    soc_bias=0.00,
                    wind_mean_mps=wind,
                    wind_std_mps=0.75,
                    wind_direction=wind_direction,
                    extra_drain_probability=0.05,
                    extra_drain_soc=0.04,
                )
            )

    for noise in [0.02, 0.05, 0.10, 0.15]:
        scenarios.append(
            Scenario(
                name=f"battery_noise_{int(noise * 100)}pct",
                distance_m=950.0,
                initial_soc=0.58,
                soc_noise_std=noise,
                soc_bias=0.00,
                wind_mean_mps=5.0,
                wind_std_mps=1.0,
                wind_direction="headwind",
                extra_drain_probability=0.05,
                extra_drain_soc=0.04,
            )
        )

    for bias in [-0.08, -0.04, 0.04, 0.08]:
        label = "optimistic" if bias > 0 else "pessimistic"
        scenarios.append(
            Scenario(
                name=f"soc_bias_{label}_{abs(int(bias * 100))}pct",
                distance_m=1000.0,
                initial_soc=0.56,
                soc_noise_std=0.05,
                soc_bias=bias,
                wind_mean_mps=6.0,
                wind_std_mps=1.25,
                wind_direction="headwind",
                extra_drain_probability=0.10,
                extra_drain_soc=0.05,
            )
        )

    scenarios.append(
        Scenario(
            name="fault_injection_extra_drain",
            distance_m=1050.0,
            initial_soc=0.57,
            soc_noise_std=0.06,
            soc_bias=0.04,
            wind_mean_mps=7.0,
            wind_std_mps=1.5,
            wind_direction="headwind",
            extra_drain_probability=0.30,
            extra_drain_soc=0.08,
        )
    )
    return scenarios


def default_policies(mc_samples: int) -> List[PolicyConfig]:
    return [
        PolicyConfig(name="deterministic_threshold", deterministic_soc_threshold=0.30, mc_samples=mc_samples),
        PolicyConfig(name="risk_aware_mc", risk_threshold=0.90, mc_samples=mc_samples),
        PolicyConfig(name="adaptive_risk_mc", risk_threshold=0.88, mc_samples=mc_samples),
        PolicyConfig(name="health_aware_risk_mc", risk_threshold=0.90, mc_samples=mc_samples),
    ]


def write_trials_csv(results: List[TrialResult], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(results[0]).keys()))
        writer.writeheader()
        for row in results:
            writer.writerow(asdict(row))


def aggregate(results: Iterable[TrialResult], keys: Tuple[str, ...]) -> List[Dict[str, object]]:
    groups: Dict[Tuple[object, ...], List[TrialResult]] = {}
    for result in results:
        key = tuple(getattr(result, item) for item in keys)
        groups.setdefault(key, []).append(result)

    rows: List[Dict[str, object]] = []
    for key, items in sorted(groups.items()):
        battery_values = [item.battery_left for item in items]
        completion_values = [item.mission_completion_ratio for item in items]
        distance_values = [item.distance_completed for item in items]
        margin_values = [item.energy_margin for item in items]
        normalized_margin_values = [item.normalized_energy_margin for item in items]
        row: Dict[str, object] = {name: value for name, value in zip(keys, key)}
        row.update(
            trials=len(items),
            rth_trigger_rate=mean(item.rth_triggered for item in items),
            safe_return_rate=mean(item.safe_return for item in items),
            failure_rate=mean(item.failure for item in items),
            early_rth_rate=mean(item.early_rth for item in items),
            mission_completion_rate=mean(completion_values),
            mean_distance_completed=mean(distance_values),
            mean_energy_margin=mean(margin_values),
            std_energy_margin=pstdev(margin_values) if len(margin_values) > 1 else 0.0,
            mean_normalized_energy_margin=mean(normalized_margin_values),
            negative_margin_rate=mean(item.energy_margin < 0.0 for item in items),
            mean_battery_left=mean(battery_values),
            std_battery_left=pstdev(battery_values) if len(battery_values) > 1 else 0.0,
        )
        rows.append(row)
    return rows


def write_dict_csv(rows: List[Dict[str, object]], path: Path) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def make_plots(summary_by_policy: List[Dict[str, object]], output_dir: Path) -> None:
    if plt is None:
        print("matplotlib is not installed; skipping plots")
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    policies = [str(row["policy"]) for row in summary_by_policy]

    for metric, ylabel, filename in [
        ("safe_return_rate", "Safe return rate", "safe_return_rate.png"),
        ("failure_rate", "Failure rate", "failure_rate.png"),
        ("early_rth_rate", "Early RTH rate", "early_rth_rate.png"),
        ("mission_completion_rate", "Mission completion ratio", "mission_completion.png"),
        ("mean_energy_margin", "Mean energy margin", "energy_margin.png"),
        ("negative_margin_rate", "Negative margin rate", "negative_margin_rate.png"),
        ("mean_battery_left", "Mean battery left", "battery_left.png"),
    ]:
        values = [float(row[metric]) for row in summary_by_policy]
        fig = plt.figure(figsize=(8, 5))
        plt.bar(policies, values)
        plt.ylabel(ylabel)
        plt.xlabel("Policy")
        plt.xticks(rotation=20, ha="right")
        plt.tight_layout()
        fig.savefig(output_dir / filename, dpi=200)
        plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run UAV RTH Monte Carlo policy experiments")
    parser.add_argument("--trials", type=int, default=200, help="Trials per scenario-policy pair")
    parser.add_argument("--mc-samples", type=int, default=500, help="Monte Carlo samples per policy decision")
    parser.add_argument("--seed", type=int, default=7, help="Base random seed")
    parser.add_argument("--output-dir", type=Path, default=Path("results"), help="Output directory")
    args = parser.parse_args()

    scenarios = default_scenarios()
    policies = default_policies(args.mc_samples)
    results: List[TrialResult] = []

    for scenario_index, scenario in enumerate(scenarios):
        for policy_index, policy in enumerate(policies):
            for trial in range(args.trials):
                seed = args.seed + 100000 * scenario_index + 1000 * policy_index + trial
                results.append(run_trial(scenario, policy, trial, seed))

    write_trials_csv(results, args.output_dir / "experiment_trials.csv")
    summary_by_policy = aggregate(results, ("policy",))
    summary_by_scenario = aggregate(results, ("scenario", "policy"))
    write_dict_csv(summary_by_policy, args.output_dir / "summary_by_policy.csv")
    write_dict_csv(summary_by_scenario, args.output_dir / "summary_by_scenario.csv")
    make_plots(summary_by_policy, args.output_dir)

    print(f"Wrote {len(results)} trial rows to {args.output_dir / 'experiment_trials.csv'}")
    print(f"Wrote policy summary to {args.output_dir / 'summary_by_policy.csv'}")
    print(f"Wrote scenario summary to {args.output_dir / 'summary_by_scenario.csv'}")


if __name__ == "__main__":
    main()
