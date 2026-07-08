"""Run controlled RTH simulation experiments from YAML."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from risk_rth.evaluation.metrics import summarize_results
from risk_rth.models.battery import BatteryModel
from risk_rth.models.energy import EnergyModel
from risk_rth.models.wind import WindModel
from risk_rth.planning.policies import (
    DeterministicEnergyPolicy,
    DistanceBasedPolicy,
    FixedBatteryThresholdPolicy,
    RiskAwareMonteCarloPolicy,
)
from risk_rth.simulation.simulator import MissionSimulator2D, SimulatorConfig
from risk_rth.uncertainty.monte_carlo import MonteCarloRiskEstimator
from risk_rth.utils.config import load_yaml
from risk_rth.visualization.plots import plot_timeseries, plot_trajectory, plot_wind_timeseries


def build_policy(name: str, cfg: dict):
    """Build an RTH policy from YAML configuration."""
    if name == "fixed_battery_threshold":
        return FixedBatteryThresholdPolicy(threshold_soc=float(cfg.get("threshold_soc", 0.25)))
    if name == "distance_based_rth":
        return DistanceBasedPolicy(max_distance_m=float(cfg.get("max_distance_m", 350.0)))
    if name == "deterministic_energy_estimate":
        return DeterministicEnergyPolicy()
    if name == "risk_aware_mc":
        estimator = MonteCarloRiskEstimator(n_samples=int(cfg.get("mc_samples", 512)))
        return RiskAwareMonteCarloPolicy(tau=float(cfg.get("tau", 0.95)), estimator=estimator)
    raise ValueError(f"Unknown policy: {name}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/experiments/nominal.yaml")
    parser.add_argument("--output-dir", default="results/controlled")
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    output_dir = Path(args.output_dir) / str(cfg.get("name", "experiment"))
    figures_dir = output_dir / "figures"
    output_dir.mkdir(parents=True, exist_ok=True)

    sim_cfg = SimulatorConfig(**cfg.get("simulation", {}))
    battery = BatteryModel(**cfg.get("battery", {}))
    wind = WindModel(**cfg.get("wind", {}))
    energy = EnergyModel(**cfg.get("energy", {}))
    policy_cfg = cfg.get("policy", {})
    policy = build_policy(str(policy_cfg.get("name", "risk_aware_mc")), policy_cfg)

    trials = int(cfg.get("trials", 1))
    seed = int(cfg.get("seed", 7))
    results = []
    for i in range(trials):
        simulator = MissionSimulator2D(sim_cfg, battery, wind, energy, seed=seed + i)
        results.append(simulator.run(policy))

    metrics = summarize_results(results)
    with (output_dir / "metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(metrics))
        writer.writeheader()
        writer.writerow(metrics)

    representative = results[0]
    if representative.history:
        plot_trajectory(representative.history, figures_dir / "trajectory.png")
        plot_timeseries(representative.history, "soc", "battery_soc", figures_dir / "battery_soc.png")
        plot_timeseries(
            representative.history,
            "p_safe",
            "estimated safe-return probability",
            figures_dir / "p_safe.png",
        )
        plot_wind_timeseries(representative.history, figures_dir / "wind.png")

    print(metrics)


if __name__ == "__main__":
    main()
