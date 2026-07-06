from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from .benchmark import run_trials, safe_rate_with_ci
from .config import load_scenario
from .simulator import RiskAwareMissionSimulator


def main() -> None:
    parser = argparse.ArgumentParser(description="Run risk-aware UAV platform benchmarks.")
    parser.add_argument("--scenario", default="configs/platform/default_scenario.json", help="Path to scenario JSON file.")
    parser.add_argument("--trials", type=int, default=200, help="Number of benchmark trials.")
    parser.add_argument("--output", default="results_platform/platform_benchmark.csv", help="CSV output path.")
    parser.add_argument("--timeline", default="results_platform/latest_timeline.json", help="JSON timeline output path.")
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    simulator = RiskAwareMissionSimulator(scenario)
    results = run_trials(simulator, args.trials)
    summary = simulator.run_benchmark(args.trials)
    safe_rate, safe_ci95 = safe_rate_with_ci(results)
    summary["safe_rate"] = safe_rate
    summary["safe_rate_ci95"] = safe_ci95
    summary["failure_rate"] = 1.0 - safe_rate
    summary["failure_rate_ci95"] = safe_ci95
    latest_trial = simulator.run_trial(args.trials + 1)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(summary.keys()))
        writer.writeheader()
        writer.writerow(summary)

    timeline_path = Path(args.timeline)
    timeline_path.parent.mkdir(parents=True, exist_ok=True)
    timeline_path.write_text(json.dumps(latest_trial.timeline, indent=2), encoding="utf-8")

    print(json.dumps(summary, indent=2))
    print(f"Saved summary to {output_path}")
    print(f"Saved latest timeline to {timeline_path}")


if __name__ == "__main__":
    main()
