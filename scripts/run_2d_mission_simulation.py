#!/usr/bin/env python3
"""Run the 2D mission simulator and export telemetry as JSON."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from risk_rth_platform.simulation.engine import load_engine_from_file


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a 2D risk-aware UAV mission simulation.")
    parser.add_argument("--config", default="configs/platform/world_urban_mission.json")
    parser.add_argument("--output", default="results_platform/mission_2d_telemetry.json")
    args = parser.parse_args()

    engine = load_engine_from_file(args.config)
    result = engine.run()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(
            {
                "scenario": result.scenario_name,
                "final_decision": result.final_decision.value,
                "completed": result.completed,
                "safe": result.safe,
                "telemetry": result.telemetry,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"Scenario: {result.scenario_name}")
    print(f"Final decision: {result.final_decision.value}")
    print(f"Completed: {result.completed}")
    print(f"Safe: {result.safe}")
    print(f"Telemetry samples: {len(result.telemetry)}")
    print(f"Saved telemetry to {output_path}")


if __name__ == "__main__":
    main()
