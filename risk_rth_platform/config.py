from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from .models import MissionScenario


def load_scenario(path: str | Path) -> MissionScenario:
    """Load a scenario from a JSON configuration file."""

    data: Dict[str, Any] = json.loads(Path(path).read_text(encoding="utf-8"))
    return MissionScenario(**data)


def save_scenario_template(path: str | Path) -> None:
    """Write a default scenario template that can be edited by users."""

    scenario = MissionScenario(
        name="template_scenario",
        initial_soc=0.85,
        battery_health=0.92,
        distance_to_home_m=1500.0,
        wind_speed_mps=6.5,
        wind_gust_std_mps=2.0,
        payload_kg=0.4,
        mission_drain_per_step=0.006,
        return_energy_per_meter=0.00022,
        landing_energy_reserve=0.08,
        risk_threshold=0.72,
        divert_threshold=0.35,
        emergency_threshold=0.12,
        max_steps=120,
        monte_carlo_samples=300,
        random_seed=7,
    )
    Path(path).write_text(json.dumps(scenario.__dict__, indent=2), encoding="utf-8")
