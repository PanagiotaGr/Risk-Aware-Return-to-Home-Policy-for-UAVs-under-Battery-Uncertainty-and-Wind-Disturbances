from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Tuple


class Decision(str, Enum):
    """High-level mission decision actions supported by the platform."""

    CONTINUE = "continue_mission"
    RETURN_HOME = "return_home"
    EMERGENCY_LAND = "emergency_land"
    DIVERT = "divert_to_safe_site"
    REDUCE_SPEED = "reduce_speed"


@dataclass(frozen=True)
class UAVState:
    """Current UAV state used by the decision layer."""

    battery_soc: float
    battery_health: float
    distance_to_home_m: float
    mission_progress: float
    wind_speed_mps: float
    payload_kg: float = 0.0


@dataclass(frozen=True)
class MissionScenario:
    """Scenario parameters for reproducible risk-aware mission evaluation."""

    name: str
    initial_soc: float
    battery_health: float
    distance_to_home_m: float
    wind_speed_mps: float
    wind_gust_std_mps: float
    payload_kg: float
    mission_drain_per_step: float
    return_energy_per_meter: float
    landing_energy_reserve: float
    risk_threshold: float
    divert_threshold: float
    emergency_threshold: float
    max_steps: int
    monte_carlo_samples: int
    random_seed: int


@dataclass(frozen=True)
class SimulationResult:
    """Single-trial simulation output."""

    scenario: str
    decision: Decision
    safe_return_probability: float
    final_soc: float
    mission_progress: float
    safe: bool
    steps: int
    energy_margin: float
    timeline: List[Dict[str, float | str]]


Point = Tuple[float, float]
