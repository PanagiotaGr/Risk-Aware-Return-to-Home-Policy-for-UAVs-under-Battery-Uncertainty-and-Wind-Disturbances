"""State containers for risk-aware UAV return-to-home simulation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

Vector2 = NDArray[np.float64]


@dataclass(frozen=True)
class UAVState:
    """Minimal 2D UAV state used by the research simulator."""

    position: Vector2
    velocity: Vector2
    soc: float
    time_s: float = 0.0


@dataclass(frozen=True)
class MissionState:
    """Mission geometry and terminal-state bookkeeping."""

    home: Vector2
    target: Vector2
    returning_home: bool = False
    landed: bool = False
    failed: bool = False
    rth_trigger_time_s: float | None = None


@dataclass(frozen=True)
class SimulationResult:
    """Trajectory-level result produced by the simulator."""

    policy_name: str
    success: bool
    unsafe_failure: bool
    early_return: bool
    remaining_soc: float
    distance_completed_m: float
    rth_trigger_time_s: float | None
    history: list[dict[str, float]]
