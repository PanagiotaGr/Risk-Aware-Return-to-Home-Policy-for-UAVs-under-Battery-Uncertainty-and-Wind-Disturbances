"""Risk-aware UAV mission decision platform.

This package provides a lightweight, dependency-free simulation layer that can
be used for benchmarking Return-to-Home, diversion, and emergency-landing
policies under battery and wind uncertainty.
"""

from .models import Decision, MissionScenario, SimulationResult, UAVState
from .simulator import RiskAwareMissionSimulator

__all__ = [
    "Decision",
    "MissionScenario",
    "SimulationResult",
    "UAVState",
    "RiskAwareMissionSimulator",
]
