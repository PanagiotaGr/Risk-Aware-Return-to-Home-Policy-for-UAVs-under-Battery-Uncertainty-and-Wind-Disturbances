"""Sequential multi-action mission-management policy."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum

class MissionAction(str, Enum):
    CONTINUE_MISSION = "CONTINUE_MISSION"
    RETURN_HOME_DIRECT = "RETURN_HOME_DIRECT"
    RETURN_HOME_ENERGY_OPTIMAL = "RETURN_HOME_ENERGY_OPTIMAL"
    REROUTE_AROUND_HEADWIND = "REROUTE_AROUND_HEADWIND"
    HOLD_POSITION = "HOLD_POSITION"
    REDUCE_SPEED = "REDUCE_SPEED"
    CLIMB_FOR_WIND_ADVANTAGE = "CLIMB_FOR_WIND_ADVANTAGE"
    DESCEND_FOR_WIND_AVOIDANCE = "DESCEND_FOR_WIND_AVOIDANCE"
    LAND_AT_ALTERNATE_SITE = "LAND_AT_ALTERNATE_SITE"
    EMERGENCY_LAND = "EMERGENCY_LAND"
    ABORT_MISSION = "ABORT_MISSION"

@dataclass(frozen=True)
class ActionEvaluation:
    action: MissionAction
    mission_progress: float
    expected_energy_wh: float
    probability_safe_completion: float
    probability_safe_return: float
    residual_energy_mean_wh: float
    residual_energy_variance_wh2: float
    wind_exposure: float
    time_cost_s: float
    safety_feasible: bool
    recovery_value: float = 0.0
    score: float = float("-inf")

@dataclass(frozen=True)
class PolicyWeights:
    energy: float = 0.02
    risk: float = 10.0
    uncertainty: float = 0.2
    reserve: float = 0.05
    recoverability: float = 1.0

class SequentialRiskPolicy:
    """Scores feasible actions while enforcing explicit chance constraints."""
    def __init__(self, safe_return_threshold: float = 0.95, minimum_reserve_wh: float = 5.0, weights: PolicyWeights | None = None) -> None:
        self.safe_return_threshold = safe_return_threshold
        self.minimum_reserve_wh = minimum_reserve_wh
        self.weights = weights or PolicyWeights()

    def score(self, candidate: ActionEvaluation) -> ActionEvaluation:
        uncertainty = candidate.residual_energy_variance_wh2**0.5
        failure_risk = 1.0 - min(candidate.probability_safe_completion, candidate.probability_safe_return)
        value = candidate.mission_progress - self.weights.energy * candidate.expected_energy_wh - self.weights.risk * failure_risk - self.weights.uncertainty * uncertainty + self.weights.reserve * candidate.residual_energy_mean_wh + self.weights.recoverability * candidate.recovery_value
        feasible = candidate.safety_feasible and candidate.residual_energy_mean_wh >= self.minimum_reserve_wh
        if candidate.action == MissionAction.CONTINUE_MISSION:
            feasible = feasible and candidate.probability_safe_return >= self.safe_return_threshold
        return ActionEvaluation(**{**candidate.__dict__, "safety_feasible": feasible, "score": value if feasible else float("-inf")})

    def select(self, candidates: list[ActionEvaluation]) -> ActionEvaluation:
        if not candidates:
            raise ValueError("At least one candidate action is required")
        scored = [self.score(c) for c in candidates]
        feasible = [c for c in scored if c.safety_feasible]
        if feasible:
            return max(feasible, key=lambda c: c.score)
        emergency = [c for c in scored if c.action in {MissionAction.EMERGENCY_LAND, MissionAction.ABORT_MISSION}]
        return max(emergency or scored, key=lambda c: c.score)
