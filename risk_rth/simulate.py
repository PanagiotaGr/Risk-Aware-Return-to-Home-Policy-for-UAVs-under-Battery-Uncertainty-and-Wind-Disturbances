from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin
from .models import EnergyModel, UAVState, Uncertainty, estimate_safe_return_probability, should_trigger_rth


@dataclass(frozen=True)
class StepResult:
    step: int
    x_m: float
    y_m: float
    soc_wh: float
    probability_safe_return: float
    mode: str


def run_mission(
    *,
    threshold: float = 0.70,
    initial_soc_wh: float = 80.0,
    outbound_speed_mps: float = 12.0,
    step_s: float = 5.0,
    max_steps: int = 240,
    uncertainty: Uncertainty | None = None,
    samples: int = 800,
    seed: int = 7,
) -> list[StepResult]:
    model = EnergyModel()
    uncertainty = uncertainty or Uncertainty(wind_mean_mps=3.0, wind_sigma_mps=1.2)
    state = UAVState(x_m=0.0, y_m=0.0, soc_wh=initial_soc_wh, speed_mps=outbound_speed_mps)
    results: list[StepResult] = []
    mode = "MISSION"

    for step in range(max_steps):
        p_safe = estimate_safe_return_probability(state, uncertainty, model, samples=samples, seed=seed + step)
        if mode == "MISSION" and should_trigger_rth(p_safe, threshold):
            mode = "RTH"
        results.append(StepResult(step, state.x_m, state.y_m, state.soc_wh, p_safe, mode))

        if mode == "MISSION":
            # Simple outbound diagonal survey leg.
            state = UAVState(
                x_m=state.x_m + outbound_speed_mps * step_s,
                y_m=state.y_m + 0.35 * outbound_speed_mps * step_s,
                soc_wh=max(0.0, state.soc_wh - model.base_power_w * step_s / 3600.0),
                speed_mps=outbound_speed_mps,
                mode=mode,
            )
        else:
            distance = state.distance_home_m
            if distance < 1.0:
                results.append(StepResult(step + 1, 0.0, 0.0, state.soc_wh, 1.0, "LANDED"))
                break
            heading_home = atan2(-state.y_m, -state.x_m)
            travel = min(distance, outbound_speed_mps * step_s)
            state = UAVState(
                x_m=state.x_m + travel * cos(heading_home),
                y_m=state.y_m + travel * sin(heading_home),
                soc_wh=max(0.0, state.soc_wh - model.base_power_w * step_s / 3600.0),
                speed_mps=outbound_speed_mps,
                mode=mode,
            )
            if state.soc_wh <= 0.0:
                results.append(StepResult(step + 1, state.x_m, state.y_m, state.soc_wh, 0.0, "FAILED"))
                break
    return results
