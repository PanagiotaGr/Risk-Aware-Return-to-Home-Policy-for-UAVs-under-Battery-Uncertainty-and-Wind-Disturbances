"""2D UAV mission simulator for controlled RTH experiments."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from risk_rth.models.battery import BatteryModel
from risk_rth.models.energy import EnergyModel
from risk_rth.models.state import SimulationResult
from risk_rth.models.wind import WindModel
from risk_rth.planning.policies import RTHPolicy


@dataclass(frozen=True)
class SimulatorConfig:
    dt_s: float = 1.0
    cruise_speed_mps: float = 8.0
    max_time_s: float = 300.0
    initial_soc: float = 1.0
    home_xy_m: tuple[float, float] = (0.0, 0.0)
    target_xy_m: tuple[float, float] = (600.0, 0.0)
    landing_radius_m: float = 5.0


class MissionSimulator2D:
    """Deterministic integration loop with stochastic policy internals."""

    def __init__(
        self,
        config: SimulatorConfig,
        battery: BatteryModel,
        wind: WindModel,
        energy: EnergyModel,
        seed: int = 7,
    ) -> None:
        self.config = config
        self.battery = battery
        self.wind = wind
        self.energy = energy
        self.rng = np.random.default_rng(seed)

    def run(self, policy: RTHPolicy) -> SimulationResult:
        home = np.asarray(self.config.home_xy_m, dtype=float)
        target = np.asarray(self.config.target_xy_m, dtype=float)
        position = home.copy()
        soc = self.config.initial_soc
        returning_home = False
        rth_trigger_time_s: float | None = None
        history: list[dict[str, float]] = []
        distance_completed = 0.0

        for step in range(int(self.config.max_time_s / self.config.dt_s)):
            time_s = step * self.config.dt_s
            goal = home if returning_home else target
            delta = goal - position
            distance_to_goal = float(np.linalg.norm(delta))
            if distance_to_goal <= self.config.landing_radius_m:
                if returning_home or np.allclose(goal, home):
                    return SimulationResult(
                        policy_name=policy.name,
                        success=True,
                        unsafe_failure=False,
                        early_return=returning_home,
                        remaining_soc=soc,
                        distance_completed_m=distance_completed,
                        rth_trigger_time_s=rth_trigger_time_s,
                        history=history,
                    )
                returning_home = True
                rth_trigger_time_s = time_s
                goal = home
                delta = goal - position
                distance_to_goal = float(np.linalg.norm(delta))

            if not returning_home:
                trigger, estimate = policy.should_return(
                    position, home, soc, self.battery, self.wind, self.energy, self.rng, time_s
                )
                if trigger:
                    returning_home = True
                    rth_trigger_time_s = time_s
                    goal = home
                    delta = goal - position
                    distance_to_goal = float(np.linalg.norm(delta))
            else:
                estimate = None

            direction = delta / max(distance_to_goal, 1e-9)
            velocity = direction * self.config.cruise_speed_mps
            wind_xy = self.wind.deterministic(time_s)
            energy_used = self.energy.step_energy_wh(velocity, wind_xy, self.config.dt_s)
            soc = self.battery.drain_soc(soc, energy_used)
            if soc <= 0.0:
                return SimulationResult(
                    policy.name,
                    False,
                    True,
                    returning_home,
                    0.0,
                    distance_completed,
                    rth_trigger_time_s,
                    history,
                )

            previous = position.copy()
            position = position + velocity * self.config.dt_s
            if not returning_home:
                distance_completed += float(np.linalg.norm(position - previous))

            history.append(
                {
                    "time_s": time_s,
                    "x_m": float(position[0]),
                    "y_m": float(position[1]),
                    "soc": float(soc),
                    "wind_x_mps": float(wind_xy[0]),
                    "wind_y_mps": float(wind_xy[1]),
                    "p_safe": float(estimate.p_safe) if estimate is not None else float("nan"),
                    "returning_home": float(returning_home),
                }
            )

        return SimulationResult(
            policy.name,
            False,
            True,
            returning_home,
            soc,
            distance_completed,
            rth_trigger_time_s,
            history,
        )
