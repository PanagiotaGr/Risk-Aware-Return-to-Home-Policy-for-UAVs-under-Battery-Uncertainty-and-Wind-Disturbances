from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

from risk_rth_platform.models import Decision, UAVState
from risk_rth_platform.simulator import RiskAwareMissionSimulator

from .mission import MissionPlan
from .uav import UAVKinematics, UAVTelemetry
from .wind import WindField
from .world import SimulationWorld


@dataclass(frozen=True)
class MissionSimulationOutput:
    scenario_name: str
    final_decision: Decision
    completed: bool
    safe: bool
    telemetry: List[Dict[str, float | str | bool]]


class MissionSimulationEngine:
    """2D world simulator connected to the risk-aware decision layer."""

    def __init__(
        self,
        world: SimulationWorld,
        mission: MissionPlan,
        wind: WindField,
        uav: UAVKinematics,
        risk_model: RiskAwareMissionSimulator,
        dt_s: float = 1.0,
        max_steps: int = 500,
    ):
        self.world = world
        self.mission = mission
        self.wind = wind
        self.uav = uav
        self.risk_model = risk_model
        self.dt_s = dt_s
        self.max_steps = max_steps

    def run(self) -> MissionSimulationOutput:
        waypoint_index = 0
        telemetry: List[Dict[str, float | str | bool]] = []
        final_decision = Decision.CONTINUE
        safe = True

        for step in range(1, self.max_steps + 1):
            target = self._target_for_decision(final_decision, waypoint_index)
            wind_vector = self.wind.sample(self.uav.position, step)
            self.uav, energy_used = self.uav.step_towards(
                target=target,
                wind=wind_vector,
                dt_s=self.dt_s,
                world_width_m=self.world.width_m,
                world_height_m=self.world.height_m,
            )
            waypoint_index = self.mission.update_waypoint_index(self.uav.position, waypoint_index)
            progress = self.mission.progress(waypoint_index)
            distance_home = self.uav.position.distance_to(self.world.home)
            inside_zone = self.world.violates_no_fly_zone(self.uav.position)

            state = UAVState(
                battery_soc=self.uav.battery_soc,
                battery_health=self.uav.battery_health,
                distance_to_home_m=distance_home,
                mission_progress=progress,
                wind_speed_mps=wind_vector.speed_mps,
                payload_kg=self.uav.payload_kg,
            )
            probability = self.risk_model.estimate_safe_return_probability(state)
            final_decision = self.risk_model.choose_decision(state, probability)

            row = UAVTelemetry(
                step=step,
                position=self.uav.position,
                target=target,
                battery_soc=self.uav.battery_soc,
                battery_health=self.uav.battery_health,
                mission_progress=progress,
                distance_to_home_m=distance_home,
                wind_speed_mps=wind_vector.speed_mps,
                energy_used=energy_used,
                inside_no_fly_zone=inside_zone,
            )
            telemetry.append(
                {
                    "step": row.step,
                    "x": round(row.position.x, 3),
                    "y": round(row.position.y, 3),
                    "target_x": round(row.target.x, 3),
                    "target_y": round(row.target.y, 3),
                    "battery_soc": round(row.battery_soc, 4),
                    "battery_health": round(row.battery_health, 4),
                    "mission_progress": round(row.mission_progress, 4),
                    "distance_to_home_m": round(row.distance_to_home_m, 3),
                    "wind_speed_mps": round(row.wind_speed_mps, 3),
                    "safe_return_probability": round(probability, 4),
                    "decision": final_decision.value,
                    "energy_used": round(row.energy_used, 5),
                    "inside_no_fly_zone": row.inside_no_fly_zone,
                }
            )

            if inside_zone or self.uav.battery_soc <= 0.02:
                safe = False
                break
            if final_decision is Decision.EMERGENCY_LAND:
                safe = self._near_safe_landing_site()
                break
            if final_decision is Decision.RETURN_HOME and distance_home <= self.mission.acceptance_radius_m:
                break
            if progress >= 1.0:
                break

        completed = telemetry[-1]["mission_progress"] >= 1.0 if telemetry else False
        return MissionSimulationOutput(
            scenario_name=self.risk_model.scenario.name,
            final_decision=final_decision,
            completed=completed,
            safe=safe,
            telemetry=telemetry,
        )

    def _target_for_decision(self, decision: Decision, waypoint_index: int):
        if decision is Decision.RETURN_HOME:
            return self.world.home
        if decision is Decision.DIVERT or decision is Decision.EMERGENCY_LAND:
            site = self.world.nearest_landing_site(self.uav.position)
            return site.position if site else self.world.home
        return self.mission.current_target(waypoint_index)

    def _near_safe_landing_site(self) -> bool:
        site = self.world.nearest_landing_site(self.uav.position)
        return bool(site and site.contains(self.uav.position) and site.risk_score <= 0.5)


def load_engine_from_file(path: str | Path) -> MissionSimulationEngine:
    from risk_rth_platform.config import load_scenario

    data = json.loads(Path(path).read_text(encoding="utf-8"))
    world = SimulationWorld.from_dict(data["world"])
    mission = MissionPlan.from_dict(data["mission"])
    wind = WindField.from_dict(data["wind"])
    uav_data = data["uav"]
    uav = UAVKinematics(
        position=world.home,
        cruise_speed_mps=float(uav_data.get("cruise_speed_mps", 15.0)),
        battery_soc=float(uav_data.get("battery_soc", 0.9)),
        battery_health=float(uav_data.get("battery_health", 0.9)),
        payload_kg=float(uav_data.get("payload_kg", 0.0)),
    )
    risk_model = RiskAwareMissionSimulator(load_scenario(data["risk_scenario_path"]))
    return MissionSimulationEngine(
        world=world,
        mission=mission,
        wind=wind,
        uav=uav,
        risk_model=risk_model,
        dt_s=float(data.get("dt_s", 1.0)),
        max_steps=int(data.get("max_steps", 500)),
    )
