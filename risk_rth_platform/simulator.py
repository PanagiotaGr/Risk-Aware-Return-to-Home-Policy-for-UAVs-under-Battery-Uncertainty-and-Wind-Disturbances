from __future__ import annotations

import random
from dataclasses import replace
from statistics import mean
from typing import Dict, List

from .models import Decision, MissionScenario, SimulationResult, UAVState


class RiskAwareMissionSimulator:
    """Monte Carlo simulator for risk-aware UAV mission decisions.

    The model is intentionally lightweight and dependency-free. It is designed
    as a platform layer: future work can replace the energy, battery, wind, or
    decision model without changing the benchmark interface.
    """

    def __init__(self, scenario: MissionScenario):
        self.scenario = scenario
        self.rng = random.Random(scenario.random_seed)

    def estimate_safe_return_probability(self, state: UAVState) -> float:
        feasible_samples = 0
        samples = max(1, self.scenario.monte_carlo_samples)

        for _ in range(samples):
            sampled_soc = self._bounded_normal(state.battery_soc, 0.025, 0.0, 1.0)
            sampled_health = self._bounded_normal(state.battery_health, 0.015, 0.4, 1.0)
            sampled_wind = max(0.0, self.rng.gauss(state.wind_speed_mps, self.scenario.wind_gust_std_mps))
            sampled_distance = max(0.0, self.rng.gauss(state.distance_to_home_m, 0.04 * state.distance_to_home_m))
            required_energy = self._required_return_energy(sampled_distance, sampled_wind, sampled_health, state.payload_kg)

            if sampled_soc - required_energy >= self.scenario.landing_energy_reserve:
                feasible_samples += 1

        return feasible_samples / samples

    def choose_decision(self, state: UAVState, safe_return_probability: float) -> Decision:
        if state.battery_soc <= self.scenario.emergency_threshold or safe_return_probability < 0.05:
            return Decision.EMERGENCY_LAND
        if safe_return_probability < self.scenario.divert_threshold:
            return Decision.DIVERT
        if safe_return_probability < self.scenario.risk_threshold:
            return Decision.RETURN_HOME
        if state.wind_speed_mps > 12.0 or state.battery_health < 0.75:
            return Decision.REDUCE_SPEED
        return Decision.CONTINUE

    def run_trial(self, trial_seed_offset: int = 0) -> SimulationResult:
        if trial_seed_offset:
            self.rng.seed(self.scenario.random_seed + trial_seed_offset)

        state = UAVState(
            battery_soc=self.scenario.initial_soc,
            battery_health=self.scenario.battery_health,
            distance_to_home_m=self.scenario.distance_to_home_m,
            mission_progress=0.0,
            wind_speed_mps=self.scenario.wind_speed_mps,
            payload_kg=self.scenario.payload_kg,
        )
        timeline: List[Dict[str, float | str]] = []
        decision = Decision.CONTINUE
        probability = 1.0
        energy_margin = 0.0

        for step in range(1, self.scenario.max_steps + 1):
            wind = max(0.0, self.rng.gauss(self.scenario.wind_speed_mps, self.scenario.wind_gust_std_mps))
            state = replace(
                state,
                wind_speed_mps=wind,
                battery_soc=max(0.0, state.battery_soc - self._mission_step_energy(wind, state.payload_kg)),
                mission_progress=min(1.0, state.mission_progress + 1.0 / self.scenario.max_steps),
                distance_to_home_m=max(0.0, state.distance_to_home_m * 0.985),
            )

            probability = self.estimate_safe_return_probability(state)
            decision = self.choose_decision(state, probability)
            required_energy = self._required_return_energy(
                state.distance_to_home_m,
                state.wind_speed_mps,
                state.battery_health,
                state.payload_kg,
            )
            energy_margin = state.battery_soc - required_energy - self.scenario.landing_energy_reserve

            timeline.append(
                {
                    "step": step,
                    "battery_soc": round(state.battery_soc, 4),
                    "wind_speed_mps": round(state.wind_speed_mps, 3),
                    "safe_return_probability": round(probability, 4),
                    "mission_progress": round(state.mission_progress, 4),
                    "decision": decision.value,
                    "energy_margin": round(energy_margin, 4),
                }
            )

            if decision is not Decision.CONTINUE and decision is not Decision.REDUCE_SPEED:
                break

        safe = decision in {Decision.RETURN_HOME, Decision.DIVERT} and energy_margin >= 0.0
        if decision is Decision.CONTINUE:
            safe = probability >= self.scenario.risk_threshold and state.battery_soc > self.scenario.landing_energy_reserve

        return SimulationResult(
            scenario=self.scenario.name,
            decision=decision,
            safe_return_probability=probability,
            final_soc=state.battery_soc,
            mission_progress=state.mission_progress,
            safe=safe,
            steps=len(timeline),
            energy_margin=energy_margin,
            timeline=timeline,
        )

    def run_benchmark(self, trials: int) -> Dict[str, float | str]:
        results = [self.run_trial(i) for i in range(trials)]
        decisions = {decision.value: 0 for decision in Decision}
        for result in results:
            decisions[result.decision.value] += 1

        return {
            "scenario": self.scenario.name,
            "trials": trials,
            "safe_rate": mean(1.0 if r.safe else 0.0 for r in results),
            "mean_safe_return_probability": mean(r.safe_return_probability for r in results),
            "mean_final_soc": mean(r.final_soc for r in results),
            "mean_mission_progress": mean(r.mission_progress for r in results),
            "mean_energy_margin": mean(r.energy_margin for r in results),
            **{f"decision_{name}_rate": count / trials for name, count in decisions.items()},
        }

    def _mission_step_energy(self, wind_speed_mps: float, payload_kg: float) -> float:
        wind_penalty = 1.0 + 0.035 * wind_speed_mps
        payload_penalty = 1.0 + 0.08 * payload_kg
        return self.scenario.mission_drain_per_step * wind_penalty * payload_penalty

    def _required_return_energy(self, distance_m: float, wind_speed_mps: float, battery_health: float, payload_kg: float) -> float:
        wind_penalty = 1.0 + 0.04 * wind_speed_mps
        payload_penalty = 1.0 + 0.08 * payload_kg
        health_penalty = 1.0 / max(0.4, battery_health)
        return distance_m * self.scenario.return_energy_per_meter * wind_penalty * payload_penalty * health_penalty

    def _bounded_normal(self, value: float, std: float, lower: float, upper: float) -> float:
        return min(upper, max(lower, self.rng.gauss(value, std)))
