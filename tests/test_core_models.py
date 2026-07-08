import numpy as np

from risk_rth.evaluation.metrics import summarize_results
from risk_rth.models.battery import BatteryModel
from risk_rth.models.energy import EnergyModel
from risk_rth.models.state import SimulationResult
from risk_rth.models.wind import WindModel
from risk_rth.planning.policies import RiskAwareMonteCarloPolicy
from risk_rth.simulation.simulator import MissionSimulator2D, SimulatorConfig
from risk_rth.uncertainty.monte_carlo import MonteCarloRiskEstimator


def test_battery_available_energy_respects_reserve():
    battery = BatteryModel(capacity_wh=100.0, reserve_soc=0.1)
    assert battery.available_energy_wh(0.5) == 40.0


def test_wind_sampling_shape():
    wind = WindModel(mean_xy_mps=(1.0, 0.0), std_mps=0.1)
    samples = wind.sample(5, np.random.default_rng(1))
    assert samples.shape == (5, 2)


def test_energy_increases_with_headwind():
    energy = EnergyModel()
    position = np.array([100.0, 0.0])
    home = np.array([0.0, 0.0])
    calm = energy.required_energy_wh(position, home, np.array([0.0, 0.0]))
    headwind = energy.required_energy_wh(position, home, np.array([3.0, 0.0]))
    assert headwind > calm


def test_monte_carlo_probability_bounds():
    estimator = MonteCarloRiskEstimator(n_samples=32)
    estimate = estimator.estimate(
        np.array([100.0, 0.0]),
        np.array([0.0, 0.0]),
        0.8,
        BatteryModel(),
        WindModel(),
        EnergyModel(),
        np.random.default_rng(2),
    )
    assert 0.0 <= estimate.p_safe <= 1.0
    assert estimate.ci_low <= estimate.p_safe <= estimate.ci_high


def test_simulation_loop_returns_result():
    simulator = MissionSimulator2D(
        SimulatorConfig(max_time_s=30.0, target_xy_m=(50.0, 0.0)),
        BatteryModel(),
        WindModel(),
        EnergyModel(),
        seed=3,
    )
    result = simulator.run(RiskAwareMonteCarloPolicy(estimator=MonteCarloRiskEstimator(n_samples=16)))
    assert isinstance(result.history, list)
    assert result.policy_name == "risk_aware_mc"


def test_evaluation_metrics():
    result = SimulationResult("p", True, False, True, 0.4, 10.0, 2.0, [])
    metrics = summarize_results([result])
    assert metrics["mission_success_rate"] == 1.0
    assert metrics["early_return_rate"] == 1.0
