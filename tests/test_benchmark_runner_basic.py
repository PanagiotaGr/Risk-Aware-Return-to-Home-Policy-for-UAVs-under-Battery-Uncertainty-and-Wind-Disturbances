from risk_rth_platform.benchmark import run_trials
from risk_rth_platform.config import load_scenario
from risk_rth_platform.simulator import RiskAwareMissionSimulator


def test_benchmark_runner_returns_requested_trial_count():
    scenario = load_scenario("configs/platform/default_scenario.json")
    simulator = RiskAwareMissionSimulator(scenario)
    results = run_trials(simulator, 3)
    assert len(results) == 3
