# Experimental Evaluation Protocol

This folder contains reproducible experiments for evaluating return-to-home policies for UAVs under battery uncertainty and wind disturbances.

## Policies

The experiment runner compares three policies:

1. **Deterministic threshold**
   - Triggers return-to-home when the estimated state of charge is below a fixed threshold.
   - This is the main baseline.

2. **Risk-aware Monte Carlo**
   - Estimates the probability of safe return by sampling battery state, wind disturbance, and return energy demand.
   - Triggers return-to-home when the estimated probability of safe return is below a fixed risk threshold.

3. **Adaptive risk-aware Monte Carlo**
   - Uses the same Monte Carlo estimator.
   - Adjusts the risk threshold according to wind intensity, battery uncertainty, and distance from home.

## Scenario Families

The scripts evaluate several scenario families:

- wind stress tests: tailwind, crosswind, and headwind at multiple wind speeds
- battery uncertainty tests: increasing SoC noise
- biased SoC estimation tests: optimistic and pessimistic battery estimates
- fault injection: sudden extra energy drain during mission execution

## Metrics

The following metrics are computed:

- RTH trigger rate
- safe return rate
- failure rate
- early RTH rate
- mean battery remaining after decision outcome
- standard deviation of battery remaining
- 95% confidence intervals for ablation studies
- runtime per trial for Monte Carlo sample-count analysis

## Main Experiment

From the repository root:

```bash
python3 experiments/run_monte_carlo_experiments.py --trials 200 --mc-samples 500
```

Outputs are written to `results/`:

```text
results/experiment_trials.csv
results/summary_by_policy.csv
results/summary_by_scenario.csv
results/safe_return_rate.png
results/failure_rate.png
results/early_rth_rate.png
results/battery_left.png
```

## Ablation Studies

Run:

```bash
python3 experiments/run_ablation_studies.py --trials 200
```

Outputs are written to `results_ablation/`:

```text
results_ablation/deterministic_threshold_sweep.csv
results_ablation/risk_threshold_tau_sweep.csv
results_ablation/mc_sample_sweep.csv
results_ablation/deterministic_failure_vs_threshold.png
results_ablation/deterministic_safe_return_vs_threshold.png
results_ablation/risk_failure_vs_tau.png
results_ablation/risk_safe_return_vs_tau.png
results_ablation/failure_vs_mc_samples.png
results_ablation/runtime_vs_mc_samples.png
```

## Suggested Paper Table

Use `results/summary_by_policy.csv` to construct the main comparison table:

| Policy | Safe return rate | Failure rate | Early RTH rate | Mean battery left |
|---|---:|---:|---:|---:|
| Deterministic threshold | from CSV | from CSV | from CSV | from CSV |
| Risk-aware Monte Carlo | from CSV | from CSV | from CSV | from CSV |
| Adaptive risk-aware Monte Carlo | from CSV | from CSV | from CSV | from CSV |

## Publication-Level Ablations

The ablation runner tests:

- deterministic SoC threshold: 0.25, 0.30, 0.35, 0.40, 0.45, 0.50
- risk threshold tau: 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98
- Monte Carlo sample count: 50, 100, 250, 500, 1000

These ablations help answer whether the proposed policy is robust, statistically stable, computationally feasible, and superior to deterministic triggering.
