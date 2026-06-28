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

The script evaluates several scenario families:

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

## Usage

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

## Suggested Paper Table

Use `results/summary_by_policy.csv` to construct the main comparison table:

| Policy | Safe return rate | Failure rate | Early RTH rate | Mean battery left |
|---|---:|---:|---:|---:|
| Deterministic threshold | from CSV | from CSV | from CSV | from CSV |
| Risk-aware Monte Carlo | from CSV | from CSV | from CSV | from CSV |
| Adaptive risk-aware Monte Carlo | from CSV | from CSV | from CSV | from CSV |

## Suggested Ablations

For a publication-level evaluation, repeat the experiments while varying:

- Monte Carlo sample count: 50, 100, 250, 500, 1000
- risk threshold: 0.75, 0.80, 0.85, 0.90, 0.95
- SoC noise standard deviation: 0.02, 0.05, 0.10, 0.15
- wind speed and direction
- extra-drain fault probability

These ablations can show whether the proposed policy is robust, statistically stable, and superior to deterministic triggering.
