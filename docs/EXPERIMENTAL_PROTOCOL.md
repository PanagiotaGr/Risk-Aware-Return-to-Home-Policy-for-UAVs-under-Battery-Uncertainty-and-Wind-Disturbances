# Experimental Protocol

## Objective

This protocol defines how the risk-aware UAV Return-to-Home policies should be evaluated in a research-grade, statistically meaningful way.

The goal is not only to show that one policy performs better than another in one scenario. The goal is to measure how safety, mission completion and conservativeness change as uncertainty increases.

## Policies Under Evaluation

| Policy | Description |
|---|---|
| `deterministic_threshold` | Fixed battery threshold baseline. |
| `risk_aware_mc` | Monte Carlo safe-return probability estimator. |
| `adaptive_risk_mc` | Risk-aware policy with adaptive thresholding. |
| `health_aware_risk_mc` | Risk-aware policy that accounts for battery degradation. |

Future policies may include:

- chance-constrained RTH,
- belief-space RTH,
- safe reinforcement learning threshold adaptation,
- sequential safety supervisor,
- alternate landing-site policy.

## Independent Variables

### Battery Variables

| Variable | Description |
|---|---|
| Initial SoC | Starting battery state of charge. |
| SoC noise | Estimation uncertainty. |
| Battery health | Usable capacity degradation. |
| Discharge model | Nominal or perturbed energy consumption. |

### Wind Variables

| Variable | Description |
|---|---|
| Wind speed | Mean wind disturbance. |
| Gust amplitude | Time-varying wind perturbation. |
| Gust frequency | Temporal variation. |
| Wind direction | Tailwind, headwind or crosswind effect. |

### Mission Variables

| Variable | Description |
|---|---|
| Return distance | Distance from current position to home. |
| Mission geometry | Waypoint configuration and direction. |
| Mission progress | Fraction of task completed before decision. |
| Alternate landing availability | Whether emergency landing sites exist. |

### Policy Variables

| Variable | Description |
|---|---|
| Risk threshold `τ` | Minimum acceptable safe-return probability. |
| Monte Carlo samples | Number of sampled return outcomes. |
| Adaptive threshold parameters | Conservativeness tuning. |
| Safety margin | Additional energy buffer. |

## Dependent Metrics

| Metric | Meaning |
|---|---|
| Safe-return rate | Fraction of trials where RTH is triggered and return is feasible. |
| Failure rate | Fraction of trials where energy is insufficient for safe return. |
| RTH trigger rate | Fraction of trials where policy decides to return. |
| Mission completion rate | Fraction of mission completed before return/failure. |
| Mean energy margin | Remaining energy minus required return energy. |
| Negative-margin rate | Fraction of trials with insufficient return energy margin. |
| Runtime per decision | Computational cost of policy evaluation. |
| Confidence intervals | Statistical uncertainty of estimated metrics. |

## Minimum Experimental Suites

### 1. Baseline Policy Comparison

Purpose:

Compare deterministic and probabilistic policies under the same mission distribution.

Command:

```bash
python3 experiments/run_monte_carlo_experiments.py --trials 200 --mc-samples 500
```

Expected analysis:

- compare safe-return rate,
- compare failure rate,
- compare mission completion,
- interpret safety-efficiency trade-off.

### 2. Battery Aging Study

Purpose:

Measure the effect of degraded usable capacity.

Command:

```bash
python3 experiments/run_battery_aging_study.py --trials 200
```

Required health levels:

```text
100%, 95%, 90%, 80%, 70%
```

Expected analysis:

- identify when degradation becomes safety-critical,
- compare health-aware and non-health-aware policies,
- report failure trends.

### 3. Dynamic Wind Gust Study

Purpose:

Evaluate robustness under time-varying disturbances.

Command:

```bash
python3 experiments/run_wind_gust_study.py --trials 200
```

Required gust scenarios:

```text
gust_low
gust_medium
gust_high
gust_extreme
```

Expected analysis:

- compare safe-return behavior under increasing wind intensity,
- identify scenarios where deterministic thresholds fail,
- quantify policy conservativeness.

### 4. Monte Carlo Ablation Study

Purpose:

Measure the trade-off between computational cost and decision quality.

Command:

```bash
python3 experiments/run_ablation_studies.py --trials 200
```

Required sample counts:

```text
50, 100, 250, 500, 1000
```

Expected analysis:

- runtime scaling,
- stability of safe-return estimates,
- minimum useful number of samples.

## Statistical Reporting

Each experiment should report:

1. Number of trials.
2. Random seed or seed-generation method.
3. Mean metric values.
4. 95% confidence intervals where applicable.
5. Full CSV output.
6. Plots for comparison.
7. Interpretation of safety-efficiency trade-offs.

## Recommended Stress Tests

To reach PhD-level evaluation, the framework should include combined stress tests:

| Stress Test | Description |
|---|---|
| Low battery + high wind | Severe return-energy demand. |
| Degraded battery + noisy SoC | Incorrect energy availability estimate. |
| Long distance + gusts | Late return decision risk. |
| Model mismatch | Energy model underestimates true demand. |
| Sensor bias | Systematic SoC or wind-estimation error. |
| Delayed decision | RTH triggered after safety boundary is crossed. |

## Reproducibility Requirements

For every result table:

- store the script name,
- store command-line arguments,
- store random seed,
- store output CSV,
- store generated plots,
- store code version or commit hash,
- avoid manual editing of numeric results.

## Interpretation Guidelines

A policy should not be considered better only because it has a higher mission completion rate. In safety-critical autonomy, a policy must be evaluated across multiple objectives:

```text
high safe-return rate
low failure rate
reasonable mission completion
positive energy margin
acceptable runtime
stable behavior under uncertainty
```

The key scientific result is the shape of the trade-off between safety and mission efficiency.

## Future Experimental Extensions

1. Hardware-in-the-loop evaluation.
2. PX4/Gazebo digital-twin experiments.
3. Real flight-log replay.
4. Multi-UAV mission supervision.
5. Formal verification of risk bounds.
6. Comparison with chance-constrained MPC.
7. Comparison with safe reinforcement learning baselines.
