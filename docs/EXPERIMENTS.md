# Experiments

All experiments are YAML-driven and reproducible through `scripts/run_experiment.py`.

## Included YAML scenarios

- `configs/experiments/nominal.yaml`
- `configs/experiments/low_battery_uncertainty.yaml`
- `configs/experiments/high_battery_uncertainty.yaml`
- `configs/experiments/headwind.yaml`
- `configs/experiments/tailwind.yaml`
- `configs/experiments/crosswind.yaml`
- `configs/experiments/gust.yaml`
- `configs/experiments/high_wind_variance.yaml`
- `configs/experiments/long_distance.yaml`
- `configs/experiments/aggressive_tau.yaml`
- `configs/experiments/conservative_tau.yaml`

## Single experiment

```bash
python scripts/run_experiment.py --config configs/experiments/nominal.yaml --output-dir results/controlled
```

Outputs are generated under:

```text
results/controlled/<experiment-name>/metrics.csv
results/controlled/<experiment-name>/figures/
```

## Full controlled suite

```bash
python scripts/run_all_experiments.py --output-dir results/controlled
```

This executes the committed YAML scenario set and creates an aggregate scenario-comparison figure from generated CSV files.

## Scenario intent

| Scenario | Purpose |
|---|---|
| nominal | Basic sanity-check mission. |
| low/high battery uncertainty | Stress the effect of noisy SoC estimates. |
| headwind/tailwind/crosswind | Test direction-dependent wind-energy effects. |
| gust | Test time-varying disturbance behaviour. |
| high wind variance | Test stochastic wind uncertainty. |
| long-distance mission | Stress return-energy feasibility. |
| aggressive/conservative tau | Test risk-threshold sensitivity. |

## Reporting rule

Do not copy metrics into the README unless the exact command, configuration, seed, and output file are cited. These scenarios are controlled simulations, not real-flight benchmarks.
