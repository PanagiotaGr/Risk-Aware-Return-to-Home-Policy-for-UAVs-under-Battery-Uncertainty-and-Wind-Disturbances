# Experiments

All experiments are YAML-driven and reproducible through `scripts/run_experiment.py`.

## Included YAML scenarios

- `configs/experiments/nominal.yaml`
- `configs/experiments/headwind.yaml`
- `configs/experiments/gust.yaml`

## Requested scenario family

The framework is ready for additional YAML variants covering low/high battery uncertainty, constant tailwind, crosswind, high wind variance, long-distance mission, aggressive `tau`, and conservative `tau`.

## Example

```bash
python scripts/run_experiment.py --config configs/experiments/nominal.yaml --output-dir results/controlled
```

Outputs are generated under `results/controlled/<experiment-name>/`.

## Reporting rule

Do not copy metrics into the README unless the exact command, configuration, seed, and output file are cited.
