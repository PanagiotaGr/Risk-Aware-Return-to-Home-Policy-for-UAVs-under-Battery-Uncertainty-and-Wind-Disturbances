# Reproducibility Protocol

This repository is designed so that scientific claims can be traced back to code, configuration files, random seeds, and generated artifacts.

## Core principle

Do not report a number unless it can be regenerated from:

1. a committed YAML configuration,
2. a committed script,
3. a deterministic seed,
4. a generated output file under `results/`.

## Environment

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .[dev]
```

## Unit tests

```bash
pytest -q
```

## Single experiment

```bash
python scripts/run_experiment.py \
  --config configs/experiments/nominal.yaml \
  --output-dir results/controlled
```

This writes:

```text
results/controlled/<experiment-name>/metrics.csv
results/controlled/<experiment-name>/figures/*.png
results/controlled/<experiment-name>/figures/*.pdf
```

## Full controlled suite

```bash
python scripts/run_all_experiments.py --output-dir results/controlled
```

This runs the committed scenario set and creates an aggregate comparison figure from generated CSV files.

## Demo animation

```bash
python scripts/make_demo_gif.py
```

This writes:

```text
assets/demo.gif
results/videos/demo.mp4    # only when ffmpeg is available
```

The animation is a visual demonstration of the simulation and policy state; it is not experimental evidence by itself.

## Randomness

Experiment scripts use explicit seeds from YAML. Multi-trial experiments increment the base seed by trial index. This makes controlled simulation outputs reproducible while still sampling uncertainty.

## Reporting rules

- Report the YAML file, command, seed, and generated CSV path.
- Keep generated metrics separate from claims in prose.
- Do not compare methods unless all methods were run under comparable scenarios.
- Do not present simulation-only findings as hardware safety validation.

## Limitations

The current framework uses simplified simulation models. Reproducibility here means computational reproducibility, not real-world validation. Flight-level claims require calibrated platform models, logs, and hardware testing.
