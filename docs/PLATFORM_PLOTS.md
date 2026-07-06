# Platform Benchmark Plots

## Purpose

The platform benchmark now includes a lightweight plotting workflow for turning per-trial CSV outputs into paper-style diagnostic figures.

## Run

First generate benchmark outputs:

```bash
python3 scripts/run_research_platform.py --trials 200
```

Then generate figures:

```bash
python3 scripts/plot_platform_trials.py \
  --trials results_platform/platform_trials.csv \
  --output-dir results_platform/figures
```

## Generated Figures

| Figure | Meaning |
|---|---|
| `safe_return_probability.png` | Safe-return probability across trials. |
| `energy_margin.png` | Final energy margin across trials. |
| `decision_distribution.png` | Distribution of selected safety decisions. |

## Research Value

These plots help inspect policy behaviour beyond aggregate averages. They support failure-mode analysis, conservativeness assessment and quick visual checks before preparing publication-quality figures.
