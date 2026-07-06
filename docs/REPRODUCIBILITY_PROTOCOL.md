# Reproducibility Protocol

## Goal

Every experimental claim in this repository should be reproducible from code, configuration, random seed and saved result files. The repository should be usable both as a research prototype and as a transparent experimental appendix for future publications.

## Minimum Reproducibility Standard

Each experiment must document:

1. command used to run the experiment,
2. number of trials,
3. Monte Carlo sample count,
4. random seed or seed schedule,
5. scenario family,
6. policy set,
7. output directory,
8. software version,
9. hardware notes when runtime is reported,
10. generated CSV and plot files.

## Standard Command Pattern

```bash
python3 experiments/run_monte_carlo_experiments.py \
  --trials 1000 \
  --mc-samples 1000 \
  --seed 42
```

If a script does not yet support `--seed`, it should be extended before using its results in a paper.

## Recommended Result Layout

```text
results/
├── YYYY-MM-DD_experiment_name/
│   ├── config.json
│   ├── metrics.csv
│   ├── per_trial_results.csv
│   ├── summary.md
│   └── figures/
│       ├── policy_comparison.png
│       ├── safety_tradeoff.png
│       └── runtime_ablation.png
```

## Statistical Reporting

For publication-quality reporting, include:

- mean,
- standard deviation,
- 95% confidence interval,
- number of trials,
- failure count,
- safe-return count,
- mission-completion count,
- runtime distribution,
- negative-energy-margin rate.

## Baselines

Every new policy must be compared against:

1. fixed deterministic threshold,
2. risk-aware Monte Carlo RTH,
3. adaptive risk threshold,
4. battery-health-aware risk policy,
5. an ablated version without the new uncertainty source.

## Scenario Families

The following scenario families should be maintained:

| Scenario Family | Purpose |
|---|---|
| nominal | baseline sanity check |
| degraded battery | battery-health stress test |
| SoC bias | estimator robustness |
| dynamic gust | wind robustness |
| long-distance mission | late-decision stress test |
| combined uncertainty | coupled worst-case analysis |
| alternate landing | sequential action evaluation |

## Acceptance Criteria for a New Experiment

A new experiment is considered research-ready only when:

- it can be run from one documented command,
- it writes machine-readable outputs,
- it includes at least one deterministic baseline,
- it reports confidence intervals,
- it has a short interpretation section,
- it does not overwrite previous results silently,
- it has enough trials to make the reported trend meaningful.

## Paper Artifact Checklist

Before using repository results in a paper:

- freeze the code version with a Git tag,
- archive raw CSV outputs,
- export plots as PNG and PDF,
- record all experiment parameters,
- add a `summary.md` explaining the observed trend,
- include limitations and failure modes,
- verify that README commands still run.
