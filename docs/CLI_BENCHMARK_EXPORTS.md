# CLI Benchmark Exports

## Purpose

The platform CLI uses the reusable benchmark runner and reports confidence intervals for the main safe-rate and failure-rate metrics. It also exports one row per trial so that results can be re-analysed later without rerunning the simulator.

## Run

```bash
python3 scripts/run_research_platform.py \
  --scenario configs/platform/default_scenario.json \
  --trials 200 \
  --output results_platform/platform_benchmark.csv \
  --trials-output results_platform/platform_trials.csv \
  --timeline results_platform/latest_timeline.json
```

## Outputs

| File | Purpose |
|---|---|
| `platform_benchmark.csv` | One-row aggregate benchmark summary. |
| `platform_trials.csv` | Per-trial records for statistical analysis and reproducibility. |
| `latest_timeline.json` | Step-by-step decision trace for one representative trial. |

## Added Metrics

- `safe_rate_ci95`
- `failure_rate`
- `failure_rate_ci95`

## Per-Trial Columns

- `trial_id`
- `scenario`
- `decision`
- `safe_return_probability`
- `final_soc`
- `mission_progress`
- `safe`
- `steps`
- `energy_margin`

## Research Value

The CLI output is now closer to a paper artifact because it includes uncertainty around safety and failure rates, plus raw trial-level records that support independent analysis, plotting, filtering and failure-mode inspection.
