# CLI Benchmark Exports

## Purpose

The platform CLI now uses the reusable benchmark runner and reports confidence intervals for the main safe-rate and failure-rate metrics.

## Run

```bash
python3 scripts/run_research_platform.py \
  --scenario configs/platform/default_scenario.json \
  --trials 200 \
  --output results_platform/platform_benchmark.csv \
  --timeline results_platform/latest_timeline.json
```

## Outputs

| File | Purpose |
|---|---|
| `platform_benchmark.csv` | One-row aggregate benchmark summary. |
| `latest_timeline.json` | Step-by-step decision trace for one representative trial. |

## Added Metrics

- `safe_rate_ci95`
- `failure_rate`
- `failure_rate_ci95`

## Research Value

The CLI output is now closer to a paper artifact because it includes uncertainty around safety and failure rates, not only point estimates.
