# Evaluation Matrix

## Purpose

This matrix defines how each research claim should be evaluated. It prevents the project from becoming only a collection of simulations and turns it into a structured doctoral research program.

## Claim-to-Evidence Mapping

| Research Claim | Required Evidence | Main Metrics | Minimum Baselines |
|---|---|---|---|
| Probabilistic RTH improves safety awareness | Monte Carlo comparison under uncertainty | safe-return rate, failure rate, trigger timing | deterministic threshold |
| Battery health affects recoverability | degradation sweep | failure rate, energy margin, negative margin | non-health-aware policy |
| Wind uncertainty changes optimal RTH timing | gust and wind-direction scenarios | safe-return rate, trigger rate, return-energy error | wind-agnostic policy |
| Adaptive risk can tune conservativeness | risk-threshold ablation | mission completion, failure, safe return | fixed risk threshold |
| Sequential decisions outperform binary RTH | multi-action scenarios | mission utility, failure, emergency land rate | continue/return only |
| Planning-aware safety improves feasibility | path-planning scenarios | path cost, risk constraint violation | shortest path / nominal planner |

## Core Metrics

| Metric | Definition | Desired Direction |
|---|---|---|
| `safe_return_rate` | RTH triggered and enough energy remained to return | higher |
| `failure_rate` | UAV could not complete mission or return safely | lower |
| `mission_completion_rate` | mission objective completed without unsafe outcome | higher |
| `rth_trigger_rate` | fraction of trials where RTH was triggered | context-dependent |
| `mean_energy_margin` | final available minus required energy | higher |
| `negative_margin_rate` | fraction of trials ending with insufficient energy | lower |
| `decision_latency_ms` | computation time per decision | lower |
| `risk_constraint_violation_rate` | empirical violation of allowed risk bound | lower |

## Experimental Axes

| Axis | Values |
|---|---|
| Battery health | 100%, 95%, 90%, 80%, 70%, 60% |
| SoC noise | none, low, medium, high, biased |
| Wind intensity | calm, low, medium, high, extreme |
| Wind direction | tailwind, headwind, crosswind, rotating |
| Mission range | short, medium, long, near-limit |
| Monte Carlo samples | 50, 100, 250, 500, 1000, 5000 |
| Risk tolerance | 1%, 5%, 10%, 20% |
| Model mismatch | none, low, medium, severe |

## Publication-Quality Tables

Each paper should include at least:

1. policy comparison table,
2. uncertainty sweep table,
3. ablation table,
4. runtime table,
5. failure-mode table,
6. qualitative decision examples.

## Failure-Mode Taxonomy

| Failure Mode | Meaning |
|---|---|
| late RTH | policy triggered after safe return was no longer feasible |
| over-conservative RTH | policy returned too early and sacrificed mission utility |
| battery optimism | available energy was overestimated |
| wind underestimation | required return energy was underestimated |
| model mismatch | simplified energy model produced unsafe prediction |
| computational delay | decision took too long for real-time supervision |

## Minimum Threshold for a Strong Result

A result is scientifically interesting when it shows one of the following:

- lower failure rate at comparable mission completion,
- same failure rate with higher mission completion,
- earlier unsafe-state detection under degraded battery or wind,
- calibrated risk estimates that match empirical failure rates,
- clear explanation of safety-efficiency trade-off.

## Negative Results

Negative results should be kept. A policy that is safer but too conservative is still scientifically useful because it reveals the cost of risk aversion and motivates adaptive or sequential decision making.
