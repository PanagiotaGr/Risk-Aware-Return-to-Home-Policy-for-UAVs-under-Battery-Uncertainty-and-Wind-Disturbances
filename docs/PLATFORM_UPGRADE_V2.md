# Platform Upgrade v2

This document defines the next evolution of the project: from a Return-to-Home experiment repository into a modular research platform for risk-aware UAV autonomy.

## Goals

- Support multiple high-level mission decisions, not only `continue` and `return_home`.
- Keep simulation scenarios reproducible through configuration files.
- Produce benchmark summaries and decision timelines automatically.
- Provide a clean software layer for future dashboard, map, AI, and ROS 2 integration.

## New Decisions

| Decision | Meaning |
|---|---|
| `continue_mission` | Mission remains feasible under current risk. |
| `return_home` | RTH is triggered because safe-return probability is below the risk threshold. |
| `divert_to_safe_site` | Risk is too high for normal RTH, but controlled diversion may still be feasible. |
| `emergency_land` | Battery or feasibility is critically low. |
| `reduce_speed` | Risk is elevated; the UAV should reduce energy consumption before committing to RTH. |

## New Platform Layer

```text
risk_rth_platform/
├── models.py       # Decision, UAV state, scenario, result dataclasses
├── simulator.py    # Monte Carlo decision simulator
├── config.py       # Scenario loading and template generation
└── cli.py          # Benchmark command-line interface
```

## Run

```bash
python3 -m risk_rth_platform.cli --trials 200
```

or:

```bash
python3 scripts/run_platform_benchmark.py --trials 200
```

Outputs:

```text
results_platform/platform_benchmark.csv
results_platform/latest_timeline.json
```

## Scientific Rationale

The main extension is that the project is no longer restricted to a binary RTH decision. A UAV operating under uncertainty may need to choose among several risk responses: continue, return, divert, emergency land, or reduce speed. This makes the framework closer to real autonomous mission management and creates a stronger foundation for future work in adaptive planning, constrained optimization, and learning-based risk prediction.

## Next Implementation Steps

1. Add multiple scenario files: urban, coastal, mountain, high-wind, low-battery, and degraded-battery missions.
2. Add plotting scripts for platform benchmark outputs.
3. Add a lightweight dashboard that visualizes battery, wind, risk, probability of return, and selected decision.
4. Connect the platform layer to the existing interactive HTML simulator.
5. Add ROS 2 publishers for decision, risk, battery, wind, and trajectory status.
