# Research Architecture

## Purpose

This document describes the intended architecture of the repository as a research framework for probabilistic safe autonomy in energy-constrained UAVs.

The system is organized around one central idea: the UAV should not rely only on a fixed battery threshold. It should estimate the probability that different safety actions remain feasible under uncertainty and then choose the action that best balances safety and mission performance.

## Conceptual Architecture

```text
┌──────────────────────────────────────────────────────────────┐
│                         UAV Mission                           │
│      waypoints, distance-to-home, mission progress, state      │
└──────────────────────────────┬───────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────┐
│                   State and Uncertainty Layer                  │
│  battery SoC, health, wind, model uncertainty, return distance │
└──────────────────────────────┬───────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────┐
│                 Energy and Return Feasibility Model            │
│       predicts required return energy under uncertain inputs    │
└──────────────────────────────┬───────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────┐
│                    Monte Carlo Risk Estimator                  │
│      samples possible futures and estimates safe-return prob.   │
└──────────────────────────────┬───────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────┐
│                   Safety Decision Supervisor                   │
│       continue, return, reroute, land, abort, adapt mission     │
└──────────────────────────────┬───────────────────────────────┘
                               │
┌──────────────────────────────▼───────────────────────────────┐
│                    Evaluation and Logging Layer                │
│   safety metrics, mission metrics, confidence intervals, plots  │
└──────────────────────────────────────────────────────────────┘
```

## Current Implemented Layer

The current implementation focuses on Return-to-Home decision making.

| Layer | Current Status |
|---|---|
| Mission state | Simulated mission state and return distance. |
| Battery uncertainty | Monte Carlo perturbation of available energy. |
| Battery health | Reduced usable capacity through health factor. |
| Wind uncertainty | Dynamic gust scenarios and disturbance modelling. |
| Energy model | Simplified return-energy requirement model. |
| Risk estimator | Monte Carlo safe-return probability. |
| Decision supervisor | Continue mission or trigger RTH. |
| Evaluation | Monte Carlo experiments, ablation, battery aging, wind gust studies. |

## Target PhD-Level Architecture

The future architecture should expand the current binary RTH framework into a full mission safety supervisor.

### 1. Perception and State Estimation

Inputs:

- vehicle pose,
- velocity,
- battery state of charge,
- battery voltage/current,
- temperature,
- wind estimate,
- mission phase,
- distance to home and alternate landing sites.

Research direction:

- Bayesian filtering,
- Gaussian-process regression,
- uncertainty-aware battery state estimation,
- online wind estimation.

### 2. Probabilistic Energy Prediction

Goal:

Estimate the distribution of required energy for each candidate safety action.

Candidate outputs:

```math
p(E_{return}), \quad p(E_{reroute}), \quad p(E_{land}), \quad p(E_{continue})
```

The decision system should compare these distributions against uncertain available energy:

```math
p(E_{available})
```

### 3. Risk-Aware Decision Layer

The current decision is:

```text
continue mission OR return home
```

The target decision set is:

```text
continue mission
return home
reroute
reduce speed
change altitude
land at alternate site
emergency land
abort mission
```

Each action can be evaluated through a risk-cost objective:

```math
J(a) = C_{mission}(a) + \lambda C_{risk}(a) + \gamma C_{energy}(a)
```

where `a` is a candidate action.

### 4. Planning Layer

Candidate planning methods:

- chance-constrained model predictive control,
- risk-aware graph search,
- risk-aware sampling-based planning,
- belief-space planning,
- POMDP-based mission supervision.

### 5. Learning Layer

Learning should support, not replace, safety.

Possible uses:

- learn energy-model residuals,
- tune adaptive risk thresholds,
- learn wind correction factors,
- predict battery-health degradation,
- select less conservative but still safe policies.

### 6. Verification and Validation Layer

Required before real flight:

- unit tests,
- Monte Carlo stress testing,
- hardware-in-the-loop validation,
- ROS 2 integration testing,
- PX4/Gazebo digital twin,
- real-world flight logs,
- independent safety review.

## Design Principles

1. **Explicit uncertainty.** Do not hide uncertainty inside a single deterministic value.
2. **Safety before optimality.** Mission completion is secondary to recoverability.
3. **Explainable decisions.** Every RTH trigger should be traceable to risk, energy margin or uncertainty.
4. **Simulation first.** Use high-volume simulation before hardware experiments.
5. **Modular research code.** Keep models, policies and evaluation separate.
6. **Reproducibility.** Store seeds, parameters, metrics and plots.

## Suggested Module Structure

```text
risk_rth/
├── estimation/
│   ├── battery_state.py
│   ├── battery_health.py
│   └── wind_estimator.py
│
├── models/
│   ├── energy_model.py
│   ├── wind_model.py
│   └── degradation_model.py
│
├── policies/
│   ├── deterministic_threshold.py
│   ├── risk_aware_mc.py
│   ├── adaptive_risk.py
│   ├── health_aware.py
│   └── sequential_supervisor.py
│
├── planning/
│   ├── chance_constraints.py
│   ├── risk_aware_rrt.py
│   └── belief_space.py
│
├── evaluation/
│   ├── metrics.py
│   ├── confidence_intervals.py
│   └── scenario_runner.py
│
└── visualization/
    ├── plots.py
    └── dashboards.py
```

## Safety Boundary

This architecture is intended for research. It is not a substitute for certified autopilot failsafes, regulatory compliance, redundant hardware, geofencing, pilot override or independent flight safety validation.
