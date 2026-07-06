# Theoretical Framework

## Thesis Position

This project studies autonomous Return-to-Home (RTH) as a special case of probabilistic safety supervision for energy-constrained UAVs. The central scientific position is that RTH should not be triggered by a fixed battery percentage alone. Instead, the vehicle should estimate the probability that each available safety action remains feasible under uncertainty and select the action that satisfies a required safety risk bound while preserving as much mission utility as possible.

## Formal Problem Statement

Let the UAV mission state at decision time `t` be

```math
x_t = [p_t, v_t, \hat{s}_t, \hat{h}_t, \hat{w}_t, m_t]
```

where `p_t` is position, `v_t` is velocity, `\hat{s}_t` is estimated state of charge, `\hat{h}_t` is estimated battery health, `\hat{w}_t` is estimated wind state and `m_t` is mission progress.

The decision layer chooses an action

```math
a_t \in A = \{continue, return, reroute, loiter, land, emergency\_land\}
```

under uncertain available energy, uncertain energy demand and uncertain environmental disturbance.

The safety objective is to maintain

```math
P(E_{available,t} - E_{required,t}(a_t) \ge 0 \mid x_t) \ge 1 - \epsilon
```

where `epsilon` is the tolerated probability of infeasible recovery.

## Risk-Aware Decision Rule

For every candidate action `a`, estimate:

```math
R(a \mid x_t) = P(E_{required}(a) > E_{available} \mid x_t)
```

The supervisor selects the lowest-cost action satisfying the risk constraint:

```math
a_t^* = \arg\min_{a \in A} J(a, x_t)
```

subject to:

```math
R(a \mid x_t) \le \epsilon
```

A practical cost can include mission loss, energy use, time delay and operational severity:

```math
J(a, x_t) = c_{mission}(a) + \lambda c_{energy}(a) + \gamma c_{time}(a) + \eta c_{severity}(a)
```

## Uncertainty Sources

| Source | Random Variable | Research Treatment |
|---|---|---|
| State-of-charge estimation | `S_t` | Bayesian filtering, measurement-noise modelling |
| Battery health | `H_t` | degradation model, capacity uncertainty |
| Wind disturbance | `W_t` | stochastic gust model, online estimation |
| Energy model residual | `Theta_E` | calibrated residual distribution |
| Mission geometry | `D_t` | distance-to-home and alternate-site uncertainty |
| Decision latency | `Delta_t` | delayed trigger stress tests |

## Current Repository Scope

The current code base implements the first tractable layer of this theory:

```math
P(E_{return} \le E_{available})
```

estimated through Monte Carlo simulation for deterministic, risk-aware, adaptive and battery-health-aware RTH policies.

## PhD-Level Extension

The doctoral extension is to generalize the current binary RTH decision into a multi-action probabilistic supervisor. This requires:

1. probabilistic battery state and health estimation,
2. wind-aware return-energy prediction,
3. calibrated model residuals,
4. chance-constrained or belief-space planning,
5. sequential safety decisions,
6. statistical validation across scenario families,
7. ROS 2 / PX4 / Gazebo digital-twin evaluation,
8. sim-to-real validation using flight logs or hardware-in-the-loop experiments.

## Scientific Contribution Boundary

The contribution is not only a new implementation of RTH. The contribution is a reusable probabilistic decision architecture for UAV mission safety under coupled energy, wind and model uncertainty.

## Safety Boundary

This repository is a research prototype. It must not be treated as a certified flight-safety system. Real UAV deployment requires certified autopilot failsafes, geofencing, redundancy, pilot override, regulatory compliance and independent validation.
