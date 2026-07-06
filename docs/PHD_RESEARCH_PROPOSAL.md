# PhD Research Proposal

## Title

**Probabilistic Safe Autonomy for Energy-Constrained UAVs under Battery, Wind and Model Uncertainty**

## Abstract

This research investigates how unmanned aerial vehicles can make safe autonomous decisions when energy availability, battery health, wind disturbances and model parameters are uncertain. The current repository implements the first research stage: a risk-aware Return-to-Home policy that estimates the probability of safe return using Monte Carlo simulation. The long-term objective is to develop a general probabilistic decision framework that supervises UAV missions and selects safety actions such as continue, return, reroute, loiter or emergency land.

## Core Research Question

How can an autonomous UAV maximize mission performance while maintaining probabilistic safety under coupled energy, environmental and model uncertainty?

## Motivation

Battery-limited UAV missions are safety-critical because the vehicle may not be able to recover once the return decision is delayed. Fixed battery thresholds are easy to implement, but they do not represent the probability of successful return under wind, degradation or state-estimation uncertainty. A PhD-level approach should treat return decisions as part of a larger sequential decision-making problem under uncertainty.

## Research Objectives

1. Develop a probabilistic safety estimator for return feasibility.
2. Model battery state, battery health and usable energy uncertainty.
3. Integrate wind disturbance and dynamic gust modelling into energy-aware planning.
4. Extend RTH from a binary trigger into sequential mission safety decisions.
5. Compare deterministic, probabilistic, adaptive and learning-based policies.
6. Evaluate safety-efficiency trade-offs through statistically meaningful Monte Carlo studies.
7. Prepare the framework for ROS 2, digital-twin and hardware-in-the-loop validation.

## Hypotheses

### H1
Probabilistic RTH policies can improve safe-return behavior compared with fixed-threshold policies when battery and wind uncertainty are significant.

### H2
Battery-health-aware policies become increasingly important as battery degradation increases and usable capacity becomes less predictable.

### H3
Adaptive risk thresholds can improve safety under high uncertainty but may reduce mission completion, creating a measurable safety-efficiency trade-off.

### H4
Sequential safety policies that include rerouting and emergency landing can outperform binary continue/return policies under severe disturbance scenarios.

## Methodology

### Stage 1: Risk-Aware Return-to-Home

The current baseline estimates:

```math
P(safe\ return \mid \hat{SoC}, h_b, w, d, \theta_m)
```

using Monte Carlo samples of uncertain return outcomes.

### Stage 2: Battery State and Health Estimation

Candidate methods:

- Bayesian filtering for state-of-charge uncertainty.
- Gaussian processes for data-driven capacity prediction.
- Physics-informed battery degradation models.
- Temperature-aware discharge modelling.

### Stage 3: Wind-Aware Energy Prediction

Candidate methods:

- Online wind disturbance estimation.
- Spatial wind-field mapping.
- Gaussian-process wind prediction.
- Weather-informed prior models.

### Stage 4: Probabilistic Planning

Candidate methods:

- Chance-constrained model predictive control.
- Risk-aware RRT*.
- Belief-space planning.
- Partially observable Markov decision processes.

### Stage 5: Sequential Safety Supervision

The binary decision `continue` versus `return` should be expanded into:

- continue mission,
- return home,
- reroute to a safer path,
- reduce speed,
- change altitude,
- divert to an alternate landing site,
- execute emergency landing.

### Stage 6: Safe Learning

Learning should not replace the safety layer directly. Instead, learning can tune risk thresholds, improve uncertainty models or adapt policy parameters while remaining constrained by explicit safety checks.

## Evaluation Plan

Metrics:

- safe-return rate,
- failure rate,
- mission completion rate,
- RTH trigger rate,
- energy margin,
- negative-margin rate,
- decision latency,
- policy conservativeness,
- confidence intervals across Monte Carlo trials.

Scenarios:

- nominal battery,
- degraded battery,
- low to extreme wind gusts,
- long-range mission geometry,
- noisy SoC estimation,
- model mismatch,
- combined worst-case uncertainty.

## Expected Contributions

1. A formal probabilistic formulation of UAV RTH under energy and wind uncertainty.
2. A reusable simulation framework for risk-aware UAV mission supervision.
3. Comparative analysis of deterministic, probabilistic and adaptive RTH policies.
4. Extension toward sequential safety decisions under uncertainty.
5. A roadmap for digital-twin and sim-to-real validation.

## Publication Strategy

Potential publication sequence:

1. Risk-aware Return-to-Home under battery uncertainty and wind disturbances.
2. Battery-health-aware probabilistic mission safety for UAVs.
3. Wind-aware energy prediction for safe UAV return planning.
4. Sequential decision making for UAV mission safety under uncertainty.
5. Integrated probabilistic safe autonomy framework for energy-constrained UAVs.

## Current Status

The repository currently provides the first research layer: simulation-based risk-aware RTH with Monte Carlo evaluation, battery aging analysis, wind gust experiments and ROS 2 integration potential.
