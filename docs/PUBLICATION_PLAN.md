# Publication Plan

## Purpose

This document translates the repository into a publishable doctoral research trajectory. Each paper should correspond to one clear scientific contribution, one experimental protocol and one reproducible software component.

## Paper 1: Risk-Aware Return-to-Home under Battery and Wind Uncertainty

### Core Question

Can probabilistic return-feasibility estimation improve UAV RTH decisions compared with fixed battery thresholds?

### Main Contribution

A Monte Carlo RTH decision layer that estimates the probability of safe return under uncertain battery state, battery health and wind disturbance.

### Experiments

- deterministic threshold baseline,
- risk-aware Monte Carlo policy,
- adaptive risk policy,
- health-aware policy,
- battery aging study,
- dynamic wind gust study,
- Monte Carlo sample ablation.

### Expected Claim

Risk-aware policies provide more explicit safety behavior than fixed-threshold RTH, but introduce a measurable safety-efficiency trade-off.

---

## Paper 2: Battery-Health-Aware Probabilistic Safety Estimation

### Core Question

How does battery degradation affect the reliability of UAV return decisions?

### Main Contribution

A probabilistic battery state and health model integrated into return-feasibility estimation.

### Experiments

- battery health from 100% to severe degradation,
- SoC noise and bias,
- discharge-rate uncertainty,
- temperature-aware synthetic scenarios,
- comparison with non-health-aware RTH.

### Expected Claim

Ignoring battery health causes overly optimistic return-feasibility estimates under degraded capacity.

---

## Paper 3: Wind-Aware Energy Prediction for Safe UAV Return Planning

### Core Question

How should dynamic wind uncertainty be represented in energy-aware UAV safety decisions?

### Main Contribution

A wind-aware return-energy predictor and risk estimator for dynamic gust scenarios.

### Experiments

- headwind, tailwind and crosswind cases,
- low, medium, high and extreme gusts,
- spatial wind maps,
- online wind-estimation uncertainty,
- wind-aware versus wind-agnostic policies.

### Expected Claim

Wind-aware risk estimation improves return-decision timing when disturbances significantly affect energy demand.

---

## Paper 4: Sequential Safety Decision Making for Energy-Constrained UAVs

### Core Question

Can UAV safety improve when the system chooses among multiple safety actions instead of only continue or return?

### Main Contribution

A sequential decision supervisor that evaluates continue, return, reroute, loiter, alternate landing and emergency landing actions.

### Experiments

- binary RTH baseline,
- multi-action safety supervisor,
- alternate landing-site scenarios,
- long-distance missions,
- combined battery and wind uncertainty,
- mission-level cost analysis.

### Expected Claim

Multi-action safety supervision can reduce failure risk in scenarios where returning home is no longer the safest action.

---

## Paper 5: Integrated Probabilistic Safe Autonomy Framework

### Core Question

Can battery, wind, planning and decision uncertainty be integrated into a unified UAV safety architecture?

### Main Contribution

A modular probabilistic safe autonomy framework validated across simulation, ROS 2 digital twin and realistic stress tests.

### Experiments

- full framework comparison,
- ROS 2 integration,
- PX4/Gazebo or equivalent simulator,
- hardware-in-the-loop if available,
- real flight-log replay if available,
- ablation of each uncertainty module.

### Expected Claim

A unified probabilistic safety supervisor provides a more scientifically grounded foundation for energy-constrained UAV autonomy than isolated threshold-based failsafes.

---

## Target Venues

Potential venues depend on the final methodological depth and validation quality:

- IEEE International Conference on Robotics and Automation,
- IEEE/RSJ International Conference on Intelligent Robots and Systems,
- IEEE Robotics and Automation Letters,
- Journal of Field Robotics,
- Autonomous Robots,
- Aerospace Science and Technology,
- Drones,
- IEEE Access for broader systems-oriented work.

## Repository Requirements Before Submission

Before submitting any paper, the repository should include:

1. reproducible commands,
2. fixed random seeds or documented randomization,
3. CSV result outputs,
4. generated plots,
5. clean policy APIs,
6. statistical confidence intervals,
7. ablation studies,
8. clear limitations,
9. safety notice,
10. citation metadata.

## Thesis-Level Unifying Statement

The doctoral contribution can be framed as:

> This thesis develops a probabilistic decision architecture for safe UAV mission supervision under coupled energy, battery-health, wind and planning uncertainty, demonstrating how explicit risk estimation can improve autonomous safety decisions beyond deterministic battery-threshold logic.
