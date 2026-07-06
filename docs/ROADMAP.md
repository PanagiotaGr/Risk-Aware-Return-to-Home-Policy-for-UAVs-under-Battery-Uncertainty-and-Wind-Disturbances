# Research Roadmap

## Goal

Transform the current risk-aware Return-to-Home repository into a full PhD-level framework for probabilistic safe autonomy in energy-constrained UAVs.

## Roadmap Overview

| Phase | Theme | Main Outcome |
|---|---|---|
| Phase 1 | Risk-aware RTH baseline | Current repository as first research contribution. |
| Phase 2 | Battery uncertainty modelling | Probabilistic state and health estimator. |
| Phase 3 | Wind-aware energy prediction | Dynamic wind and gust-aware return modelling. |
| Phase 4 | Probabilistic planning | Chance-constrained or belief-space return planning. |
| Phase 5 | Sequential safety decisions | Continue, return, reroute, land, abort. |
| Phase 6 | Safe learning | Risk adaptation without removing explicit safety constraints. |
| Phase 7 | Digital twin and validation | ROS 2, PX4/Gazebo, HIL and real data. |

---

## Phase 1: Risk-Aware Return-to-Home Baseline

### Status

Implemented as the current repository baseline.

### Tasks

- Maintain deterministic threshold baseline.
- Maintain Monte Carlo risk-aware policy.
- Maintain adaptive risk policy.
- Maintain battery-health-aware policy.
- Report safety and mission-efficiency metrics.
- Keep results reproducible through scripts and CSV outputs.

### Research Output

Potential paper:

**Risk-Aware Return-to-Home for UAVs under Battery Uncertainty and Wind Disturbances**

---

## Phase 2: Probabilistic Battery State and Health Estimation

### Motivation

Battery state of charge is not perfectly known, and usable capacity decreases with aging, temperature and discharge history.

### Tasks

- Add battery state uncertainty model.
- Add battery health estimation module.
- Include temperature and discharge-rate effects.
- Compare deterministic SoC with probabilistic SoC distributions.
- Validate with synthetic and real battery discharge curves if available.

### Candidate Methods

- Bayesian filtering,
- Kalman or particle filtering,
- Gaussian processes,
- physics-informed degradation models,
- neural residual models.

### Research Output

Potential paper:

**Battery-Health-Aware Probabilistic Safety Estimation for Energy-Constrained UAVs**

---

## Phase 3: Wind-Aware Energy Prediction

### Motivation

Wind can increase return-energy demand and can make a previously feasible return unsafe.

### Tasks

- Model wind as a time-varying disturbance.
- Add wind direction relative to return path.
- Add spatial wind-field maps.
- Estimate wind online from UAV motion and energy residuals.
- Evaluate wind-aware versus wind-agnostic policies.

### Candidate Methods

- Gaussian-process wind maps,
- online disturbance observers,
- weather-informed priors,
- residual energy modelling.

### Research Output

Potential paper:

**Wind-Aware Probabilistic Energy Prediction for Safe UAV Return Planning**

---

## Phase 4: Probabilistic Return Planning

### Motivation

The safest path home is not always the shortest path. Wind, terrain, landing sites and energy uncertainty may change the optimal safety action.

### Tasks

- Add trajectory-aware return planning.
- Evaluate multiple return paths.
- Include chance constraints on energy feasibility.
- Compare direct return with risk-aware rerouting.
- Add alternate landing-site selection.

### Candidate Methods

- chance-constrained MPC,
- risk-aware A*,
- risk-aware RRT*,
- belief-space planning,
- stochastic shortest path planning.

### Research Output

Potential paper:

**Chance-Constrained Return Planning for UAVs under Energy and Wind Uncertainty**

---

## Phase 5: Sequential Mission Safety Supervisor

### Motivation

A PhD-level system should not make only one binary decision. It should decide among several safety actions over time.

### Tasks

- Define the full action set.
- Add state machine or decision process representation.
- Evaluate continue, return, reroute, loiter and emergency land decisions.
- Add mission-level cost function.
- Quantify safety versus mission-performance trade-offs.

### Candidate Methods

- Markov decision processes,
- partially observable Markov decision processes,
- model predictive safety supervision,
- risk-sensitive dynamic programming.

### Research Output

Potential paper:

**Sequential Safety Decision Making for Energy-Constrained UAV Missions under Uncertainty**

---

## Phase 6: Safe Learning and Risk Adaptation

### Motivation

Learning can improve performance, but safety-critical UAV control should not rely on unconstrained black-box decisions.

### Tasks

- Learn adaptive risk thresholds.
- Learn residual corrections for the energy model.
- Learn wind and battery uncertainty parameters.
- Add safety constraints around learned components.
- Compare learned policies against explicit risk-aware baselines.

### Candidate Methods

- safe reinforcement learning,
- constrained policy optimization,
- Bayesian optimization,
- offline reinforcement learning from simulation logs,
- shielded learning.

### Research Output

Potential paper:

**Safe Learning of Risk-Aware Mission Policies for Battery-Limited UAVs**

---

## Phase 7: Digital Twin, ROS 2 and Sim-to-Real Validation

### Motivation

Simulation-only results are not enough for strong robotics research. The framework should move toward realistic robotic validation.

### Tasks

- Integrate with ROS 2 nodes.
- Connect with PX4/Gazebo or equivalent simulator.
- Add hardware-in-the-loop testing.
- Replay real flight logs.
- Validate energy prediction with real discharge data.
- Document safety assumptions and limitations.

### Research Output

Potential paper:

**Digital-Twin Validation of Probabilistic Safe Autonomy for UAV Return-to-Home Decisions**

---

## Repository-Level Milestones

### Milestone A: Research Presentation

- PhD-level README.
- Research proposal.
- Architecture document.
- Experimental protocol.
- Roadmap.
- Citation metadata.

### Milestone B: Code Modularity

- Separate models, policies, metrics and experiments.
- Add configuration files.
- Add reproducible seed handling.
- Add unit tests for policy logic.

### Milestone C: Advanced Models

- Battery health model.
- Wind model.
- Energy-model uncertainty.
- Confidence intervals and statistical reporting.

### Milestone D: Planning and Sequential Decisions

- Rerouting.
- Alternate landing.
- Multi-action safety supervisor.
- Planning under uncertainty.

### Milestone E: Validation

- ROS 2 integration.
- Digital-twin experiments.
- Realistic logs.
- Hardware-in-the-loop.

## Definition of PhD-Level Maturity

The repository reaches PhD-level maturity when it provides:

1. a formal problem definition,
2. explicit uncertainty modelling,
3. multiple competing baselines,
4. statistically meaningful experiments,
5. reproducible results,
6. a modular architecture,
7. publishable research questions,
8. an extension path toward real robotic validation.

The current repository now represents the first layer of that larger research program.
