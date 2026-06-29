# Research Development Roadmap

This roadmap organizes the repository into reviewable research-engineering phases. Each phase should remain small enough to validate with tests and reproducible experiments.

## Phase 1: Research Architecture

Goal: make the existing standalone experiments easier to test, configure, and extend.

- Add Python project metadata.
- Add reusable metrics for rates, confidence intervals, and lower-tail CVaR.
- Add structured experiment configuration.
- Add unit tests for core utilities.
- Keep current experiment outputs reproducible.

## Phase 2: Dynamic Wind Field

Goal: replace scalar wind assumptions with spatially and temporally varying wind models.

Suggested components:

- steady wind baseline,
- time-varying gust model,
- spatial wind grid,
- stochastic wind sampler,
- scenario-level wind-field configuration.

## Phase 3: Probabilistic Battery Model

Goal: represent battery state as a distribution rather than a single deterministic value.

Suggested components:

- state-of-charge estimator,
- sensor bias and noise model,
- battery-health degradation factor,
- optional particle-filter or Kalman-filter estimator,
- uncertainty propagation into return-energy feasibility.

## Phase 4: Risk Engine

Goal: separate risk estimation from policy logic.

Suggested components:

- Monte Carlo feasibility estimator,
- success probability,
- failure probability,
- expected energy margin,
- lower-tail CVaR of return-energy margin,
- confidence intervals for safety metrics.

## Phase 5: Adaptive Decision Policy

Goal: move from a single trigger rule to a policy that can choose among mission actions.

Candidate actions:

- continue mission,
- return home,
- hover and re-estimate,
- emergency land.

## Phase 6: Risk-Aware Path Planning

Goal: make return decisions trajectory-aware rather than assuming only direct return distance.

Suggested planners:

- shortest-path baseline,
- A* with energy-aware cost,
- wind-aware A*,
- risk-aware A* using probability of safe return or CVaR.

## Phase 7: Emergency Landing Selection

Goal: add a safety fallback when return-to-home is less safe than landing locally.

Suggested scoring factors:

- distance to candidate landing zone,
- remaining battery,
- wind exposure,
- terrain or obstacle risk,
- landing-zone uncertainty.

## Phase 8: Statistical Evaluation

Goal: make claims publication-ready.

Recommended outputs:

- confidence intervals,
- ablation studies,
- effect-size reporting,
- hypothesis tests where appropriate,
- publication-quality plots.

## Phase 9: ROS 2 / Gazebo Integration

Goal: validate the policy in a robotics simulation stack.

Suggested steps:

- expose policy as a ROS 2 node,
- publish battery and wind-state messages,
- integrate with PX4 SITL or Gazebo wind plugins,
- compare standalone and ROS-backed experiment results.
