# Scientific Report

## Scope

This repository studies risk-aware UAV return-to-home (RTH) decisions under battery uncertainty and wind disturbances. It is a simulation research project and does not claim certified real-world flight safety.

## Research Question

How can a UAV decide whether to continue a mission or return home when the available battery energy and return energy requirement are uncertain?

## Scientific Contributions

- Formalizes RTH as a probabilistic safe-return decision rather than a fixed battery-percentage rule.
- Implements a 2D mission simulator with stochastic battery and wind perturbations.
- Implements a Monte Carlo estimator for safe-return probability.
- Provides baseline policies for fixed-threshold, distance-based, deterministic-energy, and risk-aware RTH decisions.
- Generates trajectory and risk visualizations from code-created simulation traces.

## Expected Scientific Impact

The project makes the uncertainty assumptions explicit and testable. It can support MSc/PhD-level extensions in chance-constrained planning, belief-space control, battery state estimation, and risk calibration from flight logs.

## Model Limitations

- The energy model is heuristic and must be replaced by platform-identified multirotor power models before physical claims.
- Wind is represented through simplified stochastic models and does not yet include spatially resolved wind-field estimation.
- Battery state-of-charge uncertainty is simplified and does not yet model temperature, age, internal resistance, or nonlinear discharge dynamics.
- No real flight validation, formal verification, or certified safety guarantee is included.

## Future Research Directions

1. Bayesian battery-health and state-of-charge estimation.
2. Online wind-field inference from onboard telemetry.
3. Chance-constrained RTH and emergency-landing policies.
4. ROS 2 / PX4 / Gazebo integration with hardware-in-the-loop validation.
5. Risk calibration using real mission logs.
