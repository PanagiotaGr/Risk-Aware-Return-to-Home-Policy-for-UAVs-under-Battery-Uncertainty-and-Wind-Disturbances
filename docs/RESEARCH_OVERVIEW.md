# Research Overview

This repository studies risk-aware return-to-home (RTH) decision-making for an energy-constrained UAV operating with uncertain battery state and stochastic wind disturbances.

## Implemented

- 2D mission simulation with outbound and return phases.
- Nominal battery drain with uncertain SoC sampling.
- Constant, stochastic, and gust wind models.
- Wind-aware required-return-energy approximation.
- Monte Carlo safe-return probability estimation.
- Deterministic and probabilistic RTH policy baselines.
- YAML-driven controlled experiments.
- Basic tests and generated visualization scripts.

## Prototype

- Confidence intervals for Monte Carlo feasibility estimates.
- GIF/MP4 demo generation from simulator traces.
- Risk-calibration and convergence workflows through generated logs.

## Planned

- 3D dynamics and attitude-aware energy models.
- Platform-identified power model from flight logs.
- Bayesian battery state and health estimation.
- Online wind-field estimation.
- Chance-constrained or belief-space planning.
- ROS 2/PX4/Gazebo integration and hardware validation.

## Scientific positioning

The repository should be read as simulation software for controlled studies. It does not claim real-world UAV safety without platform calibration, validation data, and flight testing.
