# Reproducibility Report

## Position

All reported outcomes should be tied to a configuration file, deterministic seed, command-line invocation, generated output directory, and software environment. The project must not present synthetic simulation results as flight-test validation.

## Required Experiment Artifacts

Each experiment should preserve:

- resolved YAML configuration;
- deterministic random seed;
- execution timestamp;
- command used to run the experiment;
- metric summary;
- trajectory and risk plots;
- logs and warnings;
- software versions;
- status: `Implemented`, `Prototype`, `Pending`, or `Failed`.

## Benchmark Integrity

If an oracle, ROS 2 baseline, PX4/Gazebo baseline, or real-flight benchmark is unavailable, the benchmark row must be marked `Pending`. Numeric values must only come from executed code and saved outputs.

## Synthetic Data Policy

Simulation traces are synthetic. They are useful for algorithm development, ablation studies, and visualization, but they are not substitutes for calibrated flight logs.

## Current Reproducibility Status

- Configuration-driven simulations: implemented.
- Experiment smoke test in CI: implemented.
- Demo GIF smoke test in CI: implemented.
- Docker environment: implemented in this branch.
- Real flight validation: pending.
- Hardware-in-the-loop validation: pending.
- Full uncertainty calibration against logs: pending.
