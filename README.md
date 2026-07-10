<div align="center">

# RiskAwareUAV-RTH

### Probabilistic mission management and return-to-home decision-making for UAVs under battery and wind uncertainty

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue)](https://www.python.org/)
[![CI](https://img.shields.io/badge/CI-research--ci-informational)](.github/workflows/research-ci.yml)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)
[![Status](https://img.shields.io/badge/Status-simulation--only-orange)](#implementation-and-validation-status)

**Research prototype for studying when an autonomous UAV should continue its mission, return home, reroute, reduce speed, hold, or land safely when battery state, battery health, wind, and energy demand are uncertain.**

<img src="assets/demo.gif" alt="Risk-aware UAV return-to-home simulation" width="760">

</div>

---

## Abstract

Conventional return-to-home logic often triggers at a fixed battery percentage. Such a rule ignores distance to safety, wind direction, uncertainty in state of charge, degradation of usable capacity, route-dependent energy demand, and the possibility that home is no longer the safest reachable destination.

RiskAwareUAV-RTH is a simulation-only research platform for probabilistic UAV mission management. The framework combines uncertain battery-state estimation, battery-health uncertainty, stochastic wind disturbances, physically motivated multirotor energy prediction, Monte Carlo safe-return estimation, decision policies, reproducible scenario execution, and automatically generated evidence. The long-term objective is a sequential decision system that evaluates mission continuation, direct return, wind-aware return, rerouting, speed reduction, holding, alternate landing, and emergency landing under explicit safety constraints.

The repository is intentionally scientifically conservative. It does not claim electrochemical fidelity, aerodynamic identification, certified chance constraints, formal safety guarantees, or real-flight validation.

## Research question

> **How should an autonomous UAV decide whether to continue a mission, return home, modify its route, or land at an alternate site when battery state, battery health, wind, energy consumption, and future disturbances are uncertain?**

## Scientific hypotheses

The project is structured around the following hypotheses. They are research questions to be tested, not claims of confirmed performance.

- **H1 — Probabilistic RTH:** A probabilistic return policy can reduce unsafe battery depletion relative to fixed state-of-charge thresholds.
- **H2 — Wind-aware planning:** Route planning that models spatially or temporally varying wind can improve safe-arrival probability relative to direct geometric return.
- **H3 — Bayesian battery estimation:** Filtering noisy SoC observations and representing usable-capacity uncertainty can support safer decisions than using raw measurements alone.
- **H4 — Sequential decisions:** Multi-action policies can outperform one-time binary continue/return decisions when conditions evolve during flight.
- **H5 — Alternate landing:** Explicit alternate-site evaluation can reduce catastrophic energy-depletion failures when home becomes unreachable.
- **H6 — Calibration:** Estimated safe-arrival probabilities require empirical calibration before they can be interpreted as reliable probabilities.
- **H7 — Chance constraints:** Probability thresholds can expose a measurable safety-efficiency trade-off between mission progress and conservative return behaviour.

No hypothesis is considered supported until experiments are executed on identical seeds, aggregate statistics are reported, and failure cases are inspected.

## Probabilistic formulation

Let the latent mission state at time \(t\) contain position, velocity, altitude, battery state, battery health, and local wind:

```math
x_t = \left(p_t, v_t, h_t, SoC_t, q_t, w_t\right).
```

The estimator maintains a belief over uncertain quantities rather than assuming that measurements are exact:

```math
b_t = p\left(SoC_t, q_t, C_{usable}, w_t \mid z_{0:t}, u_{0:t-1}\right).
```

For a candidate return path \(\pi\), safe arrival is represented by the event

```math
E_{required}(\pi, w, \theta_E) + E_{reserve}
\leq
E_{available}(SoC, q, C_{usable}).
```

The corresponding probability is

```math
P_{safe}(\pi)
=
P\!\left(
E_{required}(\pi) + E_{reserve}
\leq E_{available}
\mid b_t
\right).
```

A chance-constrained decision layer is intended to enforce conditions such as

```math
P_{safe}(\pi_{return}) \geq \tau_{return}
```

and

```math
P\!\left(E_{mission}+E_{return}+E_{reserve}\leq E_{available}\right)
\geq \tau_{continue}.
```

In the current repository, Monte Carlo safe-return estimation and a Gaussian battery-state estimator are implemented. Full sequential chance-constrained mission management remains under active development.

## Research architecture

```text
Experiment configuration and deterministic seed
                    │
                    ▼
        Lightweight UAV mission simulator
                    │
       ┌────────────┼─────────────┐
       ▼            ▼             ▼
Battery truth   Wind disturbance  Mission geometry
and sensor      and forecast      and candidate paths
model           uncertainty
       │            │             │
       ▼            ▼             ▼
Bayesian SoC    Wind belief     Multirotor-inspired
and health      scaffold        power and path-energy model
belief
       └────────────┼─────────────┘
                    ▼
       Probabilistic safe-arrival estimation
       deterministic / analytical / Monte Carlo
                    │
                    ▼
       Policy and safety-supervisor layer
                    │
                    ▼
 Trajectories, decisions, confidence intervals,
 metrics, manifests, figures, GIFs, and reports
```

Serious numerical logic is kept in importable modules. Scripts should orchestrate experiments rather than contain duplicate implementations.

## Implemented scientific components

### Bayesian battery-state estimation

The repository contains a transparent scalar Gaussian estimator under `risk_rth/estimation/battery_state.py`.

The estimator supports:

- latent true SoC and noisy synthetic observations,
- Gaussian prediction and measurement updates,
- posterior mean and variance,
- uncertainty growth from process and discharge-rate uncertainty,
- usable-capacity uncertainty,
- battery-health mean and variance,
- first-order propagation to usable-energy uncertainty,
- deterministic random-number generation for synthetic observations,
- structured logging of truth, observation, estimate, variance, and estimation error.

This is a Kalman-style research model. It is not an electrochemical battery model and has not been identified using real flight data.

### Multirotor-inspired power model

The scientific-hardening branch adds `risk_rth/energy/multirotor_power.py` and `risk_rth/energy/path_energy.py`.

The power model decomposes expected electrical demand into transparent terms:

```math
P = \frac{s_f}{\eta_b}
\left(
P_{induced}
+ P_{profile}
+ P_{parasitic}
+ P_{vertical}
+ P_{avionics}
\right),
```

where \(s_f\) is a model safety factor and \(\eta_b\) is battery/drivetrain efficiency.

The model represents:

- hover and induced-power contribution,
- profile-power scaffold,
- parasitic forward-flight contribution,
- climb and descent contribution,
- wind-relative airspeed,
- payload scaling,
- drivetrain efficiency,
- model safety factor,
- explicit uncertainty estimate,
- component-level power breakdown.

The path-energy integrator evaluates piecewise 2.5D segments and reports geometric distance, travel time, expected energy, uncertainty, and per-segment component information.

See [`docs/MULTIROTOR_ENERGY_MODEL.md`](docs/MULTIROTOR_ENERGY_MODEL.md).

### Wind and disturbance modelling

The existing simulation supports controlled wind disturbances including constant components, Gaussian uncertainty, and sinusoidal gust behaviour. Spatially varying wind fields, online wind-belief updates, wind corridors, forecast bias, and altitude-dependent wind remain research-development targets.

### Monte Carlo risk estimation

The safe-return estimator samples uncertain battery availability, wind effects, and return-energy demand:

```math
\widehat{P}_{safe}
=
\frac{1}{N}
\sum_{i=1}^{N}
\mathbf{1}
\left[
E^{(i)}_{required}+E_{reserve}
\leq
E^{(i)}_{available}
\right].
```

The estimator also reports a binomial confidence interval for diagnostic use. This confidence interval quantifies finite-sample uncertainty in the Monte Carlo estimate; it does not certify model correctness.

### Policies and baselines

The current baseline framework includes fixed-threshold, distance-based, deterministic energy-margin, and Monte Carlo risk-aware return logic. An oracle is treated only as a future-information upper bound and must never be used as a realistic deployable policy.

## Installation

Python 3.10 or later is required.

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[dev]'
```

On Windows PowerShell:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e '.[dev]'
```

## Verification

Run static checks and tests before interpreting any result:

```bash
ruff check .
black --check .
pytest -q
```

Run one controlled experiment:

```bash
python scripts/run_experiment.py \
  --config configs/experiments/nominal.yaml \
  --output-dir results/verification/nominal
```

Run the controlled experiment suite:

```bash
python scripts/run_all_experiments.py \
  --output-dir results/verification/suite
```

Generate the existing demonstration animation:

```bash
python scripts/make_demo_gif.py
```

A result should not be treated as evidence unless the exact configuration, seed, command, software environment, and output path are preserved.

## Experimental methodology

A rigorous comparison should use:

1. identical scenario definitions across policies,
2. identical random seeds across competing methods,
3. independent repetitions across multiple seeds,
4. explicit failed-run reporting,
5. uncertainty intervals where justified,
6. calibration assessment for predicted probabilities,
7. ablations that remove one subsystem at a time,
8. failure-case inspection rather than aggregate metrics alone.

Recommended outcome variables include:

- mission completion,
- safe landing,
- return to home,
- alternate landing,
- battery depletion,
- reserve violation,
- early return,
- mission progress,
- remaining energy,
- decision latency,
- policy switches,
- warning lead time,
- missed failures and false alarms.

Aggregate reports should include run count, successful and failed runs, mean, standard deviation, median, interquartile range, extrema, and confidence intervals where assumptions justify them.

## Reproducibility contract

Every research run should preserve:

- exact YAML configuration,
- seed,
- Python version,
- package versions,
- operating system,
- Git commit,
- timestamp,
- command,
- runtime,
- output paths,
- summary metrics.

The project policy is simple:

> **No numerical performance claim without an executable configuration and generated evidence.**

Generated data belong under `results/`. Public-facing visual assets belong under `assets/` only after they can be regenerated from code.

## Repository structure

```text
RiskAwareUAV-RTH/
├── configs/                  # experiment and future platform profiles
├── docs/                     # models, assumptions, evaluation, reproducibility
├── paper/                    # manuscript-oriented research notes
├── scripts/                  # experiment orchestration and visualization entry points
├── tests/                    # deterministic unit and integration tests
├── risk_rth/
│   ├── estimation/           # Bayesian battery-state estimation
│   ├── energy/               # multirotor power and path-energy prediction
│   ├── models/               # battery, wind, state, and legacy model containers
│   ├── planning/             # return policies and planning components
│   ├── simulation/           # controlled mission simulator
│   ├── uncertainty/          # Monte Carlo safe-return estimation
│   ├── evaluation/           # experiment metrics
│   ├── visualization/        # figures and animation helpers
│   └── utils/                # configuration and reproducibility utilities
├── ros2_ws/                  # research-integration scaffold; not flight validated
├── website/                  # project-page prototype
├── assets/                   # generated public-facing visuals
├── results/                  # generated evidence and reports
└── .github/workflows/        # continuous integration
```

## Implementation and validation status

| Subsystem | Status | Scientific interpretation |
|---|---|---|
| Controlled 2D mission simulator | **Implemented** | Simulation-only outbound and RTH experiments. |
| Gaussian battery SoC estimator | **Implemented — Research Prototype** | Bayesian/Kalman-style estimator; no electrochemical validation. |
| Battery-health uncertainty | **Research Prototype** | Mean/variance representation exists; repeated-mission identification remains pending. |
| Constant, stochastic, and gust wind | **Implemented — Simulation-Only** | Controlled disturbance generation. |
| Spatial wind field and online wind belief | **Planned** | Required for wind-corridor and route-planning hypotheses. |
| Legacy heuristic energy model | **Implemented — Baseline** | Retained for comparison. |
| Multirotor-inspired power decomposition | **Implemented — Research Prototype** | Transparent component model; not platform identified. |
| 2.5D path-energy integration | **Implemented — Research Prototype** | Segment-wise energy and uncertainty prediction. |
| Monte Carlo safe-return estimator | **Implemented** | Includes finite-sample confidence interval. |
| Deterministic and threshold baselines | **Implemented** | Used as controlled comparators. |
| Sequential multi-action policy | **Planned** | Continue, hold, reroute, return, alternate landing, emergency landing. |
| Chance-constrained policy | **Planned / Partial foundations** | Probability estimation exists; full sequential enforcement pending. |
| Alternate landing-site reasoning | **Planned** | Must be evaluated before declaring no safe action. |
| Probability calibration | **Planned** | Reliability diagrams, Brier score, ECE, threshold diagnostics. |
| Ablation and sensitivity suites | **Planned** | Required for causal subsystem assessment. |
| ROS 2 / PX4 / Gazebo bridge | **Research Prototype / Pending Validation** | No flight-validation claim. |
| Real-flight validation | **Required** | Mandatory before any real-world safety interpretation. |
| Formal safety guarantee | **Not implemented** | Simulation-based probabilities are not certification. |

## Current limitations

- The simulator is a controlled research environment, not a high-fidelity flight dynamics simulator.
- The battery estimator is Gaussian and scalar; nonlinear voltage, current, temperature, hysteresis, and electrochemical effects are not represented.
- Battery-health and usable-capacity parameters are assumptions unless identified from measurements.
- The multirotor power model is physically motivated but generic; its coefficients are not tied to a verified commercial platform.
- Wind uncertainty is not yet represented as a fully inferred spatial-temporal field.
- Current return logic is primarily binary and does not yet implement the complete sequential action set.
- Monte Carlo output is only as valid as the assumed battery, wind, and energy distributions.
- An estimated probability is not necessarily calibrated.
- No formal verification, reachability certificate, or aviation certification is provided.
- No real-flight safety claim is made.

## Research roadmap

The next scientific milestones are:

1. integrate battery belief into every simulator decision step,
2. implement spatial-temporal wind fields and online wind-belief updates,
3. add direct, minimum-energy, wind-aware, and risk-aware return paths,
4. implement sequential multi-action decision-making,
5. separate policy utility from an independent safety supervisor,
6. evaluate alternate landing sites when home is unsafe,
7. implement and test chance constraints,
8. compare analytical and Monte Carlo probability estimates,
9. run identical-seed baseline, ablation, and sensitivity studies,
10. evaluate probability calibration and failure-warning quality,
11. generate reports and figures only from actual experiment outputs,
12. validate ROS 2/PX4/Gazebo integration before considering hardware experiments.

## Documentation

- [`docs/RESEARCH_OVERVIEW.md`](docs/RESEARCH_OVERVIEW.md)
- [`docs/MATHEMATICAL_FORMULATION.md`](docs/MATHEMATICAL_FORMULATION.md)
- [`docs/SYSTEM_ARCHITECTURE.md`](docs/SYSTEM_ARCHITECTURE.md)
- [`docs/BATTERY_MODEL.md`](docs/BATTERY_MODEL.md)
- [`docs/WIND_MODEL.md`](docs/WIND_MODEL.md)
- [`docs/MULTIROTOR_ENERGY_MODEL.md`](docs/MULTIROTOR_ENERGY_MODEL.md)
- [`docs/MONTE_CARLO_RISK_ESTIMATION.md`](docs/MONTE_CARLO_RISK_ESTIMATION.md)
- [`docs/EVALUATION_PROTOCOL.md`](docs/EVALUATION_PROTOCOL.md)
- [`docs/EXPERIMENTS.md`](docs/EXPERIMENTS.md)
- [`docs/REPRODUCIBILITY.md`](docs/REPRODUCIBILITY.md)
- [`docs/ROADMAP.md`](docs/ROADMAP.md)

## Citation

If this repository supports academic work, cite the software using [`CITATION.cff`](CITATION.cff). Literature citations should be added only after verification against primary sources; placeholder or fabricated references are not acceptable.

## Responsible use

This software is intended for research, education, simulation, and algorithmic evaluation. It must not be treated as a flight-certified safety system. Deployment on physical aircraft requires platform identification, hardware-in-the-loop testing, operational risk assessment, regulatory compliance, fault handling, independent validation, and real-flight evidence.

## License

Released under the MIT License. See [`LICENSE`](LICENSE).
