<div align="center">

# 🚁 Probabilistic Safe Autonomy for Energy-Constrained UAVs

### Risk-Aware Return-to-Home, Battery Uncertainty, Wind Disturbances & Sequential Safety Decisions

<p>
  <b>Probabilistic robotics</b> · <b>Risk-aware decision making</b> · <b>Battery-health-aware autonomy</b> · <b>Wind-robust UAV planning</b> · <b>Safe mission supervision</b>
</p>

<p>
  <a href="https://python.org"><img src="https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python 3.10+"/></a>
  <a href="https://docs.ros.org"><img src="https://img.shields.io/badge/ROS2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white" alt="ROS2 Jazzy"/></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-4CAF50?style=for-the-badge" alt="MIT License"/></a>
  <img src="https://img.shields.io/badge/Research-PhD%20Level-8E44AD?style=for-the-badge" alt="PhD-level research"/>
  <img src="https://img.shields.io/badge/Safety-Probabilistic-E67E22?style=for-the-badge" alt="Probabilistic safety"/>
</p>

<p>
  <a href="#-research-vision">Vision</a> •
  <a href="#-scientific-problem">Scientific Problem</a> •
  <a href="#-framework">Framework</a> •
  <a href="#-experiments">Experiments</a> •
  <a href="#-doctoral-research-agenda">Doctoral Agenda</a> •
  <a href="#-research-documentation">Research Docs</a> •
  <a href="#-citation">Citation</a>
</p>

</div>

---

## Research Vision

This repository is being developed as a **PhD-level research framework for probabilistic safe autonomy in energy-constrained unmanned aerial vehicles (UAVs)**.

The current implementation starts from **risk-aware Return-to-Home (RTH)** under battery uncertainty, battery degradation and dynamic wind disturbances. The doctoral objective is broader: to develop a probabilistic mission-safety supervisor that reasons about uncertain energy availability, uncertain environmental conditions, mission progress, model mismatch and action feasibility in real time.

Instead of asking only:

> "At what battery percentage should the UAV return home?"

this project asks:

> **How can an autonomous UAV make safe, mission-aware decisions under coupled battery, wind, model and planning uncertainty?**

---

## Scientific Problem

Small UAVs operate with limited energy reserves and are exposed to disturbances that are difficult to predict precisely. A mission that appears feasible under nominal assumptions may become unsafe when the following uncertainties interact:

| Uncertainty Source | Effect on UAV Safety |
|---|---|
| State-of-charge estimation error | Available energy may be overestimated or underestimated. |
| Battery aging and health degradation | Usable capacity may be lower than expected. |
| Wind and gust disturbances | Return energy demand may increase nonlinearly. |
| Mission geometry | Return distance and heading change during the mission. |
| Model mismatch | The simulated energy model may differ from the real platform. |
| Decision timing | A late RTH trigger can make recovery impossible. |

Classical deterministic RTH logic is usually based on a fixed battery threshold. This is simple, but it does not explicitly estimate the probability that returning home is still feasible.

This repository formulates RTH as a probabilistic safety decision problem:

```math
P(\text{safe return} \mid \hat{SoC}, h_b, w, d, \theta_m)
```

where `SoC` is estimated battery state, `h_b` is battery health, `w` is wind disturbance, `d` is return distance and `theta_m` represents uncertain model parameters.

A risk-aware RTH policy triggers safety action when:

```math
P(\text{safe return}) < \tau
```

---

## Framework

```text
UAV mission state
      ↓
Battery / health / wind uncertainty estimation
      ↓
Energy and return-feasibility model
      ↓
Monte Carlo risk estimation
      ↓
Safety decision supervisor
      ↓
Continue mission / return home / reroute / land
```

The current implementation includes:

| Component | Description |
|---|---|
| `deterministic_threshold` | Fixed SoC baseline policy. |
| `risk_aware_mc` | Monte Carlo estimation of safe-return probability. |
| `adaptive_risk_mc` | Risk-aware policy with adaptive conservativeness. |
| `health_aware_risk_mc` | Policy that accounts for battery degradation. |
| Battery aging study | Evaluates RTH behavior under reduced battery health. |
| Dynamic gust study | Evaluates decision robustness under time-varying wind. |
| Ablation study | Measures Monte Carlo sample count versus runtime and decision quality. |
| ROS 2 simulation path | Provides a robotic-system integration direction. |
| HTML simulator | Enables lightweight visual exploration of the policy. |

---

## Main Contributions

1. **Risk-aware RTH formulation** using probabilistic return feasibility instead of a deterministic battery threshold.
2. **Monte Carlo safety estimator** for uncertain energy demand and uncertain battery availability.
3. **Battery-health-aware decision layer** that models degraded usable capacity.
4. **Dynamic wind disturbance evaluation** for testing return decisions under gust scenarios.
5. **Safety-efficiency trade-off analysis** using mission completion, failure rate, safe-return rate and energy margin.
6. **Reproducible experimental structure** for Monte Carlo evaluation, ablation, battery aging and wind gust studies.
7. **Doctoral roadmap** toward sequential decision making, chance-constrained planning, Bayesian prediction and sim-to-real validation.

---

## Experiments

### Main Monte Carlo Policy Comparison

```bash
python3 experiments/run_monte_carlo_experiments.py --trials 200 --mc-samples 500
```

| Policy | RTH Trigger | Safe Return | Failure | Mission Completion | Mean Energy Margin |
|---|---:|---:|---:|---:|---:|
| deterministic_threshold | 0.15% | 0.15% | 20.54% | 99.47% | 0.0836 |
| risk_aware_mc | 40.33% | 19.94% | 20.83% | 78.79% | 0.0836 |
| adaptive_risk_mc | 52.23% | 31.00% | 21.27% | 73.31% | 0.0837 |
| health_aware_risk_mc | 39.90% | 19.90% | 20.75% | 79.04% | 0.0843 |

**Interpretation.** The deterministic baseline preserves mission completion because it rarely triggers RTH. The risk-aware policies trigger RTH more often because they explicitly evaluate return feasibility under uncertainty.

### Research Platform Benchmark

```bash
python3 scripts/run_research_platform.py \
  --scenario configs/platform/default_scenario.json \
  --trials 200 \
  --output results_platform/platform_benchmark.csv \
  --timeline results_platform/latest_timeline.json
```

This command writes aggregate metrics with 95% confidence intervals and a representative decision timeline.

Generate diagnostic figures from the per-trial CSV:

```bash
python3 scripts/plot_platform_trials.py \
  --trials results_platform/platform_trials.csv \
  --output-dir results_platform/figures
```

### Battery Aging Study

```bash
python3 experiments/run_battery_aging_study.py --trials 200
```

### Dynamic Wind Gust Study

```bash
python3 experiments/run_wind_gust_study.py --trials 200
```

### Monte Carlo Ablation Study

```bash
python3 experiments/run_ablation_studies.py --trials 200
```

---

## Quick Start

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-dev.txt
```

Run the full experimental suite:

```bash
python3 experiments/run_monte_carlo_experiments.py --trials 200 --mc-samples 500
python3 experiments/run_ablation_studies.py --trials 200
python3 experiments/run_battery_aging_study.py --trials 200
python3 experiments/run_wind_gust_study.py --trials 200
python3 scripts/run_research_platform.py --trials 200
python3 scripts/plot_platform_trials.py
```

Generated result folders:

```text
results/
results_ablation/
results_battery_aging/
results_wind_gust/
results_platform/
```

---

## ROS 2 Simulation

### Prerequisites

- Ubuntu with ROS 2 Jazzy
- Python 3.10+
- `colcon`
- standard ROS 2 build tools

### Installation

```bash
git clone https://github.com/PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances.git
cd Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances
colcon build
source install/setup.bash
```

### Run

```bash
source install/setup.bash
ros2 run uav_sim uav_sim
```

In another terminal:

```bash
source install/setup.bash
ros2 run uav_planner uav_planner
```

---

## 🕹️ Interactive Simulator

A lightweight HTML simulator is included for visual exploration of the RTH policy:

```text
uav_rth_simulator.html
```

Live demo:

<p align="center">
  <a href="https://panagiotagr.github.io/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances/uav_rth_simulator.html">
    <img src="https://img.shields.io/badge/Open-Interactive%20Simulator-2ECC71?style=for-the-badge" alt="Open Interactive Simulator"/>
  </a>
</p>

---

## Doctoral Research Agenda

| Research Stage | Scientific Goal | Expected Output |
|---|---|---|
| Stage 1 | Risk-aware RTH under battery and wind uncertainty | Baseline framework and first publication |
| Stage 2 | Bayesian battery state and health prediction | Probabilistic battery estimator |
| Stage 3 | Online wind-field estimation | Wind-aware safety prediction |
| Stage 4 | Chance-constrained or belief-space planning | Probabilistic path-planning module |
| Stage 5 | Sequential mission safety decisions | Continue / return / reroute / land policy |
| Stage 6 | Safe reinforcement learning for risk adaptation | Learned risk threshold and policy tuning |
| Stage 7 | Digital twin and sim-to-real validation | ROS 2 / PX4 / Gazebo / hardware experiments |

---

## Research Documentation

| Document | Purpose |
|---|---|
| [`docs/PHD_RESEARCH_PROPOSAL.md`](docs/PHD_RESEARCH_PROPOSAL.md) | Doctoral proposal, hypotheses and publication direction. |
| [`docs/THEORETICAL_FRAMEWORK.md`](docs/THEORETICAL_FRAMEWORK.md) | Mathematical formulation of risk-aware UAV safety supervision. |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | System architecture and target PhD-level module structure. |
| [`docs/EXPERIMENTAL_PROTOCOL.md`](docs/EXPERIMENTAL_PROTOCOL.md) | Experimental design for current simulations. |
| [`docs/REPRODUCIBILITY_PROTOCOL.md`](docs/REPRODUCIBILITY_PROTOCOL.md) | Rules for reproducible experiments and paper artifacts. |
| [`docs/EVALUATION_MATRIX.md`](docs/EVALUATION_MATRIX.md) | Claim-to-evidence matrix for doctoral evaluation. |
| [`docs/PUBLICATION_PLAN.md`](docs/PUBLICATION_PLAN.md) | Paper sequence and target contribution plan. |
| [`docs/CONTRIBUTING_RESEARCH.md`](docs/CONTRIBUTING_RESEARCH.md) | Contribution rules for scientific extensions. |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | Implementation roadmap toward full safe autonomy. |
| [`docs/CLI_BENCHMARK_EXPORTS.md`](docs/CLI_BENCHMARK_EXPORTS.md) | CLI benchmark outputs for paper-style runs. |
| [`docs/PLATFORM_PLOTS.md`](docs/PLATFORM_PLOTS.md) | Plotting workflow for platform benchmark outputs. |

---

## Project Structure

```text
Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances/
│
├── experiments/                 # Reproducible Monte Carlo studies
├── results*/                    # Generated result folders
├── src/                         # ROS 2 source packages
├── risk_rth/                    # Python implementation components
├── risk_rth_platform/           # Modular platform benchmark layer
├── scripts/                     # Utility scripts
├── tests/                       # Tests
├── docs/                        # PhD-level research documentation
├── .github/workflows/           # Research CI
├── uav_rth_simulator.html       # Interactive simulator
├── requirements-dev.txt
├── README.md
└── CITATION.cff
```

---

## Evaluation Metrics

| Metric | Meaning |
|---|---|
| `safe_return_rate` | Fraction of trials where RTH was triggered and enough energy remained to return. |
| `failure_rate` | Fraction of trials where the UAV did not have enough energy to return safely. |
| `rth_trigger_rate` | Fraction of trials where the policy triggered RTH. |
| `mission_completion_rate` | Approximate fraction of mission completed before RTH or failure. |
| `mean_energy_margin` | Mean difference between available energy and required return energy. |
| `negative_margin_rate` | Fraction of trials with negative return energy margin. |
| `mean_battery_left` | Mean battery remaining after decision outcome. |
| `safe_return_ci95` | 95% confidence interval half-width for safe-return rate. |
| `failure_ci95` | 95% confidence interval half-width for failure rate. |

---

## Research Positioning

This project is relevant to probabilistic robotics, UAV safety, autonomous mission supervision, decision making under uncertainty, battery-aware mission planning, energy-constrained robotics, dynamic wind disturbance modelling, safe reinforcement learning and digital twins for robotic validation.

The intended research contribution is not merely a better RTH trigger. The broader contribution is a **probabilistic decision architecture for UAV safety under coupled environmental and system uncertainty**.

---

## Safety Notice

This repository is a research and simulation framework. It is **not flight-certified software** and should not be deployed on real UAV hardware without independent verification, hardware-in-the-loop testing, fail-safe mechanisms and compliance with applicable aviation regulations.

---

## Citation

If you use this project in research, you may cite it as:

```bibtex
@software{grosdouli2026_probabilistic_safe_uav_autonomy,
  author    = {Grosdouli, Panagiota},
  title     = {Probabilistic Safe Autonomy for Energy-Constrained UAVs under Battery Uncertainty and Wind Disturbances},
  year      = {2026},
  publisher = {GitHub},
  url       = {https://github.com/PanagiotaGr/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances},
  license   = {MIT}
}
```

---

## Author

<div align="center">

**Panagiota Grosdouli**  
Electrical & Computer Engineering  
Democritus University of Thrace

</div>

---

## License

This project is released under the **MIT License**. See the [LICENSE](LICENSE) file for details.

---

<div align="center">

### Toward safe, probabilistic and energy-aware UAV autonomy.

⭐ **Star this repository** if you find it useful for UAV safety, robotics, probabilistic planning, or autonomous systems research.

</div>
