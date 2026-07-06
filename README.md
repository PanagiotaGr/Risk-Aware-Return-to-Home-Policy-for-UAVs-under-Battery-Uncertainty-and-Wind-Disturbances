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
  <a href="#-phd-research-agenda">PhD Agenda</a> •
  <a href="#-citation">Citation</a>
</p>

</div>

---

##  Research Vision

This repository is being developed as a **PhD-level research framework for probabilistic safe autonomy in energy-constrained unmanned aerial vehicles (UAVs)**.

The initial implementation focuses on **risk-aware Return-to-Home (RTH)** under battery uncertainty, battery degradation, and dynamic wind disturbances. The broader research goal is to move beyond fixed battery thresholds and build an autonomous safety supervisor that can reason about uncertain energy availability, uncertain environmental conditions, mission progress, and safety risk in real time.

Instead of asking only:

> "At what battery percentage should the UAV return home?"

this project asks the more general research question:

> **How can an autonomous UAV make safe, mission-aware decisions under coupled battery, wind, model, and planning uncertainty?**

---

##  Scientific Problem

Small UAVs operate with limited energy reserves and are exposed to disturbances that are difficult to predict precisely. A mission that appears feasible under nominal assumptions may become unsafe when the following uncertainties interact:

| Uncertainty Source | Effect on UAV Safety |
|---|---|
| State-of-charge estimation error | The available energy may be overestimated or underestimated. |
| Battery aging and health degradation | The usable capacity may be lower than expected. |
| Wind and gust disturbances | The return energy demand may increase nonlinearly. |
| Mission geometry | Return distance and heading may change during the mission. |
| Model mismatch | The simulated energy model may differ from the real platform. |
| Decision timing | A late RTH trigger can make recovery impossible. |

Classical deterministic RTH logic is usually based on a fixed threshold, for example `return when battery < 25%`. This is simple, but it does not explicitly estimate the probability that returning home is still feasible.

This repository formulates RTH as a **probabilistic safety decision problem**:

```math
P(\text{safe return} \mid \hat{SoC}, h_b, w, d, \theta_m)
```

where:

| Symbol | Meaning |
|---|---|
| `\hat{SoC}` | estimated state of charge |
| `h_b` | battery health / usable capacity factor |
| `w` | wind field or wind disturbance profile |
| `d` | return distance and mission geometry |
| `\theta_m` | uncertain energy-model parameters |

A risk-aware RTH policy triggers safety action when:

```math
P(\text{safe return}) < \tau
```

where `τ` is a configurable safety threshold.

---

##  Framework

```text
                  ┌────────────────────────────┐
                  │ UAV Mission State           │
                  │ position, distance, phase   │
                  └──────────────┬─────────────┘
                                 │
                  ┌──────────────▼─────────────┐
                  │ Uncertainty Estimation      │
                  │ battery, health, wind       │
                  └──────────────┬─────────────┘
                                 │
                  ┌──────────────▼─────────────┐
                  │ Monte Carlo Risk Evaluation │
                  │ sampled return feasibility  │
                  └──────────────┬─────────────┘
                                 │
                  ┌──────────────▼─────────────┐
                  │ Safety Decision Supervisor  │
                  │ continue / return / abort   │
                  └──────────────┬─────────────┘
                                 │
                  ┌──────────────▼─────────────┐
                  │ Mission Outcome Analysis    │
                  │ safety-efficiency trade-off │
                  └────────────────────────────┘
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
| Ablation study | Measures the effect of Monte Carlo sample count on runtime and decision quality. |
| ROS 2 simulation | Provides a robotic-system integration path. |
| HTML simulator | Enables lightweight visual exploration of the policy. |

---

##  Main Contributions

This repository contributes a research-oriented baseline for safe UAV autonomy:

1. **Risk-aware RTH formulation** using probabilistic return feasibility instead of a deterministic battery threshold.
2. **Monte Carlo safety estimator** for uncertain energy demand and uncertain battery availability.
3. **Battery-health-aware decision layer** that models degraded usable capacity.
4. **Dynamic wind disturbance evaluation** for testing return decisions under gust scenarios.
5. **Safety-efficiency trade-off analysis** using mission completion, failure rate, safe-return rate, and energy margin.
6. **Reproducible experimental scripts** for Monte Carlo evaluation, ablation, battery aging, and wind gust studies.
7. **Research roadmap toward PhD-level safe autonomy**, including sequential decision making, chance-constrained planning, Bayesian prediction, and sim-to-real validation.

---

##  Experiments

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

### Battery Aging Study

```bash
python3 experiments/run_battery_aging_study.py --trials 200
```

| Battery Health | Deterministic Safe Return | Risk-Aware MC | Adaptive Risk | Health-Aware MC |
|---:|---:|---:|---:|---:|
| 100% | 2.56% | 22.54% | 30.44% | 23.29% |
| 95% | 2.15% | 21.00% | 28.31% | 30.21% |
| 90% | 1.60% | 20.52% | 28.73% | 37.21% |
| 80% | 2.38% | 27.21% | 32.48% | 40.54% |
| 70% | 2.10% | 9.58% | 11.52% | 13.02% |

**Interpretation.** Battery degradation is a dominant safety factor. Health-aware RTH becomes more conservative as usable capacity decreases.

### Dynamic Wind Gust Study

```bash
python3 experiments/run_wind_gust_study.py --trials 200
```

| Gust Scenario | Deterministic Safe Return | Risk-Aware MC | Adaptive Risk | Health-Aware MC |
|---|---:|---:|---:|---:|
| `gust_low` | 1.5% | 75.0% | 89.5% | 91.0% |
| `gust_medium` | 3.0% | 76.5% | 85.0% | 85.0% |
| `gust_high` | 0.0% | 70.0% | 73.5% | 78.5% |
| `gust_extreme` | 1.0% | 54.5% | 60.5% | 56.0% |

**Interpretation.** Dynamic wind strongly affects return feasibility. Monte Carlo policies provide a stronger safety response than fixed-threshold logic.

### Monte Carlo Ablation Study

```bash
python3 experiments/run_ablation_studies.py --trials 200
```

| Samples | Runtime per Trial | Safe Return | Failure |
|---:|---:|---:|---:|
| 50 | 0.064 ms | 19.81% | 20.13% |
| 100 | 0.128 ms | 19.31% | 21.21% |
| 250 | 0.288 ms | 19.73% | 20.83% |
| 500 | 0.562 ms | 19.52% | 20.63% |
| 1000 | 1.086 ms | 19.50% | 20.69% |

**Interpretation.** The estimator stabilizes quickly in the current simulation setting. Higher sample counts increase runtime approximately linearly.

---

##  Quick Start

### Python Experiments

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Run the full experimental suite:

```bash
python3 experiments/run_monte_carlo_experiments.py --trials 200 --mc-samples 500
python3 experiments/run_ablation_studies.py --trials 200
python3 experiments/run_battery_aging_study.py --trials 200
python3 experiments/run_wind_gust_study.py --trials 200
```

Generated result folders:

```text
results/
results_ablation/
results_battery_aging/
results_wind_gust/
```

---

##  ROS 2 Simulation

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

Terminal 1:

```bash
source install/setup.bash
ros2 run uav_sim uav_sim
```

Terminal 2:

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

##  PhD Research Agenda

The current repository can serve as the first stage of a larger doctoral research program.

| Research Stage | Scientific Goal | Expected Output |
|---|---|---|
| Stage 1 | Risk-aware RTH under battery and wind uncertainty | Baseline framework and first publication |
| Stage 2 | Bayesian battery state and health prediction | Probabilistic battery estimator |
| Stage 3 | Online wind-field estimation | Wind-aware safety prediction |
| Stage 4 | Chance-constrained or belief-space planning | Probabilistic path planning module |
| Stage 5 | Sequential mission safety decisions | Continue / return / reroute / land policy |
| Stage 6 | Safe reinforcement learning for risk adaptation | Learned risk threshold and policy tuning |
| Stage 7 | Digital twin and sim-to-real validation | ROS 2 / PX4 / Gazebo / hardware experiments |

See:

- [`docs/PHD_RESEARCH_PROPOSAL.md`](docs/PHD_RESEARCH_PROPOSAL.md)
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)
- [`docs/EXPERIMENTAL_PROTOCOL.md`](docs/EXPERIMENTAL_PROTOCOL.md)
- [`docs/ROADMAP.md`](docs/ROADMAP.md)

---

##  Project Structure

```text
Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances/
│
├── experiments/
│   ├── run_monte_carlo_experiments.py
│   ├── run_ablation_studies.py
│   ├── run_battery_aging_study.py
│   └── run_wind_gust_study.py
│
├── results/
├── results_ablation/
├── results_battery_aging/
├── results_wind_gust/
│
├── src/                         # ROS 2 source packages
├── risk_rth/                    # Python implementation components
├── scripts/                     # Utility scripts
├── tests/                       # Tests
├── docs/                        # PhD-level research documentation
├── uav_rth_simulator.html       # Interactive simulator
├── requirements.txt
├── README.md
└── CITATION.cff
```

---

##  Evaluation Metrics

| Metric | Meaning |
|---|---|
| `safe_return_rate` | Fraction of trials where RTH was triggered and the UAV had enough energy to return. |
| `failure_rate` | Fraction of trials where the UAV did not have enough energy to return safely. |
| `rth_trigger_rate` | Fraction of trials where the policy triggered RTH. |
| `mission_completion_rate` | Approximate fraction of mission completed before RTH or failure. |
| `mean_energy_margin` | Mean difference between available energy and required return energy. |
| `negative_margin_rate` | Fraction of trials with negative return energy margin. |
| `mean_battery_left` | Mean battery remaining after decision outcome. |
| `safe_return_ci95` | 95% confidence interval half-width for safe-return rate. |
| `failure_ci95` | 95% confidence interval half-width for failure rate. |

---

##  Research Positioning

This project is relevant to:

- probabilistic robotics,
- UAV safety and autonomous mission supervision,
- decision making under uncertainty,
- battery-aware mission planning,
- energy-constrained robotics,
- dynamic wind disturbance modelling,
- safe reinforcement learning,
- digital twins for robotic validation.

The intended research contribution is not merely a better RTH trigger. The broader contribution is a **probabilistic decision architecture for UAV safety under coupled environmental and system uncertainty**.

---

##  Safety Notice

This repository is a research and simulation framework. It is **not flight-certified software** and should not be deployed on real UAV hardware without independent verification, hardware-in-the-loop testing, fail-safe mechanisms, and compliance with applicable aviation regulations.

---

##  Citation

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

##  Author

<div align="center">

**Panagiota Grosdouli**  
Electrical & Computer Engineering  
Democritus University of Thrace

</div>

---

##  License

This project is released under the **MIT License**. See the [LICENSE](LICENSE) file for details.

---

<div align="center">

### Toward safe, probabilistic and energy-aware UAV autonomy.

⭐ **Star this repository** if you find it useful for UAV safety, robotics, probabilistic planning, or autonomous systems research.

</div>
