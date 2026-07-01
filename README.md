<div align="center">

# 🚁 Risk-Aware Return-to-Home Policy for UAVs

### Safe UAV Mission Execution under Battery Uncertainty, Battery Aging & Dynamic Wind Disturbances

<p>
  <b>Probabilistic RTH decision-making</b> · <b>Monte Carlo risk estimation</b> · <b>Battery-health-aware autonomy</b> · <b>Dynamic wind evaluation</b>
</p>

<p>
  <a href="https://python.org"><img src="https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python 3.10+"/></a>
  <a href="https://docs.ros.org"><img src="https://img.shields.io/badge/ROS2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white" alt="ROS2 Jazzy"/></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-4CAF50?style=for-the-badge" alt="MIT License"/></a>
  <img src="https://img.shields.io/badge/Risk-Monte%20Carlo-8E44AD?style=for-the-badge" alt="Monte Carlo Risk"/>
  <img src="https://img.shields.io/badge/UAV-Return--to--Home-E67E22?style=for-the-badge" alt="UAV Return to Home"/>
</p>

<p>
  <a href="#-project-overview">Overview</a> •
  <a href="#-key-results">Key Results</a> •
  <a href="#-quick-start">Quick Start</a> •
  <a href="#-interactive-simulator">Simulator</a> •
  <a href="#-citation">Citation</a>
</p>

---

###  What this project does

This repository develops and evaluates a **risk-aware Return-to-Home (RTH) policy** for unmanned aerial vehicles operating under uncertain battery state, battery degradation, and wind disturbances. Instead of relying only on a fixed battery threshold, the UAV estimates the probability of returning safely and triggers RTH when the estimated risk becomes unacceptable.

</div>

---

##  Project Overview

Safety-critical UAV missions are affected by several uncertain factors: the remaining state of charge may be noisy, the battery may be degraded, and wind conditions can change during flight. A deterministic RTH threshold can therefore be too late or too conservative.

This project studies whether probabilistic decision-making can improve return safety by estimating:

```math
P(\text{safe return} \mid \hat{SoC}, w, d, h_b)
```

where:

| Symbol | Meaning |
|---|---|
| `\hat{SoC}` | estimated state of charge |
| `w` | wind disturbance profile |
| `d` | return distance / mission geometry |
| `h_b` | battery health factor |

The planner triggers Return-to-Home when:

```math
P(\text{safe return}) < \tau
```

where `τ` is the risk threshold.

---

##  Core Idea

```text
UAV State + Battery Estimate + Wind Estimate
                    │
                    ▼
        Monte Carlo Return Feasibility
                    │
                    ▼
   Risk-Aware / Adaptive / Health-Aware RTH
                    │
                    ▼
      Continue Mission or Trigger Return-to-Home
```

The decision layer samples possible return outcomes and evaluates whether the UAV has enough energy to reach home safely under uncertainty.

---

##  Main Contributions

| Contribution | Description |
|---|---|
|  **Monte Carlo RTH policy** | Estimates the probability of safe return using sampled battery, wind, and energy-demand uncertainty. |
|  **Adaptive risk threshold** | Adjusts conservativeness according to uncertainty, wind intensity, distance, and battery health. |
|  **Battery-health-aware planning** | Accounts for degraded battery capacity during return feasibility estimation. |
|  **Dynamic wind gust evaluation** | Tests the policies under time-varying gust scenarios instead of only steady wind. |
|  **Safety-efficiency analysis** | Measures both safe-return behavior and mission completion trade-offs. |
| **Reproducible experiments** | Provides scripts, CSV outputs, and plots for systematic evaluation. |

---

##  Policies Compared

| Policy | Decision Logic | Main Characteristic |
|---|---|---|
| `deterministic_threshold` | Fixed SoC threshold | Simple baseline, weak uncertainty handling |
| `risk_aware_mc` | Monte Carlo safe-return probability | Probabilistic feasibility estimation |
| `adaptive_risk_mc` | Monte Carlo + adaptive threshold | More conservative under risky conditions |
| `health_aware_risk_mc` | Monte Carlo + battery health | Explicit battery degradation awareness |

---

##  Key Results

### Main Policy Comparison

```bash
python3 experiments/run_monte_carlo_experiments.py --trials 200 --mc-samples 500
```

| Policy | RTH Trigger | Safe Return | Failure | Mission Completion | Mean Energy Margin |
|---|---:|---:|---:|---:|---:|
| deterministic_threshold | 0.15% | 0.15% | 20.54% | 99.47% | 0.0836 |
| risk_aware_mc | 40.33% | 19.94% | 20.83% | 78.79% | 0.0836 |
| adaptive_risk_mc | 52.23% | 31.00% | 21.27% | 73.31% | 0.0837 |
| health_aware_risk_mc | 39.90% | 19.90% | 20.75% | 79.04% | 0.0843 |

> **Takeaway:** the deterministic baseline preserves mission completion because it almost never triggers RTH, while Monte Carlo policies increase safe-return behavior by explicitly evaluating return feasibility.

---

### Battery Aging Study

```bash
python3 experiments/run_battery_aging_study.py --trials 200
```

#### Safe Return Rate under Battery Health Degradation

| Battery Health | Deterministic | Risk-Aware MC | Adaptive Risk | Health-Aware MC |
|---:|---:|---:|---:|---:|
| 100% | 2.56% | 22.54% | 30.44% | 23.29% |
| 95% | 2.15% | 21.00% | 28.31% | 30.21% |
| 90% | 1.60% | 20.52% | 28.73% | 37.21% |
| 80% | 2.38% | 27.21% | 32.48% | 40.54% |
| 70% | 2.10% | 9.58% | 11.52% | 13.02% |

#### Failure Rate under Battery Health Degradation

| Battery Health | Deterministic | Risk-Aware MC | Adaptive Risk | Health-Aware MC |
|---:|---:|---:|---:|---:|
| 100% | 20.96% | 21.35% | 21.04% | 20.77% |
| 95% | 31.25% | 31.56% | 32.50% | 31.79% |
| 90% | 43.19% | 43.00% | 43.27% | 43.35% |
| 80% | 59.35% | 58.40% | 59.06% | 59.08% |
| 70% | 87.38% | 87.83% | 87.44% | 86.98% |

> **Takeaway:** battery degradation is a dominant safety factor. Health-aware planning improves safe-return behavior under degraded batteries, especially around 90% and 80% health, but it reduces mission completion because the policy becomes more conservative.

---

### Dynamic Wind Gust Study

```bash
python3 experiments/run_wind_gust_study.py --trials 200
```

#### Safe Return Rate under Dynamic Gusts

| Gust Scenario | Deterministic | Risk-Aware MC | Adaptive Risk | Health-Aware MC |
|---|---:|---:|---:|---:|
| `gust_low` | 1.5% | 75.0% | 89.5% | 91.0% |
| `gust_medium` | 3.0% | 76.5% | 85.0% | 85.0% |
| `gust_high` | 0.0% | 70.0% | 73.5% | 78.5% |
| `gust_extreme` | 1.0% | 54.5% | 60.5% | 56.0% |

#### Failure Rate under Dynamic Gusts

| Gust Scenario | Deterministic | Risk-Aware MC | Adaptive Risk | Health-Aware MC |
|---|---:|---:|---:|---:|
| `gust_low` | 9.5% | 9.0% | 5.5% | 8.0% |
| `gust_medium` | 9.0% | 13.0% | 8.5% | 15.0% |
| `gust_high` | 17.0% | 24.0% | 22.5% | 20.5% |
| `gust_extreme` | 51.0% | 39.0% | 38.5% | 44.0% |

> **Takeaway:** dynamic gusts strongly stress return feasibility. Monte Carlo policies maintain much stronger safe-return behavior than deterministic thresholding, especially under low-to-high gust conditions.

---

### Ablation Study

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

> **Takeaway:** the Monte Carlo estimator stabilizes quickly. In this simulation, 50–100 samples often provide comparable decision quality, while higher sample counts increase runtime almost linearly.

---

##  Main Conclusions

1. **Deterministic thresholding is not sufficient for safety-critical RTH decisions.**
2. **Monte Carlo risk estimation improves safe-return behavior under uncertainty.**
3. **Adaptive risk policies provide stronger safety behavior, but reduce mission completion.**
4. **Battery aging has a strong negative effect on return feasibility.**
5. **Health-aware planning is valuable under degraded battery conditions.**
6. **Dynamic wind gusts significantly increase mission risk.**
7. **The central design trade-off is safety versus mission efficiency.**

---

## ⚙️ Quick Start

### Python Experiments

Create and activate a Python environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install "numpy<2" matplotlib
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
├── uav_rth_simulator.html       # Interactive simulator
├── README.md
└── CITATION.cff
```

---

## 📏 Evaluation Metrics

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

##  Research Relevance

This repository is relevant to:

- UAV safety and autonomy,
- risk-aware decision making,
- probabilistic robotics,
- battery-aware mission planning,
- dynamic wind disturbance modelling,
- simulation-based evaluation of safety-critical systems.

Potential future extensions include:

- trajectory-aware return planning,
- spatial wind-field maps,
- more realistic electrochemical battery aging models,
- formal probabilistic safety bounds,
- sim-to-real validation on UAV hardware.

---

##  Citation

If you use this project in research, you may cite it as:

```bibtex
@software{grosdouli2025uavrth,
  author    = {Grosdouli, Panagiota},
  title     = {Risk-Aware Return-to-Home Policy for UAVs under Battery Uncertainty and Wind Disturbances},
  year      = {2025},
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

### Designed for safe autonomy under uncertainty.

⭐ **Star this repository** if you find it useful for UAV safety, robotics, or probabilistic planning research.

</div>
