<div align="center">

# 🚁 Risk-Aware Return-to-Home Policy for UAVs

### Battery Uncertainty & Wind Disturbances for Safe Mission Execution

*Probabilistic decision-making · Monte Carlo risk estimation · ROS 2 simulation · Safety-aware autonomy*

---

[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://python.org)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org)
[![License](https://img.shields.io/badge/License-MIT-4CAF50?style=for-the-badge)](LICENSE)
[![Simulation](https://img.shields.io/badge/Simulation-UAV%20RTH-orange?style=for-the-badge)](.)
[![Risk](https://img.shields.io/badge/Risk-Monte%20Carlo-purple?style=for-the-badge)](.)
[![University](https://img.shields.io/badge/DUTH-ECE-1565C0?style=for-the-badge)](https://ee.duth.gr)

<br/>

**[📖 Overview](#overview) · [🏗 Architecture](#system-architecture) · [🚀 Quick Start](#quick-start) · [🧪 Scenarios](#scenarios) · [📊 Evaluation](#evaluation-metrics) · [📁 Project Structure](#project-structure) · [🤝 Citation](#citation)**

</div>

---

## Overview

Unmanned Aerial Vehicles (UAVs) operate under strict **energy constraints** and often face substantial **environmental uncertainty** during real missions. Two of the most critical factors affecting mission safety are:

- **battery uncertainty**, caused by noisy state-of-charge (SoC) estimation, temperature effects, aging, and modelling errors
- **wind disturbances**, which alter the true energy required to continue the mission or return safely

This project implements a **risk-aware Return-to-Home (RTH) policy** for UAVs. Instead of relying on fixed deterministic thresholds only, the system estimates the **probability of successful return** under uncertainty and triggers a return action when the mission becomes unsafe.

The framework is designed for:

- **autonomous UAV safety**
- **decision-making under uncertainty**
- **probabilistic robotics**
- **ROS 2 simulation and evaluation**
- **research on safety-critical aerial autonomy**

---

## Core Idea

The planner estimates the probability that the UAV can safely return to home:

```math
P(\text{safe return} \mid x, \hat{SoC}, w)
```

and applies the decision rule:

```math
\text{Trigger RTH if } P(\text{safe return}) < \tau
```

where:

- `τ ∈ (0,1)` is the **risk tolerance threshold**
- **low τ** corresponds to more aggressive mission continuation
- **high τ** corresponds to more conservative early return

### Probabilistic Interpretation

The return decision is not made from a single battery or wind estimate. Instead, the planner reasons over **uncertainty distributions**:

- uncertain battery state-of-charge
- uncertain wind magnitude and direction
- uncertain energy required to return home

This allows the UAV to make a **risk-sensitive safety decision** rather than a naive deterministic one.

---

## Estimation Method

The probability of safe return is estimated using **Monte Carlo sampling**.

At each decision step, the planner samples:

- battery states from an SoC uncertainty model
- wind disturbances from a stochastic wind model
- return energy consumption under sampled conditions

Then it computes the fraction of samples for which the UAV can still return safely.

### Monte Carlo Decision Logic

1. Observe current UAV state
2. Estimate distance-to-home
3. Sample possible battery and wind realizations
4. Compute return feasibility for each sample
5. Estimate:

```math
\hat{P}(\text{safe return}) = \frac{\#\text{ feasible samples}}{N}
```

6. Trigger `RTH` if:

```math
\hat{P}(\text{safe return}) < \tau
```

This makes the policy naturally robust to uncertainty and disturbances.

---

## System Architecture

```text
┌──────────────────────────────────────────────────────────────┐
│                   Risk-Aware UAV RTH Stack                  │
├──────────────────────────────────────────────────────────────┤
│                        UAV Simulator                        │
│   State propagation · Battery drain · Wind disturbances     │
├──────────────────────────────────────────────────────────────┤
│                     Perception / Estimation                 │
│   UAV state · SoC estimate · SoC uncertainty · Wind state   │
├──────────────────────────────────────────────────────────────┤
│                    Risk-Aware Planner                       │
│   Return cost model · Monte Carlo sampling · Risk metric    │
├──────────────────────────────────────────────────────────────┤
│                     Decision Layer                          │
│      Mission continuation / Return-to-Home triggering       │
├──────────────────────────────────────────────────────────────┤
│                  Visualisation & Logging                    │
│   Topic streams · Mission traces · Plots · Outcome stats    │
└──────────────────────────────────────────────────────────────┘
```

### Runtime Flow

```text
uav_sim  --->  state / battery / wind  --->  uav_planner  --->  RTH decision
   |                                           |               |
   |                                           v               v
   +---------------------- visualization / logs ----------> mission outcomes
```

### Runtime Loop

1. The simulator publishes UAV state, battery estimate, and wind conditions
2. The planner evaluates return feasibility under uncertainty
3. The risk-aware policy decides whether to continue the mission or trigger RTH
4. Logs and evaluation metrics are stored for analysis

---

## ROS 2 Packages

| Package | Description |
|--------|-------------|
| `uav_sim` | UAV simulation including kinematics, battery consumption, and wind disturbances |
| `uav_planner` | Risk-aware return planner using probabilistic safety evaluation |
| `uav_interfaces` | Custom ROS 2 message definitions for UAV state and planner communication |

---

## ROS 2 Interfaces

### Subscribed Topics

| Topic | Description |
|------|-------------|
| `/uav/state` | UAV pose, velocity, heading, and mission state |
| `/uav/battery` | SoC estimate and uncertainty information |
| `/uav/wind` | Wind vector, gust effects, and disturbances |

### Published Topics

| Topic | Description |
|------|-------------|
| `/uav/rth_trigger` | Boolean trigger for Return-to-Home |
| `/uav/risk_return` | Estimated probability of safe return |
| `/uav/mode` | Current mode: `MISSION` or `RTH` |

---

## Quick Start

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

#### Terminal 1 — Simulator

```bash
source install/setup.bash
ros2 run uav_sim uav_sim
```

#### Terminal 2 — Planner

```bash
source install/setup.bash
ros2 run uav_planner uav_planner
```

---

## Interactive Simulator

A lightweight interactive simulator is included for visual exploration of the policy:

- real-time UAV motion
- wind disturbance visualisation
- battery uncertainty effects
- live Monte Carlo risk estimation
- visible RTH triggering behaviour

### Live Demo

<p align="center">
  <a href="https://panagiotagr.github.io/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances/uav_rth_simulator.html">
    <b>▶ Open Live HTML Simulator</b>
  </a>
</p>

### Local File

```bash
uav_rth_simulator.html
```

---

## Scenarios

The framework supports multiple mission scenarios for testing robustness under uncertainty:

- **Calm Flight**  
  Nominal outbound mission under mild environmental conditions

- **Increasing Headwind**  
  Return becomes progressively more expensive due to rising headwind

- **Gust Disturbances**  
  Sudden short-duration wind bursts close to the mission end

- **Biased SoC Estimation**  
  Battery estimate is systematically optimistic or pessimistic

- **Fault Injection**  
  Unexpected additional energy drain or battery degradation event

These scenarios allow controlled evaluation of how the RTH policy reacts under both nominal and adverse conditions.

---

## Evaluation Metrics

The project evaluates mission safety and planner quality using:

- **Mission success rate**
- **Safe return rate**
- **Failure rate**
- **RTH trigger timing**
- **Sensitivity to risk threshold `τ`**

### Example Experimental Questions

- How early should RTH be triggered under uncertainty?
- How sensitive is performance to the threshold `τ`?
- How much do wind disturbances degrade return feasibility?
- How robust is the policy to biased battery estimation?
- Does Monte Carlo risk estimation improve safety compared to deterministic thresholds?

### Example Result Summary

| Metric | Deterministic Threshold | Risk-Aware Monte Carlo Policy |
|--------|-------------------------|-------------------------------|
| Mission continuation aggressiveness | High | Tunable via `τ` |
| Sensitivity to wind uncertainty | Limited | Explicitly modelled |
| Safety under SoC estimation error | Moderate | Improved |
| Interpretability of decision | Low | High |
| Return probability estimate | Not available | Available online |

> Replace this table with your actual experimental values once the runs are finalised.

---

## Project Structure

```text
Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances/
│
├── uav_sim/                     # UAV simulator package
│   ├── sim_node.py              # State propagation, battery model, wind model
│   └── ...
│
├── uav_planner/                 # Risk-aware planner package
│   ├── planner_node.py          # RTH policy and decision logic
│   └── ...
│
├── uav_interfaces/              # Custom ROS 2 messages
│   ├── msg/
│   └── ...
│
├── docs/                        # GIFs, figures, documentation material
│   ├── demo.gif
│   └── ...
│
├── uav_rth_simulator.html       # Standalone interactive visual simulator
├── LICENSE
└── README.md
```

---

## Research Contribution

This project contributes a compact framework for studying:

- **risk-aware decision-making in UAV autonomy**
- **probabilistic returnability under battery uncertainty**
- **stochastic mission safety under wind disturbances**
- **uncertainty-aware control logic in ROS 2**
- **simulation-based evaluation of safety-critical aerial policies**

It is especially relevant for research in:

- robotics
- autonomous aerial systems
- uncertainty-aware planning
- probabilistic safety analysis
- dependable autonomy

---

## Possible Extensions

The framework can be extended toward more advanced research directions:

- **adaptive risk thresholding**  
  Dynamic adjustment of `τ` depending on mission phase or environmental conditions

- **trajectory-aware return planning**  
  Optimisation of the actual return path under wind fields

- **battery degradation modelling**  
  Integration of physics-based or data-driven battery aging models

- **learning-based risk prediction**  
  Neural estimators for return feasibility and mission risk

- **sim-to-real validation**  
  Deployment on real UAV platforms with onboard state estimation

- **formal safety guarantees**  
  Reachability analysis or probabilistic safety bounds for RTH activation

---

## Example Decision Rule Summary

| Condition | Planner Action |
|----------|----------------|
| High SoC, mild wind, high return feasibility | Continue mission |
| Moderate SoC, uncertain wind, low margin | Monitor closely |
| Low estimated probability of safe return | Trigger RTH |
| Severe gusts or unexpected drain | Early conservative RTH |

---

## Citation

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

## Author

<div align="center">

**Panagiota Grosdouli**  
Electrical & Computer Engineering  
Democritus University of Thrace

</div>

---

## License

This project is released under the **MIT License**.  
See the [LICENSE](LICENSE) file for details.

---

<div align="center">

*Designed for safe autonomy under uncertainty.*

⭐ **Star this repository** if you find it useful for UAV safety, robotics, or probabilistic planning research.

</div>
