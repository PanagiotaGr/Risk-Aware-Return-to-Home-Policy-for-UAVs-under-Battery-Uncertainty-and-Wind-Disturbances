# Risk-Aware Return-to-Home Policy for UAVs

### Battery Uncertainty & Wind Disturbances (ROS 2 Jazzy)

<p align="center">
  <img src="docs/demo.gif" width="720" alt="UAV Risk-Aware RTH Simulator Demo"/>
</p>

<p align="center">
  <a href="https://panagiotagr.github.io/Risk-Aware-Return-to-Home-Policy-for-UAVs-under-Battery-Uncertainty-and-Wind-Disturbances/uav_rth_simulator.html">
    <b>▶ Open Live HTML Simulator</b>
  </a>
</p>

---

##  Overview

Unmanned Aerial Vehicles (UAVs) operate under **energy constraints** and **environmental uncertainty**.
In realistic missions, battery state estimation (SoC) is uncertain due to:

* sensor noise
* temperature effects
* battery aging

Additionally, **wind disturbances** introduce stochastic energy consumption.

This project implements a **risk-aware Return-to-Home (RTH) policy**, enabling the UAV to decide whether to:

* continue the mission, or
* safely return to home

based on **probabilistic reasoning under uncertainty**.

---

## 🧠 Core Idea

The system estimates the probability of successful return:

[
P(\text{safe return} \mid x, \hat{SoC}, w)
]

and applies the decision rule:

[
\text{Trigger RTH if } P(\text{safe return}) < \tau
]

where:

* ( \tau \in (0,1) ): risk tolerance threshold
* low ( \tau ): aggressive behaviour
* high ( \tau ): conservative behaviour

### Estimation Method

The probability is computed using **Monte Carlo sampling** over:

* battery uncertainty (SoC variance)
* wind uncertainty (speed & direction)

---

##  System Architecture

```text
uav_sim  --->  state / battery / wind  --->  uav_planner  --->  RTH decision
   |                                           |               |
   |                                           v               v
   +---------------------- visualization / logs ----------> mission outcomes
```

### Runtime Loop

1. Simulator publishes UAV state, battery and wind
2. Planner evaluates return feasibility
3. Risk-aware policy decides mission continuation or RTH

---

##  ROS 2 Packages

| Package          | Description                           |
| ---------------- | ------------------------------------- |
| `uav_sim`        | UAV simulation (battery + wind model) |
| `uav_planner`    | Risk-aware decision policy            |
| `uav_interfaces` | Custom ROS 2 messages                 |

---

## ROS 2 Interfaces

### Subscriptions

| Topic          | Description                     |
| -------------- | ------------------------------- |
| `/uav/state`   | UAV position, velocity, heading |
| `/uav/battery` | SoC estimate + uncertainty      |
| `/uav/wind`    | Wind vector + disturbances      |


| Topic              | Description                |
| ------------------ | -------------------------- |
| `/uav/rth_trigger` | RTH decision               |
| `/uav/risk_return` | Probability of safe return |
| `/uav/mode`        | `MISSION` / `RTH`          |

---

##  Installation

```bash
colcon build
source install/setup.bash
```

---

## ▶️ Run

### Terminal 1 — Simulator

```bash
source install/setup.bash
ros2 run uav_sim uav_sim
```

### Terminal 2 — Planner

```bash
source install/setup.bash
ros2 run uav_planner uav_planner
```

---

##  Scenarios

The framework supports multiple uncertainty scenarios:

* Increasing headwind during outbound mission
* Gust disturbances near mission end
* Biased SoC estimation
* Sudden energy drain (fault injection)

---

##  Evaluation Metrics

* **Mission success rate**
* **Safe return rate**
* **Failure rate**
* **RTH trigger timing**
* **Sensitivity to risk threshold ( \tau )**

---

##  Simulator

A lightweight interactive simulator is included:

* Real-time UAV motion
* Dynamic wind disturbances
* Monte Carlo risk estimation
* Visual RTH triggering

 File:

```bash
uav_rth_simulator.html
```

---

## 🔬 Extensions

* Adaptive risk threshold ( \tau )
* Energy-aware trajectory optimization
* Real battery models integration
* Sim-to-real UAV validation
* Learning-based risk prediction

---

##  Author

**Panagiota Grosdouli**

---

##  License

MIT License

---

## Research Context

This project targets:

* autonomous systems
* robotics (ROS 2)
* decision-making under uncertainty
* safety-critical UAV operations

---

