# Risk-Aware Return-to-Home Policy for UAVs

### Battery Uncertainty + Wind Disturbances (ROS 2 Jazzy)

> ROS 2 (Jazzy) simulation of a UAV executing a mission with a **risk-aware Return-to-Home (RTH)** policy under **battery uncertainty** and **wind disturbances**. The goal is to decide *when to abort the mission and return safely* by explicitly reasoning about uncertainty and risk.

---

## Table of Contents

* [Overview](#overview)
* [Core Idea](#core-idea)
* [ROS 2 Packages](#ros-2-packages)
* [System Architecture](#system-architecture)
* [ROS 2 Interfaces](#ros-2-interfaces)
* [Installation](#installation)
* [Run](#run)
* [Scenarios](#scenarios)
* [Evaluation Metrics](#evaluation-metrics)
* [Extensions](#extensions)
* [Author](#author)
* [License](#license)

---

## Overview

UAV missions are often constrained by uncertain battery state (SoC estimation error, temperature effects, aging) and external disturbances such as wind and gusts. A risk-neutral policy may continue too long and fail to return safely, while an overly conservative policy wastes mission time.

This project implements a **risk-aware RTH decision policy** that balances mission progress against safety under uncertainty.

---

## Core Idea

At runtime, the system estimates the probability of safe return given current conditions:

<p align="center">
  <img alt="p(return)" src="https://latex.codecogs.com/svg.image?\Large%20p_{\mathrm{return}}(t)=\Pr\big(\text{reach home before battery depletion}\ \big|\ \text{state,wind}\big)" />
</p>

A simple decision rule can be expressed as:

<p align="center">
  <img alt="decision rule" src="https://latex.codecogs.com/svg.image?\Large%20\text{if }p_{\mathrm{return}}(t)<\tau\text{ then trigger RTH}" />
</p>

where:

* <img alt="tau" src="https://latex.codecogs.com/svg.image?\tau" /> is a user-defined safety threshold (risk tolerance)
* lower <img alt="tau" src="https://latex.codecogs.com/svg.image?\tau" /> → more aggressive missions
* higher <img alt="tau" src="https://latex.codecogs.com/svg.image?\tau" /> → more conservative, safer behaviour

---

## ROS 2 Packages

* `uav_sim` — UAV simulation (wind + battery model)
* `uav_planner` — risk-aware RTH decision policy
* `uav_interfaces` — custom ROS 2 messages

---

## System Architecture

```text
uav_sim  --->  publishes state, battery, wind  --->  uav_planner  --->  RTH decision
   |                                              |                  |
   |                                              v                  v
   +--------------------------- visualization / logs ---------> mission outcomes
```

Typical loop:

1. simulator publishes UAV state, battery estimate, and wind disturbance
2. planner predicts return feasibility / risk
3. planner either continues mission or triggers RTH

---

## ROS 2 Interfaces

(Example interfaces; keep or adapt to your implementation.)

### Subscriptions

| Topic          | Type                          | Description                 |
| -------------- | ----------------------------- | --------------------------- |
| `/uav/state`   | `uav_interfaces/UavState`     | position, velocity, heading |
| `/uav/battery` | `uav_interfaces/BatteryState` | SoC + uncertainty           |
| `/uav/wind`    | `uav_interfaces/Wind`         | wind vector + gusts         |

### Publications

| Topic              | Type               | Description               |
| ------------------ | ------------------ | ------------------------- |
| `/uav/rth_trigger` | `std_msgs/Bool`    | RTH decision              |
| `/uav/risk_return` | `std_msgs/Float32` | risk / return probability |
| `/uav/mode`        | `std_msgs/String`  | `MISSION` or `RTH`        |

---

## Installation

```bash
colcon build
source install/setup.bash
```

---

## Run

In separate terminals:

### 1) Start simulator

```bash
source install/setup.bash
ros2 run uav_sim uav_sim
```

### 2) Start planner

```bash
source install/setup.bash
ros2 run uav_planner uav_planner
```

---

## Scenarios

Suggested scenarios to validate behaviour:

* increasing headwind during outbound leg
* gust bursts near the end of the mission
* biased SoC estimate (battery uncertainty)
* sudden energy drain event (fault injection)

---

## Evaluation Metrics

* **Mission success rate** (goal reached and safe return)
* **Safe-return rate** (RTH triggered in time)
* **Failure rate** (battery depletion before home)
* **Abort efficiency** (how early/late RTH is triggered)
* **Sensitivity to** <img alt="tau" src="https://latex.codecogs.com/svg.image?\tau" /> (risk tolerance sweep)

---

## Extensions

* Monte Carlo return prediction under wind and battery uncertainty
* adaptive <img alt="tau" src="https://latex.codecogs.com/svg.image?\tau" /> based on mission criticality
* integration with trajectory optimization (energy-aware paths)
* sim-to-real validation with real battery models and wind estimation

---

## Author

**Panagiota Grosdouli**

---

## License

MIT License
