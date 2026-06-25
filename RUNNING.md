# Running the UAV RTH simulator

This repository now includes a dependency-light Python path for validating the risk-aware return-to-home logic without requiring a full ROS 2 Jazzy installation.

## Local Python demo

```bash
python -m unittest discover -s tests
python run_experiment.py --threshold 0.70 --battery-wh 80 --wind 3 --samples 800 --csv trace.csv
```

The simulation estimates `P(safe return)` by Monte Carlo sampling battery state-of-charge and wind conditions. RTH is triggered when the estimated probability drops below the selected risk threshold.

## ROS 2 path

The README still describes the intended ROS 2 architecture. The Python modules here are separated so they can be wrapped later by ROS 2 nodes (`uav_sim`, `uav_planner`, `uav_interfaces`) without duplicating the core decision logic.
