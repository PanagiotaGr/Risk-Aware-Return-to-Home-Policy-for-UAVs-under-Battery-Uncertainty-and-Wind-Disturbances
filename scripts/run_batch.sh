#!/usr/bin/env bash
set -e

N=30
T=15   # seconds per run

# Example sweep: different z_delta values
Z_LIST=(0.0 0.5 1.0 1.645 2.0 2.5)

# Ablation toggles (set to 0 for off)
GUST_PROB=0.05
WIND_SIGMA=0.5
MEAS_SIGMA=0.02

source ~/uav_ws/install/setup.bash

for Z in "${Z_LIST[@]}"; do
  echo "=== z_delta=$Z ==="
  for i in $(seq 1 $N); do
    echo "Run $i/$N"

    ros2 run uav_sim uav_sim \
      --ros-args \
      -p gust_prob:=${GUST_PROB} \
      -p wind_sigma:=${WIND_SIGMA} \
      -p meas_sigma:=${MEAS_SIGMA} \
      -p log_dir:=/home/$USER/uav_ws/logs >/dev/null 2>&1 &

    SIM_PID=$!

    ros2 run uav_planner uav_planner \
      --ros-args \
      -p z_delta:=${Z} \
      -p sigma_b:=0.05 >/dev/null 2>&1 &

    PLANNER_PID=$!

    sleep $T

    kill $PLANNER_PID 2>/dev/null || true
    kill $SIM_PID 2>/dev/null || true
    wait $PLANNER_PID 2>/dev/null || true
    wait $SIM_PID 2>/dev/null || true
  done
done

echo "Done."
