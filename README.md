# Risk-Aware Return-to-Home Policy for UAVs under Battery Uncertainty and Wind Disturbances

ROS 2 (Jazzy) simulation of a UAV executing a mission with a risk-aware
Return-to-Home (RTH) policy under battery uncertainty and wind disturbances.

## Packages
- uav_sim: UAV simulation with wind and battery model
- uav_planner: Risk-aware RTH decision policy
- uav_interfaces: Custom ROS 2 messages

## Run
```bash
colcon build
source install/setup.bash
ros2 run uav_sim uav_sim
ros2 run uav_planner uav_planner
