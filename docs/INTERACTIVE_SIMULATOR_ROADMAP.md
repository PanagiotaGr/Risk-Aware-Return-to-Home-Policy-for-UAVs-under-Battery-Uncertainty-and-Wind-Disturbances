# Interactive Mission Simulator Roadmap

This upgrade introduces the first implementation layer for an interactive 2D UAV mission simulator.

## Implemented in this step

```text
risk_rth_platform/simulation/
├── geometry.py   # 2D points and geometry helpers
├── world.py      # map boundaries, no-fly zones, landing sites
├── wind.py       # spatially varying wind and gust model
├── uav.py        # lightweight UAV kinematics and energy usage
├── mission.py    # waypoint mission progress
└── engine.py     # simulation loop connected to risk-aware decisions
```

## Scenario file

```text
configs/platform/world_urban_mission.json
```

The scenario contains:

- map size,
- home position,
- waypoints,
- no-fly zones,
- emergency landing sites,
- wind field parameters,
- UAV parameters,
- link to the risk scenario configuration.

## Run

```bash
python3 scripts/run_2d_mission_simulation.py
```

Output:

```text
results_platform/mission_2d_telemetry.json
```

## Why this matters scientifically

The previous benchmark layer evaluates the risk-aware decision policy numerically. The new 2D mission simulation layer adds spatial context: position, distance to home, no-fly-zone violations, wind variation over space, and emergency landing sites. This is a necessary step toward more realistic UAV autonomy experiments, because safety is not only a battery-threshold problem; it also depends on the geometry and constraints of the operating environment.

## Next steps

1. Add plotting for trajectory, wind vectors, no-fly zones, and landing sites.
2. Add a browser dashboard that reads `mission_2d_telemetry.json`.
3. Add more world scenarios: coastal, mountain, forest, high-wind, and degraded-battery.
4. Add path planning around no-fly zones.
5. Connect telemetry to ROS 2 topics for RViz visualization.
