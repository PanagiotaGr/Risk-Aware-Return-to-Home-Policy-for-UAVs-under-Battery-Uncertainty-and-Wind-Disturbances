# System Architecture

```text
configs/*.yaml
    ↓
scripts/run_experiment.py
    ↓
risk_rth.simulation.MissionSimulator2D
    ↓
BatteryModel + WindModel + EnergyModel
    ↓
RTH policy
    ↓
MonteCarloRiskEstimator
    ↓
SimulationResult + metrics + figures
```

## Package layout

- `risk_rth/models`: state, battery, wind, and energy models.
- `risk_rth/simulation`: 2D mission loop and future 3D scaffold location.
- `risk_rth/planning`: deterministic and risk-aware policies.
- `risk_rth/uncertainty`: Monte Carlo risk estimation.
- `risk_rth/evaluation`: metrics and comparison utilities.
- `risk_rth/visualization`: figure generation.
- `risk_rth/utils`: configuration and reproducibility helpers.

## Engineering motivation

All important logic lives in Python modules rather than notebooks. Scripts should only orchestrate experiments and visualization.

## Scientific motivation

The structure separates models, uncertainty, policies, and evaluation so claims can be traced to assumptions and generated artifacts.
