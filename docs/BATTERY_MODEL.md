# Battery Model

## Implemented

The implemented model treats battery state-of-charge as a scalar in `[0, 1]`. Available return energy is computed as usable SoC after a configurable reserve, multiplied by nominal capacity and battery health.

```math
E_{available} = \max(SoC - SoC_{reserve}, 0) C h_b.
```

SoC uncertainty is represented by clipped Gaussian samples around the estimated SoC.

## Prototype

Battery health appears as a capacity multiplier. This supports controlled degradation studies but is not a physics-based aging model.

## Planned

- Temperature-dependent usable capacity.
- Online SoC filtering.
- Battery health estimation from logs.
- Validation against real UAV power data.

## Limitation

This model is for decision-policy research and reproducible simulation, not for certifying real battery safety.
