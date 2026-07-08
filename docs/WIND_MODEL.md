# Wind Model

## Implemented

The wind model supports constant wind, Gaussian stochastic wind samples, and a sinusoidal gust component for controlled experiments.

```math
w_t \sim \mathcal{N}(\mu_w(t), \sigma_w^2 I).
```

Headwind and crosswind affect return-energy estimates through the energy model.

## Prototype

The gust model is a simple deterministic time-varying disturbance used to test policy sensitivity.

## Planned

- Spatial wind fields.
- Online wind estimation.
- Turbulence models.
- Use of real wind logs.

## Limitation

The current model is intentionally lightweight and should not be interpreted as a validated atmospheric model.
