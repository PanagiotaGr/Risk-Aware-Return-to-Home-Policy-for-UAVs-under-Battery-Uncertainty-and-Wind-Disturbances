# Mathematical Formulation

## Research question

How can a UAV decide when to return home under uncertain battery state, stochastic wind disturbances, and safety-critical mission constraints?

## State and uncertainty

At decision time `t`, the simulator represents the UAV state as

```math
x_t = (p_t, v_t, \widehat{SoC}_t),
```

where `p_t ∈ R^2` is position, `v_t ∈ R^2` is velocity, and `\widehat{SoC}_t` is the estimated battery state-of-charge. The home location is `p_H`; the mission target is `p_G`.

Battery uncertainty is represented by a random variable

```math
SoC_t \sim \mathcal{N}(\widehat{SoC}_t, \sigma_{SoC}^2),
```

clipped to `[0, 1]`. This is a prototype estimator model, not a validated electrochemical battery filter.

Wind is represented as

```math
w_t \sim \mathcal{N}(\mu_w(t), \Sigma_w),
```

where `\mu_w(t)` may include a constant component and a sinusoidal gust component.

## Safe-return probability

Let `E_required(p_t, p_H, w_t)` be the wind-adjusted energy required to return home and `E_available(SoC_t)` be the energy available after preserving a safety reserve. The safe-return probability is

```math
P_{safe}(t) = P(E_{required}(p_t, p_H, w_t) < E_{available}(SoC_t) \mid x_t, \widehat{SoC}_t, w_t).
```

The Monte Carlo estimator samples battery and wind states and estimates

```math
\widehat{P}_{safe}(t) = \frac{1}{N}\sum_{i=1}^{N}\mathbf{1}[E_{required}^{(i)} < E_{available}^{(i)}].
```

A binomial normal-approximation confidence interval is reported for diagnostics.

## Decision rule

The implemented risk-aware policy triggers return-to-home if

```math
\widehat{P}_{safe}(t) < \tau.
```

A conservative threshold uses larger `\tau`, requiring high probability of safe return before continuing the mission. An aggressive threshold uses smaller `\tau`, allowing more mission progress but accepting a higher estimated risk that return may become infeasible.

## Why deterministic thresholds are insufficient

A fixed SoC threshold cannot distinguish between an easy return under tailwind and a difficult return under headwind, nor can it express the uncertainty introduced by noisy SoC estimation. The probabilistic rule explicitly couples return distance, wind, battery reserve, and uncertainty. Its current limitation is that the energy model is simplified and should not be interpreted as platform-validated flight physics.
