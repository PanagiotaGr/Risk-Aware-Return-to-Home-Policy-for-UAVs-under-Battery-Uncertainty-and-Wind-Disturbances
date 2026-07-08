# Monte Carlo Risk Estimation

The estimator computes a simulation estimate of safe-return probability:

```math
\widehat{P}_{safe}=\frac{1}{N}\sum_i \mathbf{1}[E^{(i)}_{required}<E^{(i)}_{available}].
```

## Implemented

- Battery SoC sampling.
- Wind sampling.
- Wind-adjusted return-energy samples.
- Binomial normal-approximation confidence interval.

## Engineering benefit

Monte Carlo estimation keeps the decision rule transparent and easy to test. It also exposes the trade-off between sample count and runtime.

## Scientific limitation

The estimator is only as valid as the assumed uncertainty distributions and energy model. Calibration against flight data is planned, not implemented.
