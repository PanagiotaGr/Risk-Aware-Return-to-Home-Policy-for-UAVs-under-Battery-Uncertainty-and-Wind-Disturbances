# Evaluation Protocol

## Goal

Evaluate whether a policy returns early enough to avoid simulated energy failure while avoiding unnecessary early return.

## Metrics

- `mission_success_rate`: fraction of trials ending safely.
- `unsafe_failure_rate`: fraction of trials exhausting battery before safe landing.
- `early_return_rate`: fraction of trials where RTH was triggered before target completion.
- `mean_remaining_soc`: average SoC at landing or termination.
- `mean_distance_completed_m`: average outbound distance before return or failure.
- Monte Carlo diagnostics: `P_safe`, confidence interval, and sample count.

## Baselines

- Fixed battery threshold.
- Distance-based RTH.
- Deterministic energy estimate.
- Risk-aware Monte Carlo policy.
- Oracle policy placeholder: planned only, because no future-disturbance oracle is implemented.

## Reproducibility

Each experiment must define random seed, number of trials, battery parameters, wind parameters, energy-model parameters, policy name, and policy threshold in YAML. Generated outputs must be stored under `results/` and figures under `results/figures/` or `assets/`.

## Limitations

Current experiments are controlled simulations. They should not be reported as real-world flight safety results. Any paper-style claim must be tied to the exact configuration and generated outputs.
