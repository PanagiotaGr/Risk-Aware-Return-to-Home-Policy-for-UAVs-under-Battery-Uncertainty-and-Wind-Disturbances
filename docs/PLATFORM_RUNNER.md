# Platform Runner

This update begins the transition from demonstration code toward a reusable research platform.

## Added Modules

- `risk_rth_platform.statistics`: statistical helper functions.
- `risk_rth_platform.benchmark`: repeated trial execution helpers.

## Research Benefit

The runner separates repeated trial execution from simulator internals. Future policies can be compared through the same benchmark interface.

## Current Capabilities

- repeated simulation trials,
- deterministic trial offsets,
- rate estimation,
- interval reporting,
- unit tests for the new utility layer.

## Next Engineering Step

The next upgrade should connect the CLI to this benchmark layer and export aggregate summaries plus per-trial records.
