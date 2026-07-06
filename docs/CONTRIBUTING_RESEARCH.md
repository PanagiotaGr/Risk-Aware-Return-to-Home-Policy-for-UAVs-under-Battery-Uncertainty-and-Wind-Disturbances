# Research Contribution Guidelines

## Purpose

This repository is intended to grow as a doctoral research code base. Contributions should improve scientific clarity, reproducibility, safety interpretation or software quality.

## Contribution Types

Accepted contribution types:

- new uncertainty models,
- new RTH or safety-supervision policies,
- new baselines,
- new experiment scripts,
- statistical analysis utilities,
- ROS 2 / simulator integration,
- documentation that improves scientific interpretation,
- tests and reproducibility improvements.

## Scientific Requirements for New Policies

A new policy should include:

1. mathematical motivation,
2. implementation in a modular policy file,
3. at least one deterministic baseline comparison,
4. documented parameters,
5. experiment script or integration into an existing experiment,
6. CSV output,
7. interpretation of failure cases,
8. unit tests where practical.

## Coding Principles

- Keep models, policies, experiments and plotting separate.
- Avoid hard-coded experiment constants inside policy logic.
- Prefer explicit parameter objects or configuration dictionaries.
- Make randomness controllable through seeds.
- Keep safety-related assumptions visible in the code.
- Do not claim real-flight safety from simulation alone.

## Documentation Requirements

Every substantial research feature should update at least one of:

- `docs/THEORETICAL_FRAMEWORK.md`,
- `docs/EVALUATION_MATRIX.md`,
- `docs/REPRODUCIBILITY_PROTOCOL.md`,
- `docs/PUBLICATION_PLAN.md`,
- `docs/ROADMAP.md`.

## Pull Request Checklist

Before merging a research PR:

- [ ] The scientific claim is stated clearly.
- [ ] The code is modular.
- [ ] The experiment can be reproduced.
- [ ] Results are saved in machine-readable form.
- [ ] A baseline is included.
- [ ] Limitations are documented.
- [ ] Safety claims are not overstated.
- [ ] README or docs are updated.

## Safety Language

Use careful language. Prefer:

- "improves simulated safe-return behavior under the tested assumptions"
- "reduces empirical failure rate in Monte Carlo evaluation"
- "supports risk-aware decision making"

Avoid unsupported claims such as:

- "guarantees safety"
- "certified autonomous failsafe"
- "ready for real flight"

unless independent certification and hardware validation exist.
