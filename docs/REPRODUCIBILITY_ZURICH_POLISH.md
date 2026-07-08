# Zurich-Style Research Polish Notes

This note records the repository-presentation changes in the current phase.

## Why this phase is needed

A strong robotics research repository should communicate the method visually and reproducibly. The README and website should make the research question clear within seconds, while all numerical outputs remain tied to generated artifacts.

## Scientific motivation

The project studies a probabilistic RTH decision rule, so the demo should show more than a path: it should expose the safety estimate, threshold, battery state, wind disturbance, and RTH trigger.

## Engineering motivation

All visuals should be generated from code so they can be regenerated after model or policy changes. This avoids manually edited figures that silently drift away from the implementation.

## Expected benefit

- Clearer first impression for reviewers and MSc/PhD applications.
- Better separation between demonstration artifacts and experimental evidence.
- More maintainable project-page and README visuals.

## Limitations

The animation remains a controlled simulation visualization. It does not validate real UAV safety, battery physics, or aerodynamic performance.
