# Multirotor Energy Model

**Status:** Research Prototype · Simulation-Only · Real-Flight Validation Required

The energy package replaces a single opaque distance coefficient with a transparent component model. It is intended for controlled comparisons and uncertainty studies, not aircraft certification or platform-specific performance claims.

## Power decomposition

The model computes

\[
P = s\left(P_i + P_0 + P_{par} + P_{climb} + P_{av}\right),
\]

where `s >= 1` is an explicit safety factor.

- Induced hover power uses the ideal momentum-theory scaling
  \(P_i=T^{3/2}/\sqrt{2\rho A}\), corrected by drivetrain efficiency.
- Profile power is represented by a documented quadratic airspeed scaffold.
- Parasitic power uses \(\tfrac12\rho C_D A_r V^3\).
- Climb power uses the potential-energy rate \(mgv_z\); descent applies a conservative recovery factor rather than assuming ideal regeneration.
- Avionics power is a fixed hotel load.

Payload changes induced and climb power through an explicit mass multiplier. Headwind enters the path model by increasing required airspeed for a fixed commanded ground speed. Path energy is integrated segment-by-segment in watt-hours.

## Uncertainty

`relative_uncertainty` produces a component-level standard deviation used by the path model. Independent segment variances are summed. This is a modelling assumption, not a calibrated stochastic error law.

## Limitations

The current implementation does not model rotor inflow transitions, blade-element aerodynamics, motor maps, battery voltage sag, thermal effects, aggressive manoeuvres, ground effect, or vortex-ring-state dynamics. Coefficients are generic unless replaced by measured platform data. Consequently, results must be labelled simulation-only and pending validation.
