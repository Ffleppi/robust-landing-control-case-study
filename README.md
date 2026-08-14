# Robust Landing Control Under Actuator Uncertainty

I designed and evaluated a hybrid landing controller for a simulated rocket
subject to actuator degradation, mass uncertainty, and off-nominal approach
states. The design combines constrained finite-model MPC, local LQR feedback,
recovery supervision, and dedicated touchdown logic.

## Approach

- A shared-input MPC plans against a finite set of actuator-effectiveness
  models while enforcing local state and actuator constraints.
- Online effectiveness estimates adjust hover trim and available control
  authority; clipped LQR feedback supplies a small local correction.
- A hybrid supervisor handles recovery outside the local model region and
  switches to braking, touchdown gating, and contact settling near the deck.

![Sanitized controller architecture](assets/architecture_sanitized.png)

## Results

**All three withheld malfunction tests ended in two-leg landings. Among 124
full-credit submissions, this controller had the fastest end-to-end evaluation
(191 submissions overall).**

End-to-end evaluation refers to execution of the complete evaluation notebook,
not an individual MPC solve. The recorded trajectories below use simulator
output; the shaded area is the deck and the markers are final positions.

![Recorded controller trajectories](assets/controller_output.png)

## Code Reference

The executable [supervisor reference](src/hybrid_landing_supervisor_reference.py)
captures mode selection, entry hysteresis, solver fallback, touchdown
commitment, and contact-settle handoff behind generic policy interfaces. The
[control formulation](docs/control_formulation.md) documents the sanitized MPC
and LQR structure.

```bash
python3 -m unittest discover -s tests -v
```

## Technical Scope

**Control:** robust MPC, LQR, hybrid supervision, online effectiveness
estimation, and constrained touchdown control.

**Tools:** Python, CVXPY, OSQP, simulation-based validation, and unit testing.

## Scope

This simulator study is not a flight-certified system. Exact models, constants,
test cases, simulator code, and submission code are intentionally omitted; see
the [disclosure boundary](DISCLAIMER.md).
