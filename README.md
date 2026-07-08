# Robust Landing Control Under Actuator Uncertainty

Sanitized public case study of a university rocket-landing control project.

The original project focused on designing a controller for a simulated rocket landing task under actuator uncertainty, degraded control authority, and off-nominal approach states. This public version is not a runnable solution. It is a portfolio artifact that documents the controller architecture, design reasoning, and qualitative results while omitting course-provided material.

Course framework code, simulator code, notebooks, assignment files, validation scripts, exact constants, scenario parameters, and runnable submission code are intentionally excluded.

## Problem

The task was to control a simulated rocket descending toward a moving landing target. A nominal baseline controller could handle easier descent cases, but degraded actuator authority and larger initial deviations exposed a lack of robustness.

The main control challenge was to combine:

- recovery behavior when the vehicle was outside the local model region,
- constrained landing control near the target,
- robustness against actuator and model uncertainty,
- and careful low-altitude touchdown behavior.

## Controller Architecture

The final design used a hybrid control architecture:

- **Recovery supervision** outside the local landing corridor, where the linearized model is not reliable.
- **Robust MPC** inside the local corridor, planning constrained actions over a finite set of plausible actuator/mass models.
- **Local LQR feedback** around the predictive command, using the structure `u_k = v_k + L x_k`.
- **Terminal touchdown logic** for low-altitude braking and contact-sensitive behavior.
- **Contact-settle logic** after touchdown to avoid aggressive lateral or gimbal commands.

![Sanitized controller architecture](assets/architecture_sanitized.png)

The controller is structured as a hybrid supervisor. Large off-nominal states
are first handled by a recovery mode. Once the vehicle enters the local landing
corridor, robust MPC is used over a finite model set, with an LQR correction
around the planned input. Close to touchdown, the controller switches to a
terminal mode with braking and contact-settle logic.

## Representative Result

In representative malfunction cases, the baseline controller drifted away from the target, while the hybrid controller recovered into the landing region. The plot below is anonymized and non-reproducible; it is intended to communicate the engineering result without exposing assignment-specific parameters or solution code.

![Anonymized before/after result](assets/before_after_sanitized.png)

## What I Implemented

The public reference implementation is here:

```text
src/hybrid_landing_controller_reference.py
```

It is a rewritten, non-runnable architecture reference. It shows the structure of the controller without using course APIs, exact thresholds, exact model matrices, validation cases, or submitted class/function names.

The implementation illustrates:

- mode selection between recovery, local MPC, terminal, and contact-settle behavior,
- robust MPC over a finite model set,
- clipped LQR-style correction around the planned input,
- actuator saturation and command smoothing,
- terminal touchdown readiness checks,
- and post-contact command suppression.

For a higher-level explanation of the control formulation, see:

```text
docs/control_formulation.md
```

## Skills Demonstrated

- Robust model predictive control
- Local LQR feedback design
- Hybrid control architecture
- Constraint-aware control logic
- Recovery behavior outside a nominal model region
- Simulation-based controller evaluation
- Python implementation of control architecture
- Safety-oriented controller design

## Repository Structure

```text
.
├── README.md
├── DISCLAIMER.md
├── assets/
│   ├── architecture_sanitized.png
│   └── before_after_sanitized.png
├── docs/
│   └── control_formulation.md
└── src/
    └── hybrid_landing_controller_reference.py
```

## Limitations

This was a simulator project, not a flight-certified control system.

The public version omits the simulator, exact validation cases, controller constants, model matrices, assignment files, and runnable submission code. The reported behavior should be understood as evidence from a controlled simulation study, not as a formal guarantee for a physical vehicle.

## Disclaimer

This repository is intended for portfolio review only. It does not contain the submitted controller and cannot be used as a drop-in solution for the original course project.
