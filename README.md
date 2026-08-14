# Robust Landing Control Under Actuator Uncertainty

I designed and evaluated a hybrid landing controller for a simulated rocket
subject to actuator degradation, mass uncertainty, and off-nominal approach
states.

The controller combines finite-model constrained MPC, ancillary LQR feedback,
nonlinear recovery supervision, and dedicated touchdown logic. In the final
evaluation, it achieved successful two-leg landings in all three withheld
malfunction scenarios.

This repository is a sanitized technical case study. Course-provided software,
exact controller constants, test scenarios, and runnable submission code are
intentionally omitted.

## Problem

The task was to control a simulated rocket descending toward a moving landing target. A nominal baseline controller could handle easier descent cases, but degraded actuator authority and larger initial deviations exposed a lack of robustness.

The main control challenge was to combine:

- recovery behavior when the vehicle was outside the local model region,
- constrained landing control near the target,
- robustness against actuator and model uncertainty,
- and careful low-altitude touchdown behavior.

## My Contribution

I was responsible for the controller design, implementation, tuning, and
simulation-based evaluation. The main contributions were:

- modeling actuator uncertainty with a finite set of effective input-gain models,
- implementing a shared-input constrained MPC across those models,
- combining MPC with local LQR correction and nonlinear recovery supervision,
- and diagnosing the terminal drop failure that motivated the touchdown and
  contact-settle logic.

## Controller Architecture

The final design used a hybrid control architecture:

- **Recovery supervision** outside the local landing corridor, where the linearized model is not reliable.
- **Robust MPC** inside the local corridor, planning constrained actions over a finite set of plausible actuator-effectiveness models.
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

## Evaluation

**Achieved two-leg landings in all three withheld malfunction tests, with the
fastest end-to-end evaluation among 124 submissions achieving the same
full-credit outcome.**

End-to-end evaluation refers to execution of the complete evaluation notebook,
not an individual MPC solve. This is empirical validation on the project
benchmark, not a global robustness guarantee for all nonlinear landing states.

## Public Architecture Reference

The executable
[supervisor reference](src/hybrid_landing_supervisor_reference.py) demonstrates
the software boundary around the controller. It implements:

- mode selection between recovery, local MPC, terminal, and contact-settle behavior,
- hysteresis before the local predictive policy receives control authority,
- fallback when the predictive policy cannot provide a valid command,
- persistent touchdown commitment and post-contact command suppression,
- and normalized actuator saturation.

Recovery, MPC/LQR, and terminal policies are injected through generic
interfaces. The reference therefore tests the hybrid architecture without
reproducing the course simulator, optimization problem, controller gains, or
submitted APIs. The actual MPC design is documented separately in the
[control formulation](docs/control_formulation.md).

Run the dependency-free supervisor tests with:

```bash
python3 -m unittest discover -s tests -v
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
├── src/
│   └── hybrid_landing_supervisor_reference.py
└── tests/
    └── test_hybrid_landing_supervisor.py
```

## Limitations

This was a simulator project, not a flight-certified control system.

The public version omits the simulator, exact validation cases, controller constants, model matrices, assignment files, and runnable submission code. The reported behavior should be understood as evidence from a controlled simulation study, not as a formal guarantee for a physical vehicle.

## Disclaimer

This repository is intended for portfolio review only. It does not contain the submitted controller and cannot be used as a drop-in solution for the original course project.
