# Control Formulation

This note describes the public, sanitized controller structure. It is not the
original submission and cannot be run in the course simulator.

## State And Input

The controller works with a reduced landing state containing:

- lateral and vertical error to the target,
- lateral and vertical velocity,
- attitude error and angular rate,
- a contact flag near touchdown.

The input is represented generically as:

- main thrust,
- lateral correction,
- thrust-vector or gimbal correction.

Exact state ordering, actuator limits, and simulator conventions are omitted.

## Robust MPC Layer

Inside a local landing corridor, the controller uses constrained finite-horizon
predictive control. Instead of optimizing only for one nominal model, it plans
against a small finite model set representing plausible uncertainty in mass and
actuator effectiveness.

Conceptually, the optimization penalizes:

- predicted state error over the horizon,
- control effort,
- terminal landing error,
- violation of actuator and local safety limits.

The first input is applied and the problem is solved again at the next time
step. This is the receding-horizon part of the controller.

## Local Feedback

The MPC command is corrected with a small clipped linear feedback term:

```text
u_k = v_k + L x_k
```

Here `v_k` is the planned predictive-control input and `L x_k` is a stabilizing
local correction. The correction is intentionally limited so it cannot override
the actuator constraints handled by the predictive controller.

## Supervisor

The supervisor decides whether the linear model is credible:

- far from the landing corridor: use recovery control,
- inside the corridor: use robust MPC plus local feedback,
- near the deck: use terminal touchdown logic,
- after contact: suppress aggressive lateral/gimbal commands.

The terminal layer is important because low-altitude contact behavior is not
well represented by a linear hover model. It prevents early shutdown while the
vehicle still has too much lateral, vertical, or attitude error.

## What Is Intentionally Omitted

The public version does not include:

- exact matrices,
- exact weighting values,
- exact actuator limits,
- exact supervisor thresholds,
- exact validation cases,
- simulator imports,
- the submitted controller file or notebook.

Those details are omitted to avoid redistributing course material or publishing
a reusable solution.
