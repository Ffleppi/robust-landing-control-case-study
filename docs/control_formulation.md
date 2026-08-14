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

## Finite-Model Robust MPC

At each MPC update, one input sequence is optimized for a finite set of
actuator-effectiveness scenarios. Every scenario starts from the measured state
and receives the same input sequence, but predicts a different trajectory:

$$
x_{j+1}^{(s)} =
A_d x_j^{(s)} +
B_d\,\mathrm{diag}(\eta^{(s)})
\left(v_j-\bar{u}^{(s)}\right),
\qquad s\in\mathcal S.
$$

The optimization problem has the structure

$$
\begin{aligned}
\min_{\{v_j\}}\quad
&\sum_{s\in\mathcal S} w_s
\left[
\sum_{j=0}^{N-1}
\left\|x_j^{(s)}-x_{j,\mathrm{ref}}\right\|_Q^2
+
\left\|x_N^{(s)}-x_{N,\mathrm{ref}}\right\|_{Q_N}^2
\right] \\
&+
\sum_{j=0}^{N-1}\left\|v_j-\hat{u}_{\mathrm{hover}}\right\|_R^2
+
\sum_{j=1}^{N-1}\left\|v_j-v_{j-1}\right\|_{R_\Delta}^2
\end{aligned}
$$

subject to the actuator limits and selected vertical-speed and attitude
constraints holding for every scenario.

The shared input sequence prevents the optimizer from selecting a different
solution for each assumed actuator condition. The input-change penalty reduces
command chatter, while the scenario constraints require the planned trajectory
to remain acceptable across the complete finite model set.

The local model and nominal hover input are rebuilt using the current vehicle
mass. Online actuator-effectiveness estimates subsequently adapt the hover trim
and available actuator bounds. The first optimized input is applied before the
problem is solved again, giving the controller its receding-horizon behavior.

## Ancillary LQR Feedback

A discrete LQR gain is computed from the same local linear model by solving the
discrete algebraic Riccati equation. It is only used inside a tighter local
envelope where linear feedback is credible.

The implemented action can be summarized as

$$
u_k =
\mathrm{sat}\!\left(
v_0^\star +
\alpha\,\mathrm{clip}\!\left(
u_{\mathrm{LQR}}(x_k,v_0^\star)-\hat{u}_{\mathrm{hover}}
\right)
\right).
$$

The MPC input $v_0^\star$ therefore remains the main command. The LQR supplies
only a bounded local correction and cannot take over the gross trajectory.

This follows the feedback-MPC idea $u_k=v_k+Lx_k$, but it is not tube MPC: the
submitted implementation does not propagate a robust invariant error tube. The
robustness mechanism is the shared-input finite-model MPC combined with the
hybrid supervisor.

## Supervisor

The supervisor decides whether the linear model is credible:

- far from the landing corridor: use recovery control,
- inside the corridor: use robust MPC plus local feedback,
- near the deck: use terminal touchdown logic,
- after contact: suppress aggressive lateral/gimbal commands.

The terminal layer is important because low-altitude contact behavior is not
well represented by a linear hover model. It prevents early shutdown while the
vehicle still has too much lateral, vertical, or attitude error.

## Public Scope

Exact matrices, tuning values, actuator limits, supervisor thresholds,
validation cases, and simulator interfaces are intentionally omitted. See the
[repository disclosure](../DISCLAIMER.md) for the public/private boundary.
