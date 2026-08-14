# Control Formulation

This note records the sanitized control structure. Exact models, constants, and
simulator interfaces are omitted.

## State And Input

The reduced state contains target-relative position and velocity, attitude and
angular rate, and contact state. Inputs represent main thrust, lateral thrust,
and gimbal correction.

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

Every model receives the same input sequence and must satisfy the selected
constraints. Input and input-change penalties limit effort and chatter. The
first input is applied before replanning at the next sample.

The local model and hover input use the current vehicle mass. Online
effectiveness estimates adjust hover trim and actuator bounds.

## Ancillary LQR Feedback

A discrete LQR gain from the local model is used only inside a tighter envelope
where linear feedback is credible.

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

The MPC input remains the main command; LQR supplies only a bounded correction.
This follows $u_k=v_k+Lx_k$ but is not tube MPC: no invariant error tube is
propagated.

## Supervisor

The supervisor uses recovery outside the local corridor, MPC plus LQR inside
it, terminal logic near the deck, and bounded settling after contact. This keeps
the linear model away from large-error and contact-dominated regimes.

## Public Scope

Exact matrices, tuning values, actuator limits, supervisor thresholds,
validation cases, and simulator interfaces are intentionally omitted. See the
[repository disclosure](../DISCLAIMER.md) for the public/private boundary.
