"""Sanitized reference architecture for a robust landing controller.

This file is intentionally not a runnable solution for any course simulator.
It documents the control structure used in the portfolio case study without
including assignment-specific APIs, exact constants, validation scenarios, or
drop-in class names.

The design follows a common control engineering pattern:

1. Use a constrained model-predictive controller while the vehicle is inside a
   local region where the linearized model is credible.
2. Add a small stabilizing feedback correction around the planned MPC input.
3. Use a recovery supervisor outside that local region.
4. Use separate terminal logic near contact, where the flight model is no
   longer a good description of the physics.

The names and thresholds below are generic. They are meant to communicate the
architecture, not reproduce a submitted controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Protocol


Vector = tuple[float, ...]


class Mode(Enum):
    """High-level supervisor modes."""

    RECOVERY = "recovery"
    LOCAL_MPC = "local_mpc"
    TERMINAL = "terminal"
    CONTACT_SETTLE = "contact_settle"


@dataclass(frozen=True)
class LandingState:
    """Generic reduced state used by the supervisor.

    A real implementation would map from the simulator or vehicle estimator into
    this representation. The public version avoids exposing the original state
    ordering or exact normalization.
    """

    lateral_error: float
    vertical_error: float
    lateral_velocity: float
    vertical_velocity: float
    attitude_error: float
    angular_rate: float
    has_contact: bool = False


@dataclass(frozen=True)
class LandingCommand:
    """Generic actuator command.

    Values are normalized. The public version does not include exact actuator
    limits or the simulator's original control convention.
    """

    main: float
    lateral: float
    gimbal: float


@dataclass(frozen=True)
class LinearModel:
    """One model in the finite uncertainty set used by robust MPC."""

    name: str
    state_matrix: object
    input_matrix: object
    input_scale: Vector


@dataclass(frozen=True)
class ControllerTuning:
    """Tuning parameters exposed at architecture level only.

    The exact numerical values are omitted intentionally. In a real controller,
    these parameters define the MPC horizon, tracking weights, uncertainty set,
    local model validity region, recovery aggressiveness, and terminal touchdown
    tolerances.
    """

    horizon_steps: int
    model_set: tuple[LinearModel, ...]
    local_corridor_radius: float
    terminal_altitude: float
    touchdown_speed_limit: float
    attitude_limit: float
    command_smoothing: float


class MpcSolver(Protocol):
    """Solver interface used by the reference architecture."""

    def solve(self, state: LandingState, models: Iterable[LinearModel]) -> LandingCommand:
        """Return the first receding-horizon input for the finite model set."""


class LqrFeedback(Protocol):
    """Local feedback correction around the planned MPC input."""

    def correction(self, state: LandingState) -> LandingCommand:
        """Return a small clipped correction term."""


class HybridLandingController:
    """Sanitized hybrid robust landing controller.

    The controller is deliberately split into explainable pieces:

    - ``RECOVERY`` captures states that are too far from the local linear model.
    - ``LOCAL_MPC`` handles constrained tracking in the credible model region.
    - ``TERMINAL`` protects the final descent by delaying touchdown if lateral
      motion, vertical speed, or attitude are not ready.
    - ``CONTACT_SETTLE`` reduces aggressive lateral/gimbal action after contact.

    The MPC path uses a finite set of degraded models. This is a pragmatic
    robust-MPC approximation: optimize one command sequence against several
    plausible models instead of trusting a single nominal linearization.
    """

    def __init__(
        self,
        tuning: ControllerTuning,
        mpc_solver: MpcSolver,
        lqr_feedback: LqrFeedback,
    ) -> None:
        self.tuning = tuning
        self.mpc_solver = mpc_solver
        self.lqr_feedback = lqr_feedback
        self.mode = Mode.RECOVERY
        self._touchdown_committed = False

    def command(self, state: LandingState) -> LandingCommand:
        """Choose and evaluate the active control law."""

        self.mode = self._select_mode(state)

        if self.mode is Mode.CONTACT_SETTLE:
            return self._contact_settle_command(state)
        if self.mode is Mode.TERMINAL:
            return self._terminal_command(state)
        if self.mode is Mode.LOCAL_MPC:
            return self._mpc_with_local_feedback(state)
        return self._recovery_command(state)

    def _select_mode(self, state: LandingState) -> Mode:
        """Supervisor logic based on model validity and touchdown readiness."""

        if state.has_contact:
            return Mode.CONTACT_SETTLE

        if self._near_terminal_region(state):
            return Mode.TERMINAL

        if self._inside_local_corridor(state):
            return Mode.LOCAL_MPC

        return Mode.RECOVERY

    def _mpc_with_local_feedback(self, state: LandingState) -> LandingCommand:
        """Robust MPC command plus clipped local feedback correction.

        In course notation this has the structure ``u_k = v_k + L x_k``:
        the optimizer chooses the nominal sequence ``v_k`` and the feedback term
        compensates for small deviations between replanning steps.
        """

        planned = self.mpc_solver.solve(state, self.tuning.model_set)
        correction = self.lqr_feedback.correction(state)
        return self._clip_and_smooth(
            LandingCommand(
                main=planned.main + correction.main,
                lateral=planned.lateral + correction.lateral,
                gimbal=planned.gimbal + correction.gimbal,
            )
        )

    def _recovery_command(self, state: LandingState) -> LandingCommand:
        """Broad nonlinear capture law outside the MPC validity region.

        This layer is intentionally simple and conservative. Its job is not to
        optimize the final landing, but to bring the state back into the region
        where the constrained model-based controller is trustworthy.
        """

        main = self._vertical_capture_thrust(state)
        lateral = self._lateral_capture_thrust(state)
        gimbal = self._attitude_capture_gimbal(state)
        return self._clip_and_smooth(LandingCommand(main, lateral, gimbal))

    def _terminal_command(self, state: LandingState) -> LandingCommand:
        """Low-altitude governor for touchdown preparation.

        The terminal layer reduces the failure mode where the vehicle appears
        stable above the target, shuts down too early, and then drops onto the
        deck. It keeps braking until vertical speed, lateral motion, and attitude
        are all inside a touchdown-ready envelope.
        """

        ready = (
            abs(state.vertical_velocity) <= self.tuning.touchdown_speed_limit
            and abs(state.attitude_error) <= self.tuning.attitude_limit
            and abs(state.lateral_velocity) <= self.tuning.touchdown_speed_limit
        )

        if ready:
            self._touchdown_committed = True

        if self._touchdown_committed:
            return self._clip_and_smooth(
                LandingCommand(main=self._gentle_descent_thrust(state), lateral=0.0, gimbal=0.0)
            )

        return self._clip_and_smooth(
            LandingCommand(
                main=self._hover_or_brake_thrust(state),
                lateral=self._lateral_capture_thrust(state),
                gimbal=self._attitude_capture_gimbal(state),
            )
        )

    def _contact_settle_command(self, state: LandingState) -> LandingCommand:
        """Suppress aggressive commands once contact has occurred."""

        if self._settled_after_contact(state):
            return LandingCommand(main=0.0, lateral=0.0, gimbal=0.0)
        return self._clip_and_smooth(LandingCommand(main=0.0, lateral=0.0, gimbal=0.0))

    def _inside_local_corridor(self, state: LandingState) -> bool:
        return (
            abs(state.lateral_error) <= self.tuning.local_corridor_radius
            and abs(state.attitude_error) <= self.tuning.attitude_limit
        )

    def _near_terminal_region(self, state: LandingState) -> bool:
        return state.vertical_error <= self.tuning.terminal_altitude

    def _settled_after_contact(self, state: LandingState) -> bool:
        return (
            abs(state.vertical_velocity) <= self.tuning.touchdown_speed_limit
            and abs(state.lateral_velocity) <= self.tuning.touchdown_speed_limit
            and abs(state.attitude_error) <= self.tuning.attitude_limit
        )

    def _clip_and_smooth(self, command: LandingCommand) -> LandingCommand:
        """Apply normalized actuator saturation and command smoothing."""

        return LandingCommand(
            main=self._clip(command.main, 0.0, 1.0),
            lateral=self._clip(command.lateral, -1.0, 1.0),
            gimbal=self._clip(command.gimbal, -1.0, 1.0),
        )

    @staticmethod
    def _clip(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _vertical_capture_thrust(self, state: LandingState) -> float:
        return 0.5 - 0.1 * state.vertical_velocity + 0.05 * state.vertical_error

    def _lateral_capture_thrust(self, state: LandingState) -> float:
        return -0.1 * state.lateral_error - 0.2 * state.lateral_velocity

    def _attitude_capture_gimbal(self, state: LandingState) -> float:
        return -0.2 * state.attitude_error - 0.1 * state.angular_rate

    def _hover_or_brake_thrust(self, state: LandingState) -> float:
        return 0.5 - 0.2 * state.vertical_velocity

    def _gentle_descent_thrust(self, state: LandingState) -> float:
        return 0.45 - 0.05 * state.vertical_velocity
