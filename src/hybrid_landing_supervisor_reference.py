"""Executable, simulator-independent reference for the hybrid supervisor.

This module documents the software boundary around the landing policies without
reproducing the submitted controller. Recovery, predictive control, and
terminal control are injected behind small interfaces. The public code owns
only mode selection, hysteresis, touchdown commitment, fallback behavior, and
normalized actuator saturation.

No course APIs, model matrices, optimizer setup, controller gains, or validation
scenarios appear here.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class Mode(Enum):
    """Operating modes exposed by the supervisor."""

    RECOVERY = "recovery"
    LOCAL_MPC = "local_mpc"
    TERMINAL = "terminal"
    CONTACT_SETTLE = "contact_settle"


@dataclass(frozen=True)
class LandingState:
    """Reduced state used at the public supervisor boundary.

    Positions and velocities are errors relative to the landing target. A real
    integration would map estimator or simulator output into this representation.
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
    """Normalized main-thrust, lateral-thrust, and gimbal commands."""

    main: float
    lateral: float
    gimbal: float

    @classmethod
    def zero(cls) -> LandingCommand:
        """Return a fully suppressed command for established contact."""

        return cls(main=0.0, lateral=0.0, gimbal=0.0)

    def saturated(self) -> LandingCommand:
        """Apply the normalized public actuator contract."""

        return LandingCommand(
            main=_clip(self.main, 0.0, 1.0),
            lateral=_clip(self.lateral, -1.0, 1.0),
            gimbal=_clip(self.gimbal, -1.0, 1.0),
        )


@dataclass(frozen=True)
class SupervisorConfig:
    """Generic mode-transition limits supplied by an integration.

    The public reference deliberately provides no defaults: callers must choose
    values that match their own vehicle model, units, and safety analysis.
    """

    local_lateral_error_limit: float
    local_lateral_speed_limit: float
    local_attitude_limit: float
    local_angular_rate_limit: float
    terminal_altitude: float
    touchdown_lateral_error_limit: float
    touchdown_lateral_speed_limit: float
    touchdown_vertical_speed_limit: float
    touchdown_attitude_limit: float
    touchdown_angular_rate_limit: float
    mpc_entry_hold_steps: int

    def __post_init__(self) -> None:
        positive_limits = (
            self.local_lateral_error_limit,
            self.local_lateral_speed_limit,
            self.local_attitude_limit,
            self.local_angular_rate_limit,
            self.touchdown_lateral_error_limit,
            self.touchdown_lateral_speed_limit,
            self.touchdown_vertical_speed_limit,
            self.touchdown_attitude_limit,
            self.touchdown_angular_rate_limit,
        )
        if any(limit <= 0.0 for limit in positive_limits):
            raise ValueError("state and touchdown limits must be positive")
        if self.terminal_altitude < 0.0:
            raise ValueError("terminal_altitude must be non-negative")
        if self.mpc_entry_hold_steps < 1:
            raise ValueError("mpc_entry_hold_steps must be at least one")


@dataclass(frozen=True)
class TerminalContext:
    """Persistent information supplied to the terminal policy."""

    touchdown_committed: bool


class ControlPolicy(Protocol):
    """Interface implemented by recovery and local predictive policies."""

    def command(self, state: LandingState) -> LandingCommand:
        """Return one normalized actuator command."""


class TerminalPolicy(Protocol):
    """Interface for low-altitude braking and final descent."""

    def command(
        self,
        state: LandingState,
        context: TerminalContext,
    ) -> LandingCommand:
        """Return a command using the persistent touchdown state."""


class PolicyUnavailable(RuntimeError):
    """Signal that a policy cannot safely provide a command for this state."""


class HybridLandingSupervisor:
    """Coordinate independently supplied landing policies.

    Recovery handles states outside the local model region. Consecutive
    eligible samples are required before the local predictive policy receives
    authority. Low-altitude terminal logic has priority over both flight modes,
    and established contact suppresses all commands.

    A local policy can raise :class:`PolicyUnavailable` to represent solver
    failure or an invalid solution. The supervisor then returns immediately to
    recovery rather than propagating an unusable action.
    """

    def __init__(
        self,
        config: SupervisorConfig,
        recovery_policy: ControlPolicy,
        local_policy: ControlPolicy,
        terminal_policy: TerminalPolicy,
    ) -> None:
        self.config = config
        self.recovery_policy = recovery_policy
        self.local_policy = local_policy
        self.terminal_policy = terminal_policy
        self.reset()

    def reset(self) -> None:
        """Clear episode-specific hysteresis and touchdown state."""

        self.mode = Mode.RECOVERY
        self._eligible_samples = 0
        self._touchdown_committed = False

    @property
    def touchdown_committed(self) -> bool:
        """Whether the terminal gate has accepted the final descent."""

        return self._touchdown_committed

    def command(self, state: LandingState) -> LandingCommand:
        """Select a mode, evaluate its policy, and enforce actuator bounds."""

        if state.has_contact:
            self.mode = Mode.CONTACT_SETTLE
            self._eligible_samples = 0
            return LandingCommand.zero()

        if self._inside_terminal_region(state):
            self.mode = Mode.TERMINAL
            self._eligible_samples = 0
            if self._touchdown_ready(state):
                self._touchdown_committed = True
            context = TerminalContext(
                touchdown_committed=self._touchdown_committed,
            )
            return self.terminal_policy.command(state, context).saturated()

        if not self._inside_local_corridor(state):
            self._eligible_samples = 0
            return self._recovery_command(state)

        self._eligible_samples += 1
        if self._eligible_samples < self.config.mpc_entry_hold_steps:
            return self._recovery_command(state)

        try:
            result = self.local_policy.command(state)
        except PolicyUnavailable:
            self._eligible_samples = 0
            return self._recovery_command(state)

        self.mode = Mode.LOCAL_MPC
        return result.saturated()

    def _recovery_command(self, state: LandingState) -> LandingCommand:
        self.mode = Mode.RECOVERY
        return self.recovery_policy.command(state).saturated()

    def _inside_local_corridor(self, state: LandingState) -> bool:
        return (
            abs(state.lateral_error) <= self.config.local_lateral_error_limit
            and abs(state.lateral_velocity)
            <= self.config.local_lateral_speed_limit
            and abs(state.attitude_error) <= self.config.local_attitude_limit
            and abs(state.angular_rate) <= self.config.local_angular_rate_limit
        )

    def _inside_terminal_region(self, state: LandingState) -> bool:
        return state.vertical_error <= self.config.terminal_altitude

    def _touchdown_ready(self, state: LandingState) -> bool:
        return (
            abs(state.lateral_error)
            <= self.config.touchdown_lateral_error_limit
            and abs(state.lateral_velocity)
            <= self.config.touchdown_lateral_speed_limit
            and abs(state.vertical_velocity)
            <= self.config.touchdown_vertical_speed_limit
            and abs(state.attitude_error)
            <= self.config.touchdown_attitude_limit
            and abs(state.angular_rate)
            <= self.config.touchdown_angular_rate_limit
        )


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
