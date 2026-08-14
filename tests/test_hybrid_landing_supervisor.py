from __future__ import annotations

import unittest
from dataclasses import replace

from src.hybrid_landing_supervisor_reference import (
    HybridLandingSupervisor,
    LandingCommand,
    LandingState,
    Mode,
    PolicyUnavailable,
    SupervisorConfig,
    TerminalContext,
)


class ConstantPolicy:
    def __init__(self, result: LandingCommand) -> None:
        self.result = result
        self.calls = 0

    def command(self, state: LandingState) -> LandingCommand:
        self.calls += 1
        return self.result


class UnavailablePolicy:
    def command(self, state: LandingState) -> LandingCommand:
        raise PolicyUnavailable("predictive policy is unavailable")


class RecordingTerminalPolicy:
    def __init__(self, result: LandingCommand) -> None:
        self.result = result
        self.contexts: list[TerminalContext] = []

    def command(
        self,
        state: LandingState,
        context: TerminalContext,
    ) -> LandingCommand:
        self.contexts.append(context)
        return self.result


class HybridLandingSupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.recovery = ConstantPolicy(LandingCommand(0.4, -0.2, 0.1))
        self.local = ConstantPolicy(LandingCommand(0.5, 0.1, -0.1))
        self.terminal = RecordingTerminalPolicy(LandingCommand(0.3, 0.0, 0.0))
        self.config = SupervisorConfig(
            local_lateral_error_limit=1.0,
            local_lateral_speed_limit=0.8,
            local_attitude_limit=0.3,
            local_angular_rate_limit=0.6,
            terminal_altitude=1.0,
            touchdown_lateral_error_limit=0.5,
            touchdown_lateral_speed_limit=0.2,
            touchdown_vertical_speed_limit=0.3,
            touchdown_attitude_limit=0.1,
            touchdown_angular_rate_limit=0.2,
            mpc_entry_hold_steps=2,
        )
        self.supervisor = HybridLandingSupervisor(
            config=self.config,
            recovery_policy=self.recovery,
            local_policy=self.local,
            terminal_policy=self.terminal,
        )

    @staticmethod
    def state(**changes: object) -> LandingState:
        baseline = LandingState(
            lateral_error=0.0,
            vertical_error=5.0,
            lateral_velocity=0.0,
            vertical_velocity=-0.5,
            attitude_error=0.0,
            angular_rate=0.0,
            has_contact=False,
        )
        return replace(baseline, **changes)

    def test_large_state_error_uses_recovery(self) -> None:
        result = self.supervisor.command(self.state(lateral_error=2.0))

        self.assertEqual(self.supervisor.mode, Mode.RECOVERY)
        self.assertEqual(result, self.recovery.result)
        self.assertEqual(self.recovery.calls, 1)
        self.assertEqual(self.local.calls, 0)

    def test_local_policy_requires_consecutive_eligible_samples(self) -> None:
        first = self.supervisor.command(self.state())
        second = self.supervisor.command(self.state())

        self.assertEqual(first, self.recovery.result)
        self.assertEqual(second, self.local.result)
        self.assertEqual(self.supervisor.mode, Mode.LOCAL_MPC)

    def test_leaving_corridor_resets_entry_counter(self) -> None:
        self.supervisor.command(self.state())
        self.supervisor.command(self.state(lateral_velocity=1.2))
        result = self.supervisor.command(self.state())

        self.assertEqual(result, self.recovery.result)
        self.assertEqual(self.supervisor.mode, Mode.RECOVERY)

    def test_local_policy_unavailability_returns_to_recovery(self) -> None:
        supervisor = HybridLandingSupervisor(
            config=self.config,
            recovery_policy=self.recovery,
            local_policy=UnavailablePolicy(),
            terminal_policy=self.terminal,
        )

        supervisor.command(self.state())
        result = supervisor.command(self.state())

        self.assertEqual(result, self.recovery.result)
        self.assertEqual(supervisor.mode, Mode.RECOVERY)

    def test_terminal_mode_latches_touchdown_commitment(self) -> None:
        supervisor = self.supervisor
        supervisor.command(
            self.state(
                vertical_error=0.5,
                lateral_velocity=0.5,
                vertical_velocity=-0.6,
            )
        )
        supervisor.command(
            self.state(
                vertical_error=0.4,
                lateral_velocity=0.1,
                vertical_velocity=-0.2,
                attitude_error=0.05,
            )
        )
        supervisor.command(
            self.state(
                vertical_error=0.3,
                lateral_velocity=0.4,
                vertical_velocity=-0.5,
            )
        )

        self.assertEqual(supervisor.mode, Mode.TERMINAL)
        self.assertEqual(
            [context.touchdown_committed for context in self.terminal.contexts],
            [False, True, True],
        )

    def test_contact_suppresses_all_commands(self) -> None:
        result = self.supervisor.command(self.state(has_contact=True))

        self.assertEqual(self.supervisor.mode, Mode.CONTACT_SETTLE)
        self.assertEqual(result, LandingCommand.zero())

    def test_policy_commands_are_saturated(self) -> None:
        unbounded = LandingCommand(1.5, -2.0, 3.0)
        recovery = ConstantPolicy(unbounded)
        local = ConstantPolicy(unbounded)
        terminal = RecordingTerminalPolicy(unbounded)
        supervisor = HybridLandingSupervisor(
            config=replace(self.config, mpc_entry_hold_steps=1),
            recovery_policy=recovery,
            local_policy=local,
            terminal_policy=terminal,
        )

        recovery_result = supervisor.command(self.state(lateral_error=2.0))
        local_result = supervisor.command(self.state())
        terminal_result = supervisor.command(self.state(vertical_error=0.5))

        expected = LandingCommand(1.0, -1.0, 1.0)
        self.assertEqual(recovery_result, expected)
        self.assertEqual(local_result, expected)
        self.assertEqual(terminal_result, expected)

    def test_reset_clears_mode_hysteresis_and_touchdown_latch(self) -> None:
        self.supervisor.command(
            self.state(
                vertical_error=0.4,
                lateral_velocity=0.1,
                vertical_velocity=-0.2,
            )
        )
        self.supervisor.reset()
        result = self.supervisor.command(self.state())

        self.assertFalse(self.supervisor.touchdown_committed)
        self.assertEqual(self.supervisor.mode, Mode.RECOVERY)
        self.assertEqual(result, self.recovery.result)


if __name__ == "__main__":
    unittest.main()
