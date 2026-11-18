"""Offline PID behavior and simulated plant tracking."""

# Numeric regression bounds and controlled private fixtures are intentional.
# ruff: noqa: PLR2004, SLF001, PT009, SIM117
from __future__ import annotations

import unittest

import pytest

from irobot.src.robots.sawyer.core.execution.pid import JointPIDController
from irobot.src.robots.sawyer.core.planning.joint import QuinticTrajectory


class PIDTests(unittest.TestCase):
    def controller(self, **kwargs):
        settings = {
            'size': 1,
            'kp': 2,
            'ki': 0.1,
            'kd': 0.05,
            'max_velocity': 2,
            'max_acceleration': 10,
        }
        settings.update(kwargs)
        return JointPIDController(**settings)

    def test_pid_terms_and_feedforward(self):
        pid = self.controller(kp=2, ki=3, kd=0.5, max_acceleration=1000)
        # e=.1, integral=.01, derivative=.4-.2=.2; v=.4+.2+.03+.1=.73
        output = pid.update([1], [0.9], [0.4], [0.2], 0.1)
        self.assertAlmostEqual(output[0], 0.73)

    def test_velocity_acceleration_and_reset(self):
        pid = self.controller(max_velocity=0.5, max_acceleration=1)
        previous = 0
        for _ in range(30):
            output = pid.update([10], [0], [0], [0], 0.1)[0]
            assert abs(output) <= 0.5
            assert abs(output - previous) <= 0.1 + 1e-12
            previous = output
        self.assertAlmostEqual(previous, 0.5)
        pid.reset([0.2])
        assert pid.integral == [0]
        assert pid.previous_velocity == [0.2]

    def test_no_integral_windup_and_unwinding(self):
        pid = self.controller(kp=10, ki=2, kd=0, max_velocity=0.5)
        for _ in range(100):
            pid.update([1], [0], [0], [0], 0.01)
        assert pid.integral == [0]
        pid.integral = [0.2]
        pid.update([-0.1], [0], [10], [0], 0.1)
        assert pid.integral[0] < 0.2

    def test_integral_cap(self):
        pid = self.controller(kp=0, ki=1, kd=0, integral_limit=0.1)
        for _ in range(100):
            pid.update([0.01], [0], [0], [0], 0.1)
        assert abs(pid.integral[0]) <= 0.1

    def test_joint_position_bound_blocks_outward_velocity(self):
        pid = self.controller(position_lower=[-1], position_upper=[1])
        # Position guard overrides the normal slew bound if necessary.
        pid.reset([0.5])
        assert pid.update([2], [1], [1], [0], 0.01) == (0,)
        pid.reset([-0.5])
        assert pid.update([-2], [-1], [-1], [0], 0.01) == (0,)

    def test_per_joint_gains(self):
        pid = JointPIDController(2, [1, 2], 0, 0, 10, 1000)
        assert pid.update([1, 1], [0, 0], [0, 0], [0, 0], 0.1) == (1, 2)

    def test_invalid_inputs(self):
        for settings in (
            {'kp': -1},
            {'ki': float('nan')},
            {'max_velocity': 0},
            {'position_lower': [1], 'position_upper': [-1]},
            {'position_lower': [0]},
        ):
            with (
                self.subTest(settings=settings),
                pytest.raises(ValueError, match='.'),
            ):
                self.controller(**settings)
        pid = self.controller()
        for dt in (0, -1, float('nan'), float('inf')):
            with pytest.raises(ValueError, match='.'):
                pid.update([0], [0], [0], [0], dt)
        with pytest.raises(ValueError, match='.'):
            pid.update([0], [float('nan')], [0], [0], 0.1)
        with pytest.raises(ValueError, match='.'):
            pid.reset([100])

    def test_tracks_quintic_with_first_order_velocity_plant(self):
        path = QuinticTrajectory([0], [1], 0.8, duration=4, max_acceleration=1)
        pid = self.controller(max_velocity=0.8, max_acceleration=1)
        q, velocity, peak_error, dt = 0.0, 0.0, 0.0, 0.005
        for tick in range(1401):
            reference = path.sample(tick * dt)
            command = pid.update(
                reference.position, [q], reference.velocity, [velocity], dt
            )[0]
            velocity += (command - velocity) * dt / 0.04
            q += velocity * dt
            peak_error = max(peak_error, abs(q - reference.position[0]))
        assert peak_error < 0.03
        assert abs(1 - q) < 0.01
        assert abs(velocity) < 0.01


if __name__ == '__main__':
    unittest.main()
