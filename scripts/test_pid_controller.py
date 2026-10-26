"""Offline PID behavior and simulated plant tracking."""

import unittest

from pid_controller import JointPIDController
from quintic_trajectory import QuinticTrajectory


class PIDTests(unittest.TestCase):
    def controller(self, **kwargs):
        settings = dict(size=1, kp=2, ki=0.1, kd=0.05, max_velocity=2, max_acceleration=10)
        settings.update(kwargs)
        return JointPIDController(**settings)

    def test_pid_terms_and_feedforward(self):
        pid = self.controller(kp=2, ki=3, kd=0.5, max_acceleration=1000)
        # e=.1, integral=.01, derivative=.4-.2=.2; v=.4+.2+.03+.1=.73
        output = pid.update([1], [.9], [.4], [.2], .1)
        self.assertAlmostEqual(output[0], .73)

    def test_velocity_acceleration_and_reset(self):
        pid = self.controller(max_velocity=.5, max_acceleration=1)
        previous = 0
        for _ in range(30):
            output = pid.update([10], [0], [0], [0], .1)[0]
            self.assertLessEqual(abs(output), .5)
            self.assertLessEqual(abs(output - previous), .1 + 1e-12)
            previous = output
        self.assertAlmostEqual(previous, .5)
        pid.reset([.2])
        self.assertEqual(pid.integral, [0])
        self.assertEqual(pid.previous_velocity, [.2])

    def test_no_integral_windup_and_unwinding(self):
        pid = self.controller(kp=10, ki=2, kd=0, max_velocity=.5)
        for _ in range(100):
            pid.update([1], [0], [0], [0], .01)
        self.assertEqual(pid.integral, [0])
        pid.integral = [.2]
        pid.update([-.1], [0], [10], [0], .1)
        self.assertLess(pid.integral[0], .2)

    def test_integral_cap(self):
        pid = self.controller(kp=0, ki=1, kd=0, integral_limit=.1)
        for _ in range(100):
            pid.update([.01], [0], [0], [0], .1)
        self.assertLessEqual(abs(pid.integral[0]), .1)

    def test_joint_position_bound_blocks_outward_velocity(self):
        pid = self.controller(position_lower=[-1], position_upper=[1])
        # Position guard overrides the normal slew bound if necessary.
        pid.reset([.5])
        self.assertEqual(pid.update([2], [1], [1], [0], .01), (0,))
        pid.reset([-.5])
        self.assertEqual(pid.update([-2], [-1], [-1], [0], .01), (0,))

    def test_per_joint_gains(self):
        pid = JointPIDController(2, [1, 2], 0, 0, 10, 1000)
        self.assertEqual(pid.update([1, 1], [0, 0], [0, 0], [0, 0], .1), (1, 2))

    def test_invalid_inputs(self):
        for settings in (dict(kp=-1), dict(ki=float('nan')), dict(max_velocity=0),
                         dict(position_lower=[1], position_upper=[-1]), dict(position_lower=[0])):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                self.controller(**settings)
        pid = self.controller()
        for dt in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                pid.update([0], [0], [0], [0], dt)
        with self.assertRaises(ValueError):
            pid.update([0], [float('nan')], [0], [0], .1)
        with self.assertRaises(ValueError):
            pid.reset([100])

    def test_tracks_quintic_with_first_order_velocity_plant(self):
        path = QuinticTrajectory([0], [1], .8, duration=4, max_acceleration=1)
        pid = self.controller(max_velocity=.8, max_acceleration=1)
        q, velocity, peak_error, dt = 0.0, 0.0, 0.0, .005
        for tick in range(1401):
            reference = path.sample(tick * dt)
            command = pid.update(reference.position, [q], reference.velocity, [velocity], dt)[0]
            velocity += (command - velocity) * dt / .04
            q += velocity * dt
            peak_error = max(peak_error, abs(q - reference.position[0]))
        self.assertLess(peak_error, .03)
        self.assertLess(abs(1 - q), .01)
        self.assertLess(abs(velocity), .01)


if __name__ == '__main__':
    unittest.main()
