"""Analytical and sampled trajectory checks; no ROS or robot connection."""

# Numeric regression bounds and controlled private fixtures are intentional.
# ruff: noqa: PLR2004, SLF001, PT009, SIM117
from __future__ import annotations

import math
import random
import unittest

import pytest

from irobot.src.robots.sawyer.core.controller import QuinticTrajectory


class QuinticTests(unittest.TestCase):
    def test_endpoints_and_stationary_joint(self):
        path = QuinticTrajectory([1, -2, 3], [-1, 2, 3], [10] * 3, duration=5)
        assert path.sample(-1).position == (1, -2, 3)
        assert path.sample(6).position == (-1, 2, 3)
        for t in (0, 5):
            sample = path.sample(t)
            assert sample.velocity == (0, 0, 0)
            assert sample.acceleration == (0, 0, 0)
        midpoint = path.sample(2.5)
        assert midpoint.position == (0, 0, 3)
        assert midpoint.velocity[2] == 0

    def test_exact_velocity_limited_minimum(self):
        path = QuinticTrajectory([0, 0], [2, -3], [1, 3])
        assert path.minimum_duration == 3.75
        assert path.duration == path.minimum_duration
        self.assertAlmostEqual(path.sample(path.duration / 2).velocity[0], 1)
        with pytest.raises(ValueError, match='minimum feasible'):
            QuinticTrajectory([0, 0], [2, -3], [1, 3], duration=3.749)

    def test_acceleration_extremum_determines_duration(self):
        path = QuinticTrajectory([0], [1], 100, max_acceleration=1)
        expected = math.sqrt(10 / math.sqrt(3))
        self.assertAlmostEqual(path.duration, expected)
        for fraction in ((3 - math.sqrt(3)) / 6, (3 + math.sqrt(3)) / 6):
            self.assertAlmostEqual(
                abs(path.sample(path.duration * fraction).acceleration[0]), 1
            )
        with pytest.raises(ValueError, match='.'):
            QuinticTrajectory(
                [0], [1], 100, duration=expected * 0.99, max_acceleration=1
            )

    def test_longer_duration_reduces_derivatives(self):
        fast = QuinticTrajectory([0], [1], 1, max_acceleration=2)
        slow = QuinticTrajectory(
            [0], [1], 1, duration=2 * fast.duration, max_acceleration=2
        )
        a, b = fast.sample(fast.duration / 3), slow.sample(slow.duration / 3)
        self.assertAlmostEqual(a.position[0], b.position[0])
        self.assertAlmostEqual(a.velocity[0], 2 * b.velocity[0])
        self.assertAlmostEqual(a.acceleration[0], 4 * b.acceleration[0])
        self.assertAlmostEqual(a.jerk[0], 8 * b.jerk[0])

    def test_derivatives_match_finite_differences(self):
        path = QuinticTrajectory([0], [1], 1, duration=3)
        t, step = 0.8, 1e-5
        left, center, right = (
            path.sample(t - step),
            path.sample(t),
            path.sample(t + step),
        )
        self.assertAlmostEqual(
            (right.position[0] - left.position[0]) / (2 * step),
            center.velocity[0],
            places=8,
        )
        self.assertAlmostEqual(
            (right.velocity[0] - left.velocity[0]) / (2 * step),
            center.acceleration[0],
            places=8,
        )
        self.assertAlmostEqual(
            (right.acceleration[0] - left.acceleration[0]) / (2 * step),
            center.jerk[0],
            places=8,
        )

    def test_random_seven_joint_moves_obey_limits_and_monotonicity(self):
        randomizer = random.Random(42)
        for _ in range(25):
            start = [randomizer.uniform(-3, 3) for _ in range(7)]
            end = [randomizer.uniform(-3, 3) for _ in range(7)]
            velocity = [randomizer.uniform(0.1, 2) for _ in range(7)]
            acceleration = [randomizer.uniform(0.2, 5) for _ in range(7)]
            path = QuinticTrajectory(
                start, end, velocity, max_acceleration=acceleration
            )
            for n in range(201):
                sample = path.sample(path.duration * n / 200)
                for i in range(7):
                    assert abs(sample.velocity[i]) <= velocity[i] + 1e-10
                    assert (
                        abs(sample.acceleration[i]) <= acceleration[i] + 1e-10
                    )
                    assert (
                        min(start[i], end[i]) - 1e-10
                        <= sample.position[i]
                        <= max(start[i], end[i]) + 1e-10
                    )

    def test_stationary_auto_and_requested_duration(self):
        path = QuinticTrajectory([1, 2], [1, 2], 1)
        assert path.duration == 0
        assert path.sample(0).position == (1, 2)
        path = QuinticTrajectory([1], [1], 1, duration=5)
        assert path.duration == 5
        assert path.sample(2).velocity == (0,)

    def test_invalid_parameters(self):
        cases = [
            ([], [], 1),
            ([0], [float('nan')], 1),
            ([0], [1, 2], 1),
            ([0], [1], 0),
            ([0], [1], -1),
            ([0], [1], float('inf')),
        ]
        for arguments in cases:
            with (
                self.subTest(arguments=arguments),
                pytest.raises(ValueError, match='.'),
            ):
                QuinticTrajectory(*arguments)
        for duration in (0, -1, float('nan'), float('inf')):
            with (
                self.subTest(duration=duration),
                pytest.raises(ValueError, match='.'),
            ):
                QuinticTrajectory([0], [1], 1, duration=duration)
        with pytest.raises(ValueError, match='.'):
            QuinticTrajectory([0], [1], 1, max_acceleration=0)
        with pytest.raises(ValueError, match='.'):
            QuinticTrajectory([0], [1], 1).sample(float('nan'))


if __name__ == '__main__':
    unittest.main()
