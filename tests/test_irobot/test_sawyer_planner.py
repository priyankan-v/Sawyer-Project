"""Offline geometry, IK-path interpolation, timing, and validation tests."""

# Numeric regression bounds and controlled private fixtures are intentional.
# ruff: noqa: PLR2004, SLF001, PT009, SIM117
from __future__ import annotations

import math
import unittest
from unittest.mock import MagicMock

import numpy as np
import pytest

from irobot.src.robots.sawyer.core.base import CartesianPose, pose_error
from irobot.src.robots.sawyer.core.controller import (
    CartesianLine,
    CartesianPlanner,
    JointPath,
    QuinticTrajectory,
    TimedJointPath,
)

IDENTITY = (0, 0, 0, 1)


def affine_inverse(pose, seed):
    return (*pose.position, *seed[3:])


def affine_forward(configurations):
    return [CartesianPose(values[:3], IDENTITY) for values in configurations]


class GeometryTests(unittest.TestCase):
    def test_straight_line_fixed_orientation_and_rest_endpoints(self):
        start = CartesianPose((0.4, 0.1, 0.2), IDENTITY)
        line = CartesianLine(start, (5, -3, 10))
        assert line.start == start
        np.testing.assert_allclose(line.end.position, (0.45, 0.07, 0.3))
        for u in np.linspace(0, 1, 101):
            pose = line.sample(float(u))
            displacement = np.asarray(pose.position) - start.position
            assert np.linalg.norm(np.cross(displacement, line.delta)) < 1e-14
            assert pose.orientation == start.orientation
        for u in (0, 1):
            sample = line.progress.sample(u)
            assert sample.velocity == (0,)
            assert sample.acceleration == (0,)

    def test_pose_validation_and_quaternion_sign_equivalence(self):
        a = CartesianPose((0, 0, 0), IDENTITY, '/base')
        b = CartesianPose((0, 0, 0), (0, 0, 0, -1))
        assert pose_error(a, b) == (0, 0)
        for position, orientation in (((float('nan'), 0, 0), IDENTITY), ((0, 0, 0), (0, 0, 0, 0))):
            with pytest.raises(ValueError, match='.'):
                CartesianPose(position, orientation)
        with pytest.raises(ValueError, match='frame mismatch'):
            pose_error(a, CartesianPose((0, 0, 0), IDENTITY, 'world'))


class JointPathTests(unittest.TestCase):
    def path(self):
        reference = QuinticTrajectory([0, 0.2], [1, -0.3], 2, duration=1)
        fractions = np.linspace(0, 1, 21)
        return reference, JointPath(fractions, [reference.sample(float(t)).position for t in fractions])

    def test_recovers_a_quintic_and_continuous_derivatives(self):
        reference, path = self.path()
        for u in np.linspace(0, 1, 137):
            actual, expected = path.sample(float(u)), reference.sample(float(u))
            np.testing.assert_allclose(actual.position, expected.position, atol=1e-12)
            np.testing.assert_allclose(actual.velocity, expected.velocity, atol=1e-10)
            np.testing.assert_allclose(actual.acceleration, expected.acceleration, atol=1e-8)
        for knot in path.fractions[1:-1]:
            left, right = path.sample(knot - 1e-9), path.sample(knot + 1e-9)
            np.testing.assert_allclose(left.position, right.position, atol=1e-8)
            np.testing.assert_allclose(left.velocity, right.velocity, atol=1e-7)
            np.testing.assert_allclose(left.acceleration, right.acceleration, atol=1e-6)

    def test_extrema_cover_between_knot_values_and_timing(self):
        _, path = self.path()
        np.testing.assert_allclose(path.peak_velocity, (1.875, 0.9375), atol=1e-9)
        np.testing.assert_allclose(path.peak_acceleration, (10 / math.sqrt(3), 5 / math.sqrt(3)), atol=1e-8)
        trajectory = TimedJointPath(path, [0.7, 1], [0.8, 2])
        for t in np.linspace(0, trajectory.duration, 701):
            sample = trajectory.sample(float(t))
            assert all(
                abs(v) <= limit + 1e-09 for v, limit in zip(sample.velocity, trajectory.max_velocity, strict=False)
            )
            assert all(
                abs(a) <= limit + 1e-08
                for a, limit in zip(sample.acceleration, trajectory.max_acceleration, strict=False)
            )
        with pytest.raises(ValueError, match='too short'):
            TimedJointPath(path, [0.7, 1], [0.8, 2], trajectory.minimum_duration * 0.999)

    def test_longer_duration_scales_all_derivatives(self):
        _, path = self.path()
        fast = TimedJointPath(path, [1, 1], [1, 1])
        slow = TimedJointPath(path, [1, 1], [1, 1], 2 * fast.duration)
        a, b = fast.sample(fast.duration * 0.3), slow.sample(slow.duration * 0.3)
        np.testing.assert_allclose(a.position, b.position)
        np.testing.assert_allclose(a.velocity, np.asarray(b.velocity) * 2)
        np.testing.assert_allclose(a.acceleration, np.asarray(b.acceleration) * 4)

    def test_endpoint_distance_alone_does_not_determine_path_duration(self):
        path = JointPath([0, 0.25, 0.5, 0.75, 1], [[0], [0.1], [0.2], [0.1], [0]])
        assert path.start == path.end
        assert TimedJointPath(path, 1, 1).minimum_duration > 0

    def test_stationary_path_and_invalid_inputs(self):
        path = JointPath([0, 0.5, 1], [[1], [1], [1]])
        timed = TimedJointPath(path, 1, 1)
        assert timed.duration == 0
        assert timed.sample(0).position == (1,)
        for fractions, positions in (
            ([0, 1], [[0], [1]]),
            ([0, 0.5, 0.5, 1], [[0]] * 4),
            ([0, 0.5, 1], [[0], [float('nan')], [1]]),
        ):
            with pytest.raises(ValueError, match='.'):
                JointPath(fractions, positions)
        with pytest.raises(ValueError, match='.'):
            timed.sample(float('nan'))


class PlannerTests(unittest.TestCase):
    def planner(self, inverse=affine_inverse, forward=affine_forward, **options):
        return CartesianPlanner(inverse, forward, [-3] * 7, [3] * 7, **options)

    def test_affine_kinematics_matches_straight_quintic_and_minimum(self):
        start = CartesianPose((0, 0, 0), IDENTITY)
        plan = self.planner().plan(start, (0, 0, 10), [0] * 7, [1] * 7, [1] * 7)
        assert plan.max_position_error < 1e-12
        self.assertAlmostEqual(plan.trajectory.minimum_duration, math.sqrt((10 / math.sqrt(3)) * 0.1), places=8)
        for t in np.linspace(0, plan.trajectory.duration, 101):
            joints = plan.trajectory.sample(float(t)).position
            actual = affine_forward([joints])[0]
            desired = plan.line.sample(float(t) / plan.trajectory.duration)
            assert pose_error(actual, desired)[0] < 1e-12

    def test_nonlinear_ik_and_previous_solution_seeding(self):
        seeds, solutions = [], []

        def inverse(pose, seed):
            seeds.append(tuple(seed))
            solution = (pose.position[0], math.asin(pose.position[1]), pose.position[2], 0, 0, 0, 0)
            solutions.append(solution)
            return solution

        def forward(configurations):
            return [CartesianPose((q[0], math.sin(q[1]), q[2]), IDENTITY) for q in configurations]

        start, seed = CartesianPose((0, 0.6, 0), IDENTITY), (0, math.asin(0.6), 0, 0, 0, 0, 0)
        plan = self.planner(inverse, forward).plan(start, (5, 10, 3), seed, [1] * 7, [1] * 7, duration=4)
        assert seeds[0] == seed
        assert seeds[1:] == solutions[:-1]
        assert plan.max_position_error < 0.001
        assert plan.trajectory.duration == 4

    def test_unreachable_and_discontinuous_ik_rejected(self):
        with pytest.raises(ValueError, match='unreachable'):
            self.planner(MagicMock(side_effect=ValueError('unreachable'))).plan(
                CartesianPose((0, 0, 0), IDENTITY), (0, 0, 10), [0] * 7, [1] * 7, [1] * 7
            )

        def discontinuous(pose, _seed):
            return (*pose.position, 1, 0, 0, 0)

        with pytest.raises(ValueError, match='discontinuity'):
            self.planner(discontinuous, max_samples=11, sample_step=0.1).plan(
                CartesianPose((0, 0, 0), IDENTITY), (0, 0, 10), [0] * 7, [1] * 7, [1] * 7
            )

    def test_fk_mismatch_collision_and_unvalidated_interpolation_rejected(self):
        start = CartesianPose((0, 0, 0), IDENTITY)
        with pytest.raises(ValueError, match='does not match'):
            self.planner().plan(CartesianPose((0.1, 0, 0), IDENTITY), (0, 0, 10), [0] * 7, [1] * 7, [1] * 7)
        forward = MagicMock(side_effect=ValueError('FK collision'))
        with pytest.raises(ValueError, match='collision'):
            self.planner(forward=forward).plan(start, (0, 0, 10), [0] * 7, [1] * 7, [1] * 7)

        def inaccurate(pose, _seed):
            return (pose.position[0] + 0.01, pose.position[1], pose.position[2], 0, 0, 0, 0)

        with pytest.raises(ValueError, match='FK deviation'):
            self.planner(inaccurate, sample_step=0.1, max_samples=11).plan(start, (0, 0, 10), [0] * 7, [1] * 7, [1] * 7)

    def test_joint_bounds_and_too_short_time(self):
        start = CartesianPose((0, 0, 0), IDENTITY)

        def inaccurate(pose, _seed):
            # Smooth nonlinear redundancy invisible in XYZ; interpolates below bounds.
            return (*pose.position, 0.05 * math.sin(30 * pose.position[2]), 0, 0, 0)

        plan = self.planner(inaccurate, sample_step=0.1).plan(start, (0, 0, 10), [0] * 7, [1] * 7, [1] * 7)
        assert plan.max_position_error <= 0.001
        with pytest.raises(ValueError, match='too short'):
            self.planner().plan(start, (0, 0, 10), [0] * 7, [1] * 7, [1] * 7, duration=0.01)
        with pytest.raises(ValueError, match='position limits'):
            self.planner(sample_step=0.1).plan(start, (400, 0, 0), [0] * 7, [1] * 7, [1] * 7)

    def test_nonlinear_fk_refines_sampling_until_valid(self):
        def inverse(pose, _seed):
            return (pose.position[0] ** 0.2, pose.position[1], pose.position[2], 0, 0, 0, 0)

        def forward(configurations):
            return [CartesianPose((q[0] ** 5, q[1], q[2]), IDENTITY) for q in configurations]

        plan = self.planner(inverse, forward, sample_step=0.2, path_tolerance=1e-7).plan(
            CartesianPose((0.1, 0, 0), IDENTITY), (100, 0, 0), [0.1**0.2, 0, 0, 0, 0, 0, 0], [1] * 7, [1] * 7
        )
        assert plan.samples > 11
        assert plan.max_position_error <= 1e-07

    def test_joint_interpolation_overshoot_rejected_between_valid_ik_points(self):
        def inverse(pose, _seed):
            return (*pose.position, 0.99 if pose.position[2] < 0.1 else 0, 0, 0, 0)

        planner = CartesianPlanner(
            inverse,
            affine_forward,
            [-3] * 7,
            [3, 3, 3, 0.99, 3, 3, 3],
            sample_step=0.1,
            max_joint_step=2,
            max_samples=11,
        )
        with pytest.raises(ValueError, match='overshoots position limits'):
            planner.plan(CartesianPose((0, 0, 0), IDENTITY), (0, 0, 10), [0] * 7, [1] * 7, [1] * 7)

    def test_zero_offset_does_not_call_inverse_or_move_joints(self):
        inverse = MagicMock()
        plan = self.planner(inverse).plan(CartesianPose((0, 0, 0), IDENTITY), (0, 0, 0), [0] * 7, [1] * 7, [1] * 7)
        inverse.assert_not_called()
        assert plan.trajectory.duration == 0
        assert plan.trajectory.start == plan.trajectory.end

    def test_starting_limit_error_names_joint_and_does_not_call_kinematics(self):
        inverse, forward = MagicMock(), MagicMock()
        planner = CartesianPlanner(inverse, forward, [-3] * 7, [3] * 7, joint_names=[f'right_j{i}' for i in range(7)])
        with pytest.raises(ValueError, match=r'right_j6=-3.002000 outside position limits'):
            planner.plan(CartesianPose((0, 0, 0), IDENTITY), (0, 0, 10), [0] * 6 + [-3.002], [1] * 7, [1] * 7)
        inverse.assert_not_called()
        forward.assert_not_called()


if __name__ == '__main__':
    unittest.main()
