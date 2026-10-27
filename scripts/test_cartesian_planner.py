"""Offline geometry, IK-path interpolation, timing, and validation tests."""

import math
import unittest
from unittest.mock import MagicMock

import numpy as np

from cartesian_path import CartesianLine, CartesianPose, parse_offsets, pose_error
from cartesian_planner import CartesianPlanner
from joint_path import JointPath, TimedJointPath
from quintic_trajectory import QuinticTrajectory


IDENTITY = (0, 0, 0, 1)


def affine_inverse(pose, seed):
    return (*pose.position, *seed[3:])


def affine_forward(configurations):
    return [CartesianPose(values[:3], IDENTITY) for values in configurations]


class GeometryTests(unittest.TestCase):
    def test_offset_units_axes_and_validation(self):
        self.assertEqual(parse_offsets(['z:+10']), (0, 0, 10))
        self.assertEqual(parse_offsets(['X:+5', 'y:-3', 'z:+10']), (5, -3, 10))
        for tokens in ([], ['z:1', 'z:2'], ['q:1'], ['x:nan'], ['z:inf'], ['y'], ['x:1:2']):
            with self.subTest(tokens=tokens), self.assertRaises(ValueError):
                parse_offsets(tokens)

    def test_straight_line_fixed_orientation_and_rest_endpoints(self):
        start = CartesianPose((.4, .1, .2), IDENTITY)
        line = CartesianLine(start, (5, -3, 10))
        self.assertEqual(line.start, start)
        np.testing.assert_allclose(line.end.position, (.45, .07, .3))
        for u in np.linspace(0, 1, 101):
            pose = line.sample(float(u))
            displacement = np.asarray(pose.position) - start.position
            self.assertLess(np.linalg.norm(np.cross(displacement, line.delta)), 1e-14)
            self.assertEqual(pose.orientation, start.orientation)
        for u in (0, 1):
            sample = line.progress.sample(u)
            self.assertEqual(sample.velocity, (0,))
            self.assertEqual(sample.acceleration, (0,))

    def test_pose_validation_and_quaternion_sign_equivalence(self):
        a = CartesianPose((0, 0, 0), IDENTITY, '/base')
        b = CartesianPose((0, 0, 0), (0, 0, 0, -1))
        self.assertEqual(pose_error(a, b), (0, 0))
        for position, orientation in (((float('nan'), 0, 0), IDENTITY), ((0, 0, 0), (0, 0, 0, 0))):
            with self.assertRaises(ValueError):
                CartesianPose(position, orientation)
        with self.assertRaisesRegex(ValueError, 'frame mismatch'):
            pose_error(a, CartesianPose((0, 0, 0), IDENTITY, 'world'))


class JointPathTests(unittest.TestCase):
    def path(self):
        reference = QuinticTrajectory([0, .2], [1, -.3], 2, duration=1)
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
            left, right = path.sample(knot-1e-9), path.sample(knot+1e-9)
            np.testing.assert_allclose(left.position, right.position, atol=1e-8)
            np.testing.assert_allclose(left.velocity, right.velocity, atol=1e-7)
            np.testing.assert_allclose(left.acceleration, right.acceleration, atol=1e-6)

    def test_extrema_cover_between_knot_values_and_timing(self):
        _, path = self.path()
        np.testing.assert_allclose(path.peak_velocity, (1.875, .9375), atol=1e-9)
        np.testing.assert_allclose(path.peak_acceleration, (10/math.sqrt(3), 5/math.sqrt(3)), atol=1e-8)
        trajectory = TimedJointPath(path, [.7, 1], [.8, 2])
        for t in np.linspace(0, trajectory.duration, 701):
            sample = trajectory.sample(float(t))
            self.assertTrue(all(abs(v) <= limit+1e-9 for v, limit in zip(sample.velocity, trajectory.max_velocity)))
            self.assertTrue(all(abs(a) <= limit+1e-8 for a, limit in zip(sample.acceleration, trajectory.max_acceleration)))
        with self.assertRaisesRegex(ValueError, 'too short'):
            TimedJointPath(path, [.7, 1], [.8, 2], trajectory.minimum_duration*.999)

    def test_longer_duration_scales_all_derivatives(self):
        _, path = self.path()
        fast = TimedJointPath(path, [1, 1], [1, 1])
        slow = TimedJointPath(path, [1, 1], [1, 1], 2*fast.duration)
        a, b = fast.sample(fast.duration*.3), slow.sample(slow.duration*.3)
        np.testing.assert_allclose(a.position, b.position)
        np.testing.assert_allclose(a.velocity, np.asarray(b.velocity)*2)
        np.testing.assert_allclose(a.acceleration, np.asarray(b.acceleration)*4)

    def test_endpoint_distance_alone_does_not_determine_path_duration(self):
        path = JointPath([0, .25, .5, .75, 1], [[0], [.1], [.2], [.1], [0]])
        self.assertEqual(path.start, path.end)
        self.assertGreater(TimedJointPath(path, 1, 1).minimum_duration, 0)

    def test_stationary_path_and_invalid_inputs(self):
        path = JointPath([0, .5, 1], [[1], [1], [1]])
        timed = TimedJointPath(path, 1, 1)
        self.assertEqual(timed.duration, 0)
        self.assertEqual(timed.sample(0).position, (1,))
        for fractions, positions in (([0, 1], [[0], [1]]), ([0, .5, .5, 1], [[0]]*4),
                                       ([0, .5, 1], [[0], [float('nan')], [1]])):
            with self.assertRaises(ValueError):
                JointPath(fractions, positions)
        with self.assertRaises(ValueError):
            timed.sample(float('nan'))


class PlannerTests(unittest.TestCase):
    def planner(self, inverse=affine_inverse, forward=affine_forward, **options):
        return CartesianPlanner(inverse, forward, [-3]*7, [3]*7, **options)

    def test_affine_kinematics_matches_straight_quintic_and_minimum(self):
        start = CartesianPose((0, 0, 0), IDENTITY)
        plan = self.planner().plan(start, (0, 0, 10), [0]*7, [1]*7, [1]*7)
        self.assertLess(plan.max_position_error, 1e-12)
        self.assertAlmostEqual(plan.trajectory.minimum_duration, math.sqrt((10/math.sqrt(3))*.1), places=8)
        for t in np.linspace(0, plan.trajectory.duration, 101):
            joints = plan.trajectory.sample(float(t)).position
            actual = affine_forward([joints])[0]
            desired = plan.line.sample(float(t)/plan.trajectory.duration)
            self.assertLess(pose_error(actual, desired)[0], 1e-12)

    def test_nonlinear_ik_and_previous_solution_seeding(self):
        seeds, solutions = [], []
        def inverse(pose, seed):
            seeds.append(tuple(seed))
            solution = (pose.position[0], math.asin(pose.position[1]), pose.position[2], 0, 0, 0, 0)
            solutions.append(solution)
            return solution
        def forward(configurations):
            return [CartesianPose((q[0], math.sin(q[1]), q[2]), IDENTITY) for q in configurations]
        start, seed = CartesianPose((0, .6, 0), IDENTITY), (0, math.asin(.6), 0, 0, 0, 0, 0)
        plan = self.planner(inverse, forward).plan(start, (5, 10, 3), seed, [1]*7, [1]*7, duration=4)
        self.assertEqual(seeds[0], seed)
        self.assertEqual(seeds[1:], solutions[:-1])
        self.assertLess(plan.max_position_error, .001)
        self.assertEqual(plan.trajectory.duration, 4)

    def test_unreachable_and_discontinuous_ik_rejected(self):
        with self.assertRaisesRegex(ValueError, 'unreachable'):
            self.planner(MagicMock(side_effect=ValueError('unreachable'))).plan(
                CartesianPose((0,0,0), IDENTITY), (0,0,10), [0]*7, [1]*7, [1]*7)
        def discontinuous(pose, seed):
            return (*pose.position, 1, 0, 0, 0)
        with self.assertRaisesRegex(ValueError, 'discontinuity'):
            self.planner(discontinuous, max_samples=11, sample_step=.1).plan(
                CartesianPose((0,0,0), IDENTITY), (0,0,10), [0]*7, [1]*7, [1]*7)

    def test_fk_mismatch_collision_and_unvalidated_interpolation_rejected(self):
        start = CartesianPose((0,0,0), IDENTITY)
        with self.assertRaisesRegex(ValueError, 'does not match'):
            self.planner().plan(CartesianPose((.1,0,0), IDENTITY), (0,0,10), [0]*7, [1]*7, [1]*7)
        forward = MagicMock(side_effect=ValueError('FK collision'))
        with self.assertRaisesRegex(ValueError, 'collision'):
            self.planner(forward=forward).plan(start, (0,0,10), [0]*7, [1]*7, [1]*7)
        def inaccurate(pose, seed):
            return (pose.position[0]+.01, pose.position[1], pose.position[2], 0,0,0,0)
        with self.assertRaisesRegex(ValueError, 'FK deviation'):
            self.planner(inaccurate, sample_step=.1, max_samples=11).plan(start, (0,0,10), [0]*7, [1]*7, [1]*7)

    def test_joint_bounds_and_too_short_time(self):
        start = CartesianPose((0,0,0), IDENTITY)
        def inaccurate(pose, seed):
            # Smooth nonlinear redundancy invisible in XYZ; interpolates below bounds.
            return (*pose.position, .05*math.sin(30*pose.position[2]), 0, 0, 0)
        plan = self.planner(inaccurate, sample_step=.1).plan(start, (0,0,10), [0]*7, [1]*7, [1]*7)
        self.assertLessEqual(plan.max_position_error, .001)
        with self.assertRaisesRegex(ValueError, 'too short'):
            self.planner().plan(start, (0,0,10), [0]*7, [1]*7, [1]*7, duration=.01)
        with self.assertRaisesRegex(ValueError, 'position limits'):
            self.planner(sample_step=.1).plan(start, (400,0,0), [0]*7, [1]*7, [1]*7)

    def test_nonlinear_fk_refines_sampling_until_valid(self):
        def inverse(pose, seed):
            return (pose.position[0]**.2, pose.position[1], pose.position[2], 0,0,0,0)
        def forward(configurations):
            return [CartesianPose((q[0]**5, q[1], q[2]), IDENTITY) for q in configurations]
        plan = self.planner(inverse, forward, sample_step=.2, path_tolerance=1e-7).plan(
            CartesianPose((.1,0,0), IDENTITY), (100,0,0), [.1**.2,0,0,0,0,0,0], [1]*7, [1]*7)
        self.assertGreater(plan.samples, 11)
        self.assertLessEqual(plan.max_position_error, 1e-7)

    def test_joint_interpolation_overshoot_rejected_between_valid_ik_points(self):
        def inverse(pose, seed):
            return (*pose.position, .99 if pose.position[2] < .1 else 0, 0,0,0)
        planner = CartesianPlanner(inverse, affine_forward, [-3]*7, [3,3,3,.99,3,3,3],
                                   sample_step=.1, max_joint_step=2, max_samples=11)
        with self.assertRaisesRegex(ValueError, 'overshoots position limits'):
            planner.plan(CartesianPose((0,0,0), IDENTITY), (0,0,10), [0]*7, [1]*7, [1]*7)

    def test_zero_offset_does_not_call_inverse_or_move_joints(self):
        inverse = MagicMock()
        plan = self.planner(inverse).plan(CartesianPose((0,0,0), IDENTITY), (0,0,0), [0]*7, [1]*7, [1]*7)
        inverse.assert_not_called()
        self.assertEqual(plan.trajectory.duration, 0)
        self.assertEqual(plan.trajectory.start, plan.trajectory.end)

    def test_starting_limit_error_names_joint_and_does_not_call_kinematics(self):
        inverse, forward = MagicMock(), MagicMock()
        planner = CartesianPlanner(inverse, forward, [-3]*7, [3]*7,
                                   joint_names=[f'right_j{i}' for i in range(7)])
        with self.assertRaisesRegex(ValueError, r'right_j6=-3.002000 outside position limits'):
            planner.plan(CartesianPose((0,0,0), IDENTITY), (0,0,10), [0]*6+[-3.002], [1]*7, [1]*7)
        inverse.assert_not_called()
        forward.assert_not_called()


if __name__ == '__main__':
    unittest.main()
