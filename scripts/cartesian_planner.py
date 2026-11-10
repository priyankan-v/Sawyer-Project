"""Reusable Cartesian-to-joint planning with injected IK/FK, independent of ROS."""

from dataclasses import dataclass
import math

import numpy as np

if __package__:
    from .cartesian_path import CartesianLine, pose_error
    from .joint_path import JointPath, TimedJointPath
    from .motion_utils import vector
else:
    from cartesian_path import CartesianLine, pose_error
    from joint_path import JointPath, TimedJointPath
    from motion_utils import vector


@dataclass(frozen=True)
class CartesianPlan:
    line: CartesianLine
    trajectory: TimedJointPath
    samples: int
    max_position_error: float
    max_orientation_error: float


class CartesianPlanner:
    """Seed IK from the previous solution, fit a C2 joint path, validate with FK.

    IK accepts (CartesianPose, seed) and returns ordered joints. FK accepts a
    sequence of joint configurations and returns corresponding CartesianPoses;
    it must raise on invalid/colliding configurations. Position bounds are checked
    at polynomial extrema, not only at IK samples. Cartesian FK validation uses
    knots and quarter/midpoints; it is a sampled tolerance check, not a proof of
    continuous collision freedom or exact Cartesian tracking between samples.
    """

    def __init__(self, inverse, forward, lower, upper, *, sample_step=.005,
                 path_tolerance=.001, orientation_tolerance=.01, max_joint_step=.15,
                 max_samples=401, joint_names=None, angular_step=math.radians(5)):
        self.inverse, self.forward = inverse, forward
        self.lower = tuple(lower)
        self.upper = vector(upper, len(self.lower), 'upper joint limits')
        self.lower = vector(self.lower, len(self.lower), 'lower joint limits')
        if not self.lower or any(a >= b for a, b in zip(self.lower, self.upper)):
            raise ValueError('Invalid joint position limits')
        self.joint_names = (tuple(joint_names) if joint_names is not None else
                            tuple(f'joint[{i}]' for i in range(len(self.lower))))
        if len(self.joint_names) != len(self.lower):
            raise ValueError('Joint names and position bounds must have equal lengths')
        for label, value in (('sample_step', sample_step), ('path_tolerance', path_tolerance),
                             ('orientation_tolerance', orientation_tolerance), ('max_joint_step', max_joint_step),
                             ('angular_step', angular_step)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{label} must be positive and finite')
        if not isinstance(max_samples, int) or max_samples < 11:
            raise ValueError('max_samples must be an integer >=11')
        self.sample_step, self.path_tolerance = sample_step, path_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.max_joint_step, self.max_samples = max_joint_step, max_samples
        self.angular_step = angular_step

    def checked_joints(self, values, label):
        joints = vector(values, len(self.lower), label)
        for name, q, low, high in zip(self.joint_names, joints, self.lower, self.upper):
            if q < low - 1e-8 or q > high + 1e-8:
                raise ValueError(f'{label}: {name}={q:.6f} outside position limits [{low:.6f}, {high:.6f}]')
        return joints

    def plan(self, start_pose, offset_cm, start_joints, max_velocity, max_acceleration, duration=None):
        line = CartesianLine(start_pose, offset_cm)
        return self.plan_line(line, start_joints, max_velocity, max_acceleration, duration)

    def plan_pose(self, start_pose, end_pose, start_joints, max_velocity, max_acceleration,
                  duration=None, end_joints=None):
        """Straight XYZ and shortest SLERP, optionally meeting a specific joint branch.

        A joint-constrained endpoint biases IK seeds toward that posture and is
        still subject to continuity, FK and position-limit checks. A branch that
        cannot connect continuously is rejected rather than jumped to.
        """
        return self.plan_line(CartesianLine.between(start_pose, end_pose), start_joints,
                              max_velocity, max_acceleration, duration, end_joints)

    def plan_line(self, line, start_joints, max_velocity, max_acceleration, duration=None, end_joints=None):
        start = self.checked_joints(start_joints, 'Start joints')
        end = None if end_joints is None else self.checked_joints(end_joints, 'Target joints')
        actual = self.forward([start])
        if len(actual) != 1:
            raise ValueError('FK returned the wrong number of start poses')
        position_error, orientation_error = pose_error(actual[0], line.start)
        if position_error > self.path_tolerance or orientation_error > self.orientation_tolerance:
            raise ValueError(f'Current endpoint does not match selected IK tip/FK: '
                             f'{position_error:.6f} m, {orientation_error:.6f} rad')
        if line.length == 0 and line.angle < 1e-12 and (end is None or end == start):
            path = JointPath((0.0, .5, 1.0), (start, start, start))
            return CartesianPlan(line, TimedJointPath(path, max_velocity, max_acceleration, duration),
                                 3, position_error, orientation_error)
        count = max(11, math.ceil(1.875 * line.length / self.sample_step) + 1,
                    math.ceil(1.875 * line.angle / self.angular_step) + 1)
        if count > self.max_samples:
            raise ValueError(f'Path needs at least {count} IK samples; increase max_samples or split the move')
        last_problem = ''
        while count <= self.max_samples:
            fractions = tuple(float(t) for t in np.linspace(0, 1, count))
            positions = [start]
            for fraction in fractions[1:]:
                seed = positions[-1]
                if end is not None:
                    progress = line.progress.sample(fraction).position[0]
                    seed = tuple((1-progress)*a + progress*b for a, b in zip(seed, end))
                solution = end if end is not None and fraction == 1 else self.inverse(line.sample(fraction), seed)
                joints = self.checked_joints(solution,
                                              f'IK at normalized time {fraction:.5f}')
                if max(abs(a - b) for a, b in zip(joints, positions[-1])) > self.max_joint_step:
                    last_problem = f'IK joint discontinuity near normalized time {fraction:.5f}'
                    break
                positions.append(joints)
            if len(positions) == count:
                path = JointPath(fractions, positions)
                if any(a < low - 1e-8 or b > high + 1e-8 for a, b, low, high in
                       zip(path.min_position, path.max_position, self.lower, self.upper)):
                    last_problem = 'Joint interpolation overshoots position limits'
                else:
                    checks = path.validation_fractions()
                    poses = self.forward([path.sample(t).position for t in checks])
                    if len(poses) != len(checks):
                        raise ValueError('FK returned the wrong number of validation poses')
                    errors = [pose_error(actual, line.sample(t)) for t, actual in zip(checks, poses)]
                    position_error = max(error[0] for error in errors)
                    orientation_error = max(error[1] for error in errors)
                    if position_error <= self.path_tolerance and orientation_error <= self.orientation_tolerance:
                        trajectory = TimedJointPath(path, max_velocity, max_acceleration, duration)
                        return CartesianPlan(line, trajectory, count, position_error, orientation_error)
                    last_problem = (f'FK deviation {position_error:.6f} m, {orientation_error:.6f} rad '
                                    'exceeds Cartesian interpolation tolerance')
            count = 2 * count - 1
        raise ValueError(last_problem + f'; cannot validate a continuous IK path within {self.max_samples} samples')
