"""Straight Cartesian paths and continuous joint-path planning."""

from __future__ import annotations

import math

from ...config import MIN_IK_SAMPLES, CartesianMotionOptions, vector
from ..types import CartesianPlan, CartesianPose, pose_error
from .joint import JointPath, QuinticTrajectory
from .timing import TimedJointPath

SLERP_LINEAR_THRESHOLD = 0.9995
ANGULAR_EPSILON = 1e-12
DEFAULT_ORIENTATION_TOLERANCE = (
    CartesianMotionOptions.path_orientation_tolerance
)


class CartesianLine:
    """
    Straight-line reference with shortest SLERP and quintic time scaling.

    ``sample(u)`` uses normalized time u=t/T. Velocity/acceleration of XYZ
    can be
    obtained by differentiating the shared scalar ``progress`` trajectory.
    """

    def __init__(self, start, offset_cm, end_orientation=None):
        self.start = start
        self.offset_cm = vector(offset_cm, 3, 'offset_cm')
        self.delta = tuple(value / 100 for value in self.offset_cm)
        self.end = CartesianPose(
            tuple(
                a + d for a, d in zip(start.position, self.delta, strict=False)
            ),
            start.orientation if end_orientation is None else end_orientation,
            start.frame,
        )
        self.length = math.sqrt(sum(value * value for value in self.delta))
        self.angle = pose_error(self.start, self.end)[1]
        self.progress = QuinticTrajectory([0], [1], 1.875, duration=1.0)

    @classmethod
    def between(cls, start, end):
        if start.frame != end.frame:
            msg = 'Cartesian pose frame mismatch'
            raise ValueError(msg)
        return cls(
            start,
            tuple(
                100 * (b - a)
                for a, b in zip(start.position, end.position, strict=False)
            ),
            end.orientation,
        )

    def sample(self, normalized_time):
        progress = self.progress.sample(normalized_time).position[0]
        return CartesianPose(
            tuple(
                a + d * progress
                for a, d in zip(self.start.position, self.delta, strict=False)
            ),
            slerp(self.start.orientation, self.end.orientation, progress),
            self.start.frame,
        )


def slerp(start, end, fraction):
    """
    Shortest rotation between unit quaternions, including antipodal inputs.
    """
    dot = sum(a * b for a, b in zip(start, end, strict=False))
    if dot < 0:
        end, dot = tuple(-value for value in end), -dot
    dot = min(1.0, dot)
    if dot > SLERP_LINEAR_THRESHOLD:
        values = tuple(
            a + fraction * (b - a) for a, b in zip(start, end, strict=False)
        )
    else:
        angle = math.acos(dot)
        left, right = (
            math.sin((1 - fraction) * angle),
            math.sin(fraction * angle),
        )
        values = tuple(
            (left * a + right * b) / math.sin(angle)
            for a, b in zip(start, end, strict=False)
        )
    norm = math.sqrt(sum(value * value for value in values))
    return tuple(value / norm for value in values)


class CartesianPlanner:
    """
    Seed IK from the previous solution, fit a C2 joint path, validate with FK.

    IK accepts (CartesianPose, seed) and returns ordered joints. FK accepts a
    sequence of joint configurations and returns corresponding CartesianPoses;
    it must raise on invalid/colliding configurations. Position bounds are
    checked
    at polynomial extrema, not only at IK samples. Cartesian FK validation uses
    knots and quarter/midpoints; it is a sampled tolerance check, not a
    proof of
    continuous collision freedom or exact Cartesian tracking between samples.
    """

    def __init__(  # noqa: PLR0913
        self,
        inverse,
        forward,
        lower,
        upper,
        *,
        sample_step=CartesianMotionOptions.sample_step,
        path_tolerance=CartesianMotionOptions.path_tolerance,
        orientation_tolerance=DEFAULT_ORIENTATION_TOLERANCE,
        max_joint_step=CartesianMotionOptions.max_joint_step,
        max_samples=CartesianMotionOptions.max_samples,
        joint_names=None,
        angular_step=CartesianMotionOptions.angular_step,
    ):
        self.inverse, self.forward = inverse, forward
        self.lower = tuple(lower)
        self.upper = vector(upper, len(self.lower), 'upper joint limits')
        self.lower = vector(self.lower, len(self.lower), 'lower joint limits')
        if not self.lower or any(
            a >= b for a, b in zip(self.lower, self.upper, strict=False)
        ):
            msg = 'Invalid joint position limits'
            raise ValueError(msg)
        self.joint_names = (
            tuple(joint_names)
            if joint_names is not None
            else tuple(f'joint[{i}]' for i in range(len(self.lower)))
        )
        if len(self.joint_names) != len(self.lower):
            msg = 'Joint names and position bounds must have equal lengths'
            raise ValueError(msg)
        for label, value in (
            ('sample_step', sample_step),
            ('path_tolerance', path_tolerance),
            ('orientation_tolerance', orientation_tolerance),
            ('max_joint_step', max_joint_step),
            ('angular_step', angular_step),
        ):
            if not math.isfinite(value) or value <= 0:
                msg = f'{label} must be positive and finite'
                raise ValueError(msg)
        if not isinstance(max_samples, int) or max_samples < MIN_IK_SAMPLES:
            msg = 'max_samples must be an integer >=11'
            raise ValueError(msg)
        self.sample_step, self.path_tolerance = sample_step, path_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.max_joint_step, self.max_samples = max_joint_step, max_samples
        self.angular_step = angular_step

    def checked_joints(self, values, label):
        joints = vector(values, len(self.lower), label)
        for name, q, low, high in zip(
            self.joint_names, joints, self.lower, self.upper, strict=False
        ):
            if q < low - 1e-8 or q > high + 1e-8:
                msg = (
                    f'{label}: {name}={q:.6f} outside position limits '
                    f'[{low:.6f}, {high:.6f}]'
                )
                raise ValueError(msg)
        return joints

    def plan(  # noqa: PLR0913
        self,
        start_pose,
        offset_cm,
        start_joints,
        max_velocity,
        max_acceleration,
        duration=None,
    ):
        line = CartesianLine(start_pose, offset_cm)
        return self.plan_line(
            line, start_joints, max_velocity, max_acceleration, duration
        )

    def plan_pose(  # noqa: PLR0913
        self,
        start_pose,
        end_pose,
        start_joints,
        max_velocity,
        max_acceleration,
        duration=None,
        end_joints=None,
    ):
        """
        Straight XYZ and shortest SLERP, optionally meeting a specific
        joint branch.

        A joint-constrained endpoint biases IK seeds toward that posture and is
        still subject to continuity, FK and position-limit checks. A
        branch that
        cannot connect continuously is rejected rather than jumped to.
        """
        return self.plan_line(
            CartesianLine.between(start_pose, end_pose),
            start_joints,
            max_velocity,
            max_acceleration,
            duration,
            end_joints,
        )

    def plan_line(  # noqa: PLR0912, PLR0913
        self,
        line,
        start_joints,
        max_velocity,
        max_acceleration,
        duration=None,
        end_joints=None,
    ):
        import numpy as np

        start = self.checked_joints(start_joints, 'Start joints')
        end = (
            None
            if end_joints is None
            else self.checked_joints(end_joints, 'Target joints')
        )
        actual = self.forward([start])
        if len(actual) != 1:
            msg = 'FK returned the wrong number of start poses'
            raise ValueError(msg)
        position_error, orientation_error = pose_error(actual[0], line.start)
        if (
            position_error > self.path_tolerance
            or orientation_error > self.orientation_tolerance
        ):
            msg = (
                f'Current endpoint does not match selected IK tip/FK: '
                f'{position_error:.6f} m, {orientation_error:.6f} rad'
            )
            raise ValueError(msg)
        if (
            line.length == 0
            and line.angle < ANGULAR_EPSILON
            and (end is None or end == start)
        ):
            path = JointPath((0.0, 0.5, 1.0), (start, start, start))
            return CartesianPlan(
                line,
                TimedJointPath(path, max_velocity, max_acceleration, duration),
                3,
                position_error,
                orientation_error,
            )
        count = max(
            11,
            math.ceil(1.875 * line.length / self.sample_step) + 1,
            math.ceil(1.875 * line.angle / self.angular_step) + 1,
        )
        if count > self.max_samples:
            msg = (
                f'Path needs at least {count} IK samples; increase '
                f'max_samples or split the move'
            )
            raise ValueError(msg)
        last_problem = ''
        while count <= self.max_samples:
            fractions = tuple(float(t) for t in np.linspace(0, 1, count))
            positions = [start]
            for fraction in fractions[1:]:
                seed = positions[-1]
                if end is not None:
                    progress = line.progress.sample(fraction).position[0]
                    seed = tuple(
                        (1 - progress) * a + progress * b
                        for a, b in zip(seed, end, strict=False)
                    )
                solution = (
                    end
                    if end is not None and fraction == 1
                    else self.inverse(line.sample(fraction), seed)
                )
                joints = self.checked_joints(
                    solution, f'IK at normalized time {fraction:.5f}'
                )
                if (
                    max(
                        abs(a - b)
                        for a, b in zip(joints, positions[-1], strict=False)
                    )
                    > self.max_joint_step
                ):
                    last_problem = (
                        f'IK joint discontinuity near normalized time '
                        f'{fraction:.5f}'
                    )
                    break
                positions.append(joints)
            if len(positions) == count:
                path = JointPath(fractions, positions)
                if any(
                    a < low - 1e-8 or b > high + 1e-8
                    for a, b, low, high in zip(
                        path.min_position,
                        path.max_position,
                        self.lower,
                        self.upper,
                        strict=False,
                    )
                ):
                    last_problem = (
                        'Joint interpolation overshoots position limits'
                    )
                else:
                    checks = path.validation_fractions()
                    poses = self.forward(
                        [path.sample(t).position for t in checks]
                    )
                    if len(poses) != len(checks):
                        msg = (
                            'FK returned the wrong number of validation poses'
                        )
                        raise ValueError(msg)
                    errors = [
                        pose_error(actual, line.sample(t))
                        for t, actual in zip(checks, poses, strict=False)
                    ]
                    position_error = max(error[0] for error in errors)
                    orientation_error = max(error[1] for error in errors)
                    if (
                        position_error <= self.path_tolerance
                        and orientation_error <= self.orientation_tolerance
                    ):
                        trajectory = TimedJointPath(
                            path, max_velocity, max_acceleration, duration
                        )
                        return CartesianPlan(
                            line,
                            trajectory,
                            count,
                            position_error,
                            orientation_error,
                        )
                    last_problem = (
                        f'FK deviation {position_error:.6f} m, '
                        f'{orientation_error:.6f} rad '
                        'exceeds Cartesian interpolation tolerance'
                    )
            count = 2 * count - 1
        raise ValueError(
            last_problem
            + (
                f'; cannot validate a continuous IK path within '
                f'{self.max_samples} samples'
            )
        )
