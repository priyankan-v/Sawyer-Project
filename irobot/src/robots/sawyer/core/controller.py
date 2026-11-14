"""Sawyer motion primitives and trajectory generation built on SawyerBase."""

# Named trajectory bounds and retry/stop scopes stay explicit for review.
from __future__ import annotations

import math
import os
import time
import xmlrpc.client
from bisect import bisect_right
from dataclasses import dataclass, replace

from irobot.src.robots.sawyer.config import (
    JOINT_NAMES,
    MIN_IK_SAMPLES,
    CartesianMotionOptions,
)
from irobot.src.robots.sawyer.core.base import (
    CartesianPose,
    InteraKinematics,
    MotionResult,
    SawyerBase,
    ordered_values,
    pose_error,
    vector,
)

# Mathematical interpolation constants, independent of robot hardware settings.
SLERP_LINEAR_THRESHOLD = 0.9995
POLYNOMIAL_ROOT_IMAG_TOLERANCE = 1e-8
MIN_PATH_KNOTS = 3
PATH_ARRAY_DIMENSIONS = 2
ANGULAR_EPSILON = 1e-12
DEFAULT_ORIENTATION_TOLERANCE = (
    CartesianMotionOptions.path_orientation_tolerance
)


@dataclass(frozen=True)
class TrajectorySample:
    position: tuple
    velocity: tuple
    acceleration: tuple
    jerk: tuple


class QuinticTrajectory:
    """
    q(t) = q0 + (q1-q0)*(10*u^3 - 15*u^4 + 6*u^5), u=t/T.

    All joints share T and have zero endpoint velocity and acceleration. Units
    are radians and seconds. Limits accept scalars or per-joint vectors.
    Minimum T for this polynomial family is max(15*abs(delta)/(8*vmax)),
    optionally also max(sqrt((10/sqrt(3))*abs(delta)/amax)). This is not a
    globally time-optimal trajectory among arbitrary motion profiles.
    Too-short explicit durations are rejected; stationary auto-duration is
    zero.
    """

    def __init__(
        self, start, end, max_velocity, duration=None, max_acceleration=None
    ):
        self.start = tuple(float(value) for value in start)
        if not self.start:
            msg = 'At least one joint is required'
            raise ValueError(msg)
        size = len(self.start)
        self.start = vector(self.start, size, 'start')
        self.end = vector(end, size, 'end')
        self.delta = vector(
            [b - a for a, b in zip(self.start, self.end, strict=False)],
            size,
            'displacement',
        )
        self.max_velocity = vector(
            max_velocity, size, 'max_velocity', positive=True
        )
        self.max_acceleration = (
            None
            if max_acceleration is None
            else vector(
                max_acceleration, size, 'max_acceleration', positive=True
            )
        )
        self.velocity_duration = max(
            1.875 * abs(d) / v
            for d, v in zip(self.delta, self.max_velocity, strict=False)
        )
        self.acceleration_duration = (
            0.0
            if self.max_acceleration is None
            else max(
                math.sqrt((10 / math.sqrt(3)) * abs(d) / a)
                for d, a in zip(
                    self.delta, self.max_acceleration, strict=False
                )
            )
        )
        self.minimum_duration = max(
            self.velocity_duration, self.acceleration_duration
        )
        if not math.isfinite(self.minimum_duration):
            msg = 'Joint displacement/limits produce a non-finite duration'
            raise ValueError(msg)
        if duration is None:
            duration = self.minimum_duration
        else:
            duration = float(duration)
            if not math.isfinite(duration) or duration <= 0:
                msg = 'Requested duration must be positive and finite'
                raise ValueError(msg)
            if duration < self.minimum_duration:
                msg = (
                    f'Requested duration {duration:.6f} s is too short; '
                    f'minimum feasible quintic duration is '
                    f'{self.minimum_duration:.6f} s'
                )
                raise ValueError(msg)
        self.duration = duration

    def sample(self, elapsed):
        elapsed = float(elapsed)
        if not math.isfinite(elapsed):
            msg = 'Sample time must be finite'
            raise ValueError(msg)
        zero = (0.0,) * len(self.start)
        if self.duration == 0 or elapsed >= self.duration:
            return TrajectorySample(self.end, zero, zero, zero)
        if elapsed <= 0:
            return TrajectorySample(self.start, zero, zero, zero)
        u = elapsed / self.duration
        s = u**3 * (10 - 15 * u + 6 * u**2)
        ds = 30 * u**2 * (1 - u) ** 2 / self.duration
        dds = 60 * u * (1 - u) * (1 - 2 * u) / self.duration**2
        ddds = 60 * (1 - 6 * u + 6 * u**2) / self.duration**3
        return TrajectorySample(
            tuple(
                a + d * s for a, d in zip(self.start, self.delta, strict=False)
            ),
            tuple(d * ds for d in self.delta),
            tuple(d * dds for d in self.delta),
            tuple(d * ddds for d in self.delta),
        )


class JointSequence:
    """Timed joint waypoints joined with rest-to-rest quintic segments."""

    def __init__(self, start, positions, times, velocity, acceleration):
        self.start = vector(start, len(JOINT_NAMES), 'Start joints')
        if not positions or len(positions) != len(times):
            msg = 'Provide matching nonempty positions and times'
            raise ValueError(msg)
        self.segments, self.starts = [], []
        previous, previous_time = self.start, 0.0
        for position, timestamp in zip(positions, times, strict=True):
            target = vector(position, len(JOINT_NAMES), 'Waypoint')
            if not math.isfinite(timestamp) or timestamp <= previous_time:
                msg = 'Waypoint times must increase from zero'
                raise ValueError(msg)
            self.starts.append(previous_time)
            self.segments.append(
                QuinticTrajectory(
                    previous,
                    target,
                    velocity,
                    timestamp - previous_time,
                    acceleration,
                )
            )
            previous, previous_time = target, timestamp
        self.end, self.duration = previous, previous_time
        self.max_velocity = tuple(velocity)
        self.max_acceleration = tuple(acceleration)
        points = [self.start, *[segment.end for segment in self.segments]]
        self.min_position = tuple(map(min, zip(*points, strict=True)))
        self.max_position = tuple(map(max, zip(*points, strict=True)))

    def sample(self, elapsed):
        if not math.isfinite(elapsed):
            msg = 'Sample time must be finite'
            raise ValueError(msg)
        index = max(0, bisect_right(self.starts, elapsed) - 1)
        return self.segments[index].sample(elapsed - self.starts[index])


class JointPIDController:
    """
    Velocity feedforward + Kp*position_error + Ki*integral + Kd*velocity_error.

    Gains may be scalars or per-joint vectors. Derivative uses measured
    velocity,
    avoiding numerical differentiation of noisy positions. Conditional
    integration
    prevents windup against velocity, acceleration slew, and optional position
    bounds. Position bounds take priority if they conflict with acceleration
    slew bounds. These command bounds cannot guarantee physical tracking
    limits.
    """

    def __init__(  # noqa: PLR0913
        self,
        size,
        kp,
        ki,
        kd,
        max_velocity,
        max_acceleration,
        integral_limit=0.25,
        position_lower=None,
        position_upper=None,
    ):
        if not isinstance(size, int) or size <= 0:
            msg = 'size must be a positive integer'
            raise ValueError(msg)
        self.size = size
        self.kp = vector(kp, size, 'kp', nonnegative=True)
        self.ki = vector(ki, size, 'ki', nonnegative=True)
        self.kd = vector(kd, size, 'kd', nonnegative=True)
        self.max_velocity = vector(
            max_velocity, size, 'max_velocity', positive=True
        )
        self.max_acceleration = vector(
            max_acceleration, size, 'max_acceleration', positive=True
        )
        self.integral_limit = vector(
            integral_limit, size, 'integral_limit', nonnegative=True
        )
        if (position_lower is None) != (position_upper is None):
            msg = 'Supply both position_lower and position_upper'
            raise ValueError(msg)
        self.position_lower = (
            None
            if position_lower is None
            else vector(position_lower, size, 'position_lower')
        )
        self.position_upper = (
            None
            if position_upper is None
            else vector(position_upper, size, 'position_upper')
        )
        if self.position_lower is not None and any(
            a >= b
            for a, b in zip(
                self.position_lower, self.position_upper, strict=False
            )
        ):
            msg = 'Position lower bounds must be below upper bounds'
            raise ValueError(msg)
        self.reset()

    def reset(self, velocity=None):
        self.integral = [0.0] * self.size
        self.previous_velocity = list(
            vector(
                [0.0] * self.size if velocity is None else velocity,
                self.size,
                'initial velocity',
            )
        )
        if any(
            abs(v) > limit
            for v, limit in zip(
                self.previous_velocity, self.max_velocity, strict=False
            )
        ):
            msg = 'Initial velocity exceeds limits'
            raise ValueError(msg)

    def update(
        self,
        reference_position,
        measured_position,
        reference_velocity,
        measured_velocity,
        dt,
    ):
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0:
            msg = 'dt must be positive and finite'
            raise ValueError(msg)
        qref = vector(reference_position, self.size, 'reference_position')
        q = vector(measured_position, self.size, 'measured_position')
        vref = vector(reference_velocity, self.size, 'reference_velocity')
        velocity = vector(measured_velocity, self.size, 'measured_velocity')
        output = []
        for i in range(self.size):
            error = qref[i] - q[i]
            limit = self.integral_limit[i]
            integral = max(-limit, min(limit, self.integral[i] + error * dt))
            base = (
                vref[i]
                + self.kp[i] * error
                + self.kd[i] * (vref[i] - velocity[i])
            )
            low, high = -self.max_velocity[i], self.max_velocity[i]
            if self.position_lower is not None:
                below = max(0.0, q[i] - self.position_lower[i])
                above = max(0.0, self.position_upper[i] - q[i])
                # Stopping-distance envelope and one-step position constraint.
                low = max(
                    low,
                    -math.sqrt(2 * self.max_acceleration[i] * below),
                    -below / dt,
                )
                high = min(
                    high,
                    math.sqrt(2 * self.max_acceleration[i] * above),
                    above / dt,
                )
            lower = max(
                low, self.previous_velocity[i] - self.max_acceleration[i] * dt
            )
            upper = min(
                high, self.previous_velocity[i] + self.max_acceleration[i] * dt
            )
            if lower > upper:
                lower, upper = low, high
            candidate = base + self.ki[i] * integral
            # Integrate when unsaturated, or when error unwinds saturation.
            if (
                lower <= candidate <= upper
                or (candidate > upper and error < 0)
                or (candidate < lower and error > 0)
            ):
                self.integral[i] = integral
            command = max(
                lower, min(upper, base + self.ki[i] * self.integral[i])
            )
            output.append(command)
        self.previous_velocity = output
        return tuple(output)


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


def extrema(coefficients):
    """Polynomial extrema over [0,1], including both endpoints."""
    import numpy as np
    from numpy.polynomial import polynomial

    derivative = polynomial.polyder(coefficients)
    roots = polynomial.polyroots(derivative) if np.any(derivative) else []
    candidates = [0.0, 1.0] + [
        float(root.real)
        for root in roots
        if abs(root.imag) < POLYNOMIAL_ROOT_IMAG_TOLERANCE
        and 0 < root.real < 1
    ]
    values = [
        float(polynomial.polyval(point, coefficients)) for point in candidates
    ]
    return min(values), max(values)


class JointPath:
    """
    Piecewise quintic Hermite q(u) through IK samples, u in [0,1].

    Interior derivatives use local polynomial fits of degree up to five.
    Endpoint velocity
    and acceleration are explicitly zero. Position, first derivative and second
    derivative are continuous across knots. This joint interpolation must be
    checked against the desired Cartesian reference using FK before execution.
    """

    def __init__(self, fractions, positions):
        import numpy as np
        from numpy.polynomial import polynomial

        self.fractions = tuple(float(value) for value in fractions)
        values = np.asarray(positions, dtype=float)
        if (
            len(self.fractions) < MIN_PATH_KNOTS
            or values.ndim != PATH_ARRAY_DIMENSIONS
            or values.shape[0] != len(self.fractions)
            or values.shape[1] == 0
            or not np.all(np.isfinite(values))
            or not all(math.isfinite(t) for t in self.fractions)
            or self.fractions[0] != 0
            or self.fractions[-1] != 1
            or any(
                a >= b
                for a, b in zip(
                    self.fractions, self.fractions[1:], strict=False
                )
            )
        ):
            msg = (
                'JointPath needs >=3 finite, increasing knots from 0 to 1 '
                'and matching joint vectors'
            )
            raise ValueError(msg)
        self.size = values.shape[1]
        self.start, self.end = (
            tuple(map(float, values[0])),
            tuple(map(float, values[-1])),
        )
        first, second = np.zeros_like(values), np.zeros_like(values)
        window = min(7, len(self.fractions))
        times = np.asarray(self.fractions)
        for i, center in enumerate(times):
            left = max(0, min(i - window // 2, len(times) - window))
            indices = slice(left, left + window)
            scale = max(abs(times[indices] - center))
            # Center/scale the stencil to avoid ill-conditioning near the ends.
            fitted = polynomial.polyfit(
                (times[indices] - center) / scale,
                values[indices],
                min(5, window - 1),
            )
            first[i] = fitted[1] / scale
            second[i] = 2 * fitted[2] / scale**2
        stationary = np.all(values == values[0], axis=0)
        first[:, stationary], second[:, stationary] = 0.0, 0.0
        first[[0, -1]], second[[0, -1]] = 0.0, 0.0
        self.segments = []
        self.min_position = np.full(self.size, math.inf)
        self.max_position = np.full(self.size, -math.inf)
        self.peak_velocity = np.zeros(self.size)
        self.peak_acceleration = np.zeros(self.size)
        for i, (left, right) in enumerate(
            zip(self.fractions, self.fractions[1:], strict=False)
        ):
            h = right - left
            c0, c1, c2 = values[i], first[i] * h, 0.5 * second[i] * h**2
            a = values[i + 1] - c0 - c1 - c2
            b = first[i + 1] * h - c1 - 2 * c2
            c = second[i + 1] * h**2 - 2 * c2
            coefficients = np.stack(
                (
                    c0,
                    c1,
                    c2,
                    10 * a - 4 * b + 0.5 * c,
                    -15 * a + 7 * b - c,
                    6 * a - 3 * b + 0.5 * c,
                ),
                axis=1,
            )
            self.segments.append(coefficients)
            for joint, coeff in enumerate(coefficients):
                low, high = extrema(coeff)
                self.min_position[joint] = min(self.min_position[joint], low)
                self.max_position[joint] = max(self.max_position[joint], high)
                for derivative, peaks in (
                    (1, self.peak_velocity),
                    (2, self.peak_acceleration),
                ):
                    low, high = extrema(
                        polynomial.polyder(coeff, derivative) / h**derivative
                    )
                    peaks[joint] = max(peaks[joint], abs(low), abs(high))

    def sample(self, fraction):
        from numpy.polynomial import polynomial

        if not math.isfinite(fraction):
            msg = 'Normalized time must be finite'
            raise ValueError(msg)
        zero = (0.0,) * self.size
        if fraction <= 0:
            return TrajectorySample(self.start, zero, zero, zero)
        if fraction >= 1:
            return TrajectorySample(self.end, zero, zero, zero)
        index = min(
            bisect_right(self.fractions, fraction) - 1, len(self.segments) - 1
        )
        h = self.fractions[index + 1] - self.fractions[index]
        x = (fraction - self.fractions[index]) / h
        fields = [
            tuple(
                float(
                    polynomial.polyval(
                        x, polynomial.polyder(coeff, derivative)
                    )
                )
                / h**derivative
                for coeff in self.segments[index]
            )
            for derivative in range(4)
        ]
        return TrajectorySample(*fields)

    def validation_fractions(self):
        return tuple(
            sorted(
                set(self.fractions)
                | {
                    left + (right - left) * fraction
                    for left, right in zip(
                        self.fractions, self.fractions[1:], strict=False
                    )
                    for fraction in (0.25, 0.5, 0.75)
                }
            )
        )


class TimedJointPath:
    """
    Shortest uniform time scale for the constructed polynomial joint path.

    T >= peak|dq/du|/vmax and T >= sqrt(peak|d2q/du2|/amax). It is not a
    globally time-optimal Cartesian planner. Bounds are for reference commands,
    not guaranteed physical tracking under PID corrections/network latency.
    """

    def __init__(self, path, max_velocity, max_acceleration, duration=None):
        self.path = path
        self.start, self.end = path.start, path.end
        self.min_position = tuple(map(float, path.min_position))
        self.max_position = tuple(map(float, path.max_position))
        self.max_velocity = vector(
            max_velocity, path.size, 'max_velocity', positive=True
        )
        self.max_acceleration = vector(
            max_acceleration, path.size, 'max_acceleration', positive=True
        )
        self.velocity_duration = max(
            float(v) / limit
            for v, limit in zip(
                path.peak_velocity, self.max_velocity, strict=False
            )
        )
        self.acceleration_duration = math.sqrt(
            max(
                float(a) / limit
                for a, limit in zip(
                    path.peak_acceleration, self.max_acceleration, strict=False
                )
            )
        )
        self.minimum_duration = max(
            self.velocity_duration, self.acceleration_duration
        )
        if not math.isfinite(self.minimum_duration):
            msg = 'Non-finite joint path duration'
            raise ValueError(msg)
        if duration is not None:
            if not math.isfinite(duration) or duration <= 0:
                msg = 'Requested duration must be positive and finite'
                raise ValueError(msg)
            if duration < self.minimum_duration:
                msg = (
                    f'Requested duration {duration:.6f} s is too short; '
                    f'minimum feasible '
                    f'duration for the validated IK reference is '
                    f'{self.minimum_duration:.6f} s'
                )
                raise ValueError(msg)
        self.duration = self.minimum_duration if duration is None else duration

    def sample(self, elapsed):
        if not math.isfinite(elapsed):
            msg = 'Sample time must be finite'
            raise ValueError(msg)
        if self.duration == 0:
            zero = (0.0,) * len(self.end)
            return TrajectorySample(self.end, zero, zero, zero)
        reference = self.path.sample(elapsed / self.duration)
        return TrajectorySample(
            reference.position,
            tuple(v / self.duration for v in reference.velocity),
            tuple(a / self.duration**2 for a in reference.acceleration),
            tuple(j / self.duration**3 for j in reference.jerk),
        )


@dataclass(frozen=True)
class CartesianPlan:
    line: CartesianLine
    trajectory: TimedJointPath
    samples: int
    max_position_error: float
    max_orientation_error: float


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


@dataclass(frozen=True)
class CartesianMoveResult:
    plan: CartesianPlan
    execution: MotionResult | None


@dataclass(frozen=True)
class JointMoveResult:
    plan: object
    execution: object | None


class ParameterTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection


class SawyerController(SawyerBase):
    """Synchronous direct arm/gripper control; movement previews by default."""

    def run_interaction(  # noqa: PLR0913
        self,
        message,
        duration,
        *,
        execute=False,
        limits=None,
        stream_options=None,
        cancel_event=None,
        on_step=None,
    ):
        """Run an explicit bounded interaction session, then confirm exit."""
        from .interaction import run_interaction

        return run_interaction(
            self,
            message,
            duration,
            execute=execute,
            limits=limits,
            stream_options=stream_options,
            cancel_event=cancel_event,
            on_step=on_step,
        )

    def stream_joint_commands(  # noqa: PLR0913
        self,
        mode,
        source,
        duration,
        *,
        options=None,
        execute=False,
        cancel_event=None,
        on_step=None,
    ):
        """Bounded position/velocity/torque/trajectory session; SI units."""
        from .control import stream_joint_commands

        return stream_joint_commands(
            self,
            mode,
            source,
            duration,
            options=options,
            execute=execute,
            cancel_event=cancel_event,
            on_step=on_step,
        )

    def move_native_joint_path(  # noqa: PLR0913
        self,
        positions,
        *,
        execute=False,
        plan_only=False,
        speed=0.05,
        max_acceleration=0.1,
        timeout=60.0,
        cancel_event=None,
        on_feedback=None,
    ):
        """Use Intera's planner/action server through the direct transport."""
        from .motion import run_joint_path

        return run_joint_path(
            self,
            positions,
            execute=execute,
            plan_only=plan_only,
            speed=speed,
            max_acceleration=max_acceleration,
            timeout=timeout,
            cancel_event=cancel_event,
            on_feedback=on_feedback,
        )

    def execute_native_trajectory(  # noqa: PLR0913
        self,
        trajectory,
        *,
        execute=False,
        plan_only=False,
        timeout=60.0,
        cancel_event=None,
        on_feedback=None,
    ):
        """
        Validate a typed joint/Cartesian Intera path, then preview or
        run.
        """
        from .motion import run_trajectory

        return run_trajectory(
            self,
            trajectory,
            execute=execute,
            plan_only=plan_only,
            timeout=timeout,
            cancel_event=cancel_event,
            on_feedback=on_feedback,
        )

    def calibrate_arm(
        self,
        *,
        execute=False,
        timeout=120.0,
        cancel_event=None,
        on_feedback=None,
    ):
        """
        Explicit native homing; the controller requires the gripper
        removed.
        """
        from ..config import positive
        from . import wire

        positive('timeout', timeout)
        if type(execute) is not bool:
            msg = 'execute must be a boolean'
            raise TypeError(msg)
        with self._operation():
            self._refresh()
            attached = any(
                topic == self.config.gripper_namespace + '/state'
                for topic, _ in self._node.master('getTopicTypes')
            )
            if not execute:
                return {
                    'executed': False,
                    'available': not attached,
                    'precondition': (
                        'Remove/configure out the gripper before arm '
                        'calibration'
                    ),
                }
            if attached:
                msg = (
                    'Arm calibration requires the gripper '
                    'removed/configured out'
                )
                raise RuntimeError(msg)
            self._node.cancel_requested.clear()
            self._node.check_ready()
            return self._actions.run(
                wire.CalibrationCommandGoal(command='start'),
                name='calibration',
                timeout=timeout,
                cancel_event=cancel_event,
                on_feedback=on_feedback,
            )

    def _neutral_pose(self):
        if self.config.neutral_joints is not None:
            return tuple(self.config.neutral_joints)
        uri = self.config.ros_master_uri or os.environ.get('ROS_MASTER_URI')
        if not uri:
            msg = (
                'Set ROS_MASTER_URI or SawyerConfig.neutral_joints for '
                'neutral motion'
            )
            raise RuntimeError(msg)
        transport = ParameterTransport(self.config.parameter_timeout)
        with xmlrpc.client.ServerProxy(uri, transport=transport) as master:
            code, message, named = master.getParam(
                '/irobot_sawyer', self.config.neutral_parameter
            )
        if code != 1:
            msg = f'Cannot read {self.config.neutral_parameter}: {message}'
            raise RuntimeError(msg)
        if len(named['joint_names']) != len(JOINT_NAMES):
            msg = (
                'Neutral pose must contain exactly the seven right-arm joints'
            )
            raise ValueError(msg)
        return ordered_values(
            named['joint_names'], named['poses']['neutral'], 'Neutral pose'
        )

    def _move_joints(
        self, target, options, *, cancel_event=None, on_step=None
    ):
        node = self._node
        self._refresh()
        node.wait_inputs(options.execute)
        start = tuple(node.joints)
        node.limits.check_position(start, 'Current position')
        node.limits.check_position(target, 'Target position')
        velocity = tuple(options.speed * v for v in node.limits.velocity)
        acceleration = tuple(
            min(a, options.max_accel) if options.max_accel is not None else a
            for a in node.limits.acceleration
        )
        plan = QuinticTrajectory(
            start,
            target,
            velocity,
            duration=options.time,
            max_acceleration=acceleration,
        )
        return self._run_joint_plan(
            plan,
            options,
            velocity,
            acceleration,
            cancel_event=cancel_event,
            on_step=on_step,
        )

    def _run_joint_plan(  # noqa: PLR0913
        self,
        plan,
        options,
        velocity,
        acceleration,
        *,
        cancel_event=None,
        on_step=None,
    ):
        node = self._node
        if not options.execute:
            return JointMoveResult(plan, None)
        node.wait_inputs(True)
        node.check_ready()
        if max(map(abs, node.velocity)) >= self.config.velocity_tolerance:
            msg = 'Rest-to-rest motion requires a stationary arm'
            raise RuntimeError(msg)
        if (
            max(
                abs(a - b)
                for a, b in zip(plan.start, node.joints, strict=False)
            )
            > self.config.planning_joint_tolerance
        ):
            msg = 'Arm moved during planning; rerun from the new pose'
            raise RuntimeError(msg)
        if node.cancel_requested.is_set() or (
            cancel_event is not None and cancel_event.is_set()
        ):
            msg = 'Motion cancelled'
            raise RuntimeError(msg)
        if plan.duration == 0 or (
            not isinstance(plan, JointSequence)
            and max(
                abs(a - b) for a, b in zip(plan.end, node.joints, strict=True)
            )
            < self.config.position_tolerance
        ):
            return JointMoveResult(plan, None)
        controller = JointPIDController(
            7,
            options.kp,
            options.ki,
            options.kd,
            velocity,
            acceleration,
            position_lower=node.limits.lower,
            position_upper=node.limits.upper,
        )

        def monitor(elapsed, reference):
            if cancel_event is not None and cancel_event.is_set():
                msg = 'Motion cancelled'
                raise RuntimeError(msg)
            if on_step is not None:
                on_step(
                    elapsed,
                    reference,
                    tuple(node.joints),
                    tuple(node.velocity),
                )

        execution = node.track(
            plan,
            controller,
            options,
            label='Joint target',
            announce=False,
            on_step=monitor,
        )
        return JointMoveResult(plan, execution)

    def move_to_joints(  # noqa: PLR0913
        self,
        joints,
        *,
        duration=None,
        execute=False,
        options=None,
        cancel_event=None,
        on_step=None,
    ):
        """Preview/execute seven right_j0..right_j6 angles in radians."""
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            return self._move_joints(
                vector(joints, len(JOINT_NAMES), 'Target joints'),
                self._motion_options(
                    options, duration=duration, execute=execute
                ),
                cancel_event=cancel_event,
                on_step=on_step,
            )

    def move_to_neutral(
        self,
        *,
        duration=None,
        execute=False,
        options=None,
        cancel_event=None,
        on_step=None,
    ):
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            return self._move_joints(
                self._neutral_pose(),
                self._motion_options(
                    options, duration=duration, execute=execute
                ),
                cancel_event=cancel_event,
                on_step=on_step,
            )

    def move_trajectory(  # noqa: PLR0913
        self,
        positions,
        times,
        *,
        execute=False,
        options=None,
        cancel_event=None,
        on_step=None,
    ):
        """
        Preview/execute waypoints; each segment ends at rest.

        Times are seconds from the start and must strictly increase above
        zero. on_step must be fast and nonblocking: it runs on the control
        thread. An external cancel_event is never cleared by this driver.
        """
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            motion = self._motion_options(options, execute=execute)
            if motion.time is not None:
                msg = 'Use waypoint times instead of options.time'
                raise ValueError(msg)
            self._refresh()
            self._node.wait_inputs(execute)
            limits = self._node.limits
            velocity = tuple(motion.speed * v for v in limits.velocity)
            acceleration = tuple(
                min(a, motion.max_accel) if motion.max_accel else a
                for a in limits.acceleration
            )
            plan = JointSequence(
                self._node.joints, positions, times, velocity, acceleration
            )
            limits.check_position(plan.start, 'Start joints')
            for segment in plan.segments:
                limits.check_position(segment.end, 'Waypoint')
            return self._run_joint_plan(
                plan,
                motion,
                velocity,
                acceleration,
                cancel_event=cancel_event,
                on_step=on_step,
            )

    def move_relative(  # noqa: PLR0913
        self,
        dx_cm=0.0,
        dy_cm=0.0,
        dz_cm=0.0,
        *,
        duration=None,
        execute=False,
        options=None,
    ):
        """Base-frame XYZ offsets in centimeters, with fixed orientation."""
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            self._refresh()
            motion = self._motion_options(
                options, duration=duration, execute=execute
            )
            return self._move_cartesian(
                dx_cm, dy_cm, dz_cm, duration, execute=execute, options=motion
            )

    def move_to_pose(
        self, pose, *, duration=None, execute=False, options=None
    ):
        """Move in a straight Cartesian line with shortest quaternion SLERP."""
        if not isinstance(pose, CartesianPose):
            msg = 'Use CartesianPose with metres and an xyzw quaternion'
            raise TypeError(msg)
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            self._refresh()
            return self._move_cartesian(
                duration=duration,
                execute=execute,
                target_pose=pose,
                options=self._motion_options(
                    options, duration=duration, execute=execute
                ),
            )

    def _gripper_action(self, action, execute, cancel_event=None):
        with self._operation():
            self._require_connected()
            if self._gripper is None:
                msg = 'Gripper support is disabled in SawyerConfig'
                raise RuntimeError(msg)
            self._node.spin_until(time.monotonic() + 0.02)
            if cancel_event is None:
                return self._gripper.perform(action, execute=execute)
            return self._gripper.perform(
                action, execute=execute, cancel_event=cancel_event
            )

    def open_gripper(self, *, execute=False, cancel_event=None):
        return self._gripper_action('open', execute, cancel_event)

    def move_gripper(self, position, *, execute=False, cancel_event=None):
        """
        Set finger separation in metres, with acknowledgement and
        settling.
        """
        with self._operation():
            self._require_connected()
            if self._gripper is None:
                msg = 'Gripper support is disabled in SawyerConfig'
                raise RuntimeError(msg)
            self._node.spin_until(
                time.monotonic() + self.config.callback_timeout
            )
            return self._gripper.perform(
                'move',
                position=position,
                execute=execute,
                cancel_event=cancel_event,
            )

    def close_gripper(self, *, execute=False, cancel_event=None):
        return self._gripper_action('close', execute, cancel_event)

    def calibrate_gripper(self, *, execute=False, cancel_event=None):
        return self._gripper_action('calibrate', execute, cancel_event)

    def stop_gripper(self, *, execute=False):
        return self._gripper_action('stop', execute)

    def start_gripper(self, *, execute=False, cancel_event=None):
        """
        Explicitly resume the gripper's currently reported position
        target.
        """
        return self._gripper_action('start', execute, cancel_event)

    def reboot_gripper(self, *, execute=False, cancel_event=None):
        """Power-cycle an idle gripper; calibration must then be explicit."""
        return self._gripper_action('reboot', execute, cancel_event)

    def stop(self):
        """
        Request cancellation; an active arm loop performs its normal stop.
        """
        if self._node is not None:
            self._node.cancel_requested.set()

    def _move_cartesian(  # noqa: PLR0913
        self,
        dx_cm: float = 0.0,
        dy_cm: float = 0.0,
        dz_cm: float = 0.0,
        duration: float | None = None,
        *,
        execute: bool = False,
        options: CartesianMotionOptions | None = None,
        target_pose: CartesianPose | None = None,
    ) -> CartesianMoveResult:
        """
        Preview/execute a relative XYZ translation in the pose reference frame.

        Offsets are centimeters; duration is seconds or None for
        minimum feasible
        uniform time scaling. Orientation is held. Connect the driver first;
        this method services direct-transport callbacks internally. No
        execution occurs without
        execute=True. The returned result contains the reusable plan
        and timing.
        """
        options = options or CartesianMotionOptions()
        duration = options.time if duration is None else duration
        options = replace(options, time=duration, execute=execute)
        if options.frame.lstrip('/') != self._node.pose_feedback.frame.lstrip(
            '/'
        ):
            msg = (
                'Construct CartesianMotionNode with the same frame as '
                'options.frame'
            )
            raise ValueError(msg)
        self._node.wait_inputs(execute)
        tip, start_pose = self._node.wait_pose(options.tip)
        start_joints = tuple(self._node.joints)
        self._node.planning_health(start_joints, start_pose, tip)
        velocity = tuple(
            options.speed * value for value in self._node.limits.velocity
        )
        acceleration = tuple(
            min(value, options.max_accel)
            if options.max_accel is not None
            else value
            for value in self._node.limits.acceleration
        )
        kinematics = InteraKinematics(
            self._node,
            tip,
            start_pose.frame,
            options.ik_timeout,
            options.planning_timeout,
            health_check=lambda: self._node.planning_health(
                start_joints, start_pose, tip
            ),
        )
        try:
            kinematics.wait_ready()
            planner = CartesianPlanner(
                kinematics.inverse,
                kinematics.forward,
                self._node.limits.lower,
                self._node.limits.upper,
                sample_step=options.sample_step,
                path_tolerance=options.path_tolerance,
                orientation_tolerance=options.path_orientation_tolerance,
                max_joint_step=options.max_joint_step,
                max_samples=options.max_samples,
                joint_names=JOINT_NAMES,
                angular_step=options.angular_step,
            )
            target = (
                target_pose
                or CartesianLine(start_pose, (dx_cm, dy_cm, dz_cm)).end
            )
            plan = planner.plan_pose(
                start_pose,
                target,
                start_joints,
                velocity,
                acceleration,
                duration,
            )
        finally:
            kinematics.close()
        if not execute:
            return CartesianMoveResult(plan, None)
        self._node.wait_inputs(True)
        self._node.wait_pose(tip)
        self._node.check_ready()
        self._node.planning_health(start_joints, start_pose, tip)
        if plan.trajectory.duration == 0:
            return CartesianMoveResult(plan, None)
        controller = JointPIDController(
            7,
            options.kp,
            options.ki,
            options.kd,
            velocity,
            acceleration,
            position_lower=self._node.limits.lower,
            position_upper=self._node.limits.upper,
        )

        def monitor(elapsed, _joint_reference):
            fraction = (
                elapsed / plan.trajectory.duration
                if plan.trajectory.duration
                else 1.0
            )
            position, orientation = pose_error(
                self._node.pose_feedback.read(tip), plan.line.sample(fraction)
            )
            if (
                position > options.max_cartesian_error
                or orientation > options.max_orientation_error
            ):
                msg = (
                    f'Cartesian tracking error: {position:.6f} m / '
                    f'{orientation:.6f} rad exceeds configured bounds'
                )
                raise RuntimeError(msg)

        def at_goal():
            position, orientation = pose_error(
                self._node.pose_feedback.read(tip), plan.line.end
            )
            return (
                position < options.cartesian_tolerance
                and orientation < options.goal_orientation_tolerance
            )

        execution = self._node.track(
            plan.trajectory,
            controller,
            options,
            label='Cartesian target',
            on_step=monitor,
            at_goal=at_goal,
            announce=False,
        )
        return CartesianMoveResult(plan, execution)
