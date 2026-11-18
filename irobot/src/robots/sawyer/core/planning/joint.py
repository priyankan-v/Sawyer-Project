"""Joint interpolation and trajectories."""

from __future__ import annotations

import math
from bisect import bisect_right

from ...config import JOINT_NAMES, vector
from ..types import TrajectorySample

POLYNOMIAL_ROOT_IMAG_TOLERANCE = 1e-8
MIN_PATH_KNOTS = 3
PATH_ARRAY_DIMENSIONS = 2


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
