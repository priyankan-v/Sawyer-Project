"""C2 joint interpolation and derivative-based timing for sampled IK paths.

All extrema are computed on the actual piecewise polynomial reference, including
between knots. No ROS dependency; NumPy is used for small polynomial root solves.
"""

from bisect import bisect_right
import math

import numpy as np
from numpy.polynomial import polynomial as polynomial

if __package__:
    from .motion_utils import vector
    from .quintic_trajectory import TrajectorySample
else:
    from motion_utils import vector
    from quintic_trajectory import TrajectorySample


def extrema(coefficients):
    """Polynomial extrema over [0,1], including both endpoints."""
    derivative = polynomial.polyder(coefficients)
    roots = polynomial.polyroots(derivative) if np.any(derivative) else []
    candidates = [0.0, 1.0] + [float(root.real) for root in roots
                              if abs(root.imag) < 1e-8 and 0 < root.real < 1]
    values = [float(polynomial.polyval(point, coefficients)) for point in candidates]
    return min(values), max(values)


class JointPath:
    """Piecewise quintic Hermite q(u) through IK samples, u in [0,1].

    Interior derivatives use local polynomial fits of degree up to five. Endpoint velocity
    and acceleration are explicitly zero. Position, first derivative and second
    derivative are continuous across knots. This joint interpolation must be
    checked against the desired Cartesian reference using FK before execution.
    """

    def __init__(self, fractions, positions):
        self.fractions = tuple(float(value) for value in fractions)
        values = np.asarray(positions, dtype=float)
        if (len(self.fractions) < 3 or values.ndim != 2 or values.shape[0] != len(self.fractions)
                or values.shape[1] == 0 or not np.all(np.isfinite(values))
                or not all(math.isfinite(t) for t in self.fractions)
                or self.fractions[0] != 0 or self.fractions[-1] != 1
                or any(a >= b for a, b in zip(self.fractions, self.fractions[1:]))):
            raise ValueError('JointPath needs >=3 finite, increasing knots from 0 to 1 and matching joint vectors')
        self.size = values.shape[1]
        self.start, self.end = tuple(map(float, values[0])), tuple(map(float, values[-1]))
        first, second = np.zeros_like(values), np.zeros_like(values)
        window = min(7, len(self.fractions))
        times = np.asarray(self.fractions)
        for i, center in enumerate(times):
            left = max(0, min(i - window // 2, len(times) - window))
            indices = slice(left, left + window)
            scale = max(abs(times[indices] - center))
            # Center/scale the stencil to avoid ill-conditioning near the ends.
            fitted = polynomial.polyfit((times[indices] - center) / scale,
                                        values[indices], min(5, window - 1))
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
        for i, (left, right) in enumerate(zip(self.fractions, self.fractions[1:])):
            h = right - left
            c0, c1, c2 = values[i], first[i] * h, .5 * second[i] * h**2
            a = values[i + 1] - c0 - c1 - c2
            b = first[i + 1] * h - c1 - 2 * c2
            c = second[i + 1] * h**2 - 2 * c2
            coefficients = np.stack((c0, c1, c2, 10*a - 4*b + .5*c,
                                     -15*a + 7*b - c, 6*a - 3*b + .5*c), axis=1)
            self.segments.append(coefficients)
            for joint, coeff in enumerate(coefficients):
                low, high = extrema(coeff)
                self.min_position[joint] = min(self.min_position[joint], low)
                self.max_position[joint] = max(self.max_position[joint], high)
                for derivative, peaks in ((1, self.peak_velocity), (2, self.peak_acceleration)):
                    low, high = extrema(polynomial.polyder(coeff, derivative) / h**derivative)
                    peaks[joint] = max(peaks[joint], abs(low), abs(high))

    def sample(self, fraction):
        if not math.isfinite(fraction):
            raise ValueError('Normalized time must be finite')
        zero = (0.0,) * self.size
        if fraction <= 0:
            return TrajectorySample(self.start, zero, zero, zero)
        if fraction >= 1:
            return TrajectorySample(self.end, zero, zero, zero)
        index = min(bisect_right(self.fractions, fraction) - 1, len(self.segments) - 1)
        h = self.fractions[index + 1] - self.fractions[index]
        x = (fraction - self.fractions[index]) / h
        fields = []
        for derivative in range(4):
            fields.append(tuple(float(polynomial.polyval(x, polynomial.polyder(coeff, derivative))) / h**derivative
                                for coeff in self.segments[index]))
        return TrajectorySample(*fields)

    def validation_fractions(self):
        return tuple(sorted(set(self.fractions) | {
            left + (right-left)*fraction
            for left, right in zip(self.fractions, self.fractions[1:]) for fraction in (.25, .5, .75)
        }))


class TimedJointPath:
    """Shortest uniform time scale for the constructed polynomial joint path.

    T >= peak|dq/du|/vmax and T >= sqrt(peak|d2q/du2|/amax). It is not a
    globally time-optimal Cartesian planner. Bounds are for reference commands,
    not guaranteed physical tracking under PID corrections/network latency.
    """

    def __init__(self, path, max_velocity, max_acceleration, duration=None):
        self.path = path
        self.start, self.end = path.start, path.end
        self.min_position = tuple(map(float, path.min_position))
        self.max_position = tuple(map(float, path.max_position))
        self.max_velocity = vector(max_velocity, path.size, 'max_velocity', positive=True)
        self.max_acceleration = vector(max_acceleration, path.size, 'max_acceleration', positive=True)
        self.velocity_duration = max(float(v) / limit for v, limit in zip(path.peak_velocity, self.max_velocity))
        self.acceleration_duration = math.sqrt(max(float(a) / limit
                                                  for a, limit in zip(path.peak_acceleration, self.max_acceleration)))
        self.minimum_duration = max(self.velocity_duration, self.acceleration_duration)
        if not math.isfinite(self.minimum_duration):
            raise ValueError('Non-finite joint path duration')
        if duration is not None:
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError('Requested duration must be positive and finite')
            if duration < self.minimum_duration:
                raise ValueError(f'Requested duration {duration:.6f} s is too short; minimum feasible '
                                 f'duration for the validated IK reference is {self.minimum_duration:.6f} s')
        self.duration = self.minimum_duration if duration is None else duration

    def sample(self, elapsed):
        if not math.isfinite(elapsed):
            raise ValueError('Sample time must be finite')
        if self.duration == 0:
            zero = (0.0,) * len(self.end)
            return TrajectorySample(self.end, zero, zero, zero)
        reference = self.path.sample(elapsed / self.duration)
        return TrajectorySample(reference.position,
                                tuple(v / self.duration for v in reference.velocity),
                                tuple(a / self.duration**2 for a in reference.acceleration),
                                tuple(j / self.duration**3 for j in reference.jerk))
