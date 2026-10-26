"""Synchronized rest-to-rest quintic joint trajectories; no ROS dependencies."""

from dataclasses import dataclass
import math

if __package__:
    from .motion_utils import vector
else:
    from motion_utils import vector


@dataclass(frozen=True)
class TrajectorySample:
    position: tuple
    velocity: tuple
    acceleration: tuple
    jerk: tuple


class QuinticTrajectory:
    """q(t) = q0 + (q1-q0)*(10*u^3 - 15*u^4 + 6*u^5), u=t/T.

    All joints share T and have zero endpoint velocity and acceleration. Units
    are radians and seconds. Limits accept scalars or per-joint vectors.
    Minimum T for this polynomial family is max(15*abs(delta)/(8*vmax)),
    optionally also max(sqrt((10/sqrt(3))*abs(delta)/amax)). This is not a
    globally time-optimal trajectory among arbitrary motion profiles.
    Too-short explicit durations are rejected; stationary auto-duration is zero.
    """

    def __init__(self, start, end, max_velocity, duration=None, max_acceleration=None):
        self.start = tuple(float(value) for value in start)
        if not self.start:
            raise ValueError("At least one joint is required")
        size = len(self.start)
        self.start = vector(self.start, size, "start")
        self.end = vector(end, size, "end")
        self.delta = vector([b - a for a, b in zip(self.start, self.end)], size, "displacement")
        self.max_velocity = vector(max_velocity, size, "max_velocity", positive=True)
        self.max_acceleration = (None if max_acceleration is None else
                                 vector(max_acceleration, size, "max_acceleration", positive=True))
        self.velocity_duration = max(1.875 * abs(d) / v
                                     for d, v in zip(self.delta, self.max_velocity))
        self.acceleration_duration = (0.0 if self.max_acceleration is None else max(
            math.sqrt((10 / math.sqrt(3)) * abs(d) / a)
            for d, a in zip(self.delta, self.max_acceleration)
        ))
        self.minimum_duration = max(self.velocity_duration, self.acceleration_duration)
        if not math.isfinite(self.minimum_duration):
            raise ValueError("Joint displacement/limits produce a non-finite duration")
        if duration is None:
            duration = self.minimum_duration
        else:
            duration = float(duration)
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError("Requested duration must be positive and finite")
            if duration < self.minimum_duration:
                raise ValueError(f"Requested duration {duration:.6f} s is too short; "
                                 f"minimum feasible quintic duration is {self.minimum_duration:.6f} s")
        self.duration = duration

    def sample(self, elapsed):
        elapsed = float(elapsed)
        if not math.isfinite(elapsed):
            raise ValueError("Sample time must be finite")
        zero = (0.0,) * len(self.start)
        if self.duration == 0 or elapsed >= self.duration:
            return TrajectorySample(self.end, zero, zero, zero)
        if elapsed <= 0:
            return TrajectorySample(self.start, zero, zero, zero)
        u = elapsed / self.duration
        s = u**3 * (10 - 15 * u + 6 * u**2)
        ds = 30 * u**2 * (1 - u)**2 / self.duration
        dds = 60 * u * (1 - u) * (1 - 2 * u) / self.duration**2
        ddds = 60 * (1 - 6 * u + 6 * u**2) / self.duration**3
        return TrajectorySample(
            tuple(a + d * s for a, d in zip(self.start, self.delta)),
            tuple(d * ds for d in self.delta),
            tuple(d * dds for d in self.delta),
            tuple(d * ddds for d in self.delta),
        )
