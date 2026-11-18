"""Velocity and acceleration based trajectory timing."""

from __future__ import annotations

import math

from ...config import vector
from ..types import TrajectorySample


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
