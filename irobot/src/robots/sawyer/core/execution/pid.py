"""Joint velocity PID with command limits and anti-windup."""

from __future__ import annotations

import math

from ...config import vector


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
