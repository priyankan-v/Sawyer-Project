"""Shared geometry, limits, trajectory samples and motion results."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from ..config import (
    JOINT_NAMES,
    QUATERNION_MIN_NORM,
    QUATERNION_NORM_TOLERANCE,
    vector,
)

__all__ = [
    'CartesianMoveResult',
    'CartesianPlan',
    'CartesianPose',
    'JointBounds',
    'JointMoveResult',
    'MotionResult',
    'TrajectorySample',
    'ordered_values',
    'pose_error',
    'vector',
]

if TYPE_CHECKING:
    from .planning.cartesian import CartesianLine
    from .planning.timing import TimedJointPath


@dataclass(frozen=True)
class CartesianPose:
    position: tuple
    orientation: tuple  # Quaternion x, y, z, w.
    frame: str = 'base'

    def __post_init__(self):
        object.__setattr__(
            self, 'position', vector(self.position, 3, 'Cartesian position')
        )
        quaternion = vector(self.orientation, 4, 'orientation quaternion')
        norm = math.sqrt(sum(value * value for value in quaternion))
        if (
            not math.isfinite(norm)
            or norm < QUATERNION_MIN_NORM
            or abs(norm - 1) > QUATERNION_NORM_TOLERANCE
        ):
            msg = 'Orientation must be a valid approximately unit quaternion'
            raise ValueError(msg)
        object.__setattr__(
            self, 'orientation', tuple(value / norm for value in quaternion)
        )
        frame = self.frame.lstrip('/')
        if not frame:
            msg = 'A Cartesian reference frame is required'
            raise ValueError(msg)
        object.__setattr__(self, 'frame', frame)


def pose_error(actual, desired):
    if actual.frame != desired.frame:
        msg = f'Pose frame mismatch: {actual.frame} != {desired.frame}'
        raise ValueError(msg)
    dot = abs(
        sum(
            a * b
            for a, b in zip(
                actual.orientation, desired.orientation, strict=False
            )
        )
    )
    return math.dist(actual.position, desired.position), 2 * math.acos(
        min(1.0, dot)
    )


@dataclass(frozen=True)
class MotionResult:
    reference_duration: float
    elapsed_duration: float
    final_joints: tuple


def ordered_values(names, values, label):
    if len(names) != len(values) or len(set(names)) != len(names):
        msg = f'Malformed {label} names/values'
        raise ValueError(msg)
    values = dict(zip(names, values, strict=False))
    if not all(name in values for name in JOINT_NAMES):
        msg = f'{label} is missing right arm joints'
        raise ValueError(msg)
    return vector([values[name] for name in JOINT_NAMES], 7, label)


@dataclass(frozen=True)
class JointBounds:
    lower: tuple
    upper: tuple
    velocity: tuple
    acceleration: tuple

    @classmethod
    def from_message(cls, message):
        bounds = cls(
            *(
                ordered_values(
                    message.joint_names, getattr(message, field), field
                )
                for field in (
                    'position_lower',
                    'position_upper',
                    'velocity',
                    'accel',
                )
            )
        )
        if any(
            a >= b for a, b in zip(bounds.lower, bounds.upper, strict=False)
        ):
            msg = 'Invalid joint position bounds'
            raise ValueError(msg)
        if any(value <= 0 for value in bounds.velocity + bounds.acceleration):
            msg = 'Joint velocity/acceleration limits must be positive'
            raise ValueError(msg)
        return bounds

    def check_position(self, positions, label, tolerance=0.0):
        positions = vector(positions, 7, label)
        for name, position, lower, upper in zip(
            JOINT_NAMES, positions, self.lower, self.upper, strict=False
        ):
            if not lower - tolerance <= position <= upper + tolerance:
                msg = (
                    f'{label}: {name}={position:.6f} outside [{lower}, '
                    f'{upper}]'
                )
                raise ValueError(msg)


@dataclass(frozen=True)
class TrajectorySample:
    position: tuple
    velocity: tuple
    acceleration: tuple
    jerk: tuple


@dataclass(frozen=True)
class CartesianPlan:
    line: CartesianLine
    trajectory: TimedJointPath
    samples: int
    max_position_error: float
    max_orientation_error: float


@dataclass(frozen=True)
class CartesianMoveResult:
    plan: CartesianPlan
    execution: MotionResult | None


@dataclass(frozen=True)
class JointMoveResult:
    plan: object
    execution: object | None
