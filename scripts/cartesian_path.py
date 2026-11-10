"""Straight Cartesian references with quintic timing, independent of ROS."""

from dataclasses import dataclass
import math

if __package__:
    from .motion_utils import vector
    from .quintic_trajectory import QuinticTrajectory
else:
    from motion_utils import vector
    from quintic_trajectory import QuinticTrajectory


def parse_offsets(tokens):
    """Parse x:+5 y:-3 z:+10 into an XYZ tuple in centimeters."""
    result = [0.0, 0.0, 0.0]
    seen = set()
    for token in tokens:
        try:
            axis, value = token.split(':')
            axis, value = axis.lower(), float(value)
        except (ValueError, AttributeError):
            raise ValueError(f'Invalid offset {token!r}; use x:+5, y:-3, or z:+10 (centimeters)') from None
        if axis not in ('x', 'y', 'z') or axis in seen or not math.isfinite(value):
            raise ValueError(f'Offset axes must be unique x/y/z with finite centimeter values: {token!r}')
        seen.add(axis)
        result[('x', 'y', 'z').index(axis)] = value
    if not seen:
        raise ValueError('At least one Cartesian offset is required')
    return tuple(result)


@dataclass(frozen=True)
class CartesianPose:
    position: tuple
    orientation: tuple  # Quaternion x, y, z, w.
    frame: str = 'base'

    def __post_init__(self):
        object.__setattr__(self, 'position', vector(self.position, 3, 'Cartesian position'))
        quaternion = vector(self.orientation, 4, 'orientation quaternion')
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not math.isfinite(norm) or norm < 1e-12 or abs(norm - 1) > .05:
            raise ValueError('Orientation must be a valid approximately unit quaternion')
        object.__setattr__(self, 'orientation', tuple(value / norm for value in quaternion))
        frame = self.frame.lstrip('/')
        if not frame:
            raise ValueError('A Cartesian reference frame is required')
        object.__setattr__(self, 'frame', frame)


def pose_error(actual, desired):
    if actual.frame != desired.frame:
        raise ValueError(f'Pose frame mismatch: {actual.frame} != {desired.frame}')
    dot = abs(sum(a * b for a, b in zip(actual.orientation, desired.orientation)))
    return math.dist(actual.position, desired.position), 2 * math.acos(min(1.0, dot))


class CartesianLine:
    """Straight-line reference with shortest SLERP and quintic time scaling.

    ``sample(u)`` uses normalized time u=t/T. Velocity/acceleration of XYZ can be
    obtained by differentiating the shared scalar ``progress`` trajectory.
    """

    def __init__(self, start, offset_cm, end_orientation=None):
        self.start = start
        self.offset_cm = vector(offset_cm, 3, 'offset_cm')
        self.delta = tuple(value / 100 for value in self.offset_cm)
        self.end = CartesianPose(tuple(a + d for a, d in zip(start.position, self.delta)),
                                 start.orientation if end_orientation is None else end_orientation, start.frame)
        self.length = math.sqrt(sum(value * value for value in self.delta))
        self.angle = pose_error(self.start, self.end)[1]
        self.progress = QuinticTrajectory([0], [1], 1.875, duration=1.0)

    @classmethod
    def between(cls, start, end):
        if start.frame != end.frame:
            raise ValueError('Cartesian pose frame mismatch')
        return cls(start, tuple(100 * (b-a) for a, b in zip(start.position, end.position)), end.orientation)

    def sample(self, normalized_time):
        progress = self.progress.sample(normalized_time).position[0]
        return CartesianPose(tuple(a + d * progress for a, d in zip(self.start.position, self.delta)),
                             slerp(self.start.orientation, self.end.orientation, progress), self.start.frame)


def slerp(start, end, fraction):
    """Shortest rotation between unit quaternions, including antipodal inputs."""
    dot = sum(a*b for a, b in zip(start, end))
    if dot < 0:
        end, dot = tuple(-value for value in end), -dot
    dot = min(1.0, dot)
    if dot > .9995:
        values = tuple(a + fraction*(b-a) for a, b in zip(start, end))
    else:
        angle = math.acos(dot)
        left, right = math.sin((1-fraction)*angle), math.sin(fraction*angle)
        values = tuple((left*a + right*b)/math.sin(angle) for a, b in zip(start, end))
    norm = math.sqrt(sum(value*value for value in values))
    return tuple(value/norm for value in values)
