"""Portable pick/place recordings and validated plans; no ROS imports.

Plans store IK knots, not executable Python or pickled objects. Reconstructing a
plan recomputes polynomial extrema and timing before any execution is allowed.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import os
import tempfile

import numpy as np

if __package__:
    from .cartesian_path import CartesianLine, CartesianPose, pose_error
    from .joint_path import JointPath, TimedJointPath
    from .motion_utils import vector
else:
    from cartesian_path import CartesianLine, CartesianPose, pose_error
    from joint_path import JointPath, TimedJointPath
    from motion_utils import vector


JOINTS = tuple(f'right_j{i}' for i in range(7))
SCHEMA = 'sawyer.pick_place.v1'
LIFT_METERS = .20
RELEASE_DELAY = 2.0


@dataclass(frozen=True)
class Waypoint:
    name: str
    pose: CartesianPose
    joints: tuple

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name:
            raise ValueError('Waypoint needs a name')
        object.__setattr__(self, 'joints', vector(self.joints, 7, 'waypoint joints'))


@dataclass(frozen=True)
class Segment:
    name: str
    trajectory: TimedJointPath
    line: CartesianLine | None = None
    reference_poses: tuple = ()


def pose_data(pose):
    return dict(position=pose.position, orientation=pose.orientation, frame=pose.frame)


def read_pose(data):
    return CartesianPose(**data)


def atomic_json(path, data):
    """Keep the previous complete artifact if writing is interrupted."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, suffix='.tmp', delete=False) as stream:
            temporary = stream.name
            json.dump(data, stream, indent=2, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


def recording_data(waypoints, tip, settings, limits):
    return dict(schema=SCHEMA, status='recorded', created_utc=datetime.now(timezone.utc).isoformat(),
                joint_names=JOINTS, frame=waypoints[0].pose.frame, tip=tip,
                settings=settings, limits=limits, release_delay_s=RELEASE_DELAY, lift_m=LIFT_METERS,
                waypoints=[dict(name=w.name, pose=pose_data(w.pose), joints=w.joints) for w in waypoints])


def read_recording(data):
    if data['schema'] != SCHEMA or tuple(data['joint_names']) != JOINTS:
        raise ValueError('Unsupported recording schema or joint order')
    if not isinstance(data['tip'], str) or not data['tip']:
        raise ValueError('Recording has no tool identification')
    if data['settings']['tip'] != data['tip']:
        raise ValueError('Inconsistent recorded tool identification')
    points = tuple(Waypoint(w['name'], read_pose(w['pose']), w['joints']) for w in data['waypoints'])
    expected = ['pick'] + [f'middle_{i}' for i in range(1, len(points)-1)] + ['place']
    if len(points) < 2 or [p.name for p in points] != expected:
        raise ValueError('Recording needs pick, ordered middle points, and place')
    if any(p.pose.frame != data['frame'] for p in points) or data['frame'] != 'base':
        raise ValueError('Pick/place currently requires all poses in the base frame')
    if data['release_delay_s'] != RELEASE_DELAY or data['lift_m'] != LIFT_METERS:
        raise ValueError('Unexpected release delay or lift distance')
    return points


def round_corners(*_args, **_kwargs):
    """TODO: blend corners with bounded deviation and retime the entire path.

    Future implementation must add blend-radius/tolerance controls, check IK/FK
    and collisions again, and preserve pick/place stops. Until then, exact
    straight segments use zero velocity and acceleration at every waypoint.
    """
    raise NotImplementedError('Corner rounding is reserved for a future implementation')


def segment_from_plan(name, plan):
    return Segment(name, plan.trajectory, plan.line)


def validate_segment(segment, planner):
    """Recheck loaded polynomial extrema, sampled FK/collision and Cartesian error."""
    trajectory = segment.trajectory
    if trajectory.path.size != 7:
        raise ValueError('Expected seven joints in saved trajectory')
    planner.checked_joints(trajectory.min_position, 'Path minimum')
    planner.checked_joints(trajectory.max_position, 'Path maximum')
    fractions = trajectory.path.validation_fractions()
    poses = planner.forward([trajectory.path.sample(t).position for t in fractions])
    if len(poses) != len(fractions):
        raise ValueError('Wrong FK response size')
    if any(pose.frame != 'base' for pose in poses):
        raise ValueError('FK returned an unexpected reference frame')
    if segment.line is not None:
        errors = [pose_error(pose, segment.line.sample(t)) for t, pose in zip(fractions, poses)]
        if (max(e[0] for e in errors) > planner.path_tolerance or
                max(e[1] for e in errors) > planner.orientation_tolerance):
            raise ValueError(f'{segment.name}: saved path no longer meets FK tolerances')
    return tuple((t, pose) for t, pose in zip(fractions, poses))


def plan_joint_segment(name, start, end, velocity, acceleration):
    """Construct a rest-to-rest joint-space quintic, e.g. the neutral return."""
    fractions = tuple(float(t) for t in np.linspace(0, 1, 21))
    positions = [tuple(a + (b-a)*(10*t**3 - 15*t**4 + 6*t**5)
                       for a, b in zip(start, end)) for t in fractions]
    return Segment(name, TimedJointPath(JointPath(fractions, positions), velocity, acceleration))


def replan_from_pick(segments, planner, pick_joints):
    """Keep saved Cartesian geometry, connect it to the approached pick branch.

    A redundant arm can reach the recorded pose with different joint angles.
    Retarget the entire chain, including lift and neutral, instead of jumping
    to recorded joints at pick. Preserve each segment's capped limits and never
    shorten its saved duration. This returns a run-local plan; no file is changed.
    """
    joints = planner.checked_joints(pick_joints, 'Approached pick joints')
    result = []
    for saved in segments:
        old = saved.trajectory
        try:
            if saved.line is not None:
                plan = planner.plan_pose(saved.line.start, saved.line.end, joints,
                                         old.max_velocity, old.max_acceleration)
                segment = segment_from_plan(saved.name, plan)
            else:
                segment = plan_joint_segment(saved.name, joints,
                    planner.checked_joints(old.end, 'Neutral pose'), old.max_velocity, old.max_acceleration)
            segment.trajectory.duration = max(old.duration, segment.trajectory.minimum_duration)
            references = validate_segment(segment, planner)
        except ValueError as exc:
            raise ValueError(f'{saved.name}: cannot connect saved path to approached pick posture: {exc}') from exc
        result.append(Segment(segment.name, segment.trajectory, segment.line, references))
        joints = segment.trajectory.end
    return tuple(result)


def plan_task(points, planner, velocity, acceleration, neutral):
    """Plan offline from the recorded pick branch, chaining each IK endpoint."""
    joints, start = points[0].joints, points[0].pose
    segments = []
    for point in points[1:]:
        plan = planner.plan_pose(start, point.pose, joints, velocity, acceleration)
        segments.append(segment_from_plan('to_' + point.name, plan))
        joints, start = plan.trajectory.end, plan.line.end
    lift_pose = CartesianPose((start.position[0], start.position[1], start.position[2]+LIFT_METERS),
                              start.orientation, start.frame)
    lift = planner.plan_pose(start, lift_pose, joints, velocity, acceleration)
    segments.append(segment_from_plan('lift', lift))
    neutral = planner.checked_joints(neutral, 'Neutral pose')
    segments.append(plan_joint_segment('neutral', lift.trajectory.end, neutral, velocity, acceleration))
    return tuple(Segment(s.name, s.trajectory, s.line, validate_segment(s, planner)) for s in segments)


def segment_data(segment):
    trajectory = segment.trajectory
    path = trajectory.path
    return dict(name=segment.name, fractions=path.fractions,
                positions=[path.sample(t).position for t in path.fractions],
                velocity_limits=trajectory.max_velocity, acceleration_limits=trajectory.max_acceleration,
                duration_s=trajectory.duration,
                line=None if segment.line is None else dict(start=pose_data(segment.line.start),
                                                            end=pose_data(segment.line.end)),
                reference_poses=[dict(fraction=t, pose=pose_data(p)) for t, p in segment.reference_poses])


def read_segment(data):
    path = JointPath(data['fractions'], data['positions'])
    duration = float(data['duration_s'])
    if not math.isfinite(duration) or duration < 0:
        raise ValueError('Invalid saved duration')
    trajectory = TimedJointPath(path, data['velocity_limits'], data['acceleration_limits'])
    # Decimal round trips/reconstruction can change extrema by machine precision.
    if duration + 1e-8*max(1, duration) < trajectory.minimum_duration:
        raise ValueError('Saved duration violates joint limits')
    trajectory.duration = max(duration, trajectory.minimum_duration)
    line = data['line']
    line = None if line is None else CartesianLine.between(read_pose(line['start']), read_pose(line['end']))
    poses = tuple((float(p['fraction']), read_pose(p['pose'])) for p in data['reference_poses'])
    return Segment(data['name'], trajectory, line, poses)


def read_task(path):
    with open(path) as stream:
        data = json.load(stream)
    points = read_recording(data)
    if data['status'] != 'planned':
        raise ValueError('Recording has no complete plan; finish planning before replay')
    segments = tuple(read_segment(s) for s in data['segments'])
    expected = ['to_' + p.name for p in points[1:]] + ['lift', 'neutral']
    if [s.name for s in segments] != expected:
        raise ValueError('Incomplete or out-of-order saved plan')
    previous_joints, previous_pose = points[0].joints, points[0].pose
    for segment, point in zip(segments[:-2], points[1:]):
        if segment.line is None:
            raise ValueError('Transfer segment must have a Cartesian reference')
        for actual, expected_pose in ((segment.line.start, previous_pose), (segment.line.end, point.pose)):
            if any(error > 1e-7 for error in pose_error(actual, expected_pose)):
                raise ValueError('Saved path does not match recorded waypoints')
        previous_pose = segment.line.end
    lift, neutral = segments[-2:]
    target = CartesianPose((previous_pose.position[0], previous_pose.position[1],
                            previous_pose.position[2]+LIFT_METERS), previous_pose.orientation, 'base')
    if lift.line is None or neutral.line is not None:
        raise ValueError('Invalid lift/neutral reference')
    if any(e > 1e-7 for e in pose_error(lift.line.start, previous_pose)+pose_error(lift.line.end, target)):
        raise ValueError('Saved lift is not 20 cm along base +Z with fixed orientation')
    for segment in segments:
        if max(abs(a-b) for a, b in zip(previous_joints, segment.trajectory.start)) > 1e-8:
            raise ValueError('Saved joint paths are discontinuous')
        previous_joints = segment.trajectory.end
    return data, points, segments


def retime_segment(segment, velocity, acceleration):
    """Replay may reduce recorded limits; never silently increase recorded speed."""
    old = segment.trajectory
    velocity = tuple(min(a, b) for a, b in zip(velocity, old.max_velocity))
    acceleration = tuple(min(a, b) for a, b in zip(acceleration, old.max_acceleration))
    trajectory = TimedJointPath(old.path, velocity, acceleration)
    trajectory.duration = max(old.duration, trajectory.minimum_duration)
    return Segment(segment.name, trajectory, segment.line, segment.reference_poses)


class SegmentSequence:
    """One velocity-control stream with measured, rest-to-rest waypoint gates.

    Samples hold the active endpoint until advance() is called by the feedback
    monitor. This preserves exact taught lines without mode switches, resets or
    fixed settling delays at intermediate points. Rounding corners remains a
    separate future feature.
    """
    def __init__(self, segments, settle_time, timeout=None):
        if (not math.isfinite(settle_time) or settle_time <= 0 or
                (timeout is not None and (not math.isfinite(timeout) or timeout <= 0))):
            raise ValueError('Sequence settling allowance and timeout must be positive and finite')
        segments = tuple(segments)
        if not segments:
            raise ValueError('A sequence needs at least one segment')
        self.max_velocity = tuple(min(s.trajectory.max_velocity[i] for s in segments) for i in range(7))
        self.max_acceleration = tuple(min(s.trajectory.max_acceleration[i] for s in segments) for i in range(7))
        self.segments = tuple(retime_segment(s, self.max_velocity, self.max_acceleration) for s in segments)
        for left, right in zip(self.segments, self.segments[1:]):
            if max(abs(a-b) for a,b in zip(left.trajectory.end, right.trajectory.start)) > 1e-8:
                raise ValueError('Cannot stream discontinuous joint segments')
        self.start, self.end = self.segments[0].trajectory.start, self.segments[-1].trajectory.end
        self.min_position = tuple(min(s.trajectory.min_position[i] for s in self.segments) for i in range(7))
        self.max_position = tuple(max(s.trajectory.max_position[i] for s in self.segments) for i in range(7))
        self.duration = sum(s.trajectory.duration for s in self.segments)
        self.budgets = tuple(timeout if timeout is not None else s.trajectory.duration+settle_time
                             for s in self.segments)
        if any(b < s.trajectory.duration for b,s in zip(self.budgets, self.segments)):
            raise ValueError('--timeout must be at least every segment reference duration')
        self.timeout = sum(self.budgets)
        self.index, self.segment_started = 0, 0.
        self.completed = []

    @property
    def active(self):
        return self.segments[self.index]

    @property
    def is_last(self):
        return self.index == len(self.segments)-1

    def segment_elapsed(self, elapsed):
        return max(0., elapsed-self.segment_started)

    def sample(self, elapsed):
        return self.active.trajectory.sample(self.segment_elapsed(elapsed))

    def check_timeout(self, elapsed):
        if self.segment_elapsed(elapsed) > self.budgets[self.index]:
            raise TimeoutError(f'{self.active.name}: waypoint arrival/settling timeout')

    def advance(self, elapsed):
        if self.is_last or self.segment_elapsed(elapsed)+1e-10 < self.active.trajectory.duration:
            raise ValueError('Cannot advance before the active segment ends')
        self.completed.append((self.active.name, self.segment_elapsed(elapsed)))
        self.index += 1
        self.segment_started = elapsed
