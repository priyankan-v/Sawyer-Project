"""Native Intera trajectory and interaction command construction."""

from __future__ import annotations

import math
from dataclasses import asdict

from ...config import JOINT_NAMES, InteractionLimits, positive, vector
from ..communication import wire
from ..types import CartesianPose, pose_error

CARTESIAN_AXES = 6


def check_finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        msg = 'Native trajectory values must be finite'
        raise ValueError(msg)
    if isinstance(value, dict):
        for item in value.values():
            check_finite(item)
    elif isinstance(value, tuple | list):
        for item in value:
            check_finite(item)


def validate_tracking(tracking):
    if tracking.goal_joint_tolerance:
        vector(
            tracking.goal_joint_tolerance, 7, 'Goal tolerances', positive=True
        )
    for name, active in (
        ('min_time_rate', tracking.use_min_time_rate),
        ('max_time_rate', tracking.use_max_time_rate),
        ('settling_time_at_goal', tracking.use_settling_time_at_goal),
    ):
        if active:
            positive(name, getattr(tracking, name))
    if (
        tracking.use_min_time_rate
        and tracking.use_max_time_rate
        and tracking.min_time_rate > tracking.max_time_rate
    ):
        msg = 'Minimum native tracking rate exceeds maximum'
        raise ValueError(msg)


def validate_waypoint_options(options, limits):
    positive('waypoint speed', options.max_joint_speed_ratio)
    if options.max_joint_speed_ratio > 1:
        msg = 'Waypoint speed ratio exceeds one'
        raise ValueError(msg)
    if options.joint_tolerances:
        vector(options.joint_tolerances, 7, 'Joint tolerances', positive=True)
    if options.max_joint_accel:
        acceleration = vector(
            options.max_joint_accel, 7, 'Joint acceleration', positive=True
        )
        if any(
            a > b
            for a, b in zip(acceleration, limits.acceleration, strict=True)
        ):
            msg = 'Native acceleration exceeds published limits'
            raise ValueError(msg)
    if options.corner_distance < 0:
        msg = 'Corner distance must be nonnegative'
        raise ValueError(msg)


def validate_cartesian_waypoint(  # noqa: PLR0913
    waypoint,
    *,
    initial,
    frame,
    current_pose,
    position_tolerance,
    orientation_tolerance,
):
    _, current = current_pose(waypoint.active_endpoint)
    pose = waypoint.pose
    target = CartesianPose(
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ),
        pose.header.frame_id or frame,
    )
    if target.frame != frame:
        msg = 'Native Cartesian poses must use the configured base frame'
        raise ValueError(msg)
    for name in (
        'max_linear_speed',
        'max_linear_accel',
        'max_rotational_speed',
        'max_rotational_accel',
    ):
        positive(name, getattr(waypoint.options, name))
    if initial:
        position, rotation = pose_error(current, target)
        if position > position_tolerance or rotation > orientation_tolerance:
            msg = 'First native pose differs from measured start'
            raise ValueError(msg)


def validate_trajectory(  # noqa: PLR0913
    trajectory,
    *,
    limits,
    joints,
    frame,
    current_pose,
    position_tolerance,
    orientation_tolerance,
    joint_tolerance,
):
    """Check supported joint/Cartesian native paths before any action goal."""
    if not isinstance(trajectory, wire.Trajectory):
        msg = 'Use the typed Intera Trajectory'
        raise TypeError(msg)
    check_finite(asdict(trajectory))
    if (
        tuple(trajectory.joint_names) != JOINT_NAMES
        or not trajectory.waypoints
    ):
        msg = 'Native paths need canonical joint order and nonempty waypoints'
        raise ValueError(msg)
    options = trajectory.trajectory_options
    if options.interpolation_type not in (
        wire.TrajectoryOptions.JOINT,
        wire.TrajectoryOptions.CARTESIAN,
    ):
        msg = 'Choose JOINT or CARTESIAN native interpolation'
        raise ValueError(msg)
    if options.interaction_control:
        validate_command(options.interaction_params, InteractionLimits())
    if options.path_interpolation_step < 0:
        msg = 'Path interpolation step must be nonnegative'
        raise ValueError(msg)
    validate_tracking(options.tracking_options)
    for index, waypoint in enumerate(trajectory.waypoints):
        validate_waypoint_options(waypoint.options, limits)
        if waypoint.joint_positions:
            positions = vector(waypoint.joint_positions, 7, 'Native joints')
            limits.check_position(positions, 'Native waypoint')
        if options.interpolation_type == wire.TrajectoryOptions.JOINT:
            positions = vector(waypoint.joint_positions, 7, 'Native joints')
            if (
                index == 0
                and max(
                    abs(a - b) for a, b in zip(positions, joints, strict=True)
                )
                > joint_tolerance
            ):
                msg = 'First native waypoint differs from measured start'
                raise ValueError(msg)
        else:
            validate_cartesian_waypoint(
                waypoint,
                initial=index == 0,
                frame=frame,
                current_pose=current_pose,
                position_tolerance=position_tolerance,
                orientation_tolerance=orientation_tolerance,
            )
    wire.serialize(trajectory)


def joint_trajectory(  # noqa: PLR0913
    positions,
    *,
    start,
    limits,
    tip,
    speed=0.05,
    max_acceleration=0.1,
):
    """Build a joint path including the measured start, in SDK joint order."""
    positive('speed', speed)
    positive('max_acceleration', max_acceleration)
    if speed > 1 or not positions:
        msg = 'Use nonempty waypoints and a speed ratio at most one'
        raise ValueError(msg)
    trajectory = wire.Trajectory(joint_names=list(JOINT_NAMES))
    trajectory.trajectory_options.interpolation_type = (
        wire.TrajectoryOptions.JOINT
    )
    trajectory.trajectory_options.tracking_options.goal_joint_tolerance = [
        0.01
    ] * 7
    for values in (start, *positions):
        target = vector(values, 7, 'Native waypoint')
        limits.check_position(target, 'Native waypoint')
        options = wire.WaypointOptions(
            max_joint_speed_ratio=speed,
            max_joint_accel=[
                min(max_acceleration, v) for v in limits.acceleration
            ],
            joint_tolerances=[0.05] * 7,
        )
        trajectory.waypoints.append(
            wire.Waypoint(
                joint_positions=list(target),
                options=options,
                active_endpoint=tip or 'right_hand',
            )
        )
    return trajectory


def interaction_command(*, endpoint='right_hand'):
    """SDK-compatible maximum-impedance, zero-force starting configuration."""
    message = wire.InteractionControlCommand(
        interaction_control_active=True,
        k_impedance=[1300.0, 1300.0, 1300.0, 30.0, 30.0, 30.0],
        max_impedance=[True] * 6,
        k_nullspace=[5.0, 10.0, 5.0, 10.0, 5.0, 10.0, 5.0],
        force_command=[0.0] * 6,
        endpoint_name=endpoint,
        interaction_control_mode=[
            wire.InteractionControlCommand.IMPEDANCE_MODE
        ]
        * 6,
    )
    message.header.frame_id = 'base'
    message.interaction_frame.orientation.w = 1.0
    return message


def validate_command(message, limits):
    if not isinstance(message, wire.InteractionControlCommand):
        msg = 'Use InteractionControlCommand'
        raise TypeError(msg)
    for name, bound in (
        ('k_impedance', limits.stiffness),
        ('d_impedance', limits.damping),
        ('k_nullspace', limits.nullspace_stiffness),
        ('force_command', limits.wrench),
    ):
        value = getattr(message, name)
        if name == 'd_impedance' and not value:
            # The SDK omits damping to use controller defaults.
            continue
        values = vector(
            value, len(bound), name, nonnegative=name != 'force_command'
        )
        if any(abs(a) > b for a, b in zip(values, bound, strict=True)):
            msg = f'{name} exceeds the application envelope'
            raise ValueError(msg)
    if len(message.max_impedance) != CARTESIAN_AXES or any(
        type(value) is not bool for value in message.max_impedance
    ):
        msg = 'max_impedance requires six boolean values'
        raise ValueError(msg)
    if len(message.interaction_control_mode) != CARTESIAN_AXES or any(
        value not in (1, 2, 3, 4) for value in message.interaction_control_mode
    ):
        msg = 'Each interaction axis requires a supported mode (1..4)'
        raise ValueError(msg)
    frame = message.interaction_frame
    CartesianPose(
        (frame.position.x, frame.position.y, frame.position.z),
        (
            frame.orientation.x,
            frame.orientation.y,
            frame.orientation.z,
            frame.orientation.w,
        ),
    )
    if not message.endpoint_name or not message.interaction_control_active:
        msg = 'An active interaction command needs a known endpoint'
        raise ValueError(msg)
