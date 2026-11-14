"""Controller-planned joint paths through the native Intera action server."""

from __future__ import annotations

import math
import time
from copy import deepcopy
from dataclasses import asdict

from ..config import JOINT_NAMES, positive
from . import wire
from .actions import NativeActionClient
from .base import CartesianPose, pose_error, vector


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


def validate_waypoint_options(node, options):
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
            for a, b in zip(
                acceleration, node.limits.acceleration, strict=True
            )
        ):
            msg = 'Native acceleration exceeds published limits'
            raise ValueError(msg)
    if options.corner_distance < 0:
        msg = 'Corner distance must be nonnegative'
        raise ValueError(msg)


def validate_cartesian_waypoint(node, waypoint, *, initial):
    _, current = node.wait_pose(waypoint.active_endpoint)
    pose = waypoint.pose
    target = CartesianPose(
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ),
        pose.header.frame_id or node.config.frame,
    )
    if target.frame != node.config.frame:
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
        if (
            position > node.config.planning_position_tolerance
            or rotation > node.config.planning_orientation_tolerance
        ):
            msg = 'First native pose differs from measured start'
            raise ValueError(msg)


def validate_trajectory(node, trajectory):
    """Check supported joint/Cartesian native paths before any action goal."""
    from .interaction import InteractionLimits, validate_command

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
        validate_waypoint_options(node, waypoint.options)
        if waypoint.joint_positions:
            positions = vector(waypoint.joint_positions, 7, 'Native joints')
            node.limits.check_position(positions, 'Native waypoint')
        if options.interpolation_type == wire.TrajectoryOptions.JOINT:
            positions = vector(waypoint.joint_positions, 7, 'Native joints')
            if (
                index == 0
                and max(
                    abs(a - b)
                    for a, b in zip(positions, node.joints, strict=True)
                )
                > node.config.planning_joint_tolerance
            ):
                msg = 'First native waypoint differs from measured start'
                raise ValueError(msg)
        else:
            validate_cartesian_waypoint(node, waypoint, initial=index == 0)
    wire.serialize(trajectory)


def run_trajectory(  # noqa: PLR0913
    driver,
    trajectory,
    *,
    execute=False,
    plan_only=False,
    timeout=60.0,
    cancel_event=None,
    on_feedback=None,
):
    if type(execute) is not bool or type(plan_only) is not bool:
        msg = 'execute and plan_only must be booleans'
        raise TypeError(msg)
    with driver._operation():  # noqa: SLF001
        driver._refresh()  # noqa: SLF001
        node = driver._node  # noqa: SLF001
        trajectory = deepcopy(trajectory)
        validate_trajectory(node, trajectory)
        goal = wire.MotionCommandGoal(
            command='generate' if plan_only else 'start', trajectory=trajectory
        )
        if not execute:
            return {'executed': False, 'goal': asdict(goal)}
        if (
            trajectory.trajectory_options.interaction_control
            and not plan_only
            and not driver.config.allow_unvalidated_modes
        ):
            msg = (
                'Native interaction trajectories require explicit '
                'allow_unvalidated_modes'
            )
            raise RuntimeError(msg)
        node.cancel_requested.clear()
        node.check_ready()
        return driver._actions.run(  # noqa: SLF001
            goal,
            timeout=timeout,
            cancel_event=cancel_event,
            on_feedback=on_feedback,
        )


def joint_trajectory(node, positions, *, speed=0.05, max_acceleration=0.1):
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
    for values in (node.joints, *positions):
        target = vector(values, 7, 'Native waypoint')
        node.limits.check_position(target, 'Native waypoint')
        options = wire.WaypointOptions(
            max_joint_speed_ratio=speed,
            max_joint_accel=[
                min(max_acceleration, v) for v in node.limits.acceleration
            ],
            joint_tolerances=[0.05] * 7,
        )
        trajectory.waypoints.append(
            wire.Waypoint(
                joint_positions=list(target),
                options=options,
                active_endpoint=node.config.tip or 'right_hand',
            )
        )
    return trajectory


class MotionActions:
    """Lazy clients tied to one connected driver session."""

    def __init__(self, node, config):
        self.node, self.config = node, config
        self.clients = {}

    def client(self, name):
        if name not in self.clients:
            namespace, action_type = {
                'motion': (
                    self.config.native_motion_action,
                    'intera_motion_msgs/MotionCommand',
                ),
                'calibration': (
                    self.config.calibration_action,
                    'intera_core_msgs/CalibrationCommand',
                ),
            }[name]
            self.clients[name] = NativeActionClient(
                self.node,
                namespace,
                action_type,
                server_timeout=self.config.action_server_timeout,
            )
        return self.clients[name]

    def check_health(self):
        self.node.check_ready()
        if self.node.cancel_requested.is_set():
            msg = 'Controller action cancelled'
            raise RuntimeError(msg)

    def run(
        self,
        goal,
        *,
        name='motion',
        timeout=60.0,
        cancel_event=None,
        on_feedback=None,
    ):
        result = self.client(name).run(
            goal,
            timeout=timeout,
            cancel_event=cancel_event,
            on_feedback=on_feedback,
            health_check=self.check_health,
        )
        return asdict(result) | {
            'succeeded': result.succeeded and bool(result.result.result)
        }


def run_joint_path(  # noqa: PLR0913
    driver,
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
    """
    Preview, generate on the controller, or explicitly execute a joint
    path.
    """
    if type(execute) is not bool or type(plan_only) is not bool:
        msg = 'execute and plan_only must be booleans'
        raise TypeError(msg)
    with driver._operation():  # noqa: SLF001
        driver._refresh()  # noqa: SLF001
        node = driver._node  # noqa: SLF001
        trajectory = joint_trajectory(
            node, positions, speed=speed, max_acceleration=max_acceleration
        )
        goal = wire.MotionCommandGoal(
            command='generate' if plan_only else 'start', trajectory=trajectory
        )
        if not execute:
            return {'executed': False, 'goal': asdict(goal)}
        node.cancel_requested.clear()
        node.check_ready()
        report = driver._actions.run(  # noqa: SLF001
            goal,
            timeout=timeout,
            cancel_event=cancel_event,
            on_feedback=on_feedback,
        )
        report.update(executed=not plan_only, planned_only=plan_only)
        if report['succeeded'] and not plan_only:
            node.spin_until(time.monotonic() + driver.config.callback_timeout)
            node.check_ready()
            report['max_final_error_rad'] = max(
                abs(a - b)
                for a, b in zip(
                    trajectory.waypoints[-1].joint_positions,
                    node.joints,
                    strict=True,
                )
            )
            report['arrival_confirmed'] = (
                report['max_final_error_rad']
                <= driver.config.position_tolerance
            )
        return report
