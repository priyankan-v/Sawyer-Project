"""Compatibility exports; implementation lives in execution.native."""

from __future__ import annotations

from .execution.native import MotionActions
from .planning import native as _native
from .planning.native import (
    check_finite,
    validate_tracking,
)

__all__ = [
    'MotionActions',
    'check_finite',
    'joint_trajectory',
    'run_joint_path',
    'run_trajectory',
    'validate_cartesian_waypoint',
    'validate_tracking',
    'validate_trajectory',
    'validate_waypoint_options',
]


def validate_waypoint_options(node, options):
    return _native.validate_waypoint_options(options, node.limits)


def validate_cartesian_waypoint(node, waypoint, *, initial):
    return _native.validate_cartesian_waypoint(
        waypoint,
        initial=initial,
        frame=node.config.frame,
        current_pose=node.wait_pose,
        position_tolerance=node.config.planning_position_tolerance,
        orientation_tolerance=node.config.planning_orientation_tolerance,
    )


def validate_trajectory(node, trajectory):
    return _native.validate_trajectory(
        trajectory,
        limits=node.limits,
        joints=node.joints,
        frame=node.config.frame,
        current_pose=node.wait_pose,
        position_tolerance=node.config.planning_position_tolerance,
        orientation_tolerance=node.config.planning_orientation_tolerance,
        joint_tolerance=node.config.planning_joint_tolerance,
    )


def joint_trajectory(node, positions, *, speed=0.05, max_acceleration=0.1):
    return _native.joint_trajectory(
        positions,
        start=node.joints,
        limits=node.limits,
        tip=node.config.tip,
        speed=speed,
        max_acceleration=max_acceleration,
    )


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
    return driver.move_native_joint_path(
        positions,
        execute=execute,
        plan_only=plan_only,
        speed=speed,
        max_acceleration=max_acceleration,
        timeout=timeout,
        cancel_event=cancel_event,
        on_feedback=on_feedback,
    )


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
    return driver.execute_native_trajectory(
        trajectory,
        execute=execute,
        plan_only=plan_only,
        timeout=timeout,
        cancel_event=cancel_event,
        on_feedback=on_feedback,
    )
