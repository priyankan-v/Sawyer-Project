"""Timed trajectory tracking and joint/Cartesian execution."""

from __future__ import annotations

import time

from ..communication.wire import Float64
from ..planning.joint import JointSequence
from ..types import (
    CartesianMoveResult,
    JointMoveResult,
    MotionResult,
    pose_error,
)
from .pid import JointPIDController


def check_trajectory_limits(node, trajectory):
    if any(
        a < b
        for a, b in zip(
            node.limits.velocity, trajectory.max_velocity, strict=False
        )
    ) or any(
        a < b
        for a, b in zip(
            node.limits.acceleration,
            trajectory.max_acceleration,
            strict=False,
        )
    ):
        msg = 'Robot limits changed during the move; replan required'
        raise RuntimeError(msg)
    # Cartesian IK paths can have interior joint extrema beyond both ends.
    minimum = getattr(
        trajectory,
        'min_position',
        tuple(
            min(a, b)
            for a, b in zip(trajectory.start, trajectory.end, strict=False)
        ),
    )
    maximum = getattr(
        trajectory,
        'max_position',
        tuple(
            max(a, b)
            for a, b in zip(trajectory.start, trajectory.end, strict=False)
        ),
    )
    node.limits.check_position(
        minimum, 'Reference position minimum', tolerance=1e-8
    )
    node.limits.check_position(
        maximum, 'Reference position maximum', tolerance=1e-8
    )


def track(  # noqa: PLR0912, PLR0913, PLR0915
    node,
    trajectory,
    controller,
    options,
    *,
    label='Motion',
    on_step=None,
    at_goal=None,
    announce=True,
):
    if node.cancel_requested.is_set():
        msg = 'Motion cancelled'
        raise RuntimeError(msg)
    period = 1.0 / options.rate
    timeout = (
        options.timeout
        if options.timeout is not None
        else trajectory.duration + options.settle_time
    )
    if timeout < trajectory.duration:
        msg = '--timeout must be at least the planned motion duration'
        raise ValueError(msg)
    node.check_ready()
    check_trajectory_limits(node, trajectory)
    transport = node.__dict__.get('transport')
    if transport is not None:
        transport.check_command_owner()
        node.check_ready()
    controller.reset(node.velocity)
    node.watchdog_pub.publish(Float64(data=node.config.command_timeout))
    start = time.monotonic()
    previous, next_tick = start - period, start
    stable_since = None
    next_watchdog = start + node.config.watchdog_refresh_interval
    node.streaming = True
    transport = node.__dict__.get('transport')
    generation = transport.generation if transport is not None else None
    try:
        while True:
            if transport is not None and transport.generation != generation:
                msg = (
                    'Transport registration changed; explicit restart required'
                )
                raise RuntimeError(msg)
            if node.cancel_requested.is_set():
                msg = 'Motion cancelled'
                raise RuntimeError(msg)
            now = time.monotonic()
            if now < next_tick:
                node.spin_once(timeout_sec=min(period, next_tick - now))
                continue
            dt, elapsed = now - previous, now - start
            if dt > node.config.command_timeout / 2:
                msg = (
                    f'Control loop missed its deadline '
                    f'(>{node.config.command_timeout / 2:.3f} s); stopping'
                )
                raise RuntimeError(msg)
            if elapsed > timeout:
                msg = f'Motion/settling timeout; {label} was not confirmed'
                raise TimeoutError(msg)
            node.check_ready()
            if not node.routes_connected():
                msg = 'Sawyer command subscriber disappeared'
                raise RuntimeError(msg)
            check_trajectory_limits(node, trajectory)
            reference = trajectory.sample(elapsed)
            if (
                max(
                    abs(a - b)
                    for a, b in zip(
                        reference.position, node.joints, strict=False
                    )
                )
                > options.max_error
            ):
                msg = 'Joint tracking error exceeded --max-error'
                raise RuntimeError(msg)
            if on_step is not None:
                on_step(elapsed, reference)
            command = controller.update(
                reference.position,
                node.joints,
                reference.velocity,
                node.velocity,
                dt,
            )
            node.check_ready()
            if time.monotonic() - now > node.config.command_timeout / 2:
                msg = 'Control callback missed its deadline; stopping'
                raise RuntimeError(msg)
            if node.cancel_requested.is_set():
                msg = 'Motion cancelled'
                raise RuntimeError(msg)
            node.publish_velocity(command)
            if now >= next_watchdog:
                node.watchdog_pub.publish(
                    Float64(data=node.config.command_timeout)
                )
                next_watchdog = now + node.config.watchdog_refresh_interval
            arrived = (
                elapsed >= trajectory.duration
                and max(
                    abs(a - b)
                    for a, b in zip(trajectory.end, node.joints, strict=False)
                )
                < node.config.position_tolerance
                and max(map(abs, node.velocity))
                < node.config.velocity_tolerance
                and max(map(abs, command)) < node.config.velocity_tolerance
                and (at_goal is None or at_goal())
            )
            stable_since = (
                (now if stable_since is None else stable_since)
                if arrived
                else None
            )
            if (
                stable_since is not None
                and now - stable_since >= node.config.settle_confirmation
            ):
                break
            previous, next_tick = now, now + period
    finally:
        node.stop()
    # Stop streaming before potentially blocking on terminal output.
    if announce:
        pass
    return MotionResult(trajectory.duration, elapsed, tuple(node.joints))


def execute_joint_plan(  # noqa: PLR0913
    node,
    plan,
    options,
    velocity,
    acceleration,
    *,
    cancel_event=None,
    on_step=None,
):
    if not options.execute:
        return JointMoveResult(plan, None)
    node.wait_inputs(True)
    node.check_ready()
    if max(map(abs, node.velocity)) >= node.config.velocity_tolerance:
        msg = 'Rest-to-rest motion requires a stationary arm'
        raise RuntimeError(msg)
    if (
        max(abs(a - b) for a, b in zip(plan.start, node.joints, strict=False))
        > node.config.planning_joint_tolerance
    ):
        msg = 'Arm moved during planning; rerun from the new pose'
        raise RuntimeError(msg)
    if node.cancel_requested.is_set() or (
        cancel_event is not None and cancel_event.is_set()
    ):
        msg = 'Motion cancelled'
        raise RuntimeError(msg)
    if plan.duration == 0 or (
        not isinstance(plan, JointSequence)
        and max(abs(a - b) for a, b in zip(plan.end, node.joints, strict=True))
        < node.config.position_tolerance
    ):
        return JointMoveResult(plan, None)
    controller = JointPIDController(
        7,
        options.kp,
        options.ki,
        options.kd,
        velocity,
        acceleration,
        position_lower=node.limits.lower,
        position_upper=node.limits.upper,
    )

    def monitor(elapsed, reference):
        if cancel_event is not None and cancel_event.is_set():
            msg = 'Motion cancelled'
            raise RuntimeError(msg)
        if on_step is not None:
            on_step(
                elapsed,
                reference,
                tuple(node.joints),
                tuple(node.velocity),
            )

    execution = track(
        node,
        plan,
        controller,
        options,
        label='Joint target',
        announce=False,
        on_step=monitor,
    )
    return JointMoveResult(plan, execution)


def execute_cartesian_plan(node, plan, options, tip):
    """Execute a planned Cartesian path using measured joint and tip state."""
    start_joints = plan.trajectory.start
    start_pose = plan.line.start
    velocity = plan.trajectory.max_velocity
    acceleration = plan.trajectory.max_acceleration
    node.wait_inputs(True)
    node.wait_pose(tip)
    node.check_ready()
    node.planning_health(start_joints, start_pose, tip)
    if plan.trajectory.duration == 0:
        return CartesianMoveResult(plan, None)
    controller = JointPIDController(
        7,
        options.kp,
        options.ki,
        options.kd,
        velocity,
        acceleration,
        position_lower=node.limits.lower,
        position_upper=node.limits.upper,
    )

    def monitor(elapsed, _joint_reference):
        fraction = (
            elapsed / plan.trajectory.duration
            if plan.trajectory.duration
            else 1.0
        )
        position, orientation = pose_error(
            node.pose_feedback.read(tip), plan.line.sample(fraction)
        )
        if (
            position > options.max_cartesian_error
            or orientation > options.max_orientation_error
        ):
            msg = (
                f'Cartesian tracking error: {position:.6f} m / '
                f'{orientation:.6f} rad exceeds configured bounds'
            )
            raise RuntimeError(msg)

    def at_goal():
        position, orientation = pose_error(
            node.pose_feedback.read(tip), plan.line.end
        )
        return (
            position < options.cartesian_tolerance
            and orientation < options.goal_orientation_tolerance
        )

    execution = track(
        node,
        plan.trajectory,
        controller,
        options,
        label='Cartesian target',
        on_step=monitor,
        at_goal=at_goal,
        announce=False,
    )
    return CartesianMoveResult(plan, execution)
