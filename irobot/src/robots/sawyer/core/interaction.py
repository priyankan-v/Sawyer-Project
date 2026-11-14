"""Explicit interaction sessions with state confirmation and bounded exits."""

from __future__ import annotations

import time
from copy import deepcopy
from dataclasses import asdict, dataclass

from ..config import positive
from . import wire
from .base import CartesianPose, pose_error, vector
from .control import JointStream

CARTESIAN_AXES = 6


@dataclass(frozen=True)
class InteractionLimits:
    """Application bounds, not manufacturer-certified Cartesian limits."""

    stiffness: tuple = (1300.0, 1300.0, 1300.0, 30.0, 30.0, 30.0)
    damping: tuple = (100.0,) * 6
    nullspace_stiffness: tuple = (10.0,) * 7
    wrench: tuple = (5.0, 5.0, 5.0, 1.0, 1.0, 1.0)
    translation: float = 0.01
    rotation: float = 0.1

    def __post_init__(self):
        for name, length in (
            ('stiffness', 6),
            ('damping', 6),
            ('nullspace_stiffness', 7),
            ('wrench', 6),
        ):
            object.__setattr__(
                self,
                name,
                vector(getattr(self, name), length, name, positive=True),
            )
        positive('translation', self.translation)
        positive('rotation', self.rotation)


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


class InteractionSession:
    """
    Observe active/inactive feedback and always request deactivation on exit.

    Interaction commands have no proven process-loss watchdog on this robot.
    This session cannot promise deactivation after a workstation crash.
    """

    def __init__(self, node, limits, stream_options):
        self.node, self.limits = node, limits
        self.guard = JointStream(node, 'velocity', stream_options)
        self.state = None
        self.subscription = node.observe(
            wire.InteractionControlState,
            node.config.interaction_state_topic,
            self.on_state,
        )
        self.publisher = node.publisher_for(
            wire.InteractionControlCommand,
            node.config.interaction_command_topic,
        )

    def on_state(self, message):
        self.state = message

    def state_ready(self):
        return self.state is not None and (
            time.monotonic() - self.state.received_at
            <= self.node.config.feedback_timeout
        )

    def wait_ready(self):
        deadline = time.monotonic() + self.node.config.connect_timeout
        while (
            not self.state_ready()
            or not self.publisher.get_subscription_count()
        ):
            self.node.check_ready()
            if time.monotonic() >= deadline:
                msg = 'Interaction feedback/command route unavailable'
                raise TimeoutError(msg)
            self.node.spin_once(timeout_sec=0.01)
        if self.state.interaction_control_active:
            msg = 'Another interaction controller is already active'
            raise RuntimeError(msg)

    def deactivate(self):
        message = wire.InteractionControlCommand(
            interaction_control_active=False
        )
        deadline = time.monotonic() + self.node.config.connect_timeout
        sent_at = time.monotonic()
        while time.monotonic() < deadline:
            self.publisher.publish(message)
            self.node.spin_once(timeout_sec=0.02)
            if (
                self.state_ready()
                and self.state.received_at >= sent_at
                and (not self.state.interaction_control_active)
            ):
                return
        msg = 'Interaction deactivation was not confirmed'
        raise TimeoutError(msg)

    def run(self, message, duration, *, cancel_event=None, on_step=None):
        self.wait_ready()
        self.guard.check_health(cancel_event)
        _, start_pose = self.node.wait_pose(message.endpoint_name)
        start, samples, seen_active = time.monotonic(), 0, False
        next_tick, previous = start, start
        self.node.streaming = True
        try:
            while time.monotonic() - start < duration:
                now = time.monotonic()
                if now < next_tick:
                    self.node.spin_once(timeout_sec=min(0.01, next_tick - now))
                    continue
                self.guard.check_health(cancel_event)
                if not self.state_ready():
                    msg = 'Interaction feedback is stale'
                    raise RuntimeError(msg)
                seen_active |= self.state.interaction_control_active
                _, pose = self.node.wait_pose(message.endpoint_name)
                translation, rotation = pose_error(start_pose, pose)
                if (
                    translation > self.limits.translation
                    or rotation > self.limits.rotation
                ):
                    msg = (
                        'Interaction displacement exceeded the application '
                        'envelope'
                    )
                    raise RuntimeError(msg)
                if on_step is not None:
                    on_step(
                        now - start,
                        tuple(self.node.joints),
                        tuple(self.node.velocity),
                    )
                self.guard.check_health(cancel_event)
                ready_at = time.monotonic()
                if ready_at - previous > self.node.config.command_timeout / 2:
                    msg = 'Interaction loop missed its deadline'
                    raise RuntimeError(msg)
                if ready_at - start >= duration:
                    break
                message.header.stamp = self.node.get_clock().now().to_msg()
                self.publisher.publish(message)
                samples += 1
                previous, next_tick = (
                    ready_at,
                    ready_at + 1 / self.guard.options.rate,
                )
        finally:
            try:
                self.deactivate()
            finally:
                self.node.stop(position_only=True)
        return {
            'executed': True,
            'activation_confirmed': seen_active,
            'deactivation_confirmed': True,
            'samples': samples,
            'duration_s': time.monotonic() - start,
        }


def run_interaction(  # noqa: PLR0913
    driver,
    message,
    duration,
    *,
    execute=False,
    limits=None,
    stream_options=None,
    cancel_event=None,
    on_step=None,
):
    from ..config import StreamOptions

    positive('duration', duration)
    if type(execute) is not bool:
        msg = 'execute must be a boolean'
        raise TypeError(msg)
    limits = limits or InteractionLimits()
    message = deepcopy(message)
    validate_command(message, limits)
    with driver._operation():  # noqa: SLF001
        driver._refresh()  # noqa: SLF001
        node = driver._node  # noqa: SLF001
        node.wait_pose(message.endpoint_name)
        if (message.header.frame_id or 'base') != node.config.frame:
            msg = 'Interaction header must use the configured base frame'
            raise ValueError(msg)
        if not execute:
            return {
                'executed': False,
                'command': asdict(message),
                'limits': asdict(limits),
            }
        exploratory = (
            any(
                mode != wire.InteractionControlCommand.IMPEDANCE_MODE
                for mode in message.interaction_control_mode
            )
            or not all(message.max_impedance)
            or any(message.force_command)
        )
        if exploratory and not driver.config.allow_unvalidated_modes:
            msg = (
                'Force/compliant control is not hardware-qualified; '
                'explicit allow_unvalidated_modes is required'
            )
            raise RuntimeError(msg)
        node.cancel_requested.clear()
        if driver._interaction is None:  # noqa: SLF001
            driver._interaction = InteractionSession(  # noqa: SLF001
                node, limits, stream_options or StreamOptions()
            )
        session = driver._interaction  # noqa: SLF001
        session.limits = limits
        session.guard = JointStream(
            node, 'velocity', stream_options or StreamOptions()
        )
        return session.run(
            message, duration, cancel_event=cancel_event, on_step=on_step
        )
