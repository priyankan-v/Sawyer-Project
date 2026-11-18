"""Bounded joint command sessions with watchdog and measured-position exit."""

from __future__ import annotations

import time
from dataclasses import asdict

from ...config import JOINT_NAMES, StreamOptions, positive, vector
from ..communication import wire
from ..hardware.model import effort_limits

MODES = {
    'position': (wire.JointCommand.POSITION_MODE, ('position',)),
    'velocity': (wire.JointCommand.VELOCITY_MODE, ('velocity',)),
    'torque': (wire.JointCommand.TORQUE_MODE, ('effort',)),
    'trajectory': (
        wire.JointCommand.TRAJECTORY_MODE,
        ('position', 'velocity', 'acceleration'),
    ),
}


class JointStream:
    """
    Execute one explicitly bounded session, never a retained command queue.

    Trajectory mode has the SDK's reduced controller-side collision handling.
    All modes retain local collision/readiness checks and displacement bounds.
    """

    def __init__(self, node, mode, options):
        if mode not in MODES:
            msg = f'Unknown command mode: {mode}'
            raise ValueError(msg)
        if not isinstance(options, StreamOptions):
            msg = 'Use StreamOptions for command envelopes'
            raise TypeError(msg)
        self.node, self.mode, self.options = node, mode, options
        self.start = tuple(node.joints)
        self.generation = node.generation
        self.effort = None
        if mode == 'torque':
            description = node.master(
                'getParam', node.config.robot_description_parameter
            )
            self.effort = effort_limits(description)
            self.torque_limits()

    def torque_limits(self):
        published = self.node.__dict__.get('joint_effort_limits')
        if published is None:
            msg = (
                'Published joint effort limits unavailable for torque control'
            )
            raise RuntimeError(msg)
        return tuple(
            min(a, b) for a, b in zip(self.effort, published, strict=True)
        )

    def command(self, values):
        mode, fields = MODES[self.mode]
        if not isinstance(values, dict) or set(values) != set(fields):
            msg = f'{self.mode} commands require exactly {fields}'
            raise ValueError(msg)
        message = wire.JointCommand(mode=mode, names=list(JOINT_NAMES))
        for field in fields:
            setattr(message, field, list(vector(values[field], 7, field)))
        bounds, options = self.node.limits, self.options
        if message.position:
            bounds.check_position(message.position, 'Command position')
            self.check_displacement(message.position)
        for field, published_limits in (
            ('velocity', [options.max_velocity] * 7),
            ('acceleration', [options.max_acceleration] * 7),
            (
                'effort',
                self.torque_limits() if self.mode == 'torque' else None,
            ),
        ):
            values = getattr(message, field)
            if not values:
                continue
            limits = published_limits
            if field == 'effort':
                limits = [v * options.effort_fraction for v in limits]
            elif field == 'velocity':
                limits = [
                    min(a, b)
                    for a, b in zip(limits, bounds.velocity, strict=True)
                ]
            else:
                limits = [
                    min(a, b)
                    for a, b in zip(limits, bounds.acceleration, strict=True)
                ]
            if any(abs(a) > b for a, b in zip(values, limits, strict=True)):
                msg = f'{field} command exceeds session/controller limits'
                raise ValueError(msg)
        return message

    def check_displacement(self, positions):
        if any(
            abs(a - b) > self.options.max_displacement
            for a, b in zip(positions, self.start, strict=True)
        ):
            msg = 'Joint displacement exceeds the session envelope'
            raise RuntimeError(msg)

    def check_health(self, cancel_event):
        self.node.check_ready()
        if self.node.generation != self.generation:
            msg = 'Transport changed; explicit restart required'
            raise ConnectionError(msg)
        if self.node.cancel_requested.is_set() or (
            cancel_event is not None and cancel_event.is_set()
        ):
            msg = 'Joint stream cancelled'
            raise RuntimeError(msg)
        if not self.node.routes_connected():
            msg = 'Joint command/watchdog subscriber disconnected'
            raise ConnectionError(msg)
        self.check_displacement(self.node.joints)
        if max(map(abs, self.node.velocity)) > self.options.max_velocity:
            msg = 'Measured joint velocity exceeds the session envelope'
            raise RuntimeError(msg)

    def prepare(self, cancel_event):
        node, config = self.node, self.node.config
        transport = node.__dict__.get('transport')
        if transport is not None:
            transport.check_command_owner()
            self.check_health(cancel_event)
        if self.mode == 'position':
            publisher = node.publisher_for(
                wire.Float64, config.speed_ratio_topic
            )
            deadline = time.monotonic() + config.connect_timeout
            while not publisher.get_subscription_count():
                self.check_health(cancel_event)
                if time.monotonic() >= deadline:
                    msg = 'Position speed subscriber unavailable'
                    raise TimeoutError(msg)
                node.spin_once(timeout_sec=0.01)
            speeds = [self.options.position_speed] + [
                self.options.max_velocity / limit
                for limit in node.limits.velocity
            ]
            speed = min(speeds)
            publisher.publish(wire.Float64(data=speed))
        self.check_health(cancel_event)
        node.watchdog_pub.publish(wire.Float64(data=config.command_timeout))

    def run(self, source, duration, *, cancel_event=None, on_step=None):
        if (
            self.mode in ('torque', 'trajectory')
            and not self.node.config.allow_unvalidated_modes
        ):
            msg = (
                'This mode is not hardware-qualified; explicit '
                'allow_unvalidated_modes is required'
            )
            raise RuntimeError(msg)
        positive('duration', duration)
        sample = source if callable(source) else lambda _t: source
        self.command(sample(0.0))
        self.check_health(cancel_event)
        self.prepare(cancel_event)
        node, config = self.node, self.node.config
        start = time.monotonic()
        previous, next_tick = start, start
        sent, max_gap, next_watchdog = 0, 0.0, start
        node.streaming = True
        previous_position = self.start
        try:
            while time.monotonic() - start < duration:
                now = time.monotonic()
                if now < next_tick:
                    node.spin_once(timeout_sec=min(0.01, next_tick - now))
                    continue
                self.check_health(cancel_event)
                message = self.command(sample(now - start))
                if on_step is not None:
                    on_step(
                        now - start, tuple(node.joints), tuple(node.velocity)
                    )
                self.check_health(cancel_event)
                ready_at = time.monotonic()
                if ready_at - start >= duration:
                    break
                gap = ready_at - previous
                if gap > config.command_timeout / 2:
                    msg = 'Joint stream missed its deadline; stopping'
                    raise RuntimeError(msg)
                if self.mode == 'trajectory':
                    delta_limit = self.options.max_velocity * max(
                        gap, 1 / self.options.rate
                    )
                    if (
                        max(
                            abs(a - b)
                            for a, b in zip(
                                message.position,
                                previous_position,
                                strict=True,
                            )
                        )
                        > delta_limit
                    ):
                        msg = (
                            'Trajectory position step exceeds velocity '
                            'envelope'
                        )
                        raise RuntimeError(msg)
                    previous_position = tuple(message.position)
                message.header.stamp = node.get_clock().now().to_msg()
                node.command_pub.publish(message)
                if ready_at >= next_watchdog:
                    node.watchdog_pub.publish(
                        wire.Float64(data=config.command_timeout)
                    )
                    next_watchdog = ready_at + config.watchdog_refresh_interval
                sent += 1
                max_gap = max(max_gap, gap)
                previous, next_tick = (
                    ready_at,
                    ready_at + 1 / self.options.rate,
                )
        finally:
            node.stop(position_only=self.mode in ('torque', 'trajectory'))
        self.check_displacement(node.joints)
        return {
            'executed': True,
            'mode': self.mode,
            'samples': sent,
            'duration_s': time.monotonic() - start,
            'max_command_gap_s': max_gap,
            'joints': tuple(node.joints),
        }


def stream_joint_commands(  # noqa: PLR0913
    node,
    mode,
    source,
    duration,
    *,
    options=None,
    execute=False,
    cancel_event=None,
    on_step=None,
):
    """Preview or execute a finite stream using the owned hardware session."""
    positive('duration', duration)
    if type(execute) is not bool:
        msg = 'execute must be a boolean'
        raise TypeError(msg)
    session = JointStream(node, mode, options or StreamOptions())
    preview = session.command(source(0.0) if callable(source) else source)
    if not execute:
        return {
            'executed': False,
            'command': asdict(preview),
            'duration_s': duration,
            'envelope': asdict(session.options),
        }
    node.cancel_requested.clear()
    node.wait_inputs(True)
    return session.run(
        source, duration, cancel_event=cancel_event, on_step=on_step
    )
