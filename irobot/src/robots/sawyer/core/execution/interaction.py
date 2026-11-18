"""Bounded interaction execution with confirmed exit."""

from __future__ import annotations

import time

from ..communication import wire
from ..types import pose_error
from .streaming import JointStream


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
