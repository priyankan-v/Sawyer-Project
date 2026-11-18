"""One shared arm session, state and low-level command access."""

from __future__ import annotations

import threading
import time

from ...config import JOINT_NAMES, vector
from ..types import JointBounds, ordered_values, pose_error
from .feedback import EndpointFeedback, JointFeedback, receipt_time


class ArmSession:
    def __init__(self, config, transport):
        from ..communication.wire import (
            CollisionDetectionState,
            EndpointState,
            EndpointStates,
            Float64,
            JointCommand,
            JointLimits,
            JointState,
            RobotAssemblyState,
        )

        self.config, self.transport = config, transport
        self.joints = self.velocity = self.state = None
        self.limits = self.collision = None
        self.joint_effort_limits = None
        self.feedback = JointFeedback(JOINT_NAMES)
        self.last_feedback_error = self.last_limits_error = None
        self.joints_at = self.state_at = self.collision_at = 0.0
        self.streaming = False
        self.cancel_requested = threading.Event()
        self.pose_feedback = EndpointFeedback(
            config.frame, config.feedback_timeout, config
        )
        for kind, topic, callback in (
            (JointState, config.joint_state_topic, self.on_joints),
            (RobotAssemblyState, config.robot_state_topic, self.on_state),
            (
                CollisionDetectionState,
                config.collision_topic,
                self.on_collision,
            ),
            (JointLimits, config.joint_limits_topic, self.on_limits),
            (
                EndpointState,
                config.endpoint_topic,
                self.pose_feedback.on_endpoint,
            ),
            (
                EndpointStates,
                config.tip_states_topic,
                self.pose_feedback.on_tips,
            ),
        ):
            transport.create_subscription(kind, topic, callback)
        self.command_pub = transport.create_publisher(
            JointCommand, config.joint_command_topic
        )
        self.watchdog_pub = transport.create_publisher(
            Float64, config.command_timeout_topic
        )

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, 'transport'), name)

    def spin_once(self, timeout_sec=0.02):
        self.transport.spin_once(timeout_sec=timeout_sec)

    def on_joints(self, message):
        try:
            updated = self.feedback.update(
                message.name,
                message.position,
                message.velocity,
                receipt_time(message),
                efforts=message.effort,
            )
        except ValueError as exc:
            self.last_feedback_error = str(exc)
            return
        if updated:
            self.joints, self.velocity, self.joints_at = (
                self.feedback.snapshot()
            )
            self.last_feedback_error = None

    def on_state(self, message):
        self.state, self.state_at = message, receipt_time(message)

    def on_collision(self, message):
        self.collision, self.collision_at = (
            message.collision_state,
            receipt_time(message),
        )

    def on_limits(self, message):
        self.joint_effort_limits = None
        if message.effort:
            try:
                values = ordered_values(
                    message.joint_names, message.effort, 'Effort limits'
                )
                self.joint_effort_limits = vector(
                    values, 7, 'Effort limits', positive=True
                )
            except ValueError:
                self.get_logger().warning('Invalid published effort limits')
        try:
            self.limits = JointBounds.from_message(message)
            self.last_limits_error = None
        except ValueError as exc:
            self.limits = None
            self.last_limits_error = str(exc)
            self.get_logger().error(str(exc))

    def input_issues(self, *, require_collision=True):
        now = time.monotonic()
        issues = []
        joint_issues = self.feedback.issues(now, self.config.feedback_timeout)
        if joint_issues:
            issues.append('/robot/joint_states: ' + '; '.join(joint_issues))
            if self.last_feedback_error:
                issues.append(
                    'last rejected arm sample: ' + self.last_feedback_error
                )
        for topic, value, stamp in (
            (self.config.robot_state_topic, self.state, self.state_at),
            (
                self.config.collision_topic,
                self.collision,
                self.collision_at,
            ),
        ):
            if not require_collision and topic == self.config.collision_topic:
                continue
            if value is None:
                issues.append(f'{topic}: no sample received')
            elif now - stamp >= self.config.feedback_timeout:
                issues.append(
                    f'{topic}: {now - stamp:.3f}s old (limit '
                    f'{self.config.feedback_timeout:.3f}s)'
                )
        if self.limits is None:
            issues.append(
                '/robot/joint_limits: '
                + (self.last_limits_error or 'no valid sample received')
            )
        return issues

    def fresh(self, *, require_collision=True):
        return not self.input_issues(require_collision=require_collision)

    def check_ready(self):
        transport = self.__dict__.get('transport')
        if transport is not None and not transport.master_available:
            msg = 'Robot master registration is unavailable'
            raise RuntimeError(msg)
        if transport is not None and transport.conflicting_publishers:
            msg = 'Another joint command publisher is active: ' + ', '.join(
                transport.conflicting_publishers
            )
            raise RuntimeError(msg)
        issues = self.input_issues()
        if issues:
            raise RuntimeError('Missing/stale feedback: ' + '; '.join(issues))
        if (
            not self.state.enabled
            or not self.state.ready
            or self.state.stopped
            or self.state.error
        ):
            msg = (
                f'Robot not ready: enabled={self.state.enabled}, '
                f'ready={self.state.ready}, '
                f'stopped={self.state.stopped}, error={self.state.error}'
            )
            raise RuntimeError(msg)
        if self.collision:
            msg = 'Robot reports a collision'
            raise RuntimeError(msg)
        self.limits.check_position(
            self.joints, 'Measured position', tolerance=1e-4
        )

    def wait_inputs(self, execute, timeout=None):
        deadline = time.monotonic() + (
            self.config.connect_timeout if timeout is None else timeout
        )
        while not self.fresh(require_collision=execute) or (
            execute and not self.routes_connected()
        ):
            if time.monotonic() >= deadline:
                issues = self.input_issues()
                if execute and not self.routes_connected():
                    issues.append('Sawyer command/watchdog subscribers absent')
                raise RuntimeError('Preflight timed out: ' + '; '.join(issues))
            self.spin_once(timeout_sec=self.config.input_poll_timeout)

    def wait_robot_state(self, timeout=None):
        deadline = time.monotonic() + (
            self.config.connect_timeout if timeout is None else timeout
        )
        while (
            self.state is None
            or time.monotonic() - self.state_at >= self.config.feedback_timeout
        ):
            if time.monotonic() >= deadline:
                msg = 'No fresh assembly state from Sawyer'
                raise TimeoutError(msg)
            self.spin_once(timeout_sec=self.config.callback_slice)

    def status_snapshot(self):
        """Read already-dispatched state without network access or mutation."""
        issues = self.input_issues()
        if not issues:
            try:
                self.check_ready()
                self.pose_feedback.select(self.config.tip)
            except (RuntimeError, ValueError) as exc:
                issues.append(str(exc))
        return {
            'connected': True,
            'ready': not issues,
            'issues': issues,
            'enabled': bool(self.state and self.state.enabled),
            'stopped': bool(self.state and self.state.stopped),
            'error': bool(self.state and self.state.error),
            'homed': bool(self.state and self.state.homed),
            'collision': self.collision,
            'joints': self.joints,
            'velocities': self.velocity,
            'joint_feedback_fresh': not self.feedback.issues(
                time.monotonic(), self.config.feedback_timeout
            ),
        }

    def routes_connected(self):
        return (
            self.command_pub.get_subscription_count() > 0
            and self.watchdog_pub.get_subscription_count() > 0
        )

    def publish_velocity(self, velocity):
        from ..communication.wire import JointCommand

        message = JointCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.mode = JointCommand.VELOCITY_MODE
        message.names = list(JOINT_NAMES)
        message.velocity = list(vector(velocity, 7, 'velocity command'))
        self.command_pub.publish(message)

    def spin_until(self, deadline):
        """
        Service callbacks for a real interval even when callbacks return
        immediately.
        """
        while time.monotonic() < deadline:
            self.spin_once(
                timeout_sec=max(
                    0.0,
                    min(
                        self.config.callback_slice, deadline - time.monotonic()
                    ),
                )
            )

    def stop(self, *, position_only=False):
        """
        Request zero velocity, then measured-position hold as in SDK
        exit_control_mode.

        Delivery and physical stopping cannot be confirmed after a
        connection loss.
        The robot's command watchdog remains set to 0.2 s for velocity mode.
        """
        from ..communication.wire import JointCommand

        if not self.streaming:
            return
        try:
            state = self.__dict__.get('state')
            if state is not None and (
                state.stopped or not state.enabled or state.error
            ):
                return
            zero_samples = (
                0 if position_only else self.config.stop_zero_samples
            )
            for _ in range(zero_samples):
                self.publish_velocity((0.0,) * 7)
                self.spin_until(
                    time.monotonic() + self.config.stop_publish_interval
                )
            positions = self.feedback.fresh_positions(
                time.monotonic(), self.config.feedback_timeout
            )
            if positions is not None:
                hold = JointCommand()
                hold.mode = JointCommand.POSITION_MODE
                hold.names = list(JOINT_NAMES)
                hold.position = list(positions)
                for _ in range(self.config.stop_hold_samples):
                    hold.header.stamp = self.get_clock().now().to_msg()
                    self.command_pub.publish(hold)
                    self.spin_until(
                        time.monotonic() + self.config.stop_publish_interval
                    )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f'Stop command delivery could not be confirmed: {exc}'
            )
        finally:
            self.streaming = False

    def wait_pose(self, tip=None, timeout=None):
        deadline = time.monotonic() + (
            self.config.connect_timeout if timeout is None else timeout
        )
        while True:
            try:
                return self.pose_feedback.select(tip)
            except RuntimeError as exc:
                if time.monotonic() >= deadline:
                    msg = (
                        f'Cartesian pose unavailable: {exc}. Check '
                        f'endpoint/tip-state routes'
                    )
                    raise RuntimeError(msg) from exc
                self.spin_once(timeout_sec=self.config.callback_timeout)

    def planning_health(self, start_joints, start_pose, tip):
        issues = self.input_issues()
        if issues:
            # Polynomial fitting can occupy this thread; drain queued feedback
            # before treating old receipt times as a transport failure. No
            # motion
            # is being commanded during planning. Execution retains the strict
            # per-step feedback watchdog in ArmMotionNode.
            self.wait_inputs(
                False, timeout=self.config.planning_refresh_timeout
            )
        if max(map(abs, self.velocity)) >= self.config.velocity_tolerance:
            msg = 'Cartesian rest-to-rest planning requires a stationary arm'
            raise RuntimeError(msg)
        if (
            max(
                abs(a - b)
                for a, b in zip(self.joints, start_joints, strict=False)
            )
            > self.config.planning_joint_tolerance
        ):
            msg = (
                'Arm moved during Cartesian planning; rerun from the new pose'
            )
            raise RuntimeError(msg)
        _, current_pose = self.wait_pose(
            tip, timeout=self.config.planning_refresh_timeout
        )
        position, orientation = pose_error(current_pose, start_pose)
        if (
            position > self.config.planning_position_tolerance
            or orientation > self.config.planning_orientation_tolerance
        ):
            msg = (
                'Endpoint moved during Cartesian planning; rerun from the '
                'new pose'
            )
            raise RuntimeError(msg)
