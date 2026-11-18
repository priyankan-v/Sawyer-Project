"""Sawyer connection lifecycle and public state/model access."""

from __future__ import annotations

import json
import math
import threading
import time
from contextlib import contextmanager
from dataclasses import asdict, replace

from ..config import (
    JOINT_NAMES,
    CartesianMotionOptions,
    MotionOptions,
    SawyerConfig,
)
from .hardware.feedback import (
    EndpointFeedback,
    JointFeedback,
    from_pose,
    receipt_time,
)
from .hardware.gripper import SawyerGripper
from .hardware.io import (
    Signal,
    command_args,
    read_signals,
    signal_value,
    time_key,
)
from .hardware.kinematics import InteraKinematics
from .hardware.session import ArmSession as _ArmSession
from .types import (
    CartesianPose,
    JointBounds,
    MotionResult,
    ordered_values,
    pose_error,
    vector,
)

__all__ = [
    'CartesianPose',
    'EndpointFeedback',
    'InteraKinematics',
    'JointBounds',
    'JointFeedback',
    'MotionResult',
    'SawyerBase',
    'SawyerGripper',
    'Signal',
    '_ArmSession',
    'command_args',
    'from_pose',
    'ordered_values',
    'pose_error',
    'read_signals',
    'receipt_time',
    'signal_value',
    'time_key',
    'vector',
]


class SawyerBase:
    """
    Owns the direct Python connection to Sawyer.

    Construction is offline. connect() registers transport only; it never
    enables Sawyer or calibrates the gripper. Methods are synchronous and
    process feedback on the caller thread.
    Poll get_status()/state properties to refresh feedback while idle.
    """

    def __init__(self, config: SawyerConfig | None = None):
        self.config = config or SawyerConfig()
        self._node = self._gripper = self._transport = None
        self._devices = None
        self._actions = None
        self._interaction = None
        self._connected = False
        self._busy = threading.Lock()

    @contextmanager
    def _operation(self, *, timeout=0.0):
        acquired = (
            self._busy.acquire(timeout=timeout)
            if timeout > 0
            else self._busy.acquire(blocking=False)
        )
        if not acquired:
            msg = (
                'A Sawyer operation is already running; use one command owner'
            )
            raise RuntimeError(msg)
        try:
            yield
        finally:
            self._busy.release()

    def _open_runtime(self):
        from .communication.transport import DirectTransport
        from .execution.native import MotionActions
        from .hardware.peripherals import SawyerDevices

        self._transport = DirectTransport(self.config)
        self._node = _ArmSession(self.config, self._transport)
        if self.config.enable_gripper:
            self._gripper = SawyerGripper(self._node, self.config)
        self._devices = SawyerDevices(self)
        self._actions = MotionActions(self._node, self.config)

    @property
    def devices(self):
        """Explicit robot, head, display, and camera operations."""
        self._require_connected()
        return self._devices

    @property
    def capabilities(self):
        """
        Versioned SDK coverage; hardware evidence is separate from
        readiness.
        """
        from importlib.resources import files

        report = json.loads(
            files('irobot.src.robots.sawyer')
            .joinpath('CAPABILITY_LEDGER.json')
            .read_text()
        )
        report['runtime'] = {
            'connected': self.is_connected,
            'allow_unvalidated_modes': self.config.allow_unvalidated_modes,
        }
        return report

    def check_version(self, compatible=('5.3.1',)):
        """
        Check the selected public-SDK baseline without changing the
        robot.
        """
        version = str(
            self.get_parameter(self.config.version_parameter)
        ).lstrip('v')
        if not any(
            version == prefix or version.startswith(prefix + '.')
            for prefix in compatible
        ):
            msg = f'Unqualified Intera version: {version}'
            raise RuntimeError(msg)
        return version

    def connect(self) -> bool:
        """
        Open a session without actuation, including while stopped/disabled.

        Healthy enabled sessions also validate arm/tip feedback. Every motion
        still requires fresh collision feedback and a ready controller.
        """
        with self._operation():
            if self._connected:
                self._node.wait_robot_state()
                return True
            try:
                self._open_runtime()
                deadline = time.monotonic() + self.config.connect_timeout
                self._node.wait_robot_state()
                state = self._node.state
                if (
                    state.enabled
                    and state.ready
                    and not state.stopped
                    and not state.error
                ):
                    self._node.wait_inputs(
                        False, timeout=max(0.001, deadline - time.monotonic())
                    )
                    self._node.wait_pose(
                        self.config.tip,
                        timeout=max(0.001, deadline - time.monotonic()),
                    )
                    if self._gripper is not None:
                        self._gripper.wait_for_state(
                            timeout=max(0.001, deadline - time.monotonic())
                        )
                self._connected = True
            except BaseException:
                self._close_runtime()
                raise
        return True

    def _require_connected(self):
        if not self._connected or self._node is None:
            msg = 'Sawyer is disconnected; call connect() first'
            raise RuntimeError(msg)

    def _refresh(self):
        self._require_connected()
        # Service callbacks even if a recently cached sample still looks fresh.
        self._node.spin_until(time.monotonic() + self.config.callback_timeout)
        self._node.wait_inputs(False)

    @property
    def is_connected(self) -> bool:
        """
        Whether this object has an open session; get_status() checks freshness.
        """
        return self._connected

    @property
    def joint_positions(self) -> tuple[float, ...]:
        with self._operation():
            self._refresh()
            return tuple(self._node.joints)

    @property
    def joint_velocities(self) -> tuple[float, ...]:
        with self._operation():
            self._refresh()
            return tuple(self._node.velocity)

    @property
    def joint_efforts(self):
        """Measured Nm, requiring a recent value for each arm joint."""
        with self._operation():
            self._refresh()
            deadline = time.monotonic() + self.config.connect_timeout
            feedback = self._node.feedback
            while any(
                name not in feedback.effort_at
                or time.monotonic() - feedback.effort_at[name]
                >= self.config.feedback_timeout
                for name in JOINT_NAMES
            ):
                if time.monotonic() >= deadline:
                    msg = 'Seven fresh measured joint efforts unavailable'
                    raise TimeoutError(msg)
                self._node.spin_once(self.config.callback_slice)
            return tuple(feedback.effort[name] for name in JOINT_NAMES)

    @property
    def joint_limits(self):
        with self._operation():
            self._refresh()
            return asdict(self._node.limits)

    def get_parameter(self, name):
        with self._operation():
            self._require_connected()
            return self._node.master('getParam', name)

    @property
    def robot_description(self):
        return self.get_parameter(self.config.robot_description_parameter)

    @property
    def joint_effort_limits(self):
        from .hardware.model import effort_limits

        model = effort_limits(self.robot_description)
        with self._operation():
            self._refresh()
            published = self._node.joint_effort_limits
            if published is None:
                msg = 'Published joint effort limits are unavailable'
                raise RuntimeError(msg)
            return tuple(
                min(a, b) for a, b in zip(model, published, strict=True)
            )

    @property
    def endpoint_state(self):
        return self.get_tip_state(self.config.tip)

    def get_tip_state(self, tip=None):
        """Read the complete named endpoint state in its declared frame."""
        from copy import deepcopy

        with self._operation():
            self._refresh()
            tip, _ = self._node.wait_pose(tip or self.config.tip)
            return deepcopy(self._node.pose_feedback.raw_tips[tip][0])

    @property
    def tip_names(self):
        with self._operation():
            self._refresh()
            now = time.monotonic()
            return tuple(
                name
                for name, (
                    _,
                    received,
                ) in self._node.pose_feedback.tips.items()
                if now - received <= self.config.feedback_timeout
            )

    def poll(self, timeout=0.02):
        """
        Dispatch direct feedback and registered callbacks without
        commands.
        """
        if not math.isfinite(timeout) or timeout < 0:
            msg = 'Polling timeout must be finite and nonnegative'
            raise ValueError(msg)
        with self._operation():
            self._require_connected()
            self._node.spin_once(timeout_sec=timeout)
            return self._node.status_snapshot()

    @property
    def endpoint_velocity(self):
        state = self.endpoint_state
        return {
            'linear': (
                state.twist.linear.x,
                state.twist.linear.y,
                state.twist.linear.z,
            ),
            'angular': (
                state.twist.angular.x,
                state.twist.angular.y,
                state.twist.angular.z,
            ),
            'frame': state.header.frame_id or self.config.frame,
        }

    @property
    def endpoint_effort(self):
        state = self.endpoint_state
        return {
            'force': (
                state.wrench.force.x,
                state.wrench.force.y,
                state.wrench.force.z,
            ),
            'torque': (
                state.wrench.torque.x,
                state.wrench.torque.y,
                state.wrench.torque.z,
            ),
            'frame': state.header.frame_id or self.config.frame,
        }

    @property
    def endpoint_pose(self):
        """CartesianPose: meters, quaternion (x,y,z,w), and reference frame."""
        with self._operation():
            self._refresh()
            return self._node.wait_pose(self.config.tip)[1]

    @contextmanager
    def _kinematics(self, tip):
        self._require_connected()
        deadline = time.monotonic() + self.config.connect_timeout
        while self._node.limits is None:
            if time.monotonic() >= deadline:
                msg = 'Published joint limits unavailable for kinematics'
                raise TimeoutError(msg)
            self._node.spin_once(timeout_sec=self.config.callback_slice)
        selected = tip or self.config.tip or 'right_hand'
        kinematics = InteraKinematics(self._node, selected, self.config.frame)
        try:
            kinematics.wait_ready()
            yield kinematics
        finally:
            kinematics.close()

    def forward_kinematics(self, configurations, *, tip=None):
        """Return poses for seven-joint configurations; reject invalid FK."""
        with self._operation(), self._kinematics(tip) as kinematics:
            configurations = [
                vector(values, len(JOINT_NAMES), 'FK configuration')
                for values in configurations
            ]
            for values in configurations:
                self._node.limits.check_position(values, 'FK configuration')
            return kinematics.forward(configurations)

    def configuration_collisions(self, configurations, *, tip=None):
        """Query the controller's FK collision flags without moving the arm."""
        with self._operation(), self._kinematics(tip) as kinematics:
            values = [
                vector(item, len(JOINT_NAMES), 'Collision configuration')
                for item in configurations
            ]
            for item in values:
                self._node.limits.check_position(
                    item, 'Collision configuration'
                )
            return tuple(
                collision
                for _, collision in kinematics.forward_results(values)
            )

    def inverse_kinematics(self, pose, *, seed=None, tip=None):
        """Return a seeded seven-joint solution without commanding motion."""
        if not isinstance(pose, CartesianPose):
            msg = 'pose must be CartesianPose'
            raise TypeError(msg)
        with self._operation(), self._kinematics(tip) as kinematics:
            if seed is None:
                self._node.wait_inputs(False)
            seed = (
                tuple(self._node.joints)
                if seed is None
                else vector(seed, len(JOINT_NAMES), 'IK seed')
            )
            self._node.limits.check_position(seed, 'IK seed')
            joints = kinematics.inverse(pose, seed)
            self._node.limits.check_position(joints, 'IK solution')
            return joints

    @property
    def gripper_state(self) -> dict:
        """
        Return the latest standard Intera gripper signals without commanding
        it.
        """
        with self._operation():
            self._require_connected()
            if self._gripper is None:
                msg = 'Gripper support is disabled in SawyerConfig'
                raise RuntimeError(msg)
            self._node.spin_until(
                time.monotonic() + self.config.callback_timeout
            )
            self._gripper.wait_for_state(self.config.connect_timeout)
            return {
                'device_status': self._gripper.message.device.status.tag,
                **{
                    name: signal.value
                    for name, signal in self._gripper.signals.items()
                },
            }

    def get_status(self) -> dict:
        with self._operation():
            if not self._connected:
                return {
                    'connected': False,
                    'ready': False,
                    'issues': ['Disconnected'],
                }
            self._node.spin_until(
                time.monotonic() + self.config.callback_timeout
            )
            return self._node.status_snapshot()

    def _motion_options(self, options=None, *, duration=None, execute=False):
        if options is not None and not isinstance(options, MotionOptions):
            msg = 'options must be MotionOptions or CartesianMotionOptions'
            raise ValueError(msg)
        if options is not None and not isinstance(
            options, CartesianMotionOptions
        ):
            options = replace(self.config.motion, **asdict(options))
        selected = options or self.config.motion
        return replace(
            selected,
            frame=self.config.frame,
            tip=self.config.tip
            if options is None or options.tip is None
            else options.tip,
            time=duration if duration is not None else selected.time,
            execute=execute,
        )

    def _close_runtime(self):
        try:
            if self._node is not None:
                self._node.stop()
        finally:
            try:
                if self._transport is not None:
                    self._transport.destroy_node()
            finally:
                self._node = self._gripper = self._transport = None
                self._devices = None
                self._actions = None
                self._interaction = None
                self._connected = False

    def disconnect(self):
        """Release this object's direct connection resources."""
        with self._operation():
            self._close_runtime()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_exc):
        self.disconnect()
