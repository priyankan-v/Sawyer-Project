"""Public Sawyer motion API and planning/execution coordination."""

from __future__ import annotations

import os
import time
import xmlrpc.client
from dataclasses import replace

from ..config import JOINT_NAMES, CartesianMotionOptions
from .base import SawyerBase
from .communication import wire
from .execution.pid import JointPIDController
from .execution.trajectory import execute_cartesian_plan, execute_joint_plan
from .hardware.kinematics import InteraKinematics
from .planning.cartesian import (
    CartesianLine,
    CartesianPlanner,
    slerp,
)
from .planning.joint import (
    JointPath,
    JointSequence,
    QuinticTrajectory,
    extrema,
)
from .planning.timing import TimedJointPath
from .types import (
    CartesianMoveResult,
    CartesianPlan,
    CartesianPose,
    JointMoveResult,
    MotionResult,
    TrajectorySample,
    ordered_values,
    vector,
)

__all__ = [
    'CartesianLine',
    'CartesianMoveResult',
    'CartesianPlan',
    'CartesianPlanner',
    'CartesianPose',
    'JointMoveResult',
    'JointPIDController',
    'JointPath',
    'JointSequence',
    'MotionResult',
    'ParameterTransport',
    'QuinticTrajectory',
    'SawyerController',
    'TimedJointPath',
    'TrajectorySample',
    'extrema',
    'slerp',
]


class ParameterTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection


class SawyerController(SawyerBase):
    """Synchronous direct arm/gripper control; movement previews by default."""

    def run_interaction(  # noqa: PLR0913
        self,
        message,
        duration,
        *,
        execute=False,
        limits=None,
        stream_options=None,
        cancel_event=None,
        on_step=None,
    ):
        """Preview or execute a bounded interaction and confirm its exit."""
        from copy import deepcopy
        from dataclasses import asdict

        from ..config import InteractionLimits, StreamOptions, positive
        from .execution.interaction import InteractionSession
        from .execution.streaming import JointStream
        from .planning.native import validate_command

        positive('duration', duration)
        if type(execute) is not bool:
            msg = 'execute must be a boolean'
            raise TypeError(msg)
        limits = limits or InteractionLimits()
        message = deepcopy(message)
        validate_command(message, limits)
        with self._operation():
            self._refresh()
            node = self._node
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
            if exploratory and not self.config.allow_unvalidated_modes:
                msg = (
                    'Force/compliant control is not hardware-qualified; '
                    'explicit allow_unvalidated_modes is required'
                )
                raise RuntimeError(msg)
            node.cancel_requested.clear()
            if self._interaction is None:
                self._interaction = InteractionSession(
                    node, limits, stream_options or StreamOptions()
                )
            session = self._interaction
            session.limits = limits
            session.guard = JointStream(
                node, 'velocity', stream_options or StreamOptions()
            )
            return session.run(
                message, duration, cancel_event=cancel_event, on_step=on_step
            )

    def stream_joint_commands(  # noqa: PLR0913
        self,
        mode,
        source,
        duration,
        *,
        options=None,
        execute=False,
        cancel_event=None,
        on_step=None,
    ):
        """Bounded position/velocity/torque/trajectory session; SI units."""
        from .execution.streaming import stream_joint_commands

        with self._operation():
            self._refresh()
            return stream_joint_commands(
                self._node,
                mode,
                source,
                duration,
                options=options,
                execute=execute,
                cancel_event=cancel_event,
                on_step=on_step,
            )

    def move_native_joint_path(  # noqa: PLR0913
        self,
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
        """Use Intera's planner/action server through the direct transport."""
        from .execution.native import run_joint_path

        with self._operation():
            self._refresh()
            return run_joint_path(
                self._node,
                self._actions,
                positions,
                execute=execute,
                plan_only=plan_only,
                speed=speed,
                max_acceleration=max_acceleration,
                timeout=timeout,
                cancel_event=cancel_event,
                on_feedback=on_feedback,
            )

    def execute_native_trajectory(  # noqa: PLR0913
        self,
        trajectory,
        *,
        execute=False,
        plan_only=False,
        timeout=60.0,
        cancel_event=None,
        on_feedback=None,
    ):
        """
        Validate a typed joint/Cartesian Intera path, then preview or
        run.
        """
        from .execution.native import run_trajectory

        with self._operation():
            self._refresh()
            return run_trajectory(
                self._node,
                self._actions,
                trajectory,
                execute=execute,
                plan_only=plan_only,
                timeout=timeout,
                cancel_event=cancel_event,
                on_feedback=on_feedback,
            )

    def calibrate_arm(
        self,
        *,
        execute=False,
        timeout=120.0,
        cancel_event=None,
        on_feedback=None,
    ):
        """
        Explicit native homing; the controller requires the gripper
        removed.
        """
        from ..config import positive
        from .communication import wire

        positive('timeout', timeout)
        if type(execute) is not bool:
            msg = 'execute must be a boolean'
            raise TypeError(msg)
        with self._operation():
            self._refresh()
            attached = any(
                topic == self.config.gripper_namespace + '/state'
                for topic, _ in self._node.master('getTopicTypes')
            )
            if not execute:
                return {
                    'executed': False,
                    'available': not attached,
                    'precondition': (
                        'Remove/configure out the gripper before arm '
                        'calibration'
                    ),
                }
            if attached:
                msg = (
                    'Arm calibration requires the gripper '
                    'removed/configured out'
                )
                raise RuntimeError(msg)
            self._node.cancel_requested.clear()
            self._node.check_ready()
            return self._actions.run(
                wire.CalibrationCommandGoal(command='start'),
                name='calibration',
                timeout=timeout,
                cancel_event=cancel_event,
                on_feedback=on_feedback,
            )

    def _neutral_pose(self):
        if self.config.neutral_joints is not None:
            return tuple(self.config.neutral_joints)
        uri = self.config.ros_master_uri or os.environ.get('ROS_MASTER_URI')
        if not uri:
            msg = (
                'Set ROS_MASTER_URI or SawyerConfig.neutral_joints for '
                'neutral motion'
            )
            raise RuntimeError(msg)
        transport = ParameterTransport(self.config.parameter_timeout)
        with xmlrpc.client.ServerProxy(uri, transport=transport) as master:
            code, message, named = master.getParam(
                '/irobot_sawyer', self.config.neutral_parameter
            )
        if code != 1:
            msg = f'Cannot read {self.config.neutral_parameter}: {message}'
            raise RuntimeError(msg)
        if len(named['joint_names']) != len(JOINT_NAMES):
            msg = (
                'Neutral pose must contain exactly the seven right-arm joints'
            )
            raise ValueError(msg)
        return ordered_values(
            named['joint_names'], named['poses']['neutral'], 'Neutral pose'
        )

    def _move_joints(
        self, target, options, *, cancel_event=None, on_step=None
    ):
        node = self._node
        self._refresh()
        node.wait_inputs(options.execute)
        start = tuple(node.joints)
        node.limits.check_position(start, 'Current position')
        node.limits.check_position(target, 'Target position')
        velocity = tuple(options.speed * v for v in node.limits.velocity)
        acceleration = tuple(
            min(a, options.max_accel) if options.max_accel is not None else a
            for a in node.limits.acceleration
        )
        plan = QuinticTrajectory(
            start,
            target,
            velocity,
            duration=options.time,
            max_acceleration=acceleration,
        )
        return self._run_joint_plan(
            plan,
            options,
            velocity,
            acceleration,
            cancel_event=cancel_event,
            on_step=on_step,
        )

    def _run_joint_plan(  # noqa: PLR0913
        self,
        plan,
        options,
        velocity,
        acceleration,
        *,
        cancel_event=None,
        on_step=None,
    ):
        return execute_joint_plan(
            self._node,
            plan,
            options,
            velocity,
            acceleration,
            cancel_event=cancel_event,
            on_step=on_step,
        )

    def move_to_joints(  # noqa: PLR0913
        self,
        joints,
        *,
        duration=None,
        execute=False,
        options=None,
        cancel_event=None,
        on_step=None,
    ):
        """Preview/execute seven right_j0..right_j6 angles in radians."""
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            return self._move_joints(
                vector(joints, len(JOINT_NAMES), 'Target joints'),
                self._motion_options(
                    options, duration=duration, execute=execute
                ),
                cancel_event=cancel_event,
                on_step=on_step,
            )

    def move_to_neutral(
        self,
        *,
        duration=None,
        execute=False,
        options=None,
        cancel_event=None,
        on_step=None,
    ):
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            return self._move_joints(
                self._neutral_pose(),
                self._motion_options(
                    options, duration=duration, execute=execute
                ),
                cancel_event=cancel_event,
                on_step=on_step,
            )

    def move_trajectory(  # noqa: PLR0913
        self,
        positions,
        times,
        *,
        execute=False,
        options=None,
        cancel_event=None,
        on_step=None,
    ):
        """
        Preview/execute waypoints; each segment ends at rest.

        Times are seconds from the start and must strictly increase above
        zero. on_step must be fast and nonblocking: it runs on the control
        thread. An external cancel_event is never cleared by this driver.
        """
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            motion = self._motion_options(options, execute=execute)
            if motion.time is not None:
                msg = 'Use waypoint times instead of options.time'
                raise ValueError(msg)
            self._refresh()
            self._node.wait_inputs(execute)
            limits = self._node.limits
            velocity = tuple(motion.speed * v for v in limits.velocity)
            acceleration = tuple(
                min(a, motion.max_accel) if motion.max_accel else a
                for a in limits.acceleration
            )
            plan = JointSequence(
                self._node.joints, positions, times, velocity, acceleration
            )
            limits.check_position(plan.start, 'Start joints')
            for segment in plan.segments:
                limits.check_position(segment.end, 'Waypoint')
            return self._run_joint_plan(
                plan,
                motion,
                velocity,
                acceleration,
                cancel_event=cancel_event,
                on_step=on_step,
            )

    def move_relative(  # noqa: PLR0913
        self,
        dx_cm=0.0,
        dy_cm=0.0,
        dz_cm=0.0,
        *,
        duration=None,
        execute=False,
        options=None,
    ):
        """Base-frame XYZ offsets in centimeters, with fixed orientation."""
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            self._refresh()
            motion = self._motion_options(
                options, duration=duration, execute=execute
            )
            return self._move_cartesian(
                dx_cm, dy_cm, dz_cm, duration, execute=execute, options=motion
            )

    def move_to_pose(
        self, pose, *, duration=None, execute=False, options=None
    ):
        """Move in a straight Cartesian line with shortest quaternion SLERP."""
        if not isinstance(pose, CartesianPose):
            msg = 'Use CartesianPose with metres and an xyzw quaternion'
            raise TypeError(msg)
        with self._operation():
            self._require_connected()
            self._node.cancel_requested.clear()
            self._refresh()
            return self._move_cartesian(
                duration=duration,
                execute=execute,
                target_pose=pose,
                options=self._motion_options(
                    options, duration=duration, execute=execute
                ),
            )

    def _gripper_action(self, action, execute, cancel_event=None):
        with self._operation():
            self._require_connected()
            if self._gripper is None:
                msg = 'Gripper support is disabled in SawyerConfig'
                raise RuntimeError(msg)
            self._node.spin_until(time.monotonic() + 0.02)
            if cancel_event is None:
                return self._gripper.perform(action, execute=execute)
            return self._gripper.perform(
                action, execute=execute, cancel_event=cancel_event
            )

    def open_gripper(self, *, execute=False, cancel_event=None):
        return self._gripper_action('open', execute, cancel_event)

    def move_gripper(self, position, *, execute=False, cancel_event=None):
        """
        Set finger separation in metres, with acknowledgement and
        settling.
        """
        with self._operation():
            self._require_connected()
            if self._gripper is None:
                msg = 'Gripper support is disabled in SawyerConfig'
                raise RuntimeError(msg)
            self._node.spin_until(
                time.monotonic() + self.config.callback_timeout
            )
            return self._gripper.perform(
                'move',
                position=position,
                execute=execute,
                cancel_event=cancel_event,
            )

    def close_gripper(self, *, execute=False, cancel_event=None):
        return self._gripper_action('close', execute, cancel_event)

    def calibrate_gripper(self, *, execute=False, cancel_event=None):
        return self._gripper_action('calibrate', execute, cancel_event)

    def stop_gripper(self, *, execute=False):
        return self._gripper_action('stop', execute)

    def start_gripper(self, *, execute=False, cancel_event=None):
        """
        Explicitly resume the gripper's currently reported position
        target.
        """
        return self._gripper_action('start', execute, cancel_event)

    def reboot_gripper(self, *, execute=False, cancel_event=None):
        """Power-cycle an idle gripper; calibration must then be explicit."""
        return self._gripper_action('reboot', execute, cancel_event)

    def stop(self):
        """
        Request cancellation; an active arm loop performs its normal stop.
        """
        if self._node is not None:
            self._node.cancel_requested.set()

    def _move_cartesian(  # noqa: PLR0913
        self,
        dx_cm: float = 0.0,
        dy_cm: float = 0.0,
        dz_cm: float = 0.0,
        duration: float | None = None,
        *,
        execute: bool = False,
        options: CartesianMotionOptions | None = None,
        target_pose: CartesianPose | None = None,
    ) -> CartesianMoveResult:
        """
        Preview/execute a relative XYZ translation in the pose reference frame.

        Offsets are centimeters; duration is seconds or None for
        minimum feasible
        uniform time scaling. Orientation is held. Connect the driver first;
        this method services direct-transport callbacks internally. No
        execution occurs without
        execute=True. The returned result contains the reusable plan
        and timing.
        """
        options = options or CartesianMotionOptions()
        duration = options.time if duration is None else duration
        options = replace(options, time=duration, execute=execute)
        if options.frame.lstrip('/') != self._node.pose_feedback.frame.lstrip(
            '/'
        ):
            msg = (
                'Construct CartesianMotionNode with the same frame as '
                'options.frame'
            )
            raise ValueError(msg)
        self._node.wait_inputs(execute)
        tip, start_pose = self._node.wait_pose(options.tip)
        start_joints = tuple(self._node.joints)
        self._node.planning_health(start_joints, start_pose, tip)
        velocity = tuple(
            options.speed * value for value in self._node.limits.velocity
        )
        acceleration = tuple(
            min(value, options.max_accel)
            if options.max_accel is not None
            else value
            for value in self._node.limits.acceleration
        )
        kinematics = InteraKinematics(
            self._node,
            tip,
            start_pose.frame,
            options.ik_timeout,
            options.planning_timeout,
            health_check=lambda: self._node.planning_health(
                start_joints, start_pose, tip
            ),
        )
        try:
            kinematics.wait_ready()
            planner = CartesianPlanner(
                kinematics.inverse,
                kinematics.forward,
                self._node.limits.lower,
                self._node.limits.upper,
                sample_step=options.sample_step,
                path_tolerance=options.path_tolerance,
                orientation_tolerance=options.path_orientation_tolerance,
                max_joint_step=options.max_joint_step,
                max_samples=options.max_samples,
                joint_names=JOINT_NAMES,
                angular_step=options.angular_step,
            )
            target = (
                target_pose
                or CartesianLine(start_pose, (dx_cm, dy_cm, dz_cm)).end
            )
            plan = planner.plan_pose(
                start_pose,
                target,
                start_joints,
                velocity,
                acceleration,
                duration,
            )
        finally:
            kinematics.close()
        if not execute:
            return CartesianMoveResult(plan, None)
        return execute_cartesian_plan(self._node, plan, options, tip)
