"""Optional typed ROS 2 actions/services for the direct Sawyer driver."""

# This component shares its owner's private job lifecycle.
# ruff: noqa: SLF001

from __future__ import annotations

import json
import threading
import time

from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from irobot_sawyer_interfaces.action import (
    CalibrationCommand,
    InteractionSession,
    JointStream,
    MotionCommand,
    MoveCartesian,
)
from irobot_sawyer_interfaces.msg import StreamCommand
from irobot_sawyer_interfaces.srv import (
    CheckJointCollision,
    ForwardKinematics,
    InverseKinematics,
    ReadParameter,
)
from rcl_interfaces.msg import ParameterType, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionServer, GoalResponse
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger

from .config import (
    JOINT_NAMES,
    CartesianMotionOptions,
    StreamOptions,
    positive,
)
from .core import wire
from .core.base import CartesianPose
from .core.control import MODES
from .core.lease import CommandLease
from .ros2_telemetry import to_ros, to_wire


def cartesian_pose(message):
    pose = message.pose
    return CartesianPose(
        (pose.position.x, pose.position.y, pose.position.z),
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        message.header.frame_id,
    )


def pose_message(pose):
    result = PoseStamped()
    result.header.frame_id = pose.frame
    result.pose.position.x, result.pose.position.y, result.pose.position.z = (
        pose.position
    )
    (
        result.pose.orientation.x,
        result.pose.orientation.y,
        result.pose.orientation.z,
        result.pose.orientation.w,
    ) = pose.orientation
    return result


class ExtendedInterfaces:
    def __init__(self, node):
        self.node, self.driver = node, node.driver
        self.actions, self.services = [], []
        self.feedback_lock = threading.Lock()
        self.feedback = None
        self.lease = None
        for kind, name, validate, operation in (
            (
                CalibrationCommand,
                'calibrate_arm',
                self.validate_calibration,
                self.calibration,
            ),
            (JointStream, 'joint_stream', self.validate_stream, self.stream),
            (
                MotionCommand,
                'native_motion',
                self.validate_native,
                self.native,
            ),
            (
                InteractionSession,
                'interaction',
                self.validate_interaction,
                self.interaction,
            ),
            (
                MoveCartesian,
                'move_cartesian',
                self.validate_cartesian,
                self.cartesian,
            ),
            (
                GripperCommand,
                'gripper_command',
                self.validate_gripper,
                self.gripper,
            ),
        ):
            self.actions.append(
                ActionServer(
                    node,
                    kind,
                    name,
                    callback_group=node.group,
                    goal_callback=self.goal_callback(name, validate),
                    cancel_callback=node._cancel,
                    execute_callback=self.execute_callback(kind, operation),
                )
            )
        for kind, name, callback in (
            (CheckJointCollision, 'check_joint_collision', self.collisions),
            (ForwardKinematics, 'forward_kinematics', self.forward),
            (InverseKinematics, 'inverse_kinematics', self.inverse),
            (ReadParameter, 'read_parameter', self.read_parameter),
        ):
            self.services.append(
                node.create_service(
                    kind, name, callback, callback_group=node.group
                )
            )
        self.command_sub = node.create_subscription(
            StreamCommand,
            'stream_command',
            self.receive_command,
            qos_profile_sensor_data,
        )
        self.display_sub = node.create_subscription(
            Image, 'display_image', self.display, 1
        )
        self.services.append(
            node.create_service(
                Trigger, 'inventory', self.inventory, callback_group=node.group
            )
        )
        self.services.append(
            node.create_service(
                Trigger,
                'capabilities',
                self.capabilities,
                callback_group=node.group,
            )
        )
        self.services.append(
            node.create_service(
                SetParameters,
                'digital_outputs/set_parameters',
                self.digital_outputs,
                callback_group=node.group,
            )
        )
        for name, operation in (
            ('stop_gripper', lambda: self.driver.stop_gripper(execute=True)),
            (
                'start_gripper',
                lambda: self.driver.start_gripper(
                    execute=True, cancel_event=node.cancel_event
                ),
            ),
            (
                'reboot_gripper',
                lambda: self.driver.reboot_gripper(
                    execute=True, cancel_event=node.cancel_event
                ),
            ),
        ):
            self.services.append(
                node.create_service(
                    Trigger,
                    name,
                    node._job_service(name, operation),
                    callback_group=node.group,
                )
            )
        for camera in ('head', 'hand'):
            self.services.append(
                node.create_service(
                    SetBool,
                    camera + '/streaming',
                    self.camera_callback(camera),
                    callback_group=node.group,
                )
            )
        config = self.driver.config
        for label, namespace in (
            ('gripper', config.gripper_namespace),
            ('head_camera', config.camera_namespace + '/head_camera'),
            ('hand_camera', config.camera_namespace + '/right_hand_camera'),
            ('robot_io', config.io_namespace + '/robot'),
        ):
            for category, endpoint in (
                ('signals', label + '/set_parameters'),
                ('ports', label + '/ports/set_parameters'),
            ):
                self.services.append(
                    node.create_service(
                        SetParameters,
                        endpoint,
                        self.io_callback(namespace, category=category),
                        callback_group=node.group,
                    )
                )

    def goal_callback(self, name, validate):
        def accept(request):
            try:
                validate(request)
            except (ValueError, TypeError) as exc:
                self.node.get_logger().warning(str(exc))
                return GoalResponse.REJECT
            return (
                GoalResponse.ACCEPT
                if self.node._reserve(name)
                else GoalResponse.REJECT
            )

        return accept

    def execute_callback(self, kind, operation):
        def execute(handle):
            result = kind.Result()
            state, detail = 'failed', ''
            try:
                with self.node.driver_lock:
                    if self.node.cancel_event.is_set():
                        msg = 'Action cancelled before execution'
                        raise RuntimeError(msg)  # noqa: TRY301
                    success = operation(handle, result)
                if handle.is_cancel_requested:
                    handle.canceled()
                    state = 'cancelled'
                elif success:
                    handle.succeed()
                    state = 'succeeded'
                else:
                    handle.abort()
                    detail = getattr(
                        result,
                        'detail',
                        getattr(
                            result,
                            'error_id',
                            'Controller rejected operation',
                        ),
                    )
            except (RuntimeError, ValueError, TypeError, OSError) as exc:
                if handle.is_cancel_requested:
                    handle.canceled()
                    state = 'cancelled'
                else:
                    handle.abort()
                    state = 'failed'
                if hasattr(result, 'detail'):
                    result.detail = str(exc)
                elif hasattr(result, 'error_id'):
                    result.error_id = str(exc)
                elif hasattr(result, 'status_id'):
                    result.status_id = str(exc)
                detail = str(exc)
            finally:
                self.lease = None
                with self.feedback_lock:
                    self.feedback = None
                self.node._finish(state, detail)
            return result

        return execute

    def cache_feedback(self, handle, message):
        with self.feedback_lock:
            self.feedback = (handle, message)

    def publish(self):
        with self.feedback_lock:
            feedback, self.feedback = self.feedback, None
        if feedback is not None:
            handle, message = feedback
            if handle.is_active:
                handle.publish_feedback(message)

    def validate_stream(self, request):
        modes = {value[0]: name for name, value in MODES.items()}
        if request.mode not in modes:
            msg = 'Unknown joint control mode'
            raise ValueError(msg)
        self.stream_options(request)
        positive('duration', request.duration)
        positive('lease_timeout', request.lease_timeout)
        if request.lease_timeout > self.driver.config.command_timeout / 2:
            msg = 'Lease timeout must not exceed half the hardware watchdog'
            raise ValueError(msg)

    @staticmethod
    def stream_options(request):
        return StreamOptions(
            **{
                name: getattr(request, name)
                for name in StreamOptions.__dataclass_fields__
            }
        )

    def receive_command(self, message):
        lease = self.lease
        if lease is not None:
            lease.accept(message)

    def stream(self, handle, result):
        request = handle.request
        mode = next(
            name for name, value in MODES.items() if value[0] == request.mode
        )
        identifier = bytes(handle.goal_id.uuid).hex()
        lease = self.lease = CommandLease(
            identifier,
            mode,
            request.lease_timeout,
            lambda: self.node.get_clock().now().nanoseconds,
        )
        deadline = time.monotonic() + self.driver.config.connect_timeout
        while lease.command is None:
            if lease.error is not None:
                raise RuntimeError(lease.error)
            if time.monotonic() >= deadline or self.node.cancel_event.is_set():
                msg = 'Initial stream command absent or session cancelled'
                raise TimeoutError(msg)
            time.sleep(0.005)

        def step(elapsed, joints, velocities):
            self.cache_feedback(
                handle,
                JointStream.Feedback(
                    session_id=identifier,
                    elapsed=elapsed,
                    position=list(joints),
                    velocity=list(velocities),
                ),
            )

        report = self.driver.stream_joint_commands(
            mode,
            lease.get,
            request.duration,
            options=self.stream_options(request),
            execute=True,
            cancel_event=self.node.cancel_event,
            on_step=step,
        )
        result.success = True
        result.samples = report['samples']
        result.max_command_gap = report['max_command_gap_s']
        return True

    @staticmethod
    def validate_native(request):
        if request.command not in ('start', 'generate'):
            msg = 'Use start/generate; cancel the accepted action to stop it'
            raise ValueError(msg)

    def native(self, handle, result):
        def feedback(message):
            self.cache_feedback(
                handle, MotionCommand.Feedback(status=to_ros(message.status))
            )

        report = self.driver.execute_native_trajectory(
            to_wire(handle.request.trajectory, wire.Trajectory),
            execute=True,
            plan_only=handle.request.command == 'generate',
            cancel_event=self.node.cancel_event,
            on_feedback=feedback,
        )
        message = to_ros(
            wire.from_dict(wire.MotionCommandResult, report['result'])
        )
        for name in message.get_fields_and_field_types():
            setattr(result, name, getattr(message, name))
        return report['succeeded']

    @staticmethod
    def validate_calibration(request):
        if request.command != 'start':
            msg = (
                'Use start; cancel the accepted calibration action to stop it'
            )
            raise ValueError(msg)

    def calibration(self, handle, result):
        def feedback(message):
            self.cache_feedback(
                handle,
                CalibrationCommand.Feedback(
                    current_state=message.current_state,
                    number_of_poses=message.number_of_poses,
                    current_pose_number=message.current_pose_number,
                ),
            )

        report = self.driver.calibrate_arm(
            execute=True,
            cancel_event=self.node.cancel_event,
            on_feedback=feedback,
        )
        result.result = report['result']['result']
        result.status_id = report['result']['status_id']
        return report['succeeded']

    @staticmethod
    def validate_interaction(request):
        from .core.interaction import InteractionLimits, validate_command

        positive('duration', request.duration)
        validate_command(
            to_wire(request.command, wire.InteractionControlCommand),
            InteractionLimits(),
        )

    def interaction(self, handle, result):
        def step(elapsed, joints, velocities):
            self.cache_feedback(
                handle,
                InteractionSession.Feedback(
                    elapsed=elapsed,
                    position=list(joints),
                    velocity=list(velocities),
                ),
            )

        report = self.driver.run_interaction(
            to_wire(handle.request.command, wire.InteractionControlCommand),
            handle.request.duration,
            execute=True,
            cancel_event=self.node.cancel_event,
            on_step=step,
        )
        result.activation_confirmed = report['activation_confirmed']
        result.deactivation_confirmed = report['deactivation_confirmed']
        result.success = (
            result.activation_confirmed and result.deactivation_confirmed
        )
        return result.success

    @staticmethod
    def validate_cartesian(request):
        cartesian_pose(request.target)
        positive('speed', request.speed)
        positive('max_acceleration', request.max_acceleration)
        if request.duration < 0 or request.speed > 1:
            msg = 'Invalid Cartesian duration/speed'
            raise ValueError(msg)

    def cartesian(self, handle, result):
        request = handle.request
        self.driver.move_to_pose(
            cartesian_pose(request.target),
            duration=request.duration or None,
            execute=True,
            options=CartesianMotionOptions(
                speed=request.speed, max_accel=request.max_acceleration
            ),
        )
        result.success = True
        return True

    def validate_gripper(self, request):
        if (
            not 0
            <= request.command.position
            <= self.driver.config.gripper_open_position
            or request.command.max_effort != 0
        ):
            msg = (
                'Use an in-range gripper position and max_effort=0 (fixed '
                'hardware force)'
            )
            raise ValueError(msg)

    def gripper(self, handle, result):
        self.driver.move_gripper(
            handle.request.command.position,
            execute=True,
            cancel_event=self.node.cancel_event,
        )
        state = self.driver.gripper_state
        result.position = float(state['position_response_m'])
        force = float(state['force_response_n'])
        result.effort = force if force >= 0 else float('nan')
        result.stalled = bool(state['is_gripping'])
        result.reached_goal = (
            abs(result.position - handle.request.command.position)
            <= self.driver.config.gripper_position_tolerance
        )
        return result.reached_goal or result.stalled

    def query(self, response, operation, *, error_field='detail'):
        if self.node.closing or self.node.job_lock.locked():
            setattr(response, error_field, 'Driver is busy or shutting down')
            return response
        if not self.node.driver_lock.acquire(
            timeout=self.driver.config.feedback_timeout
        ):
            setattr(response, error_field, 'Driver is busy')
            return response
        try:
            operation()
            response.success = True
        except (RuntimeError, ValueError, TypeError, OSError) as exc:
            setattr(response, error_field, str(exc))
        finally:
            self.node.driver_lock.release()
        return response

    def forward(self, request, response):
        def operation():
            if not request.positions or len(request.positions) % len(
                JOINT_NAMES
            ):
                msg = 'Provide flattened batches of seven joint positions'
                raise ValueError(msg)
            positions = [
                request.positions[index : index + 7]
                for index in range(0, len(request.positions), 7)
            ]
            response.poses = [
                pose_message(pose)
                for pose in self.driver.forward_kinematics(
                    positions, tip=request.tip or None
                )
            ]

        return self.query(response, operation)

    def inverse(self, request, response):
        def operation():
            response.position = list(
                self.driver.inverse_kinematics(
                    cartesian_pose(request.pose),
                    seed=request.seed,
                    tip=request.tip or None,
                )
            )

        return self.query(response, operation)

    def collisions(self, request, response):
        def operation():
            if not request.positions or len(request.positions) % len(
                JOINT_NAMES
            ):
                msg = 'Provide flattened batches of seven joint positions'
                raise ValueError(msg)
            positions = [
                request.positions[index : index + 7]
                for index in range(0, len(request.positions), 7)
            ]
            response.in_collision = list(
                self.driver.configuration_collisions(
                    positions, tip=request.tip or None
                )
            )

        return self.query(response, operation)

    def read_parameter(self, request, response):
        def operation():
            response.json_value = json.dumps(
                self.driver.get_parameter(request.name), allow_nan=False
            )

        return self.query(response, operation)

    def inventory(self, _request, response):
        def operation():
            response.message = json.dumps(self.driver.devices.inventory())

        return self.query(response, operation, error_field='message')

    def capabilities(self, _request, response):
        response.success = True
        response.message = json.dumps(self.driver.capabilities)
        return response

    def digital_outputs(self, request, response):
        if not self.node._reserve('digital_outputs'):
            response.results = [
                SetParametersResult(successful=False, reason='Driver busy')
                for _ in request.parameters
            ]
            return response
        failed = False
        try:
            with self.node.driver_lock:
                for parameter in request.parameters:
                    item = SetParametersResult()
                    try:
                        report = self._set_digital_parameter(parameter)
                        item.successful = True
                        item.reason = json.dumps(report)
                    except (RuntimeError, ValueError, OSError) as exc:
                        failed = True
                        item.reason = str(exc)
                    response.results.append(item)
        finally:
            self.node._finish('failed' if failed else 'succeeded')
        return response

    def _set_digital_parameter(self, parameter):
        if parameter.value.type != ParameterType.PARAMETER_BOOL:
            msg = 'Digital outputs require boolean parameters'
            raise ValueError(msg)
        return self.driver.devices.set_digital_output(
            parameter.name, parameter.value.bool_value, execute=True
        )

    def camera_callback(self, camera):
        def callback(request, response):
            return self.node._job_service(
                camera + '_streaming',
                lambda: self.driver.devices.set_camera_streaming(
                    camera, request.data, execute=True
                ),
            )(request, response)

        return callback

    def display(self, message):
        if not self.node._reserve('display_image'):
            return
        self.node.worker = threading.Thread(
            target=self.node._run_job,
            args=(
                lambda: self.driver.devices.display_image(
                    to_wire(message, wire.Image), execute=True
                ),
            ),
            daemon=True,
        )
        self.node.worker.start()

    def io_callback(self, namespace, *, category='signals'):
        def callback(request, response):
            if not self.node._reserve('configure_io'):
                response.results = [
                    SetParametersResult(successful=False, reason='Driver busy')
                    for _ in request.parameters
                ]
                return response
            failed = False
            try:
                with self.node.driver_lock:
                    metadata = self.driver.devices.read_io(namespace)[category]
                    for parameter in request.parameters:
                        item = SetParametersResult()
                        try:
                            value = self.parameter_value(parameter.value)
                            if metadata[parameter.name]['format']['type'] in (
                                'object',
                                'array',
                            ):
                                value = json.loads(value)
                            report = self.driver.devices.write_io(
                                namespace,
                                {parameter.name: value},
                                category=category,
                                execute=True,
                                cancel_event=self.node.cancel_event,
                            )
                            item.successful = report['acknowledged']
                            item.reason = json.dumps(report['measured'])
                        except (
                            KeyError,
                            ValueError,
                            TypeError,
                            RuntimeError,
                            OSError,
                        ) as exc:
                            item.reason = str(exc)
                            failed = True
                        response.results.append(item)
            except (RuntimeError, ValueError, OSError) as exc:
                response.results = [
                    SetParametersResult(successful=False, reason=str(exc))
                    for _ in request.parameters
                ]
                failed = True
            finally:
                self.node._finish('failed' if failed else 'succeeded')
            return response

        return callback

    @staticmethod
    def parameter_value(value):
        names = {
            ParameterType.PARAMETER_BOOL: 'bool_value',
            ParameterType.PARAMETER_INTEGER: 'integer_value',
            ParameterType.PARAMETER_DOUBLE: 'double_value',
            ParameterType.PARAMETER_STRING: 'string_value',
            ParameterType.PARAMETER_BOOL_ARRAY: 'bool_array_value',
            ParameterType.PARAMETER_INTEGER_ARRAY: 'integer_array_value',
            ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value',
            ParameterType.PARAMETER_STRING_ARRAY: 'string_array_value',
        }
        if value.type not in names:
            msg = 'Provide a typed scalar or array parameter'
            raise ValueError(msg)
        result = getattr(value, names[value.type])
        return (
            list(result)
            if names[value.type].endswith('array_value')
            else result
        )

    def close(self):
        for action in self.actions:
            action.destroy()
