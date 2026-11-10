"""Sawyer bridge connection, feedback, hardware IO and lifecycle."""

# Named trajectory bounds and retry/stop scopes stay explicit for review.
from __future__ import annotations

import json
import math
import threading
import time
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace

from irobot.src.robots.sawyer.config import (
    JOINT_NAMES,
    QUATERNION_MIN_NORM,
    QUATERNION_NORM_TOLERANCE,
    CartesianMotionOptions,
    MotionOptions,
    SawyerConfig,
)


def vector(values, size, name, *, positive=False, nonnegative=False):
    result = (float(values),) * size if isinstance(values, int | float) else tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(value) for value in result):
        msg = f'{name} must contain {size} finite values'
        raise ValueError(msg)
    if positive and any(value <= 0 for value in result):
        msg = f'{name} must be positive'
        raise ValueError(msg)
    if nonnegative and any(value < 0 for value in result):
        msg = f'{name} must be nonnegative'
        raise ValueError(msg)
    return result


@dataclass(frozen=True)
class CartesianPose:
    position: tuple
    orientation: tuple  # Quaternion x, y, z, w.
    frame: str = 'base'

    def __post_init__(self):
        object.__setattr__(self, 'position', vector(self.position, 3, 'Cartesian position'))
        quaternion = vector(self.orientation, 4, 'orientation quaternion')
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not math.isfinite(norm) or norm < QUATERNION_MIN_NORM or abs(norm - 1) > QUATERNION_NORM_TOLERANCE:
            msg = 'Orientation must be a valid approximately unit quaternion'
            raise ValueError(msg)
        object.__setattr__(self, 'orientation', tuple(value / norm for value in quaternion))
        frame = self.frame.lstrip('/')
        if not frame:
            msg = 'A Cartesian reference frame is required'
            raise ValueError(msg)
        object.__setattr__(self, 'frame', frame)


def pose_error(actual, desired):
    if actual.frame != desired.frame:
        msg = f'Pose frame mismatch: {actual.frame} != {desired.frame}'
        raise ValueError(msg)
    dot = abs(sum(a * b for a, b in zip(actual.orientation, desired.orientation, strict=False)))
    return math.dist(actual.position, desired.position), 2 * math.acos(min(1.0, dot))


class JointFeedback:
    """
    Cache position and velocity independently, with per-joint receipt times.

    A JointState message may contain any subset of joints, and either numeric
    array may be empty. Unrelated joints never refresh or invalidate required
    joints. Missing/invalid fields never refresh their last valid receipt time.
    """

    def __init__(self, names):
        self.names = tuple(names)
        if not self.names or len(set(self.names)) != len(self.names):
            msg = 'Required joint names must be nonempty and unique'
            raise ValueError(msg)
        self.position = {}
        self.velocity = {}
        self.position_at = {}
        self.velocity_at = {}

    def update(self, names, positions, velocities, now):
        relevant = set(self.names).intersection(names)
        if not relevant:
            return False
        if len(set(names)) != len(names):
            msg = 'Duplicate names in arm JointState message'
            raise ValueError(msg)
        if not math.isfinite(now):
            msg = 'Joint feedback receipt time must be finite'
            raise ValueError(msg)
        updates = []
        for label, values, cache, times in (
            ('position', positions, self.position, self.position_at),
            ('velocity', velocities, self.velocity, self.velocity_at),
        ):
            if len(values) == 0:
                continue
            if len(values) != len(names):
                msg = f'Arm JointState {label} length {len(values)} != names length {len(names)}'
                raise ValueError(msg)
            selected = {name: float(value) for name, value in zip(names, values, strict=False) if name in relevant}
            if not all(math.isfinite(value) for value in selected.values()):
                msg = f'Non-finite arm JointState {label}'
                raise ValueError(msg)
            updates.append((selected, cache, times))
        # Validate the whole relevant message before committing either field.
        for selected, cache, times in updates:
            cache.update(selected)
            times.update({name: now for name in selected})
        return bool(updates)

    def snapshot(self):
        if any(name not in self.position or name not in self.velocity for name in self.names):
            return None, None, 0.0
        return (
            tuple(self.position[name] for name in self.names),
            tuple(self.velocity[name] for name in self.names),
            min(*self.position_at.values(), *self.velocity_at.values()),
        )

    def fresh_positions(self, now, timeout):
        """Position hold needs fresh positions even if velocity feedback expired."""
        if any(name not in self.position_at or now - self.position_at[name] >= timeout for name in self.names):
            return None
        return tuple(self.position[name] for name in self.names)

    def issues(self, now, timeout):
        issues = []
        for label, times in (('position', self.position_at), ('velocity', self.velocity_at)):
            missing = [name for name in self.names if name not in times]
            stale = [
                f'{name} ({now - times[name]:.3f}s old)'
                for name in self.names
                if name in times and now - times[name] >= timeout
            ]
            if missing:
                issues.append(f"missing {label}: {', '.join(missing)}")
            if stale:
                issues.append(f"stale {label} (limit {timeout:.3f}s): {', '.join(stale)}")
        return issues


@dataclass(frozen=True)
class MotionResult:
    reference_duration: float
    elapsed_duration: float
    final_joints: tuple


def ordered_values(names, values, label):
    if len(names) != len(values) or len(set(names)) != len(names):
        msg = f'Malformed {label} names/values'
        raise ValueError(msg)
    values = dict(zip(names, values, strict=False))
    if not all(name in values for name in JOINT_NAMES):
        msg = f'{label} is missing right arm joints'
        raise ValueError(msg)
    return vector([values[name] for name in JOINT_NAMES], 7, label)


@dataclass(frozen=True)
class JointBounds:
    lower: tuple
    upper: tuple
    velocity: tuple
    acceleration: tuple

    @classmethod
    def from_message(cls, message):
        bounds = cls(
            *(
                ordered_values(message.joint_names, getattr(message, field), field)
                for field in ('position_lower', 'position_upper', 'velocity', 'accel')
            )
        )
        if any(a >= b for a, b in zip(bounds.lower, bounds.upper, strict=False)):
            msg = 'Invalid joint position bounds'
            raise ValueError(msg)
        if any(value <= 0 for value in bounds.velocity + bounds.acceleration):
            msg = 'Joint velocity/acceleration limits must be positive'
            raise ValueError(msg)
        return bounds

    def check_position(self, positions, label, tolerance=0.0):
        positions = vector(positions, 7, label)
        for name, position, lower, upper in zip(JOINT_NAMES, positions, self.lower, self.upper, strict=False):
            if not lower - tolerance <= position <= upper + tolerance:
                msg = f'{label}: {name}={position:.6f} outside [{lower}, {upper}]'
                raise ValueError(msg)


def from_pose(pose, frame):
    return CartesianPose(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        frame,
    )


class EndpointFeedback:
    """Named endpoint cache; avoids guessing that an anonymous pose is a fingertip."""

    def __init__(self, fallback_frame, feedback_timeout, config=None):
        self.config = config or SawyerConfig()
        self.frame = fallback_frame
        self.feedback_timeout = feedback_timeout
        self.endpoint = None
        self.tips = {}
        self.last_error = None

    def on_endpoint(self, message):
        try:
            if not message.valid:
                msg = 'Endpoint pose has valid=false'
                raise ValueError(msg)  # noqa: TRY301
            self.endpoint = (from_pose(message.pose, message.header.frame_id or self.frame), time.monotonic())
        except ValueError as exc:
            self.endpoint = None
            self.last_error = str(exc)

    def on_tips(self, message):
        if len(message.names) != len(message.states) or len(set(message.names)) != len(message.names):
            self.tips.clear()
            self.last_error = 'Malformed named tip-state array'
            return
        now = time.monotonic()
        for name, state in zip(message.names, message.states, strict=False):
            try:
                if not state.valid:
                    msg = f'Tip {name} has valid=false'
                    raise ValueError(msg)  # noqa: TRY301
                frame = state.header.frame_id or message.header.frame_id or self.frame
                self.tips[name] = (from_pose(state.pose, frame), now)
            except ValueError as exc:
                self.tips.pop(name, None)
                self.last_error = str(exc)

    def read(self, tip):
        if tip not in self.tips:
            msg = (
                f'No valid pose for {tip} on /robot/limb/right/tip_states; '
                f'available tips: {sorted(self.tips)}. {self.last_error or ""}'
            )
            raise RuntimeError(msg)
        pose, received = self.tips[tip]
        if time.monotonic() - received >= self.feedback_timeout:
            msg = f'Stale Cartesian pose for {tip}: {time.monotonic() - received:.3f}s old'
            raise RuntimeError(msg)
        return pose

    def select(self, requested=None):
        if requested:
            return requested, self.read(requested)
        if self.endpoint is None or time.monotonic() - self.endpoint[1] >= self.feedback_timeout:
            msg = 'Missing/stale /robot/limb/right/endpoint_state'
            raise RuntimeError(msg)
        matches = []
        for name in self.tips:
            try:
                position, angle = pose_error(self.read(name), self.endpoint[0])
            except (RuntimeError, ValueError):
                continue
            if (
                position <= self.config.tip_match_position_tolerance
                and angle <= self.config.tip_match_orientation_tolerance
            ):
                matches.append(name)
        if len(matches) != 1:
            msg = (
                f'Cannot uniquely identify endpoint tip (matches={matches}); '
                'inspect tip_states and select --tip NAME'
            )
            raise RuntimeError(msg)
        return matches[0], self.read(matches[0])


class InteraKinematics:
    """Seeded IK and batched FK. No motion commands are sent by this adapter."""

    def __init__(self, node, tip, frame, request_timeout=None, planning_timeout=None, health_check=None):  # noqa: PLR0913
        self.node, self.tip, self.frame = node, tip, frame
        self.request_timeout = node.config.motion.ik_timeout if request_timeout is None else request_timeout
        self.deadline = time.monotonic() + (
            node.config.motion.planning_timeout if planning_timeout is None else planning_timeout
        )
        self.health_check = health_check
        from intera_core_msgs.srv import SolvePositionFK, SolvePositionIK

        self.ik = node.create_client(SolvePositionIK, node.config.ik_service)
        self.fk = node.create_client(SolvePositionFK, node.config.fk_service)

    def close(self):
        self.node.destroy_client(self.ik)
        self.node.destroy_client(self.fk)

    def wait_ready(self, timeout=None):
        deadline = min(
            time.monotonic() + (self.node.config.connect_timeout if timeout is None else timeout), self.deadline
        )
        for client, name in ((self.ik, self.node.config.ik_service), (self.fk, self.node.config.fk_service)):
            while not client.wait_for_service(timeout_sec=self.node.config.input_poll_timeout):
                if time.monotonic() >= deadline:
                    msg = f'Kinematics service unavailable: {name}; check bridge configuration'
                    raise RuntimeError(msg)
                self.node.spin_once(timeout_sec=self.node.config.callback_slice)

    def call(self, client, request, label):
        if time.monotonic() >= self.deadline:
            msg = 'Cartesian planning exceeded its planning timeout'
            raise TimeoutError(msg)
        future = client.call_async(request)
        deadline = min(self.deadline, time.monotonic() + self.request_timeout)
        try:
            while not future.done():
                if time.monotonic() >= deadline:
                    msg = f'{label} response timed out'
                    raise TimeoutError(msg)  # noqa: TRY301
                self.node.spin_once(timeout_sec=self.node.config.callback_slice)
                if self.health_check:
                    self.health_check()
            response = future.result()
            if response is None:
                msg = f'{label} returned no response'
                raise RuntimeError(msg)  # noqa: TRY301
            return response  # noqa: TRY300
        except BaseException:
            future.cancel()
            raise

    def inverse(self, pose, seed):
        from geometry_msgs.msg import PoseStamped
        from intera_core_msgs.srv import SolvePositionIK
        from sensor_msgs.msg import JointState

        if pose.frame != self.frame:
            msg = 'IK pose frame changed during planning'
            raise ValueError(msg)
        target = PoseStamped()
        target.header.frame_id = pose.frame
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = pose.position
        (target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w) = (
            pose.orientation
        )
        request = SolvePositionIK.Request()
        request.pose_stamp = [target]
        request.tip_names = [self.tip]
        request.seed_mode = SolvePositionIK.Request.SEED_USER
        request.seed_angles = [JointState(name=list(JOINT_NAMES), position=list(seed))]
        request.use_nullspace_goal = [False]
        response = self.call(self.ik, request, 'IK')
        if len(response.result_type) != 1:
            msg = 'IK returned a malformed response'
            raise ValueError(msg)
        code = int(response.result_type[0])
        if code <= 0:
            reasons = {-1: 'unreachable pose', -2: 'self collision', -3: 'unknown endpoint'}
            msg = f'IK rejected position {pose.position}: {reasons.get(code, code)}'
            raise ValueError(msg)
        if len(response.joints) != 1:
            msg = 'IK returned a malformed joint solution'
            raise ValueError(msg)
        return ordered_values(response.joints[0].name, response.joints[0].position, 'IK solution')

    def forward(self, configurations):
        from intera_core_msgs.srv import SolvePositionFK
        from sensor_msgs.msg import JointState

        poses = []
        for offset in range(0, len(configurations), 32):
            batch = configurations[offset : offset + 32]
            request = SolvePositionFK.Request()
            request.configuration = [JointState(name=list(JOINT_NAMES), position=list(values)) for values in batch]
            request.tip_names = [self.tip] * len(batch)
            response = self.call(self.fk, request, 'FK')
            if any(
                len(field) != len(batch) for field in (response.pose_stamp, response.is_valid, response.in_collision)
            ):
                msg = 'FK returned a malformed response'
                raise ValueError(msg)
            for stamped, valid, collision in zip(
                response.pose_stamp, response.is_valid, response.in_collision, strict=False
            ):
                if not valid or collision:
                    msg = f'FK rejected path configuration: valid={valid}, in_collision={collision}'
                    raise ValueError(msg)
                pose = from_pose(stamped.pose, stamped.header.frame_id or self.frame)
                if pose.frame != self.frame:
                    msg = f'FK frame {pose.frame} does not match requested frame {self.frame}'
                    raise ValueError(msg)
                poses.append(pose)
        return poses


@dataclass(frozen=True)
class Signal:
    value: object
    kind: str
    role: str
    status: str


def read_signals(message):
    result = {}
    for signal in message.signals:
        if signal.name in result:
            msg = f'Duplicate IO signal: {signal.name}'
            raise ValueError(msg)
        formatting = json.loads(signal.format)
        data = json.loads(signal.data)
        if not isinstance(data, list) or len(data) != 1:
            # This utility handles scalar gripper/cuff signals only.
            continue
        result[signal.name] = Signal(data[0], formatting['type'], formatting.get('role', ''), signal.status.tag)
    return result


def signal_value(signals, name, kind=None):
    if name not in signals:
        msg = f'Missing signal {name}; this utility requires the standard Intera electric gripper/cuff'
        raise ValueError(msg)
    signal = signals[name]
    if signal.status != 'ready':
        msg = f'Signal {name} is {signal.status}, not ready'
        raise ValueError(msg)
    if kind is not None and signal.kind != kind:
        msg = f'Signal {name} has type {signal.kind}, expected {kind}'
        raise ValueError(msg)
    if kind == 'bool' and not isinstance(signal.value, bool):
        msg = f'Signal {name} is not a JSON boolean'
        raise ValueError(msg)
    if kind == 'float' and (
        isinstance(signal.value, bool) or not isinstance(signal.value, int | float) or not math.isfinite(signal.value)
    ):
        msg = f'Signal {name} is not a finite number'
        raise ValueError(msg)
    return signal.value


def command_args(signals, values):
    payload = {}
    for name, value in values.items():
        kind = 'bool' if isinstance(value, bool) else 'float'
        signal_value(signals, name, kind)
        if signals[name].role != 'output':
            msg = f'Signal {name} is not writable'
            raise ValueError(msg)
        if kind == 'float' and not math.isfinite(value):
            msg = f'Non-finite command for {name}'
            raise ValueError(msg)
        payload[name] = {'format': {'type': kind}, 'data': [value]}
    # Matches the SDK's SetCommand.set_signal() schema.
    return json.dumps({'signals': payload}, allow_nan=False)


def time_key(stamp):
    return stamp.sec, stamp.nanosec


class SawyerGripper:
    def __init__(self, node, config):
        from intera_core_msgs.msg import IOComponentCommand, IODeviceStatus
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self.node, self.config = node, config
        self.message = None
        self.signals = {}
        self.error = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(IODeviceStatus, config.gripper_namespace + '/state', self.on_state, qos)
        self.publisher = node.create_publisher(IOComponentCommand, config.gripper_namespace + '/command', 1)

    def on_state(self, message):
        try:
            if message.device.name != 'right_gripper':
                msg = f'Unexpected gripper device: {message.device.name}'
                raise ValueError(msg)  # noqa: TRY301
            self.signals.update(read_signals(message))
            self.message, self.error = message, None
        except (ValueError, KeyError, TypeError) as exc:
            self.message, self.error = None, str(exc)

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                msg = f'{description}. {self.error or ""}'
                raise TimeoutError(msg)
            self.node.spin_once(timeout_sec=self.config.callback_timeout)

    def wait_for_state(self, timeout):
        self.wait_for(lambda: self.message is not None, timeout, 'No valid gripper state from bridge')

    def require_ready(self, action):
        if self.message is None:
            msg = 'No valid gripper state'
            raise RuntimeError(msg)
        if action != 'stop' and signal_value(self.signals, 'has_error', 'bool'):
            msg = 'Gripper reports an error'
            raise RuntimeError(msg)
        if action in ('open', 'close'):
            if not signal_value(self.signals, 'is_calibrated', 'bool'):
                msg = 'Gripper is not calibrated; explicitly call calibrate_gripper(execute=True)'
                raise RuntimeError(msg)
            if self.message.device.status.tag != 'ready':
                msg = f'Gripper is {self.message.device.status.tag}'
                raise RuntimeError(msg)
            if signal_value(self.signals, 'is_moving', 'bool'):
                msg = 'Gripper is already moving'
                raise RuntimeError(msg)
        elif action != 'stop' and self.message.device.status.tag not in ('ready', 'unready'):
            msg = f'Gripper is {self.message.device.status.tag}'
            raise RuntimeError(msg)

    def make_message(self, values):
        from intera_core_msgs.msg import IOComponentCommand

        return IOComponentCommand(
            time=self.node.get_clock().now().to_msg(), op='set', args=command_args(self.signals, values)
        )

    def send(self, values):
        message = self.make_message(values)
        key = time_key(message.time)
        deadline = time.monotonic() + self.config.gripper_timeout
        while True:
            if not self.publisher.get_subscription_count():
                msg = 'No gripper command bridge subscription'
                raise RuntimeError(msg)
            self.publisher.publish(message)
            try:
                self.wait_for(
                    lambda: self.message is not None and key in [time_key(stamp) for stamp in self.message.commands],
                    min(self.config.gripper_ack_retry_interval, max(0.001, deadline - time.monotonic())),
                    'Waiting for IO acknowledgement',
                )
                if values != {'go': False} and signal_value(self.signals, 'has_error', 'bool'):
                    msg = 'Gripper reports an error after acknowledgement'
                    raise RuntimeError(msg)
                return  # noqa: TRY300
            except TimeoutError:
                if time.monotonic() >= deadline:
                    msg = 'Gripper did not acknowledge the command'
                    raise TimeoutError(msg) from None

    def completed(self, action, target):
        if self.message is None:
            return False
        if self.message.device.status.tag in ('down', 'error'):
            msg = f'Gripper is {self.message.device.status.tag}'
            raise RuntimeError(msg)
        # Calibration can report is_calibrated before the device leaves busy.
        # Wait for ready before accepting completion or the next command.
        if self.message.device.status.tag != 'ready':
            return False
        required = (
            ('has_error', 'is_calibrated', 'is_moving')
            if action == 'calibrate'
            else ('has_error', 'is_calibrated', 'is_moving', 'position_response_m', 'is_gripping')
        )
        if any(name in self.signals and self.signals[name].status != 'ready' for name in required):
            return False
        if signal_value(self.signals, 'has_error', 'bool'):
            msg = 'Gripper reports an error'
            raise RuntimeError(msg)
        if not signal_value(self.signals, 'is_calibrated', 'bool') or signal_value(self.signals, 'is_moving', 'bool'):
            return False
        if action == 'calibrate':
            return True
        return abs(
            signal_value(self.signals, 'position_response_m', 'float') - target
        ) <= self.config.gripper_position_tolerance or (
            action == 'close' and signal_value(self.signals, 'is_gripping', 'bool')
        )

    def wait_completion(self, action, target):
        stable_since = None

        def settled():
            nonlocal stable_since
            if not self.completed(action, target):
                stable_since = None
                return False
            now = time.monotonic()
            if stable_since is None:
                stable_since = now
            return now - stable_since >= self.config.gripper_settle_confirmation

        self.wait_for(settled, self.config.gripper_timeout, f'Gripper {action} did not complete')

    def request_stop(self):
        try:
            message = self.make_message({'go': False})
            for _ in range(self.config.stop_hold_samples):
                self.publisher.publish(message)
                self.node.spin_until(time.monotonic() + 0.05)
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warning(f'Gripper stop delivery unconfirmed: {exc}')

    def perform(self, action, *, execute=False):
        if type(execute) is not bool:
            msg = 'execute must be a boolean'
            raise ValueError(msg)
        self.wait_for_state(self.config.connect_timeout)
        target = self.config.gripper_open_position if action == 'open' else 0.0
        values = (
            {'position_m': target, 'go': True}
            if action in ('open', 'close')
            else {'calibrate': True}
            if action == 'calibrate'
            else {'go': False}
        )
        encoded = command_args(self.signals, values)
        if not execute:
            return {'action': action, 'executed': False, 'command': encoded}
        self.require_ready(action)
        self.wait_for(
            lambda: self.publisher.get_subscription_count() > 0, self.config.connect_timeout, 'No gripper command route'
        )
        self.require_ready(action)
        try:
            self.send(values)
            if action != 'stop':
                self.wait_completion(action, target)
        except BaseException:
            self.request_stop()
            raise
        return {'action': action, 'executed': True}


class _BridgeClient:
    def __init__(self, config, ros_node, executor):
        from intera_core_msgs.msg import (
            CollisionDetectionState,
            EndpointState,
            EndpointStates,
            JointCommand,
            JointLimits,
            RobotAssemblyState,
        )
        from rclpy.duration import Duration
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64

        self.config = config
        self.ros_node, self.executor = ros_node, executor
        self.joints = self.velocity = self.state = self.limits = self.collision = None
        self.feedback = JointFeedback(JOINT_NAMES)
        self.last_feedback_error = self.last_limits_error = None
        self.joints_at = self.state_at = self.collision_at = 0.0
        self.streaming = False
        self.cancel_requested = threading.Event()
        self.create_subscription(JointState, self.config.joint_state_topic, self.on_joints, qos_profile_sensor_data)
        self.create_subscription(
            RobotAssemblyState, self.config.robot_state_topic, self.on_state, qos_profile_sensor_data
        )
        self.create_subscription(
            CollisionDetectionState, self.config.collision_topic, self.on_collision, qos_profile_sensor_data
        )
        limits_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(JointLimits, self.config.joint_limits_topic, self.on_limits, limits_qos)
        command_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, lifespan=Duration(seconds=self.config.command_timeout)
        )
        self.command_pub = self.create_publisher(JointCommand, self.config.joint_command_topic, command_qos)
        self.watchdog_pub = self.create_publisher(Float64, self.config.command_timeout_topic, 1)
        self.pose_feedback = EndpointFeedback(self.config.frame, self.config.feedback_timeout, self.config)
        self.create_subscription(
            EndpointState, self.config.endpoint_topic, self.pose_feedback.on_endpoint, qos_profile_sensor_data
        )
        self.create_subscription(
            EndpointStates, self.config.tip_states_topic, self.pose_feedback.on_tips, qos_profile_sensor_data
        )

    def __getattr__(self, name):
        return getattr(self.ros_node, name)

    def spin_once(self, timeout_sec=0.02):
        """Use the persistent executor assigned by SawyerBase."""
        self.executor.spin_once(timeout_sec=timeout_sec)

    def on_joints(self, message):
        try:
            updated = self.feedback.update(message.name, message.position, message.velocity, time.monotonic())
        except ValueError as exc:
            self.last_feedback_error = str(exc)
            return
        if updated:
            self.joints, self.velocity, self.joints_at = self.feedback.snapshot()
            self.last_feedback_error = None

    def on_state(self, message):
        self.state, self.state_at = message, time.monotonic()

    def on_collision(self, message):
        self.collision, self.collision_at = message.collision_state, time.monotonic()

    def on_limits(self, message):
        try:
            self.limits = JointBounds.from_message(message)
            self.last_limits_error = None
        except ValueError as exc:
            self.limits = None
            self.last_limits_error = str(exc)
            self.get_logger().error(str(exc))

    def input_issues(self):
        now = time.monotonic()
        issues = []
        joint_issues = self.feedback.issues(now, self.config.feedback_timeout)
        if joint_issues:
            issues.append('/robot/joint_states: ' + '; '.join(joint_issues))
            if self.last_feedback_error:
                issues.append('last rejected arm sample: ' + self.last_feedback_error)
        for topic, value, stamp in (
            ('/robot/state', self.state, self.state_at),
            ('/robot/limb/right/collision_detection_state', self.collision, self.collision_at),
        ):
            if value is None:
                issues.append(f'{topic}: no sample received')
            elif now - stamp >= self.config.feedback_timeout:
                issues.append(f'{topic}: {now - stamp:.3f}s old (limit {self.config.feedback_timeout:.3f}s)')
        if self.limits is None:
            issues.append('/robot/joint_limits: ' + (self.last_limits_error or 'no valid sample received'))
        return issues

    def fresh(self):
        return not self.input_issues()

    def check_ready(self):
        issues = self.input_issues()
        if issues:
            raise RuntimeError('Missing/stale feedback: ' + '; '.join(issues))
        if not self.state.enabled or not self.state.ready or self.state.stopped or self.state.error:
            msg = (
                f'Robot not ready: enabled={self.state.enabled}, ready={self.state.ready}, '
                f'stopped={self.state.stopped}, error={self.state.error}'
            )
            raise RuntimeError(msg)
        if self.collision:
            msg = 'Robot reports a collision'
            raise RuntimeError(msg)
        self.limits.check_position(self.joints, 'Measured position', tolerance=1e-4)

    def wait_inputs(self, execute, timeout=None):
        deadline = time.monotonic() + (self.config.connect_timeout if timeout is None else timeout)
        while not self.fresh() or (execute and not self.routes_connected()):
            if time.monotonic() >= deadline:
                issues = self.input_issues()
                if execute and not self.routes_connected():
                    issues.append('command/watchdog route absent; launch bridge with commands:=true')
                raise RuntimeError('Preflight timed out: ' + '; '.join(issues))
            self.spin_once(timeout_sec=self.config.input_poll_timeout)

    def routes_connected(self):
        return self.command_pub.get_subscription_count() > 0 and self.watchdog_pub.get_subscription_count() > 0

    def publish_velocity(self, velocity):
        from intera_core_msgs.msg import JointCommand

        message = JointCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.mode = JointCommand.VELOCITY_MODE
        message.names = list(JOINT_NAMES)
        message.velocity = list(vector(velocity, 7, 'velocity command'))
        self.command_pub.publish(message)

    def spin_until(self, deadline):
        """Service callbacks for a real interval even when callbacks return immediately."""
        while time.monotonic() < deadline:
            self.spin_once(timeout_sec=max(0.0, min(self.config.callback_slice, deadline - time.monotonic())))

    def stop(self):
        """
        Request zero velocity, then measured-position hold as in SDK exit_control_mode.

        Delivery and physical stopping cannot be confirmed after a connection loss.
        The robot's command watchdog remains set to 0.2 s for velocity mode.
        """
        from intera_core_msgs.msg import JointCommand

        if not self.streaming:
            return
        try:
            for _ in range(self.config.stop_zero_samples):
                self.publish_velocity((0.0,) * 7)
                self.spin_until(time.monotonic() + self.config.stop_publish_interval)
            positions = self.feedback.fresh_positions(time.monotonic(), self.config.feedback_timeout)
            if positions is not None:
                hold = JointCommand()
                hold.mode = JointCommand.POSITION_MODE
                hold.names = list(JOINT_NAMES)
                hold.position = list(positions)
                for _ in range(self.config.stop_hold_samples):
                    hold.header.stamp = self.get_clock().now().to_msg()
                    self.command_pub.publish(hold)
                    self.spin_until(time.monotonic() + self.config.stop_publish_interval)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Stop command delivery could not be confirmed: {exc}')
        finally:
            self.streaming = False

    def check_trajectory_limits(self, trajectory):
        if any(a < b for a, b in zip(self.limits.velocity, trajectory.max_velocity, strict=False)) or any(
            a < b for a, b in zip(self.limits.acceleration, trajectory.max_acceleration, strict=False)
        ):
            msg = 'Robot limits changed during the move; replan required'
            raise RuntimeError(msg)
        # Cartesian IK paths can have interior joint extrema beyond both ends.
        minimum = getattr(
            trajectory, 'min_position', tuple(min(a, b) for a, b in zip(trajectory.start, trajectory.end, strict=False))
        )
        maximum = getattr(
            trajectory, 'max_position', tuple(max(a, b) for a, b in zip(trajectory.start, trajectory.end, strict=False))
        )
        self.limits.check_position(minimum, 'Reference position minimum', tolerance=1e-8)
        self.limits.check_position(maximum, 'Reference position maximum', tolerance=1e-8)

    def track(self, trajectory, controller, options, *, label='Motion', on_step=None, at_goal=None, announce=True):  # noqa: PLR0912, PLR0913, PLR0915
        from std_msgs.msg import Float64

        if self.cancel_requested.is_set():
            msg = 'Motion cancelled'
            raise RuntimeError(msg)
        period = 1.0 / options.rate
        timeout = options.timeout if options.timeout is not None else trajectory.duration + options.settle_time
        if timeout < trajectory.duration:
            msg = '--timeout must be at least the planned motion duration'
            raise ValueError(msg)
        self.check_ready()
        self.check_trajectory_limits(trajectory)
        controller.reset(self.velocity)
        self.watchdog_pub.publish(Float64(data=self.config.command_timeout))
        start = time.monotonic()
        previous, next_tick = start - period, start
        stable_since = None
        next_watchdog = start + self.config.watchdog_refresh_interval
        self.streaming = True
        try:
            while True:
                if self.cancel_requested.is_set():
                    msg = 'Motion cancelled'
                    raise RuntimeError(msg)
                now = time.monotonic()
                if now < next_tick:
                    self.spin_once(timeout_sec=min(period, next_tick - now))
                    continue
                dt, elapsed = now - previous, now - start
                if dt > self.config.command_timeout / 2:
                    msg = f'Control loop missed its deadline (>{self.config.command_timeout/2:.3f} s); stopping'
                    raise RuntimeError(msg)
                if elapsed > timeout:
                    msg = f'Motion/settling timeout; {label} was not confirmed'
                    raise TimeoutError(msg)
                self.check_ready()
                if not self.routes_connected():
                    msg = 'Bridge command subscription disappeared'
                    raise RuntimeError(msg)
                self.check_trajectory_limits(trajectory)
                reference = trajectory.sample(elapsed)
                if max(abs(a - b) for a, b in zip(reference.position, self.joints, strict=False)) > options.max_error:
                    msg = 'Joint tracking error exceeded --max-error'
                    raise RuntimeError(msg)
                if on_step is not None:
                    on_step(elapsed, reference)
                command = controller.update(reference.position, self.joints, reference.velocity, self.velocity, dt)
                self.publish_velocity(command)
                if now >= next_watchdog:
                    self.watchdog_pub.publish(Float64(data=self.config.command_timeout))
                    next_watchdog = now + self.config.watchdog_refresh_interval
                arrived = (
                    elapsed >= trajectory.duration
                    and max(abs(a - b) for a, b in zip(trajectory.end, self.joints, strict=False))
                    < self.config.position_tolerance
                    and max(map(abs, self.velocity)) < self.config.velocity_tolerance
                    and max(map(abs, command)) < self.config.velocity_tolerance
                    and (at_goal is None or at_goal())
                )
                stable_since = (now if stable_since is None else stable_since) if arrived else None
                if stable_since is not None and now - stable_since >= self.config.settle_confirmation:
                    break
                previous, next_tick = now, now + period
        finally:
            self.stop()
        # Stop streaming before potentially blocking on terminal output.
        if announce:
            pass
        return MotionResult(trajectory.duration, elapsed, tuple(self.joints))

    def wait_pose(self, tip=None, timeout=None):
        deadline = time.monotonic() + (self.config.connect_timeout if timeout is None else timeout)
        while True:
            try:
                return self.pose_feedback.select(tip)
            except RuntimeError as exc:
                if time.monotonic() >= deadline:
                    msg = f'Cartesian pose unavailable: {exc}. Check endpoint/tip-state routes'
                    raise RuntimeError(msg) from exc
                self.spin_once(timeout_sec=self.config.callback_timeout)

    def planning_health(self, start_joints, start_pose, tip):
        issues = self.input_issues()
        if issues:
            # Polynomial fitting can occupy this thread; drain queued feedback
            # before treating old receipt times as a transport failure. No motion
            # is being commanded during planning. Execution retains the strict
            # per-step feedback watchdog in ArmMotionNode.
            self.wait_inputs(False, timeout=self.config.planning_refresh_timeout)
        if max(map(abs, self.velocity)) >= self.config.velocity_tolerance:
            msg = 'Cartesian rest-to-rest planning requires a stationary arm'
            raise RuntimeError(msg)
        if (
            max(abs(a - b) for a, b in zip(self.joints, start_joints, strict=False))
            > self.config.planning_joint_tolerance
        ):
            msg = 'Arm moved during Cartesian planning; rerun from the new pose'
            raise RuntimeError(msg)
        _, current_pose = self.wait_pose(tip, timeout=self.config.planning_refresh_timeout)
        position, orientation = pose_error(current_pose, start_pose)
        if (
            position > self.config.planning_position_tolerance
            or orientation > self.config.planning_orientation_tolerance
        ):
            msg = 'Endpoint moved during Cartesian planning; rerun from the new pose'
            raise RuntimeError(msg)


class SawyerBase:
    """
    Owns one private ROS context, node, and persistent executor.

    Construction is offline. connect() attaches to an already running bridge;
    it never starts a bridge, enables Sawyer, or calibrates the gripper.
    Methods are synchronous and service callbacks on this private executor.
    Poll get_status()/state properties to refresh feedback while idle.
    """

    def __init__(self, config: SawyerConfig | None = None):
        self.config = config or SawyerConfig()
        self._node = self._executor = self._context = self._gripper = self._ros_node = None
        self._connected = False
        self._busy = threading.Lock()

    @contextmanager
    def _operation(self):
        if not self._busy.acquire(blocking=False):
            msg = 'A Sawyer operation is already running; use one command owner'
            raise RuntimeError(msg)
        try:
            yield
        finally:
            self._busy.release()

    def _open_runtime(self):
        # Keep public imports and configuration usable before installing ROS.
        from ..setup_runtime import require_current_runtime

        require_current_runtime()
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        self._context = Context()
        self._context.init(args=[], domain_id=self.config.domain_id)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._ros_node = Node(self.config.node_name, context=self._context)
        self._executor.add_node(self._ros_node)
        self._node = _BridgeClient(self.config, self._ros_node, self._executor)
        if self.config.enable_gripper:
            self._gripper = SawyerGripper(self._node, self.config)

    def connect(self) -> bool:
        """Wait for arm, endpoint and optional gripper feedback; send no commands."""
        with self._operation():
            if self._connected:
                self._refresh()
                return True
            try:
                self._open_runtime()
                deadline = time.monotonic() + self.config.connect_timeout
                self._node.wait_inputs(False, timeout=max(0.001, deadline - time.monotonic()))
                self._node.wait_pose(self.config.tip, timeout=max(0.001, deadline - time.monotonic()))
                if self._gripper is not None:
                    self._gripper.wait_for_state(timeout=max(0.001, deadline - time.monotonic()))
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
        """Whether this object has an open session; get_status() checks freshness."""
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
    def endpoint_pose(self):
        """CartesianPose: meters, quaternion (x,y,z,w), and reference frame."""
        with self._operation():
            self._refresh()
            return self._node.wait_pose(self.config.tip)[1]

    @property
    def gripper_state(self) -> dict:
        """Return the latest standard Intera gripper signals without commanding it."""
        with self._operation():
            self._require_connected()
            if self._gripper is None:
                msg = 'Gripper support is disabled in SawyerConfig'
                raise RuntimeError(msg)
            self._node.spin_until(time.monotonic() + self.config.callback_timeout)
            self._gripper.wait_for_state(self.config.connect_timeout)
            return {
                'device_status': self._gripper.message.device.status.tag,
                **{name: signal.value for name, signal in self._gripper.signals.items()},
            }

    def get_status(self) -> dict:
        with self._operation():
            if not self._connected:
                return {'connected': False, 'ready': False, 'issues': ['Disconnected']}
            self._node.spin_until(time.monotonic() + self.config.callback_timeout)
            issues = self._node.input_issues()
            state = self._node.state
            ready = not issues
            if not issues:
                try:
                    self._node.check_ready()
                    self._node.pose_feedback.select(self.config.tip)
                except (RuntimeError, ValueError) as exc:
                    issues.append(str(exc))
                    ready = False
            return {
                'connected': True,
                'ready': bool(ready),
                'issues': issues,
                'enabled': bool(state and state.enabled),
                'collision': self._node.collision,
                'joints': self._node.joints,
                'velocities': self._node.velocity,
            }

    def _motion_options(self, options=None, *, duration=None, execute=False):
        if options is not None and not isinstance(options, MotionOptions):
            msg = 'options must be MotionOptions or CartesianMotionOptions'
            raise ValueError(msg)
        if options is not None and not isinstance(options, CartesianMotionOptions):
            options = replace(self.config.motion, **asdict(options))
        selected = options or self.config.motion
        return replace(
            selected,
            frame=self.config.frame,
            tip=self.config.tip if options is None or options.tip is None else options.tip,
            time=duration if duration is not None else selected.time,
            execute=execute,
        )

    def _close_runtime(self):
        try:
            if self._node is not None:
                self._node.stop()
        finally:
            try:
                if self._executor is not None:
                    self._executor.shutdown()
            finally:
                try:
                    if self._ros_node is not None:
                        self._ros_node.destroy_node()
                    elif self._node is not None:
                        self._node.destroy_node()
                finally:
                    try:
                        if self._context is not None and self._context.ok():
                            self._context.shutdown()
                    finally:
                        self._node = self._executor = self._context = self._gripper = self._ros_node = None
                        self._connected = False

    def disconnect(self):
        """Release this object's resources, leaving the external bridge running."""
        with self._operation():
            self._close_runtime()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_exc):
        self.disconnect()
