"""ROS 2 telemetry conversion outside the robot's control callbacks."""

# The adapter intentionally observes the driver's shared transport.
# ruff: noqa: SLF001
from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import fields, is_dataclass, replace

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .core import wire
from .ros2_images import serialize_image


def to_ros(message):
    """
    Copy standard data, omitting ROS 1 Header.seq and local receipt
    metadata.
    """
    if message.ros_type in ('time', 'duration'):
        kind = get_message(
            'builtin_interfaces/msg/' + message.ros_type.title()
        )
    else:
        package, name = message.ros_type.split('/')
        if package.startswith('intera_'):
            package = 'irobot_sawyer_interfaces'
        kind = get_message(f'{package}/msg/{name}')
    result = kind()
    specifications = {
        wire.field_name(name): wire.split_type(field_type)
        for name, field_type in wire.SCHEMA['messages'][message.ros_type][
            'fields'
        ]
    }
    for field in fields(message):
        if field.name == 'received_at' or not hasattr(result, field.name):
            continue
        value = getattr(message, field.name)
        if is_dataclass(value):
            value = to_ros(value)
        elif isinstance(value, list) and value and is_dataclass(value[0]):
            value = [to_ros(item) for item in value]
        elif isinstance(value, list) and specifications[field.name][0] in (
            'float32',
            'float64',
        ):
            value = [float(item) for item in value]
        elif isinstance(getattr(result, field.name), float):
            value = float(value)
        elif isinstance(getattr(result, field.name), bytes) and isinstance(
            value, int
        ):
            value = bytes([value & 0xFF])
        setattr(result, field.name, value)
    return result


def to_wire(message, kind):
    """Convert a typed ROS 2 request into the driver's independent types."""
    values = {}
    for original, field_type in wire.SCHEMA['messages'][kind.ros_type][
        'fields'
    ]:
        name = wire.field_name(original)
        if not hasattr(message, name):
            continue
        value = getattr(message, name)
        base, length = wire.split_type(field_type)
        if base in wire.SCHEMA['messages']:
            child = wire.message_type(base)
            value = (
                to_wire(value, child)
                if length is None
                else [to_wire(item, child) for item in value]
            )
        elif length is not None:
            value = bytes(value) if base in ('uint8', 'char') else list(value)
        elif base == 'byte' and isinstance(value, bytes):
            value = int.from_bytes(value, 'little', signed=True)
        values[name] = value
    return kind(**values)


class SawyerTelemetry:
    """
    Receive into a small cache; conversion/DDS publication run on the executor.

    ROS 2 headers use host receipt time: robot and workstation clocks are not
    synchronized. Original controller timestamps remain available in Python.
    Subscribing to cameras never starts or changes a robot camera stream.
    """

    def __init__(self, node, driver, *, cameras=False, interfaces=False):
        self.node, self.driver = node, driver
        self.lock = threading.Lock()
        self.pending = {}
        self.publishers = {}
        self.static = {}
        self.observers = []
        self.efforts = {}
        self.errors = {}
        self.received_counts, self.published_counts = {}, {}
        self.camera_keys = {}
        self.io_states = {}
        self.io_events = deque(maxlen=256)
        self.dropped_io_events = 0
        self.connections, self.receipts = {}, {}
        self.io_health_pub = (
            node.create_publisher(String, 'io_health', 1)
            if interfaces
            else None
        )
        self.io_events_pub = (
            node.create_publisher(String, 'io_events', 256)
            if interfaces
            else None
        )
        config = driver.config
        for source, target, kind, ros_kind in (
            (config.joint_state_topic, 'joint_state', wire.JointState, None),
            (
                config.diagnostics_topic,
                'diagnostics',
                wire.DiagnosticArray,
                DiagnosticArray,
            ),
            (config.tf_topic, 'tf', wire.TFMessage, TFMessage),
            (config.tf_static_topic, 'tf_static', wire.TFMessage, TFMessage),
            (
                config.tip_states_topic,
                'endpoint_state',
                wire.EndpointStates,
                None,
            ),
        ):
            self.observe(source, target, kind, ros_kind)
        self.twist_pub = node.create_publisher(
            TwistStamped, 'endpoint_twist', qos_profile_sensor_data
        )
        self.wrench_pub = node.create_publisher(
            WrenchStamped, 'endpoint_wrench', qos_profile_sensor_data
        )
        if interfaces:
            for source, target, kind in (
                (
                    config.robot_state_topic,
                    'robot_state',
                    wire.RobotAssemblyState,
                ),
                (
                    config.tip_states_topic,
                    'endpoint_states',
                    wire.EndpointStates,
                ),
                (
                    '/robot/digital_io/right_hand_camera_power/state',
                    'digital_io/right_hand_camera_power',
                    wire.DigitalIOState,
                ),
            ):
                self.observe(
                    source,
                    target,
                    kind,
                    get_message(
                        'irobot_sawyer_interfaces/msg/' + kind.__name__
                    ),
                )
            for label, namespace in (
                ('gripper', config.gripper_namespace),
                ('head_camera', config.camera_namespace + '/head_camera'),
                (
                    'hand_camera',
                    config.camera_namespace + '/right_hand_camera',
                ),
                ('robot_io', config.io_namespace + '/robot'),
                ('cuff', config.io_namespace + '/cuff'),
                ('navigator', config.io_namespace + '/navigator'),
            ):
                for suffix, kind in (
                    ('/state', wire.IODeviceStatus),
                    ('/config', wire.IODeviceConfiguration),
                ):
                    self.observe(
                        namespace + suffix,
                        label + suffix,
                        kind,
                        get_message(
                            'irobot_sawyer_interfaces/msg/' + kind.__name__
                        ),
                    )
            self.observe(
                config.joint_limits_topic,
                'joint_limits',
                wire.JointLimits,
                get_message('irobot_sawyer_interfaces/msg/JointLimits'),
            )
        if cameras:
            for camera in ('head', 'hand'):
                namespace, image = driver.devices._camera(camera)
                self.observe(
                    namespace + '/' + image,
                    camera + '/image',
                    wire.Image,
                    Image,
                )
                self.observe(
                    namespace + '/camera_info',
                    camera + '/camera_info',
                    wire.CameraInfo,
                    CameraInfo,
                )

    def observe(self, source, target, kind, ros_kind):
        if ros_kind is not None:
            qos = (
                QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
                )
                if target in ('tf_static', 'joint_limits')
                or target.endswith(('/config', '/state', '/camera_info'))
                else qos_profile_sensor_data
            )
            self.publishers[target] = self.node.create_publisher(
                ros_kind, target, qos
            )

        def receive(message):
            self.receive(target, message)

        self.connections[target] = self.driver._node.observe(
            kind, source, receive
        )
        self.observers.append((source, receive))

    def receive(self, target, message):
        if target == 'joint_state':
            self.receive_joints(message)
            return
        stamp = self.node.get_clock().now().to_msg()
        with self.lock:
            self.received_counts[target] = (
                self.received_counts.get(target, 0) + 1
            )
            if isinstance(message, wire.Image):
                key = (
                    message.header.seq,
                    message.header.stamp.sec,
                    message.header.stamp.nanosec,
                )
                previous = self.camera_keys.get(target)
                self.camera_keys[target] = key
                if previous is None or previous == key:
                    return
            if isinstance(message, wire.IODeviceStatus):
                message = self.merge_io_state(target, message)
            self.pending[target] = (message, stamp)
            if target.endswith('/state'):
                self.receipts[target] = message.received_at

    def merge_io_state(self, target, message):
        previous = self.io_states.get(target)
        updates = {}
        for category in ('signals', 'ports'):
            cache = {
                item.name: item for item in getattr(previous, category, [])
            }
            for item in getattr(message, category):
                old = cache.get(item.name)
                if old is not None and old.data != item.data:
                    if len(self.io_events) == self.io_events.maxlen:
                        self.dropped_io_events += 1
                    self.io_events.append(
                        {
                            'device': target,
                            'category': category,
                            'name': item.name,
                            'previous_json': old.data,
                            'value_json': item.data,
                            'received_at_monotonic': message.received_at,
                        }
                    )
                cache[item.name] = item
            updates[category] = list(cache.values())
        snapshot = replace(message, **updates)
        snapshot.received_at = message.received_at
        self.io_states[target] = snapshot
        return snapshot

    def receive_joints(self, message):
        status = self.driver._node.status_snapshot()
        with self.node.cache_lock:
            self.node.cache.update(status)
            self.node.sample_time = time.monotonic()
        if len(message.name) == len(message.effort):
            with self.lock:
                for name, value in zip(
                    message.name, message.effort, strict=True
                ):
                    if math.isfinite(value):
                        self.efforts[name] = (value, message.received_at)

    def joint_efforts(self, names):
        with self.lock:
            if any(
                name not in self.efforts
                or time.monotonic() - self.efforts[name][1]
                > self.driver.config.feedback_timeout
                for name in names
            ):
                return []
            return [self.efforts[name][0] for name in names]

    def publish(self):
        with self.lock:
            pending, self.pending = self.pending, {}
            receipts = dict(self.receipts)
            events = list(self.io_events)
            self.io_events.clear()
        now = time.monotonic()
        self.publish_io_health(events, receipts, now)
        for target, (received, stamp) in pending.items():
            message = received
            if target == 'endpoint_state':
                self.publish_endpoint(message, stamp)
                continue
            if target == 'tf_static':
                for transform in message.transforms:
                    self.static[
                        (transform.header.frame_id, transform.child_frame_id)
                    ] = transform
                message = wire.TFMessage(transforms=list(self.static.values()))
            elif not isinstance(
                message,
                wire.IODeviceStatus
                | wire.IODeviceConfiguration
                | wire.JointLimits,
            ) and (
                now - message.received_at > self.driver.config.feedback_timeout
            ):
                continue
            try:
                output = (
                    serialize_image(message, stamp)
                    if isinstance(message, wire.Image)
                    else to_ros(message)
                )
            except (
                ValueError,
                TypeError,
                AssertionError,
                OverflowError,
            ) as exc:
                if self.errors.get(target) != str(exc):
                    self.node.get_logger().error(f'Telemetry {target}: {exc}')
                self.errors[target] = str(exc)
                continue
            self.errors.pop(target, None)
            if hasattr(output, 'header'):
                output.header.stamp = stamp
            if target in ('tf', 'tf_static'):
                for transform in output.transforms:
                    transform.header.stamp = stamp
            self.publishers[target].publish(output)
            self.published_counts[target] = (
                self.published_counts.get(target, 0) + 1
            )

    def publish_io_health(self, events, receipts, now):
        if self.io_events_pub is not None:
            for event in events:
                event['dropped_events'] = self.dropped_io_events
                self.io_events_pub.publish(String(data=json.dumps(event)))
        if self.io_health_pub is not None:
            health = {
                target: {
                    'connected': connection.connected(),
                    'last_receipt_age_s': None
                    if target not in receipts
                    else now - receipts[target],
                    'change_only': True,
                }
                for target, connection in self.connections.items()
                if target.endswith('/state') and connection is not None
            }
            self.io_health_pub.publish(String(data=json.dumps(health)))

    def close(self):
        for source, callback in self.observers:
            self.driver._node.remove_observer(source, callback)
        self.observers.clear()

    def publish_endpoint(self, message, stamp):
        if (
            time.monotonic() - message.received_at
            > self.driver.config.feedback_timeout
        ):
            return
        tip = self.driver.config.tip or 'right_hand'
        if tip not in message.names:
            return
        state = message.states[message.names.index(tip)]
        if not state.valid:
            return
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = (
            state.header.frame_id or self.driver.config.frame
        )
        pose.pose = to_ros(state.pose)
        self.node.pose_pub.publish(pose)
        for field, kind, publisher in (
            ('twist', TwistStamped, self.twist_pub),
            ('wrench', WrenchStamped, self.wrench_pub),
        ):
            output = kind()
            output.header.stamp = stamp
            output.header.frame_id = (
                state.header.frame_id or self.driver.config.frame
            )
            setattr(output, field, to_ros(getattr(state, field)))
            publisher.publish(output)
