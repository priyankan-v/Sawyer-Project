"""Explicit robot/head/camera operations over the direct connection."""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict

from ...config import JOINT_NAMES
from ..communication import wire
from .io import command_args, read_signals, signal_value, time_key

CAMERAS = {
    'head': ('head_camera', 'image_rect_color'),
    'hand': ('right_hand_camera', 'image_rect'),
}
SDK_LIGHTS = frozenset(
    f'{location}_{color}_light'
    for location in ('head', 'right_hand')
    for color in ('red', 'green', 'blue')
)


class SawyerDevices:
    """
    Lazy peripheral access sharing the driver's single operation owner.

    Read-only calls never start a camera, enable joints, or move a device.
    State and images are wire dataclasses, independent of ROS runtimes.
    """

    def __init__(self, driver):
        self.driver = driver
        self.config = driver.config
        self.messages, self.publishers = {}, {}
        self.io_devices = {}
        self.transform_cache = None
        self.camera_callbacks = {}
        self.callback_sequence = 0

    def transforms(self, *, static=False):
        """
        Return the known TF edges; expired dynamic transforms are
        omitted.
        """
        from .transforms import TransformCache

        with self.driver._operation():  # noqa: SLF001
            if self.transform_cache is None:
                self.transform_cache = TransformCache(self.node, self.config)
            self.node.spin_until(
                time.monotonic() + self.config.callback_timeout
            )
            return self.transform_cache.snapshot(static=static)

    def _io(self, namespace):
        from .io import IODevice

        if namespace not in self.io_devices:
            self.io_devices[namespace] = IODevice(
                self.node,
                namespace,
                self.config.device_timeout,
                writable_inputs=SDK_LIGHTS
                if namespace == self.config.io_namespace + '/robot'
                else (),
            )
        return self.io_devices[namespace]

    def read_io(self, namespace):
        with self.driver._operation():  # noqa: SLF001
            device = self._io(namespace)
            self.node.spin_until(
                time.monotonic() + self.config.callback_timeout
            )
            return device.snapshot()

    def write_io(
        self,
        namespace,
        values,
        *,
        category='signals',
        execute=False,
        cancel_event=None,
    ):
        if namespace == self.config.gripper_namespace and set(
            values
        ).intersection(
            ('position_m', 'go', 'calibrate', 'cmd_grip', 'reboot')
        ):
            msg = 'Use the explicit gripper motion methods'
            raise ValueError(msg)
        with self.driver._operation():  # noqa: SLF001
            return self._io(namespace).write(
                values,
                category=category,
                execute=execute,
                cancel_event=cancel_event,
            )

    def register_io_callback(self, namespace, name, callback):
        with self.driver._operation():  # noqa: SLF001
            return self._io(namespace).register_callback(name, callback)

    def deregister_io_callback(self, namespace, identifier):
        with self.driver._operation():  # noqa: SLF001
            self._io(namespace).deregister_callback(identifier)

    def io_events(self, namespace):
        with self.driver._operation():  # noqa: SLF001
            device = self._io(namespace)
            self.node.spin_until(
                time.monotonic() + self.config.callback_timeout
            )
            events, device.events = device.events, []
            return events

    def diagnostics(self):
        with self.driver._operation():  # noqa: SLF001
            return self._read(
                wire.DiagnosticArray, self.config.diagnostics_topic
            )

    def configure_gripper(
        self, *, speed=None, dead_zone=None, object_mass=None, execute=False
    ):
        values = {
            name: value
            for name, value in (
                ('speed_mps', speed),
                ('dead_zone_m', dead_zone),
                ('right_gripper_tip_object_kg', object_mass),
            )
            if value is not None
        }
        with self.driver._operation():  # noqa: SLF001
            device = self._io(self.config.gripper_namespace)
            device.wait_ready()
            if (
                device.signals['is_moving']['value']
                or device.signals['has_error']['value']
            ):
                msg = 'Configure only an idle, error-free gripper'
                raise RuntimeError(msg)
            return device.write(values, execute=execute)

    def configure_camera(self, camera, settings, *, execute=False):
        allowed = {
            'exposure': 'set_exposure',
            'gain': 'set_gain',
            'brightness': 'set_brightness',
            'flip': 'set_flip',
            'mirror': 'set_mirror',
            'window_x': 'set_window_x',
            'window_y': 'set_window_y',
            'resolution': 'resolution',
            'strobe': 'set_strobe',
            'embedded_data': 'set_embedded_data',
        }
        if not settings or any(name not in allowed for name in settings):
            msg = 'Unknown camera setting'
            raise ValueError(msg)
        with self.driver._operation():  # noqa: SLF001
            namespace, _ = self._camera(camera)
            return self._io(namespace).write(
                {allowed[name]: value for name, value in settings.items()},
                execute=execute,
            )

    def set_light(self, name, enabled, *, execute=False):
        if type(enabled) is not bool or name not in SDK_LIGHTS:
            msg = 'Use a light signal name and boolean value'
            raise ValueError(msg)
        return self.write_io(
            self.config.io_namespace + '/robot',
            {name: enabled},
            execute=execute,
        )

    @property
    def node(self):
        self.driver._require_connected()  # noqa: SLF001
        return self.driver._node  # noqa: SLF001

    def _wait(self, predicate, description, timeout=None):
        deadline = time.monotonic() + (timeout or self.config.device_timeout)
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(description)
            self.node.spin_once(timeout_sec=self.config.callback_timeout)

    def _subscribe(self, kind, topic):
        if topic not in self.messages:
            self.messages[topic] = None
            self.node.observe(
                kind,
                topic,
                lambda message: self.messages.__setitem__(topic, message),
            )

    def _read(self, kind, topic, *, fresh=False):
        self._subscribe(kind, topic)
        self.node.spin_until(time.monotonic() + self.config.callback_timeout)

        def available():
            message = self.messages[topic]
            return message is not None and (
                not fresh
                or time.monotonic() - message.received_at
                <= self.config.feedback_timeout
            )

        self._wait(available, f'No valid feedback from {topic}')
        return self.messages[topic]

    def _publisher(self, kind, topic):
        if topic not in self.publishers:
            self.publishers[topic] = self.node.publisher_for(kind, topic)
        publisher = self.publishers[topic]
        self._wait(
            lambda: publisher.get_subscription_count() > 0,
            f'No command subscriber for {topic}',
        )
        return publisher

    def inventory(self):
        with self.driver._operation():  # noqa: SLF001
            transport = self.node.transport
            return {
                'version': transport.master(
                    'getParam', self.config.version_parameter
                ),
                'topic_types': transport.master('getTopicTypes'),
                'graph': transport.master('getSystemState'),
                'wire_messages': sorted(wire.SCHEMA['messages']),
                'wire_services': sorted(wire.SCHEMA['services']),
            }

    def _robot_state(self):
        self.node.spin_until(time.monotonic() + self.config.callback_timeout)
        state = self.node.state
        if (
            state is None
            or time.monotonic() - self.node.state_at
            > self.config.feedback_timeout
        ):
            msg = 'Robot state is stale'
            raise RuntimeError(msg)
        return state

    def robot_state(self):
        with self.driver._operation():  # noqa: SLF001
            return asdict(self._robot_state())

    def _robot_command(self, message, topic, completed, execute):
        state = self._robot_state()
        if not execute:
            return {
                'executed': False,
                'topic': topic,
                'command': asdict(message),
            }
        if completed(state):
            return {'executed': False, 'already_complete': True}
        publisher = self._publisher(type(message), topic)

        def step():
            if completed(self._robot_state()):
                return True
            publisher.publish(message)
            self.node.spin_until(time.monotonic() + 0.05)
            return False

        self._wait(step, f'Robot did not confirm {topic}')
        return {'executed': True, 'state': asdict(self._robot_state())}

    def set_enabled(self, enabled, *, execute=False):
        if type(enabled) is not bool or type(execute) is not bool:
            msg = 'enabled and execute must be booleans'
            raise ValueError(msg)
        with self.driver._operation():  # noqa: SLF001
            state = self._robot_state()
            if enabled and (state.stopped or state.error):
                msg = 'Reset faults explicitly before enabling'
                raise RuntimeError(msg)
            if enabled and execute:
                return self._enable_robot(state)
            return self._robot_command(
                wire.Bool(data=enabled),
                self.config.enable_topic,
                lambda state: state.enabled == enabled,
                execute,
            )

    def _enable_robot(self, state):
        if state.enabled and state.ready:
            return {'executed': False, 'already_complete': True}
        start = self.node.feedback.fresh_positions(
            time.monotonic(), self.config.feedback_timeout
        )
        if start is None:
            msg = 'Fresh measured joint positions are required before enabling'
            raise RuntimeError(msg)
        publisher = self._publisher(wire.Bool, self.config.enable_topic)
        try:
            result = self._wait_enabled(publisher, start)
        except BaseException:
            try:
                self._stop_robot()
            except (OSError, RuntimeError) as exc:
                self.node.get_logger().warning(
                    f'Enable failed; software stop unconfirmed: {exc}'
                )
            raise
        return result

    def _wait_enabled(self, publisher, start):
        deadline, next_send = (
            time.monotonic() + self.config.device_timeout,
            0.0,
        )
        while True:
            state = self._robot_state()
            positions = self.node.feedback.fresh_positions(
                time.monotonic(), self.config.feedback_timeout
            )
            if positions is None:
                msg = 'Fresh joint feedback lost while enabling'
                raise RuntimeError(msg)
            deltas = [
                abs(a - b) for a, b in zip(start, positions, strict=True)
            ]
            index = max(range(len(deltas)), key=deltas.__getitem__)
            if deltas[index] > self.config.enable_max_displacement:
                msg = (
                    'Joint displacement exceeded the enable envelope: '
                    f'{JOINT_NAMES[index]} moved {deltas[index]:.6f} rad '
                    f'(limit {self.config.enable_max_displacement:.6f}); '
                    f'homed={state.homed}, ready={state.ready}'
                )
                raise RuntimeError(msg)
            if state.stopped or state.error:
                msg = 'Controller faulted while enabling'
                raise RuntimeError(msg)
            if state.enabled and state.ready:
                return {
                    'executed': True,
                    'ready_confirmed': True,
                    'state': asdict(state),
                }
            now = time.monotonic()
            if now >= deadline:
                msg = 'Enabled/ready state was not confirmed'
                raise TimeoutError(msg)
            if not state.enabled and now >= next_send:
                publisher.publish(wire.Bool(data=True))
                next_send = now + 0.1

    def reset_faults(self, *, execute=False):
        if type(execute) is not bool:
            msg = 'execute must be a boolean'
            raise ValueError(msg)
        with self.driver._operation():  # noqa: SLF001
            state = self._robot_state()
            if (
                not state.stopped
                or state.estop_button == state.ESTOP_BUTTON_PRESSED
            ):
                msg = 'Reset requires stopped state and released E-stop'
                raise RuntimeError(msg)
            return self._robot_command(
                wire.Empty(),
                self.config.reset_topic,
                lambda state: not state.stopped and not state.error,
                execute,
            )

    def software_stop(self, *, execute=False):
        """
        Latch the controller stopped state; separate from motion cancellation.
        """
        if type(execute) is not bool:
            msg = 'execute must be a boolean'
            raise ValueError(msg)
        if execute:
            self.node.cancel_requested.set()
        with self.driver._operation(  # noqa: SLF001
            timeout=self.config.device_timeout if execute else 0.0
        ):
            if not execute:
                return {
                    'executed': False,
                    'topic': self.config.software_stop_topic,
                    'command': asdict(wire.Empty()),
                }
            return self._stop_robot()

    def _stop_robot(self):
        node = self.node
        deadline = time.monotonic() + self.config.device_timeout
        publisher, sent, next_send = None, False, 0.0
        while time.monotonic() < deadline:
            state = node.state
            if (
                state is not None
                and state.stopped
                and time.monotonic() - node.state_at
                <= self.config.feedback_timeout
            ):
                return {'executed': sent, 'state': asdict(state)}
            if publisher is None:
                publisher = self._publisher(
                    wire.Empty(), self.config.software_stop_topic
                )
            now = time.monotonic()
            if publisher.get_subscription_count() and now >= next_send:
                publisher.publish(wire.Empty())
                sent, next_send = True, now + 0.05
            node.spin_once(timeout_sec=0.01)
        msg = 'Software stop was requested but not confirmed by fresh feedback'
        raise TimeoutError(msg)

    def head_state(self):
        with self.driver._operation():  # noqa: SLF001
            return self._read(
                wire.HeadState, self.config.head_state_topic, fresh=True
            )

    def _head_limits(self):
        text = self.node.transport.master(
            'getParam', self.config.robot_description_parameter
        )
        # URDF is read from the robot; do not invent limits for its head.
        root = ET.fromstring(text)  # noqa: S314
        joint = root.find("./joint[@name='head_pan']/limit")
        if joint is None:
            msg = 'Robot description lacks head_pan limits'
            raise RuntimeError(msg)
        return float(joint.attrib['lower']), float(joint.attrib['upper'])

    def move_head(self, angle, *, speed=0.1, execute=False):
        if (
            type(execute) is not bool
            or not math.isfinite(angle)
            or not math.isfinite(speed)
            or not 0 < speed <= 1
        ):
            msg = 'Use finite angle, speed in (0,1], boolean execute'
            raise ValueError(msg)
        with self.driver._operation():  # noqa: SLF001
            lower, upper = self._head_limits()
            if not lower <= angle <= upper:
                msg = 'Head target exceeds reported URDF limits'
                raise ValueError(msg)
            initial = self._read(
                wire.HeadState, self.config.head_state_topic, fresh=True
            )
            command = wire.HeadPanCommand(
                target=angle,
                speed_ratio=speed,
                pan_mode=wire.HeadPanCommand.SET_ACTIVE_MODE,
            )
            if not execute:
                return {'executed': False, 'command': asdict(command)}
            publisher = self._publisher(
                type(command), self.config.head_command_topic
            )

            def arrived():
                state = self.messages[self.config.head_state_topic]
                if (
                    time.monotonic() - state.received_at
                    > self.config.feedback_timeout
                ):
                    msg = 'Head feedback is stale'
                    raise RuntimeError(msg)
                if state.is_blocked:
                    msg = 'Head reports blocked movement'
                    raise RuntimeError(msg)
                reached = (
                    abs(state.pan - angle)
                    <= self.config.head_position_tolerance
                    and not state.is_turning
                    and state.pan_mode == wire.HeadPanCommand.SET_ACTIVE_MODE
                )
                if not reached:
                    publisher.publish(command)
                    self.node.spin_until(
                        time.monotonic() + self.config.callback_slice
                    )
                return reached

            try:
                publisher.publish(command)
                self._wait(arrived, 'Head did not reach target in active mode')
            except (OSError, RuntimeError, KeyboardInterrupt):
                state = self.messages[self.config.head_state_topic]
                try:
                    publisher.publish(
                        wire.HeadPanCommand(
                            target=state.pan,
                            speed_ratio=speed,
                            pan_mode=initial.pan_mode,
                        )
                    )
                except OSError as exc:
                    self.node.get_logger().warning(
                        f'Head hold delivery unconfirmed: {exc}'
                    )
                raise
            return {
                'executed': True,
                'state': asdict(self.messages[self.config.head_state_topic]),
            }

    def _camera(self, camera):
        if camera not in CAMERAS:
            msg = 'camera must be head or hand'
            raise ValueError(msg)
        name, image = CAMERAS[camera]
        return self.config.camera_namespace + '/' + name, image

    def camera_state(self, camera):
        with self.driver._operation():  # noqa: SLF001
            namespace, _ = self._camera(camera)
            return self._read(wire.IODeviceStatus, namespace + '/state')

    def register_camera_callback(self, camera, callback):
        """
        Callbacks run during poll/control; keep them fast and
        nonblocking.
        """
        if not callable(callback):
            msg = 'A callable camera callback is required'
            raise TypeError(msg)
        with self.driver._operation():  # noqa: SLF001
            namespace, image = self._camera(camera)
            topic = namespace + '/' + image
            self.node.observe(wire.Image, topic, callback)
            self.callback_sequence += 1
            self.camera_callbacks[self.callback_sequence] = (topic, callback)
            return self.callback_sequence

    def deregister_camera_callback(self, identifier):
        with self.driver._operation():  # noqa: SLF001
            topic, callback = self.camera_callbacks.pop(identifier)
            self.node.remove_observer(topic, callback)

    def digital_state(self, name):
        with self.driver._operation():  # noqa: SLF001
            return self._digital_state(name)

    def _digital_state(self, name):
        if not name or '/' in name or name in ('.', '..'):
            msg = 'Use a published digital I/O component name'
            raise ValueError(msg)
        return self._read(
            wire.DigitalIOState, '/robot/digital_io/' + name + '/state'
        )

    def set_digital_output(self, name, value, *, execute=False):
        if type(value) is not bool or type(execute) is not bool:
            msg = 'Digital output and execute values must be booleans'
            raise ValueError(msg)
        with self.driver._operation():  # noqa: SLF001
            state = self._digital_state(name)
            if state.is_input_only:
                msg = 'Digital I/O component is input-only'
                raise ValueError(msg)
            command = wire.DigitalOutputCommand(name=name, value=value)
            if not execute:
                return {'executed': False, 'command': asdict(command)}
            if state.state == int(value):
                return {
                    'executed': False,
                    'already_complete': True,
                    'measured': value,
                }
            publisher = self._publisher(
                wire.DigitalOutputCommand, '/robot/digital_io/command'
            )
            sent_at = time.monotonic()
            publisher.publish(command)
            topic = '/robot/digital_io/' + name + '/state'
            self._wait(
                lambda: (
                    self.messages[topic].received_at >= sent_at
                    and self.messages[topic].state == int(value)
                ),
                'Digital output state was not confirmed',
            )
            return {'executed': True, 'measured': value}

    def io_state(self, device):
        """Read raw device status, including vector and change-only signals."""
        if device not in ('cuff', 'navigator', 'robot', 'right_end_of_arm'):
            msg = 'Unknown read-only IO device'
            raise ValueError(msg)
        with self.driver._operation():  # noqa: SLF001
            return self._read(
                wire.IODeviceStatus,
                self.config.io_namespace + '/' + device + '/state',
            )

    def set_camera_streaming(self, camera, enabled, *, execute=False):
        if type(enabled) is not bool or type(execute) is not bool:
            msg = 'enabled and execute must be booleans'
            raise ValueError(msg)
        with self.driver._operation():  # noqa: SLF001
            namespace, _ = self._camera(camera)
            topic = namespace + '/state'
            state = self._read(wire.IODeviceStatus, topic)
            values = {'camera_streaming': enabled}
            message = wire.IOComponentCommand(
                time=self.node.get_clock().now().to_msg(),
                op='set',
                args=command_args(read_signals(state), values),
            )
            if not execute:
                return {'executed': False, 'command': asdict(message)}
            publisher = self._publisher(type(message), namespace + '/command')
            key = time_key(message.time)

            def acknowledged():
                current = self.messages[topic]
                if (
                    key in [time_key(stamp) for stamp in current.commands]
                    and signal_value(
                        read_signals(current), 'camera_streaming', 'bool'
                    )
                    == enabled
                ):
                    return True
                publisher.publish(message)
                self.node.spin_until(time.monotonic() + 0.1)
                return False

            self._wait(
                acknowledged, 'Camera did not acknowledge streaming state'
            )
            return {'executed': True}

    def camera_frame(self, camera):
        """
        Read a current frame and CameraInfo without changing streaming state.
        """
        with self.driver._operation():  # noqa: SLF001
            namespace, image = self._camera(camera)
            device = self._io(namespace)
            device.wait_ready()
            if not device.signals['camera_streaming']['value']:
                msg = (
                    'Camera is not streaming; start it explicitly before '
                    'reading a current frame'
                )
                raise RuntimeError(msg)
            requested_at = time.monotonic()
            self._subscribe(wire.Image, namespace + '/' + image)
            self._subscribe(wire.CameraInfo, namespace + '/camera_info')
            first = None
            topic = namespace + '/' + image

            def current():
                nonlocal first
                message = self.messages[topic]
                if message is None or message.received_at < requested_at:
                    return False
                if not device.signals['camera_streaming']['value']:
                    msg = 'Camera stopped while waiting for a current frame'
                    raise RuntimeError(msg)
                stamp = message.header.stamp
                key = (message.header.seq, stamp.sec, stamp.nanosec)
                if first is None:
                    first = key
                    return False
                return (
                    key != first
                    and time.monotonic() - message.received_at
                    <= self.config.feedback_timeout
                )

            self._wait(
                current, 'Camera did not deliver advancing current frames'
            )
            return (
                self.messages[topic],
                self._read(wire.CameraInfo, namespace + '/camera_info'),
            )

    def display_image(self, image, *, execute=False):
        """
        Publish an explicitly supplied rgb8/bgr8 image; no cv_bridge needed.
        """
        if (
            type(execute) is not bool
            or not isinstance(image, wire.Image)
            or image.encoding not in ('rgb8', 'bgr8')
            or image.height <= 0
            or image.width <= 0
            or image.step < image.width * 3
            or len(image.data) != image.height * image.step
        ):
            msg = 'Supply a valid RGB/BGR wire.Image'
            raise ValueError(msg)
        payload = wire.serialize(image)
        if not execute:
            return {'executed': False, 'bytes': len(payload)}
        with self.driver._operation():  # noqa: SLF001
            self._publisher(
                wire.Image, self.config.head_display_topic
            ).publish(image)
            return {'executed': True, 'delivery_confirmed': False}
