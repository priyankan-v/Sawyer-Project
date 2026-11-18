"""Real DDS lifecycle and serialization checks for the optional interfaces."""

# The test component deliberately inspects its internal operation ownership.
# ruff: noqa: SLF001, PLR2004
from __future__ import annotations

import os
import threading
import time
from dataclasses import asdict
from uuid import uuid4

import pytest

pytest.importorskip('irobot_sawyer_interfaces')
rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus
from irobot_sawyer_interfaces.action import JointStream, MotionCommand
from irobot_sawyer_interfaces.msg import StreamCommand
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image
from unique_identifier_msgs.msg import UUID

from irobot.src.robots.sawyer.config import JOINT_NAMES
from irobot.src.robots.sawyer.core.communication import wire
from irobot.src.robots.sawyer.ros2 import SawyerNode
from irobot.src.robots.sawyer.ros2_images import (
    deserialize_image,
    serialize_image,
)
from irobot.src.robots.sawyer.ros2_telemetry import to_ros, to_wire
from tests.test_irobot.test_sawyer_ros2 import LocalDriver, completed


def test_typed_standard_and_intera_conversions():
    for message in (
        wire.Image(data=b'\x01\x02'),
        wire.CameraInfo(),
        wire.DiagnosticArray(
            status=[wire.DiagnosticStatus(level=1, message='Retained event')]
        ),
        wire.TFMessage(),
        wire.Trajectory(),
    ):
        result = to_wire(to_ros(message), type(message))
        assert wire.serialize(result) == wire.serialize(message)
    native = wire.MotionCommandResult(
        result=False, error_id='CONTROLLER_NOT_FOLLOWING'
    )
    assert (
        to_ros(wire.from_dict(type(native), asdict(native))).error_id
        == native.error_id
    )


@pytest.mark.parametrize('frame', ['', 'base', 'camera_optical', 'κ_frame'])
@pytest.mark.parametrize(
    ('encoding', 'channels'), [('mono8', 1), ('rgb8', 3), ('bgra8', 4)]
)
def test_fast_image_serialization_matches_native_ros2_cdr(
    frame, encoding, channels
):
    image = wire.Image(
        width=3,
        height=2,
        step=3 * channels,
        encoding=encoding,
        data=bytes(range(6 * channels)),
    )
    image.header.frame_id = frame
    stamp = image.header.stamp = wire.Time(sec=123, nanosec=456)
    payload = serialize_image(image, stamp)
    # CDR alignment padding is unspecified; compare all decoded fields.
    reference = serialize_message(to_ros(image))
    assert wire.serialize(deserialize_image(reference)) == wire.serialize(
        image
    )
    native = deserialize_message(payload, Image)
    assert native == to_ros(image)
    assert bytes(native.data) == image.data
    assert native.header.frame_id == frame
    assert wire.serialize(deserialize_image(payload)) == wire.serialize(image)


@pytest.mark.skipif(
    os.environ.get('SAWYER_ROS2_TEST') != '1', reason='Enable local DDS tests'
)
def test_stream_client_loss_requires_new_goal():  # noqa: PLR0915
    context = Context()
    context.init(domain_id=99)
    driver = LocalDriver()
    driver.release_read.set()
    calls = []

    def stream(  # noqa: PLR0913
        mode, source, duration, *, options, execute, cancel_event, on_step
    ):
        assert mode == 'velocity'
        assert execute
        assert options.rate > 0
        assert driver.operation_lock.acquire(blocking=False)
        try:
            start = time.monotonic()
            samples = 0
            while time.monotonic() - start < duration:
                if cancel_event.is_set():
                    msg = 'Cancelled'
                    raise RuntimeError(msg)
                calls.append(source(time.monotonic() - start))
                on_step(time.monotonic() - start, (0.0,) * 7, (0.0,) * 7)
                samples += 1
                time.sleep(0.01)
            return {'samples': samples, 'max_command_gap_s': 0.01}
        finally:
            driver.operation_lock.release()

    driver.stream_joint_commands = stream
    driver.execute_native_trajectory = lambda *_args, **_kwargs: {
        'succeeded': False,
        'result': asdict(
            wire.MotionCommandResult(error_id='CONTROLLER_NOT_FOLLOWING')
        ),
    }
    server = SawyerNode(driver, context=context, interfaces=True)
    client_node = Node('extended_client', context=context)
    client = ActionClient(client_node, JointStream, '/sawyer/joint_stream')
    native = ActionClient(client_node, MotionCommand, '/sawyer/native_motion')
    publisher = client_node.create_publisher(
        StreamCommand, '/sawyer/stream_command', qos_profile_sensor_data
    )
    executor = MultiThreadedExecutor(num_threads=6, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    sending = threading.Event()
    closing = threading.Event()
    identifier = [uuid4()]

    def publish():
        sequence = 0
        while not closing.wait(0.01):
            if sending.is_set():
                sequence += 1
                message = StreamCommand(
                    session_id=identifier[0].hex,
                    sequence=sequence,
                    mode=2,
                    names=list(JOINT_NAMES),
                    velocity=[0.01] * 7,
                )
                message.header.stamp = client_node.get_clock().now().to_msg()
                publisher.publish(message)

    sender = threading.Thread(target=publish, daemon=True)
    sender.start()
    try:
        assert client.wait_for_server(timeout_sec=5)
        goal = JointStream.Goal(
            mode=2,
            duration=1.0,
            rate=100.0,
            max_velocity=0.1,
            max_acceleration=0.5,
            max_displacement=0.1,
            effort_fraction=0.05,
            position_speed=0.05,
            lease_timeout=0.1,
        )
        sending.set()
        handle = completed(
            client.send_goal_async(
                goal, goal_uuid=UUID(uuid=list(identifier[0].bytes))
            )
        )
        assert handle.accepted
        deadline = time.monotonic() + 2
        while not calls and time.monotonic() < deadline:
            time.sleep(0.01)
        assert calls
        sending.clear()
        result = completed(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_ABORTED
        assert 'lease expired' in result.result.detail
        count = len(calls)
        sending.set()
        time.sleep(0.15)
        assert len(calls) == count
        identifier[0] = uuid4()
        goal.duration = 0.15
        handle = completed(
            client.send_goal_async(
                goal, goal_uuid=UUID(uuid=list(identifier[0].bytes))
            )
        )
        assert handle.accepted
        assert (
            completed(handle.get_result_async()).status
            == GoalStatus.STATUS_SUCCEEDED
        )
        assert native.wait_for_server(timeout_sec=5)
        handle = completed(
            native.send_goal_async(MotionCommand.Goal(command='generate'))
        )
        assert handle.accepted
        result = completed(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_ABORTED
        assert result.result.error_id == 'CONTROLLER_NOT_FOLLOWING'
    finally:
        closing.set()
        sender.join(timeout=2)
        driver.release_read.set()
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
        server.destroy_node()
        client.destroy()
        native.destroy()
        client_node.destroy_node()
        context.shutdown()


def test_telemetry_keeps_partial_io_and_bounded_change_events():
    from collections import deque

    from irobot.src.robots.sawyer.ros2_telemetry import SawyerTelemetry

    telemetry = SawyerTelemetry.__new__(SawyerTelemetry)
    telemetry.io_states = {}
    telemetry.io_events = deque(maxlen=2)
    telemetry.dropped_io_events = 0

    def update(name, value):
        return telemetry.merge_io_state(
            'cuff/state',
            wire.IODeviceStatus(
                signals=[wire.IODataStatus(name=name, data=str([value]))]
            ),
        )

    update('button', 0)
    snapshot = update('wheel', 1)
    assert len(snapshot.signals) == 2
    assert snapshot.received_at == 0.0
    assert not telemetry.io_events
    for value in (1, 0, 1):
        update('button', value)
    assert telemetry.dropped_io_events == 1
    assert len(telemetry.io_events) == 2
    assert telemetry.io_events[-1]['previous_json'] == '[0]'
    assert telemetry.io_events[-1]['value_json'] == '[1]'


def test_telemetry_rejects_initial_and_repeated_camera_frames():
    from unittest.mock import MagicMock

    from irobot.src.robots.sawyer.ros2_telemetry import SawyerTelemetry

    telemetry = SawyerTelemetry.__new__(SawyerTelemetry)
    telemetry.node = MagicMock()
    telemetry.lock = threading.Lock()
    telemetry.received_counts, telemetry.camera_keys = {}, {}
    telemetry.pending = {}
    frame = wire.Image()
    frame.header.seq = 10
    telemetry.receive('head/image', frame)
    assert not telemetry.pending
    telemetry.receive('head/image', frame)
    assert not telemetry.pending
    frame = wire.Image()
    frame.header.seq = 11
    telemetry.receive('head/image', frame)
    assert telemetry.pending['head/image'][0].header.seq == 11


def test_ros2_port_settings_forward_category_and_report_acknowledgement():
    from unittest.mock import MagicMock

    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters

    from irobot.src.robots.sawyer.ros2_interfaces import ExtendedInterfaces

    interface = ExtendedInterfaces.__new__(ExtendedInterfaces)
    interface.node = MagicMock()
    interface.driver = MagicMock()
    interface.driver.devices.read_io.return_value = {
        'ports': {'analog': {'format': {'type': 'float'}}}
    }
    interface.driver.devices.write_io.return_value = {
        'acknowledged': True,
        'measured': {'analog': 0.25},
    }
    request = SetParameters.Request(
        parameters=[
            Parameter(
                name='analog',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=0.25
                ),
            )
        ]
    )
    callback = interface.io_callback('/io/robot/robot', category='ports')
    response = callback(request, SetParameters.Response())
    assert response.results[0].successful
    assert (
        interface.driver.devices.write_io.call_args.kwargs['category']
        == 'ports'
    )
    assert interface.driver.devices.write_io.call_args.args[1] == {
        'analog': 0.25
    }
    interface.node._finish.assert_called_once_with('succeeded')
