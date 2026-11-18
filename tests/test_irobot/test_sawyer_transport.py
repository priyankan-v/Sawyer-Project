"""Local TCPROS peers exercise actual sockets; no robot or ROS runtime."""

# Numeric values are explicit peer fixture expectations.
# ruff: noqa: PLR2004

from __future__ import annotations

import socket
import socketserver
import struct
import threading
import time
from contextlib import contextmanager
from dataclasses import replace
from xmlrpc.server import SimpleXMLRPCServer

import pytest

from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.config import JOINT_NAMES
from irobot.src.robots.sawyer.core import wire
from irobot.src.robots.sawyer.core.base import (
    JointFeedback,
    _ArmSession,
)
from irobot.src.robots.sawyer.core.transport import (
    DirectTransport,
    frame,
    receive_exact,
    receive_frame,
    receive_header,
    rpc,
    send_header,
)


class RpcServer(socketserver.ThreadingMixIn, SimpleXMLRPCServer):
    daemon_threads = True


@contextmanager
def rpc_server():
    server = RpcServer(('127.0.0.1', 0), logRequests=False)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server, f'http://127.0.0.1:{server.server_address[1]}/'
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)


def config(master_uri):
    return SawyerConfig(
        ros_master_uri=master_uri,
        ros_ip='127.0.0.1',
        transport_timeout=0.5,
        transport_idle_timeout=0.5,
        connect_timeout=2,
        enable_gripper=False,
        neutral_joints=(0.2,) * 7,
    )


def wait_until(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            pytest.fail('Local peer did not reach expected state')
        time.sleep(0.005)


def test_publisher_negotiates_and_never_replays_commands():
    calls = []
    with rpc_server() as (master, uri):
        master.register_function(
            lambda *_args: calls.append(_args) or [1, '', []],
            'registerPublisher',
        )
        master.register_function(
            lambda *_args: [1, '', 1], 'unregisterPublisher'
        )
        transport = DirectTransport(config(uri))
        try:
            publisher = transport.create_publisher(
                wire.JointCommand, '/command'
            )
            with pytest.raises(ConnectionError, match='No subscribers'):
                publisher.publish(wire.JointCommand())
            endpoint = rpc(
                transport.uri,
                'requestTopic',
                '/peer',
                '/command',
                [['TCPROS']],
                timeout=1,
            )
            with socket.create_connection(
                tuple(endpoint[1:]), timeout=1
            ) as peer:
                send_header(
                    peer,
                    {
                        'callerid': '/peer',
                        'topic': '/command',
                        'md5sum': wire.JointCommand.md5sum,
                    },
                )
                header = receive_header(peer, time.monotonic() + 1)
                assert header['latching'] == '0'
                wait_until(lambda: publisher.get_subscription_count() == 1)
                peer.settimeout(0.02)
                with pytest.raises(socket.timeout):
                    peer.recv(1)
                message = wire.JointCommand(mode=2, velocity=[0.1] * 7)
                publisher.publish(message)
                payload = receive_frame(peer, time.monotonic() + 1)
                assert wire.deserialize(message.ros_type, payload) == message
            wait_until(lambda: publisher.get_subscription_count() == 0)
        finally:
            transport.destroy_node()
        assert len(calls) == 1
        assert all(not thread.is_alive() for thread in transport.threads)


def test_bad_subscriber_hash_is_rejected():
    with rpc_server() as (master, uri):
        master.register_function(
            lambda *_args: [1, '', []], 'registerPublisher'
        )
        master.register_function(
            lambda *_args: [1, '', 1], 'unregisterPublisher'
        )
        transport = DirectTransport(config(uri))
        try:
            publisher = transport.create_publisher(wire.Float64, '/command')
            with socket.create_connection(
                ('127.0.0.1', transport.listener.getsockname()[1]), timeout=1
            ) as peer:
                send_header(peer, {'topic': '/command', 'md5sum': 'wrong'})
                assert peer.recv(1) == b''
            assert publisher.get_subscription_count() == 0
        finally:
            transport.destroy_node()


class PeerHandler(socketserver.BaseRequestHandler):
    def handle(self):
        deadline = time.monotonic() + 2
        header = receive_header(self.request, deadline)
        if 'service' in header:
            kind = wire.SolvePositionFK
            send_header(self.request, {'md5sum': kind.md5sum})
            payload = receive_frame(self.request, deadline)
            self.server.requests.append(
                wire.deserialize(kind.Request.ros_type, payload)
            )
            if self.server.service_error:
                self.request.sendall(
                    b'\x00' + frame(self.server.service_error.encode())
                )
                return
            response = kind.Response(is_valid=[True], in_collision=[False])
            # Fragment the reply to exercise exact-length socket reads.
            data = b'\x01' + frame(wire.serialize(response))
            for byte in data:
                self.request.sendall(bytes([byte]))
            return
        messages = self.server.messages[header['topic']]
        self.server.topic_connections.append(header['topic'])
        if not isinstance(messages, list):
            messages = [messages]
        message = messages[0]
        send_header(
            self.request,
            {
                'type': message.ros_type,
                'md5sum': message.md5sum,
                'callerid': '/local_fixture',
                'latching': '1',
            },
        )
        index = 0
        while not self.server.stopped.is_set():
            try:
                message = messages[index % len(messages)]
                self.request.sendall(frame(wire.serialize(message)))
                index += 1
            except OSError:
                return
            self.server.stopped.wait(self.server.publish_interval)


class PeerServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    daemon_threads = True


@contextmanager
def robot_peer():
    with rpc_server() as (master, uri):
        peer = PeerServer(('127.0.0.1', 0), PeerHandler)
        peer.messages, peer.requests = {}, []
        peer.topic_connections = []
        peer.publish_interval = 0.02
        peer.service_error = None
        peer.stopped = threading.Event()
        thread = threading.Thread(target=peer.serve_forever, daemon=True)
        thread.start()
        master.register_function(
            lambda *_args: [1, '', [uri]], 'registerSubscriber'
        )
        master.register_function(
            lambda *_args: [1, '', []], 'registerPublisher'
        )
        for method in ('unregisterSubscriber', 'unregisterPublisher'):
            master.register_function(lambda *_args: [1, '', 1], method)
        master.register_function(
            lambda *_args: [1, '', ['TCPROS', *peer.server_address]],
            'requestTopic',
        )
        master.register_function(
            lambda *_args: [
                1,
                '',
                f'rosrpc://127.0.0.1:{peer.server_address[1]}',
            ],
            'lookupService',
        )
        try:
            yield peer, config(uri)
        finally:
            peer.stopped.set()
            peer.shutdown()
            peer.server_close()
            thread.join(timeout=1)


def test_quiet_subscription_stays_connected_without_refreshing_feedback():
    with robot_peer() as (peer, settings):
        peer.publish_interval = 0.3
        peer.messages['/quiet'] = wire.Bool(data=True)
        transport = DirectTransport(
            replace(settings, transport_idle_timeout=0.05)
        )
        received = []
        try:
            subscription = transport.create_subscription(
                wire.Bool, '/quiet', received.append
            )
            wait_until(lambda: bool(transport.pending))
            transport.spin_once(timeout_sec=0)
            first_receipt = received[-1].received_at
            time.sleep(0.12)
            assert subscription.connected()
            assert received[-1].received_at == first_receipt
            wait_until(lambda: bool(transport.pending))
            transport.spin_once(timeout_sec=0)
            assert received[-1].received_at > first_receipt
            assert peer.topic_connections == ['/quiet']
        finally:
            transport.destroy_node()
        assert all(not thread.is_alive() for thread in transport.threads)


def test_direct_controller_connects_and_previews_neutral_without_ros():
    with robot_peer() as (peer, settings):
        state = wire.EndpointState(valid=True)
        state.pose.orientation.w = 1.0
        names = list(JOINT_NAMES)
        velocity_limits = (0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4)
        peer.messages = {
            settings.joint_state_topic: wire.JointState(
                name=names, position=[0.0] * 7, velocity=[0.0] * 7
            ),
            settings.robot_state_topic: wire.RobotAssemblyState(),
            settings.collision_topic: wire.CollisionDetectionState(),
            settings.joint_limits_topic: wire.JointLimits(
                joint_names=names,
                position_lower=[-3.0] * 7,
                position_upper=[3.0] * 7,
                velocity=list(velocity_limits),
                accel=[1.0] * 7,
            ),
            settings.endpoint_topic: state,
            settings.tip_states_topic: wire.EndpointStates(
                names=['right_hand'], states=[state]
            ),
        }
        with SawyerController(settings) as robot:
            assert robot.joint_positions == (0.0,) * 7
            assert robot.endpoint_pose.frame == 'base'
            assert not robot.get_status()['enabled']
            preview = robot.move_to_neutral(duration=2)
            assert preview.execution is None
            assert preview.plan.end == (0.2,) * 7
            assert preview.plan.max_velocity == velocity_limits
            assert not peer.requests
        assert not robot.is_connected


def test_direct_service_response_and_fragmented_reads():
    with robot_peer() as (peer, settings):
        transport = DirectTransport(settings)
        try:
            client = transport.create_client(wire.SolvePositionFK, '/fk')
            response = client.call_async(
                wire.SolvePositionFK.Request()
            ).result(2)
            assert response.is_valid == [True]
            assert response.in_collision == [False]
            assert len(peer.requests) == 1
        finally:
            transport.destroy_node()


def test_service_failure_is_not_reported_as_success():
    with robot_peer() as (peer, settings):
        peer.service_error = 'No valid configuration'
        transport = DirectTransport(settings)
        try:
            client = transport.create_client(wire.SolvePositionFK, '/fk')
            with pytest.raises(RuntimeError, match='No valid configuration'):
                client.call_async(wire.SolvePositionFK.Request()).result(2)
        finally:
            transport.destroy_node()


def test_joint_subsets_and_fields_survive_delayed_polling():
    with robot_peer() as (peer, settings):
        peer.messages['/joints'] = [
            wire.JointState(name=['right_j0'], position=[0.5]),
            wire.JointState(name=['right_j0'], velocity=[0.1]),
            wire.JointState(name=['head_pan'], position=[0.3]),
        ]
        received = []
        transport = DirectTransport(settings)
        try:
            transport.create_subscription(
                wire.JointState, '/joints', received.append
            )
            wait_until(
                lambda: len(transport.pending) == len(peer.messages['/joints'])
            )
            transport.spin_once(timeout_sec=0)
            arm = [
                message for message in received if message.name == ['right_j0']
            ]
            assert any(message.position == [0.5] for message in arm)
            assert any(message.velocity == [0.1] for message in arm)
        finally:
            transport.destroy_node()


def test_delayed_callback_preserves_receive_time():
    session = _ArmSession.__new__(_ArmSession)
    session.feedback = JointFeedback(JOINT_NAMES)
    message = wire.JointState(
        name=list(JOINT_NAMES), position=[0.0] * 7, velocity=[0.0] * 7
    )
    message.received_at = time.monotonic() - 1
    session.on_joints(message)
    assert session.joints_at == message.received_at
    assert session.feedback.issues(time.monotonic(), 0.2)


def test_frame_limits_before_payload_allocation():
    left, right = socket.socketpair()
    try:
        left.sendall(struct.pack('<I', 0xFFFFFFFF))
        with pytest.raises(ValueError, match='exceeds'):
            receive_frame(right, time.monotonic() + 1)
    finally:
        left.close()
        right.close()


def test_mid_frame_disconnect_is_reported():
    left, right = socket.socketpair()
    left.sendall(b'\x00')
    left.close()
    try:
        with pytest.raises(ConnectionError, match='disconnected'):
            receive_exact(right, 4, time.monotonic() + 1)
    finally:
        right.close()


def test_partial_transport_startup_closes_bound_server(monkeypatch):
    original = socket.socket
    sockets = []

    def fail_listener(*args, **kwargs):
        if sockets:
            message = 'No socket available'
            raise OSError(message)
        connection = original(*args, **kwargs)
        sockets.append(connection)
        return connection

    monkeypatch.setattr(socket, 'socket', fail_listener)
    with pytest.raises(OSError, match='No socket available'):
        DirectTransport(config('http://127.0.0.1:11311'))
    assert all(connection.fileno() == -1 for connection in sockets)


def test_master_restart_restores_registration_without_command_replay():
    state = {'pid': 1, 'uri': '', 'registrations': 0, 'available': True}
    with rpc_server() as (master, uri):

        def get_pid(*_args):
            if not state['available']:
                msg = 'Fixture master unavailable'
                raise OSError(msg)
            return [1, '', state['pid']]

        def register(*args):
            state['uri'] = args[-1]
            state['registrations'] += 1
            return [1, '', []]

        master.register_function(get_pid, 'getPid')
        master.register_function(register, 'registerPublisher')
        master.register_function(
            lambda *_args: [1, '', state['uri']], 'lookupNode'
        )
        master.register_function(
            lambda *_args: [1, '', [[], [], []]], 'getSystemState'
        )
        master.register_function(
            lambda *_args: [1, '', 1], 'unregisterPublisher'
        )
        transport = DirectTransport(
            replace(config(uri), registration_refresh_interval=0.03)
        )
        try:
            transport.create_publisher(wire.Float64, '/command')
            with socket.create_connection(
                ('127.0.0.1', transport.listener.getsockname()[1]), timeout=1
            ) as peer:
                send_header(
                    peer,
                    {
                        'callerid': '/peer',
                        'topic': '/command',
                        'md5sum': wire.Float64.md5sum,
                    },
                )
                receive_header(peer, time.monotonic() + 1)
                wait_until(lambda: transport.master_pid == 1)
                state.update(pid=2, uri='')
                wait_until(lambda: state['registrations'] == 2)
                assert transport.generation >= 1
                state['available'] = False
                wait_until(lambda: not transport.master_available)
                generation = transport.generation
                state['available'] = True
                wait_until(lambda: transport.master_available)
                assert transport.generation == generation
                assert state['registrations'] == 3
                peer.settimeout(0.05)
                with pytest.raises(socket.timeout):
                    peer.recv(1)
                publications = rpc(
                    transport.uri, 'getPublications', '/peer', timeout=1
                )
                assert publications == [['/command', 'std_msgs/Float64']]
                assert rpc(transport.uri, 'getBusInfo', '/peer', timeout=1)
        finally:
            transport.destroy_node()


def test_shared_observers_and_late_publisher_connection():
    with robot_peer() as (peer, settings):
        peer.messages['/late'] = wire.Float64(data=3.0)
        transport = DirectTransport(settings)
        first, second = [], []
        try:
            one = transport.observe(wire.Float64, '/late', first.append)
            two = transport.observe(wire.Float64, '/late', second.append)
            assert one is two
            deadline = time.monotonic() + 2
            while not first and time.monotonic() < deadline:
                transport.spin_once(0.02)
            assert first
            assert second
            transport.remove_observer('/late', first.append)
            assert second[-1].data == 3.0
            assert len(one.callbacks) == 1
            transport.remove_observer('/late', second.append)
            assert '/late' not in transport.subscriptions
        finally:
            transport.destroy_node()


def test_late_io_observer_receives_merged_retained_state_without_new_packets():
    from unittest.mock import MagicMock

    from irobot.src.robots.sawyer.core.transport import Subscription

    transport = MagicMock()
    received = []
    subscription = Subscription(
        transport, wire.IODeviceStatus, '/io/test/state', lambda _value: None
    )
    for index, name in enumerate(('button', 'wheel'), start=1):
        message = wire.IODeviceStatus(
            signals=[wire.IODataStatus(name=name, data=f'[{index}]')]
        )
        message.received_at = float(index)
        subscription.retain(('/io/test/state', 'peer', name), message)
    subscription.add_observer(received.append)
    assert transport.enqueue.call_count == 1
    _, callback, message = transport.enqueue.call_args.args
    callback(message)
    assert {item.name: item.data for item in received[0].signals} == {
        'button': '[1]',
        'wheel': '[2]',
    }
    assert {item.name: item.received_at for item in message.signals} == {
        'button': 1.0,
        'wheel': 2.0,
    }
    assert message.received_at == 2.0
