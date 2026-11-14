"""Direct XML-RPC and TCPROS connection to Sawyer's existing controller."""

from __future__ import annotations

import itertools
import logging
import os
import select
import socket
import socketserver
import struct
import threading
import time
import xmlrpc.client
from concurrent.futures import ThreadPoolExecutor
from contextlib import suppress
from copy import deepcopy
from dataclasses import replace
from functools import partial
from types import SimpleNamespace
from urllib.parse import urlsplit
from xmlrpc.server import SimpleXMLRPCServer

from .wire import MAX_MESSAGE_BYTES, Time, deserialize, serialize

LOGGER = logging.getLogger(__name__)
MAX_HEADER_BYTES = 65536
MAX_PENDING_MESSAGES = 256


def receive_exact(connection, count, deadline):
    result = bytearray()
    while len(result) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            message = 'TCPROS read timed out'
            raise TimeoutError(message)
        connection.settimeout(remaining)
        part = connection.recv(count - len(result))
        if not part:
            message = 'TCPROS peer disconnected'
            raise ConnectionError(message)
        result.extend(part)
    return bytes(result)


def receive_frame(connection, deadline, limit=MAX_MESSAGE_BYTES):
    size = struct.unpack('<I', receive_exact(connection, 4, deadline))[0]
    if size > limit:
        message = f'TCPROS frame exceeds {limit} bytes'
        raise ValueError(message)
    return receive_exact(connection, size, deadline)


def frame(payload):
    return struct.pack('<I', len(payload)) + payload


def send_header(connection, values):
    payload = b''.join(
        frame(f'{key}={value}'.encode()) for key, value in values.items()
    )
    if len(payload) > MAX_HEADER_BYTES:
        message = 'TCPROS header exceeds the byte limit'
        raise ValueError(message)
    connection.sendall(frame(payload))


def receive_header(connection, deadline):
    payload = receive_frame(connection, deadline, MAX_HEADER_BYTES)
    result, offset = {}, 0
    while offset < len(payload):
        if offset + 4 > len(payload):
            message = 'Truncated TCPROS header field'
            raise ValueError(message)
        size = struct.unpack_from('<I', payload, offset)[0]
        offset += 4
        if offset + size > len(payload):
            message = 'Truncated TCPROS header value'
            raise ValueError(message)
        key, separator, value = (
            payload[offset : offset + size].decode().partition('=')
        )
        if not separator or key in result:
            message = 'Invalid or duplicate TCPROS header field'
            raise ValueError(message)
        result[key] = value
        offset += size
    if 'error' in result:
        raise ConnectionError(result['error'])
    return result


class TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        return connection


def rpc(uri, method, *args, timeout):
    with xmlrpc.client.ServerProxy(
        uri, transport=TimeoutTransport(timeout), allow_none=True
    ) as remote:
        code, message, value = getattr(remote, method)(*args)
    if code != 1:
        message = f'{method}: {message}'
        raise ConnectionError(message)
    return value


class RpcServer(socketserver.ThreadingMixIn, SimpleXMLRPCServer):
    daemon_threads = True
    block_on_close = False


class Publisher:
    def __init__(self, transport, kind, topic):
        self.transport, self.kind, self.topic = transport, kind, topic
        self.clients = set()
        self.lock = threading.Lock()

    def get_subscription_count(self):
        with self.lock:
            return len(self.clients)

    def publish(self, message):
        if self.transport.closed.is_set():
            message = 'Sawyer transport is closed'
            raise ConnectionError(message)
        if message.ros_type != self.kind.ros_type:
            message = 'Message type does not match publisher'
            raise TypeError(message)
        payload = frame(serialize(message))
        with self.lock:
            clients = tuple(self.clients)
        for connection in clients:
            try:
                connection.settimeout(self.transport.write_timeout)
                connection.sendall(payload)
            except OSError as exc:
                self.remove(connection)
                message = f'Command delivery failed on {self.topic}'
                raise ConnectionError(message) from exc
        if not clients:
            message = f'No subscribers on {self.topic}'
            raise ConnectionError(message)

    def remove(self, connection):
        with self.lock:
            self.clients.discard(connection)
        close_socket(connection)

    def close(self):
        with self.lock:
            clients = tuple(self.clients)
        for connection in clients:
            self.remove(connection)


def close_socket(connection):
    # A peer may have already closed the socket.
    with suppress(OSError):
        connection.shutdown(socket.SHUT_RDWR)
    connection.close()


class Subscription:
    def __init__(self, transport, kind, topic, callback):
        self.transport, self.kind = transport, kind
        self.topic, self.callbacks = topic, [callback]
        self.workers = {}
        self.retained = {}
        self.lock = threading.Lock()

    def callback(self, message):
        for callback in tuple(self.callbacks):
            callback(message)

    def dispatch(self, callbacks, message):
        for callback in callbacks:
            if callback in self.callbacks:
                callback(message)

    def add_observer(self, callback):
        with self.lock:
            if callback in self.callbacks:
                return
            self.callbacks.append(callback)
            for key, message in self.retained.items():
                self.transport.enqueue(
                    (self.topic, 'observer', id(callback), key),
                    callback,
                    deepcopy(message),
                )

    def retain(self, key, message):
        """Keep feedback for late observers, preserving local receipt times."""
        received_at = message.received_at
        if self.kind.ros_type == 'intera_core_msgs/IODeviceStatus':
            key = key[:2]
            previous = self.retained.get(key)
            updates = {}
            for category in ('ports', 'signals'):
                values = {
                    item.name: item for item in getattr(previous, category, [])
                }
                for item in getattr(message, category):
                    item.received_at = message.received_at
                    values[item.name] = item
                updates[category] = list(values.values())
            message = replace(message, **updates)
        elif (
            self.kind.ros_type == 'tf2_msgs/TFMessage'
            and self.topic.endswith('tf_static')
        ):
            previous = self.retained.get(key)
            values = {
                (item.header.frame_id, item.child_frame_id): item
                for item in getattr(previous, 'transforms', [])
            }
            values.update(
                {
                    (item.header.frame_id, item.child_frame_id): item
                    for item in message.transforms
                }
            )
            message = replace(message, transforms=list(values.values()))
        message.received_at = received_at
        self.retained[key] = message
        if len(self.retained) > MAX_PENDING_MESSAGES:
            self.retained.pop(next(iter(self.retained)))

    def connected(self):
        with self.lock:
            return any(
                sock is not None and sock.fileno() >= 0
                for _, sock, _ in self.workers.values()
            )

    def update(self, publishers):
        with self.lock:
            for uri in set(self.workers) - set(publishers):
                stopped, connection, _thread = self.workers.pop(uri)
                stopped.set()
                if connection:
                    close_socket(connection)
            for uri in set(publishers) - set(self.workers):
                if self.transport.closed.is_set():
                    return
                stopped = threading.Event()
                thread = self.transport.start_thread(
                    self.receive, uri, stopped, start=False
                )
                self.workers[uri] = [stopped, None, thread]
                thread.start()

    def receive(self, uri, stopped):
        while not stopped.is_set() and not self.transport.closed.is_set():
            connection = None
            try:
                self.receive_stream(uri, stopped)
            except (
                OSError,
                ValueError,
                xmlrpc.client.Error,
                ConnectionError,
            ) as exc:
                if not stopped.is_set() and not self.transport.closed.is_set():
                    LOGGER.debug('%s: %s', self.topic, exc)
            finally:
                with self.lock:
                    worker = self.workers.get(uri)
                    if worker and worker[0] is stopped:
                        connection, worker[1] = worker[1], None
                if connection:
                    close_socket(connection)
            stopped.wait(self.transport.retry_interval)

    def receive_stream(self, uri, stopped):
        protocol, host, port = rpc(
            uri,
            'requestTopic',
            self.transport.name,
            self.topic,
            [['TCPROS']],
            timeout=self.transport.timeout,
        )
        if protocol != 'TCPROS':
            message = 'Publisher did not negotiate TCPROS'
            raise ValueError(message)
        connection = socket.create_connection(
            (host, port), timeout=self.transport.timeout
        )
        with self.lock:
            worker = self.workers.get(uri)
            if not worker or worker[0] is not stopped or stopped.is_set():
                connection.close()
                return
            worker[1] = connection
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        send_header(
            connection,
            {
                'callerid': self.transport.name,
                'topic': self.topic,
                'type': self.kind.ros_type,
                'md5sum': self.kind.md5sum,
                'tcp_nodelay': '1',
            },
        )
        header = receive_header(
            connection, time.monotonic() + self.transport.timeout
        )
        if header.get('md5sum') != self.kind.md5sum:
            message = f'Incompatible schema on {self.topic}'
            raise ValueError(message)
        if header.get('type') != self.kind.ros_type:
            message = f'Incompatible type on {self.topic}'
            raise ValueError(message)
        while not stopped.is_set() and not self.transport.closed.is_set():
            payload = receive_frame(
                connection, time.monotonic() + self.transport.idle_timeout
            )
            message = deserialize(self.kind.ros_type, payload)
            message.received_at = time.monotonic()
            key = (self.topic, uri)
            if self.kind.ros_type == 'sensor_msgs/JointState':
                # Arm/head subsets and separate fields must not overwrite
                # each other while the caller is between feedback polls.
                key += (
                    tuple(message.name),
                    bool(message.position),
                    bool(message.velocity),
                    bool(message.effort),
                )
            elif self.kind.ros_type == 'intera_core_msgs/IODeviceStatus':
                key += (next(self.transport.arrival_sequence),)
            elif hasattr(getattr(message, 'status', None), 'goal_id'):
                key += (message.status.goal_id.id,)
            with self.lock:
                self.retain(key, message)
                callbacks = tuple(self.callbacks)
                self.transport.enqueue(
                    key, partial(self.dispatch, callbacks), message
                )

    def close(self):
        self.update([])


class ServiceClient:
    def __init__(self, transport, kind, name):
        self.transport, self.kind, self.name = transport, kind, name

    def wait_for_service(self, timeout_sec):
        try:
            self.transport.master('lookupService', self.name)
        except (OSError, xmlrpc.client.Error, ConnectionError):
            self.transport.closed.wait(timeout_sec)
            return False
        return True

    def call_async(self, request):
        return self.transport.services.submit(self.call, request)

    def call(self, request):
        if request.ros_type != self.kind.Request.ros_type:
            message = 'Service request type mismatch'
            raise TypeError(message)
        uri = self.transport.master('lookupService', self.name)
        parsed = urlsplit(uri)
        if parsed.scheme != 'rosrpc' or not parsed.hostname or not parsed.port:
            message = 'Invalid ROS service URI'
            raise ValueError(message)
        deadline = time.monotonic() + self.transport.service_timeout
        with socket.create_connection(
            (parsed.hostname, parsed.port),
            timeout=self.transport.service_timeout,
        ) as connection:
            self.transport.track_socket(connection)
            try:
                send_header(
                    connection,
                    {
                        'callerid': self.transport.name,
                        'service': self.name,
                        'md5sum': self.kind.md5sum,
                        'persistent': '0',
                    },
                )
                header = receive_header(connection, deadline)
                if header.get('md5sum') != self.kind.md5sum:
                    message = f'Incompatible service {self.name}'
                    raise ValueError(message)
                connection.sendall(frame(serialize(request)))
                successful = receive_exact(connection, 1, deadline)
                payload = receive_frame(connection, deadline)
                if successful != b'\x01':
                    raise RuntimeError(
                        payload.decode('utf-8', errors='replace')
                    )
                return deserialize(self.kind.Response.ros_type, payload)
            finally:
                self.transport.untrack_socket(connection)


class DirectTransport:
    """Own connection resources; dispatch callbacks through spin_once."""

    def __init__(self, config):
        self.master_uri = config.ros_master_uri or os.environ.get(
            'ROS_MASTER_URI'
        )
        self.host = config.ros_ip or os.environ.get('ROS_IP')
        if not self.master_uri or not self.host:
            message = 'Set ros_master_uri/ros_ip or ROS_MASTER_URI/ROS_IP'
            raise ValueError(message)
        self.name = f'/{config.node_name}_{os.getpid()}_{id(self):x}'
        self.timeout = config.transport_timeout
        self.write_timeout = min(self.timeout, config.command_timeout / 4)
        self.service_timeout = config.motion.ik_timeout
        self.retry_interval = config.transport_retry_interval
        self.idle_timeout = config.transport_idle_timeout
        self.registration_interval = config.registration_refresh_interval
        self.master_available = True
        self.generation = 0
        self.master_pid = None
        self.command_topic = config.joint_command_topic
        self.conflicting_publishers = []
        self.registration_lock = threading.RLock()
        self.arrival_sequence = itertools.count()
        self.closed = threading.Event()
        self.condition = threading.Condition()
        self.pending, self.publishers, self.subscriptions = {}, {}, {}
        self.dropped_messages = 0
        self.threads, self.sockets = [], set()
        self.resource_lock = threading.Lock()
        self.services = ThreadPoolExecutor(max_workers=2)
        self.rpc_server = self.listener = None
        try:
            self.rpc_server = RpcServer(
                (self.host, 0), allow_none=True, logRequests=False
            )
            self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.listener.bind((self.host, 0))
            self.listener.listen(16)
            self.listener.settimeout(self.timeout)
            self.uri = (
                f'http://{self.host}:{self.rpc_server.server_address[1]}/'
            )
            for name, method in (
                ('publisherUpdate', self.publisher_update),
                ('requestTopic', self.request_topic),
                ('getPid', lambda _caller: [1, '', os.getpid()]),
                ('getMasterUri', lambda _caller: [1, '', self.master_uri]),
                ('shutdown', self.remote_shutdown),
                ('getPublications', self.get_publications),
                ('getSubscriptions', self.get_subscriptions),
                ('getBusInfo', self.get_bus_info),
            ):
                self.rpc_server.register_function(method, name)
            self.start_thread(
                self.rpc_server.serve_forever, poll_interval=0.05
            )
            self.start_thread(self.accept)
            self.start_thread(self.maintain_registration)
        except (OSError, RuntimeError):
            self.closed.set()
            if self.listener is not None:
                self.listener.close()
            if self.rpc_server is not None:
                if self.threads and self.threads[0].is_alive():
                    self.rpc_server.shutdown()
                self.rpc_server.server_close()
            self.services.shutdown(wait=False, cancel_futures=True)
            raise

    def master(self, method, *args):
        return rpc(
            self.master_uri, method, self.name, *args, timeout=self.timeout
        )

    def start_thread(self, target, *args, start=True, **kwargs):
        thread = threading.Thread(
            target=target, args=args, kwargs=kwargs, daemon=True
        )
        self.threads.append(thread)
        if start:
            thread.start()
        return thread

    def track_socket(self, connection):
        with self.resource_lock:
            if self.closed.is_set():
                message = 'Sawyer transport is closed'
                raise ConnectionError(message)
            self.sockets.add(connection)

    def untrack_socket(self, connection):
        with self.resource_lock:
            self.sockets.discard(connection)

    def create_subscription(self, kind, topic, callback):
        with self.registration_lock:
            if topic in self.subscriptions:
                message = f'Duplicate subscription: {topic}'
                raise ValueError(message)
            subscription = Subscription(self, kind, topic, callback)
            self.subscriptions[topic] = subscription
            publishers = self.master(
                'registerSubscriber', topic, kind.ros_type, self.uri
            )
            subscription.update(publishers)
            return subscription

    def create_publisher(self, kind, topic):
        with self.registration_lock:
            if topic in self.publishers:
                message = f'Duplicate publisher: {topic}'
                raise ValueError(message)
            publisher = Publisher(self, kind, topic)
            self.publishers[topic] = publisher
            self.master('registerPublisher', topic, kind.ros_type, self.uri)
            return publisher

    def observe(self, kind, topic, callback):
        """Share one wire subscription with additional in-process consumers."""
        with self.registration_lock:
            subscription = self.subscriptions.get(topic)
            if subscription is None:
                return self.create_subscription(kind, topic, callback)
            if subscription.kind.ros_type != kind.ros_type:
                message = f'Observer type mismatch: {topic}'
                raise TypeError(message)
            subscription.add_observer(callback)
            return subscription

    def enqueue(self, key, callback, message):
        with self.condition:
            self.pending[key] = (callback, message)
            if len(self.pending) > MAX_PENDING_MESSAGES:
                self.dropped_messages += 1
                self.pending.pop(next(iter(self.pending)))
            self.condition.notify()

    def publisher_for(self, kind, topic):
        with self.registration_lock:
            publisher = self.publishers.get(topic)
            if publisher is None:
                return self.create_publisher(kind, topic)
            if publisher.kind.ros_type != kind.ros_type:
                message = f'Publisher type mismatch: {topic}'
                raise TypeError(message)
            return publisher

    def remove_observer(self, topic, callback):
        with self.registration_lock:
            subscription = self.subscriptions[topic]
            subscription.callbacks.remove(callback)
            if not subscription.callbacks:
                del self.subscriptions[topic]
                subscription.close()
                self.master('unregisterSubscriber', topic, self.uri)

    def refresh_registration(self):
        """Restore graph registration; never publish or replay a command."""
        with self.registration_lock:
            if self.closed.is_set():
                return
            pid = self.master('getPid')
            known = False
            with suppress(ConnectionError):
                known = self.master('lookupNode', self.name) == self.uri
            changed = self.master_pid is not None and pid != self.master_pid
            if changed or not known or not self.master_available:
                if changed or (not known and self.master_pid is not None):
                    self.generation += 1
                for publisher in self.publishers.values():
                    if self.closed.is_set():
                        return
                    self.master(
                        'registerPublisher',
                        publisher.topic,
                        publisher.kind.ros_type,
                        self.uri,
                    )
                for subscription in self.subscriptions.values():
                    if self.closed.is_set():
                        return
                    publishers = self.master(
                        'registerSubscriber',
                        subscription.topic,
                        subscription.kind.ros_type,
                        self.uri,
                    )
                    subscription.update(publishers)
            self.master_pid, self.master_available = pid, True
            self.check_command_owner()

    def check_command_owner(self):
        """
        Best-effort graph check; ROS 1 provides no exclusive writer
        lease.
        """
        publishers, _, _ = self.master('getSystemState')
        self.conflicting_publishers = [
            name
            for topic, nodes in publishers
            if topic == self.command_topic
            for name in nodes
            if name != self.name
        ]

    def maintain_registration(self):
        while not self.closed.wait(self.registration_interval):
            try:
                self.refresh_registration()
            except (OSError, xmlrpc.client.Error) as exc:
                if self.master_available:
                    self.generation += 1
                self.master_available = False
                LOGGER.debug('Master registration unavailable: %s', exc)

    def get_publications(self, _caller):
        with self.registration_lock:
            return [
                1,
                '',
                [[p.topic, p.kind.ros_type] for p in self.publishers.values()],
            ]

    def get_subscriptions(self, _caller):
        with self.registration_lock:
            return [
                1,
                '',
                [
                    [s.topic, s.kind.ros_type]
                    for s in self.subscriptions.values()
                ],
            ]

    def get_bus_info(self, _caller):
        connections = []
        with self.registration_lock:
            for publisher in self.publishers.values():
                with publisher.lock:
                    for sock in publisher.clients:
                        try:
                            peer = str(sock.getpeername())
                        except OSError:
                            continue
                        connections.append(
                            [
                                sock.fileno(),
                                peer,
                                'o',
                                'TCPROS',
                                publisher.topic,
                                True,
                            ]
                        )
            for subscription in self.subscriptions.values():
                with subscription.lock:
                    connections.extend(
                        [
                            sock.fileno(),
                            uri,
                            'i',
                            'TCPROS',
                            subscription.topic,
                            True,
                        ]
                        for uri, (_, sock, _) in subscription.workers.items()
                        if sock is not None and sock.fileno() >= 0
                    )
        return [1, '', connections]

    def create_client(self, kind, name):
        return ServiceClient(self, kind, name)

    def destroy_client(self, _client):
        # Service connections are scoped to individual requests.
        return None

    def publisher_update(self, _caller, topic, publishers):
        subscription = self.subscriptions.get(topic)
        if subscription:
            subscription.update(publishers)
        return [1, '', 0]

    def request_topic(self, _caller, topic, protocols):
        if topic not in self.publishers:
            return [0, 'Unknown topic', []]
        if not any(item and item[0] == 'TCPROS' for item in protocols):
            return [0, 'TCPROS required', []]
        return [1, '', ['TCPROS', self.host, self.listener.getsockname()[1]]]

    def remote_shutdown(self, _caller, _reason):
        self.start_thread(self.destroy_node)
        return [1, '', 0]

    def accept(self):
        while not self.closed.is_set():
            try:
                connection, _address = self.listener.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            try:
                self.track_socket(connection)
            except ConnectionError:
                connection.close()
                return
            self.start_thread(self.serve_subscriber, connection)

    def serve_subscriber(self, connection):
        publisher = None
        try:
            header = receive_header(
                connection, time.monotonic() + self.timeout
            )
            publisher = self.publishers.get(header.get('topic'))
            if publisher is None:
                message = 'Unknown command topic'
                raise ValueError(message)  # noqa: TRY301
            if header.get('md5sum') != publisher.kind.md5sum:
                message = 'Command subscriber schema mismatch'
                raise ValueError(message)  # noqa: TRY301
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            send_header(
                connection,
                {
                    'callerid': self.name,
                    'type': publisher.kind.ros_type,
                    'md5sum': publisher.kind.md5sum,
                    'latching': '0',
                    'message_definition': publisher.kind.definition,
                },
            )
            with publisher.lock:
                publisher.clients.add(connection)
            while not self.closed.is_set():
                readable, _, _ = select.select(
                    [connection], [], [], self.timeout
                )
                if not readable:
                    continue
                if not connection.recv(1):
                    return
        except (OSError, ValueError, ConnectionError) as exc:
            if not self.closed.is_set():
                LOGGER.debug('Command connection closed: %s', exc)
        finally:
            if publisher:
                publisher.remove(connection)
            else:
                close_socket(connection)
            self.untrack_socket(connection)

    def spin_once(self, timeout_sec=0.02):
        with self.condition:
            if not self.pending and not self.closed.is_set():
                self.condition.wait(timeout_sec)
            pending, self.pending = self.pending, {}
        if self.closed.is_set():
            message = 'Sawyer transport is closed'
            raise ConnectionError(message)
        for callback, message in pending.values():
            callback(message)

    @staticmethod
    def get_logger():
        return LOGGER

    @staticmethod
    def get_clock():
        def stamp():
            seconds, nanoseconds = divmod(time.time_ns(), 1_000_000_000)
            return Time(sec=seconds, nanosec=nanoseconds)

        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=stamp))

    def destroy_node(self):
        if self.closed.is_set():
            return
        self.closed.set()
        with self.condition:
            self.pending.clear()
            self.condition.notify_all()
        for subscription in self.subscriptions.values():
            subscription.close()
        for publisher in self.publishers.values():
            publisher.close()
        self.listener.close()
        with self.resource_lock:
            sockets = tuple(self.sockets)
        for connection in sockets:
            close_socket(connection)
        self.rpc_server.shutdown()
        self.rpc_server.server_close()
        with self.registration_lock:
            for method, endpoints in (
                ('unregisterSubscriber', self.subscriptions),
                ('unregisterPublisher', self.publishers),
            ):
                for topic in endpoints:
                    try:
                        self.master(method, topic, self.uri)
                    except (OSError, xmlrpc.client.Error) as exc:
                        LOGGER.warning(
                            'Could not unregister %s: %s', topic, exc
                        )
        self.services.shutdown(wait=True, cancel_futures=True)
        for thread in tuple(self.threads):
            if thread is not threading.current_thread() and thread.ident:
                thread.join(timeout=self.timeout + 0.1)
