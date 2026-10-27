"""Independent ROS 1 image fallback and bridge liveness checks.

This small TCPROS publisher is used only to restore the stock SDK image when
the ROS 2 bridge is no longer available. It publishes sensor_msgs/Image only;
no robot command or enable interfaces are used. No ROS 1 Python installation
or changes to the robot are required.
"""

from dataclasses import dataclass, field
import os
import socket
import socketserver
import struct
import threading
import time
import xmlrpc.client
from xmlrpc.server import SimpleXMLRPCServer


TOPIC = '/robot/head_display'
IMAGE_TYPE = 'sensor_msgs/Image'
IMAGE_MD5 = '060021388200f6f0f447d0fcd9c64743'


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = .75
        return connection


def proxy(uri):
    return xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport(), allow_none=True)


def string(value):
    data = value.encode() if isinstance(value, str) else bytes(value)
    return struct.pack('<I', len(data)) + data


def encode_header(fields):
    data = b''.join(string(f'{key}={value}') for key, value in fields.items())
    return struct.pack('<I', len(data)) + data


def receive_exact(connection, count):
    data = bytearray()
    while len(data) < count:
        part = connection.recv(count-len(data))
        if not part:
            raise ConnectionError('TCPROS connection closed')
        data.extend(part)
    return bytes(data)


def receive_header(connection):
    count = struct.unpack('<I', receive_exact(connection, 4))[0]
    if count > 65536:
        raise ValueError('TCPROS header is too large')
    data, result, offset = receive_exact(connection, count), {}, 0
    while offset < len(data):
        if offset+4 > len(data):
            raise ValueError('Truncated TCPROS header')
        length = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        if offset+length > len(data):
            raise ValueError('Truncated TCPROS field')
        key, value = data[offset:offset+length].decode().split('=', 1)
        result[key] = value
        offset += length
    return result


def serialize_image(image, sequence=0):
    now = time.time_ns()
    body = (struct.pack('<III', sequence & 0xffffffff, now//1_000_000_000, now % 1_000_000_000)
            + string(image.header.frame_id)
            + struct.pack('<II', image.height, image.width) + string(image.encoding)
            + struct.pack('<BI', image.is_bigendian, image.step) + string(bytes(image.data)))
    return struct.pack('<I', len(body)) + body


class ThreadedRPC(socketserver.ThreadingMixIn, SimpleXMLRPCServer):
    daemon_threads = True
    block_on_close = False


@dataclass(eq=False)
class Client:
    socket: object
    lock: object = field(default_factory=threading.Lock)


class NativeImagePublisher:
    def __init__(self, master_uri=None, host=None):
        self.master_uri = master_uri or os.environ.get('ROS_MASTER_URI')
        if not self.master_uri:
            raise ValueError('ROS_MASTER_URI is unset; source scripts/env.sh')
        self.host = host or os.environ.get('ROS_IP') or os.environ.get('ROS_HOSTNAME') or socket.gethostbyname(socket.gethostname())
        self.name = f'/sawyer_display_restore_{os.getpid()}'
        self.stopping, self.connected = threading.Event(), threading.Event()
        self.lock, self.clients, self.last_payload = threading.Lock(), set(), None
        self.sequence, self.registered = 0, False
        self.rpc = ThreadedRPC(('0.0.0.0', 0), allow_none=True, logRequests=False)
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(('0.0.0.0', 0))
        self.listener.listen(4)
        self.listener.settimeout(.2)
        self.uri = f'http://{self.host}:{self.rpc.server_address[1]}/'
        self.rpc.register_function(self.request_topic, 'requestTopic')
        self.rpc.register_function(lambda caller: [1, '', os.getpid()], 'getPid')
        self.rpc.register_function(lambda caller: [1, '', self.uri], 'getUri')
        self.rpc.register_function(lambda caller: [1, '', [[TOPIC, IMAGE_TYPE]]], 'getPublications')
        self.rpc.register_function(lambda caller: [1, '', []], 'getSubscriptions')
        self.rpc.register_function(lambda caller: [1, '', []], 'getBusInfo')
        self.rpc.register_function(lambda *args: [1, '', 0], 'publisherUpdate')
        self.rpc_thread = threading.Thread(target=self.rpc.serve_forever, kwargs={'poll_interval': .05}, daemon=True)
        self.accept_thread = threading.Thread(target=self.accept, daemon=True)
        self.rpc_thread.start()
        self.accept_thread.start()
        try:
            with proxy(self.master_uri) as master:
                code, message, _ = master.registerPublisher(self.name, TOPIC, IMAGE_TYPE, self.uri)
                if code != 1:
                    raise RuntimeError(f'ROS 1 display registration failed: {message}')
            self.registered = True
        except BaseException:
            self.close()
            raise

    def request_topic(self, caller, topic, protocols):
        if topic != TOPIC or not any(protocol and protocol[0] == 'TCPROS' for protocol in protocols):
            return [0, 'Only the head-display TCPROS topic is supported', []]
        return [1, '', ['TCPROS', self.host, self.listener.getsockname()[1]]]

    def accept(self):
        while not self.stopping.is_set():
            try:
                connection, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self.serve_client, args=(connection,), daemon=True).start()

    def serve_client(self, connection):
        client = Client(connection)
        try:
            connection.settimeout(1.0)
            header = receive_header(connection)
            if header.get('md5sum') not in (IMAGE_MD5, '*') or header.get('topic') != TOPIC:
                connection.sendall(encode_header({'error': 'Head-display Image type/topic mismatch'}))
                return
            connection.sendall(encode_header({'callerid': self.name, 'type': IMAGE_TYPE,
                                              'md5sum': IMAGE_MD5, 'latching': '1'}))
            with client.lock:
                with self.lock:
                    self.clients.add(client)
                    last = self.last_payload
                if last is not None:
                    connection.sendall(last)
            self.connected.set()
            while not self.stopping.is_set():
                try:
                    if not connection.recv(1):
                        return
                except socket.timeout:
                    continue
        except (OSError, ValueError, ConnectionError):
            pass
        finally:
            with self.lock:
                self.clients.discard(client)
            connection.close()

    def publish(self, image, wait=3.0):
        payload = serialize_image(image, self.sequence)
        self.sequence += 1
        with self.lock:
            self.last_payload = payload
        deadline = time.monotonic()+wait
        while True:
            with self.lock:
                clients = list(self.clients)
            if clients or time.monotonic() >= deadline:
                break
            self.connected.wait(min(.1, max(0, deadline-time.monotonic())))
            self.connected.clear()
        delivered = 0
        for client in clients:
            try:
                with client.lock:
                    client.socket.sendall(payload)
                delivered += 1
            except OSError:
                with self.lock:
                    self.clients.discard(client)
                client.socket.close()
        return delivered

    def close(self):
        self.stopping.set()
        if self.registered:
            try:
                with proxy(self.master_uri) as master:
                    master.unregisterPublisher(self.name, TOPIC, self.uri)
            except Exception:
                pass
            self.registered = False
        self.listener.close()
        with self.lock:
            clients, self.clients = list(self.clients), set()
        for client in clients:
            client.socket.close()
        self.rpc.shutdown()
        self.rpc.server_close()
        self.rpc_thread.join(timeout=1)
        self.accept_thread.join(timeout=1)


class BridgeProbe:
    """Latch one bridge generation and detect exit, including an unclean kill."""

    def __init__(self, master_uri, name='/sawyer_bridge'):
        self.master_uri, self.name = master_uri, name
        self.status = (False, False, 'Waiting for bridge')  # seen, alive, detail
        self.generation = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def check(self):
        with proxy(self.master_uri) as master:
            code, message, uri = master.lookupNode('/sawyer_display_probe', self.name)
        if code != 1:
            raise ConnectionError(message)
        with proxy(uri) as node:
            code, message, pid = node.getPid('/sawyer_display_probe')
        if code != 1:
            raise ConnectionError(message)
        generation = uri, pid
        if self.generation is not None and generation != self.generation:
            raise ConnectionError('Bridge process was replaced')
        self.generation = generation

    def run(self):
        failures = 0
        while not self.stopping.is_set():
            try:
                self.check()
                failures = 0
                self.status = (True, True, '')
            except Exception as exc:
                failures += 1
                if self.generation is not None and failures >= 3:
                    self.status = (True, False, str(exc))
                    return
                if self.generation is None:
                    self.status = (False, False, str(exc))
            self.stopping.wait(.25)

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=2)
