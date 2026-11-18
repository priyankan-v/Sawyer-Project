"""Typed, acknowledged Intera I/O using published device metadata."""

from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from dataclasses import asdict, dataclass

from ..communication import wire

MAX_EVENTS = 256


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
        result[signal.name] = Signal(
            data[0],
            formatting['type'],
            formatting.get('role', ''),
            signal.status.tag,
        )
    return result


def signal_value(signals, name, kind=None):
    if name not in signals:
        msg = (
            f'Missing signal {name}; this utility requires the standard '
            f'Intera electric gripper/cuff'
        )
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
        isinstance(signal.value, bool)
        or not isinstance(signal.value, int | float)
        or not math.isfinite(signal.value)
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


def validate_value(value, formatting, configuration):
    """Validate scalar/vector JSON values and the device's published ranges."""
    dimensions = formatting.get('dimensions', [1])
    count = math.prod(dimensions)
    values = [value] if dimensions == [1] else value
    if not isinstance(values, list) or len(values) != count:
        msg = 'IO value does not match published dimensions'
        raise ValueError(msg)
    kind = formatting['type']
    for item in values:
        valid = {
            'bool': type(item) is bool,
            'int': type(item) is int,
            'float': type(item) in (int, float),
            'string': isinstance(item, str),
            'object': isinstance(item, dict),
            'array': isinstance(item, list),
        }.get(kind, False)
        if not valid:
            msg = f'IO value must have type {kind}'
            raise ValueError(msg)
        if kind in ('float', 'int') and not math.isfinite(item):
            msg = 'IO numeric values must be finite'
            raise ValueError(msg)
    ranges = configuration.get('params', {}).get('range', {})
    for bound, compare in (
        ('min', lambda a, b: a < b),
        ('max', lambda a, b: a > b),
    ):
        limits = ranges.get(bound)
        if limits is None and bound in formatting:
            limits = [formatting[bound]] * count
        if limits is not None:
            if len(limits) != count:
                msg = 'Malformed published IO range'
                raise ValueError(msg)
            if any(compare(a, b) for a, b in zip(values, limits, strict=True)):
                msg = f'IO value exceeds published {bound}'
                raise ValueError(msg)
    json.dumps(values, allow_nan=False)
    return {'format': {'type': kind}, 'data': values}


class IODevice:
    """
    State/configuration cache; output writes need matching acknowledgements.

    Change-only snapshots report both age and connection state. They do not
    pretend that an unchanged signal is a recent independent sensor sample.
    """

    def __init__(self, node, namespace, timeout, *, writable_inputs=()):
        if not namespace.startswith('/io/') or '..' in namespace.split('/'):
            msg = 'Use an absolute published /io/ device namespace'
            raise ValueError(msg)
        self.node, self.namespace, self.timeout = node, namespace, timeout
        self.writable_inputs = frozenset(writable_inputs)
        self.state = self.configuration = None
        self.signals, self.ports = {}, {}
        self.events = []
        self.dropped_events = 0
        self.callbacks = {}
        self.callback_sequence = 0
        self.subscription = node.observe(
            wire.IODeviceStatus, namespace + '/state', self.on_state
        )
        node.observe(
            wire.IODeviceConfiguration, namespace + '/config', self.on_config
        )
        self.publisher = None

    def on_config(self, message):
        self.configuration = message

    def on_state(self, message):
        for category, cache in (
            ('signals', self.signals),
            ('ports', self.ports),
        ):
            for item in getattr(message, category):
                data = json.loads(item.data)
                formatting = json.loads(item.format)
                if not isinstance(data, list):
                    msg = 'IO data must be a JSON array'
                    raise TypeError(msg)
                if not data:
                    msg = 'IO data array is empty'
                    raise ValueError(msg)
                value = (
                    data[0]
                    if formatting.get('dimensions', [1]) == [1]
                    else data
                )
                previous = cache.get(item.name)
                cache[item.name] = {
                    'value': value,
                    'format': formatting,
                    'status': item.status.tag,
                    'received_at': item.received_at or message.received_at,
                }
                if previous is not None and previous['value'] != value:
                    event = {
                        'category': category,
                        'name': item.name,
                        'previous': previous['value'],
                        'value': value,
                        'received_at': message.received_at,
                    }
                    self.events.append(event)
                    if len(self.events) > MAX_EVENTS:
                        self.dropped_events += 1
                    self.events = self.events[-MAX_EVENTS:]
                    for signal, callback in tuple(self.callbacks.values()):
                        if signal == item.name:
                            callback(deepcopy(event))
        self.state = message

    def wait(self, predicate, description, cancel_event=None):
        deadline = time.monotonic() + self.timeout
        while not predicate():
            if cancel_event is not None and cancel_event.is_set():
                msg = 'IO operation cancelled'
                raise RuntimeError(msg)
            if time.monotonic() >= deadline:
                raise TimeoutError(description)
            self.node.spin_once(timeout_sec=0.01)

    def wait_ready(self):
        self.wait(
            lambda: self.state is not None and self.configuration is not None,
            f'IO state/configuration unavailable: {self.namespace}',
        )

    def snapshot(self):
        self.wait_ready()
        return {
            'namespace': self.namespace,
            'device_status': self.state.device.status.tag,
            'connected': self.subscription.connected(),
            'last_message_age_s': time.monotonic() - self.state.received_at,
            'dropped_events': self.dropped_events,
            'transport_dropped_messages': self.node.dropped_messages,
            'signals': deepcopy(self.signals),
            'ports': deepcopy(self.ports),
            'configuration': asdict(self.configuration),
        }

    def register_callback(self, name, callback):
        self.wait_ready()
        if name not in self.signals or not callable(callback):
            msg = 'Use a known signal and callable callback'
            raise ValueError(msg)
        self.callback_sequence += 1
        self.callbacks[self.callback_sequence] = (name, callback)
        return self.callback_sequence

    def deregister_callback(self, identifier):
        del self.callbacks[identifier]

    def make_command(self, values, category='signals'):
        if category not in ('signals', 'ports') or not values:
            msg = 'Provide nonempty signal or port values'
            raise ValueError(msg)
        self.wait_ready()
        configs = {
            item.name: json.loads(item.config)
            for item in getattr(self.configuration, category)
        }
        cache = getattr(self, category)
        payload = {}
        for name, value in values.items():
            if name not in cache or name not in configs:
                msg = f'Unknown configured IO name: {name}'
                raise ValueError(msg)
            item = cache[name]
            role = item['format'].get('role')
            sdk_light = category == 'signals' and name in self.writable_inputs
            if (
                role != ('output' if category == 'signals' else 'sink')
                and not sdk_light
            ):
                msg = f'IO {name} is read-only'
                raise ValueError(msg)
            if item['status'] != 'ready':
                msg = f'IO {name} is not ready'
                raise RuntimeError(msg)
            payload[name] = validate_value(
                value, item['format'], configs[name]
            )
        return wire.IOComponentCommand(
            time=self.node.get_clock().now().to_msg(),
            op='set',
            args=json.dumps({category: payload}, allow_nan=False),
        )

    def write(
        self, values, *, category='signals', execute=False, cancel_event=None
    ):
        if type(execute) is not bool:
            msg = 'execute must be a boolean'
            raise ValueError(msg)
        command = self.make_command(values, category)
        if not execute:
            return {'executed': False, 'command': asdict(command)}
        if cancel_event is not None and cancel_event.is_set():
            msg = 'IO operation cancelled before execution'
            raise RuntimeError(msg)
        if not self.subscription.connected():
            msg = 'IO feedback publisher is disconnected'
            raise ConnectionError(msg)
        if self.publisher is None:
            self.publisher = self.node.publisher_for(
                wire.IOComponentCommand, self.namespace + '/command'
            )
        self.wait(
            lambda: self.publisher.get_subscription_count() > 0,
            'IO command subscriber unavailable',
            cancel_event,
        )
        key = time_key(command.time)
        deadline = time.monotonic() + self.timeout
        next_send = 0.0
        while True:
            if cancel_event is not None and cancel_event.is_set():
                msg = 'IO operation cancelled'
                raise RuntimeError(msg)
            if key in [time_key(stamp) for stamp in self.state.commands]:
                statuses = getattr(self, category)
                if any(statuses[name]['status'] != 'ready' for name in values):
                    msg = 'Device rejected an acknowledged IO write'
                    raise RuntimeError(msg)
                return {
                    'executed': True,
                    'acknowledged': True,
                    'measured': {
                        name: statuses[name]['value'] for name in values
                    },
                }
            now = time.monotonic()
            if now >= deadline:
                msg = 'IO command acknowledgement timed out'
                raise TimeoutError(msg)
            if now >= next_send:
                self.publisher.publish(command)
                next_send = now + 0.1
            self.node.spin_once(timeout_sec=0.01)
