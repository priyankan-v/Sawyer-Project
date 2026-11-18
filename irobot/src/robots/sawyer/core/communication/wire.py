"""ROS 1 wire messages and codecs without ROS runtime dependencies."""

from __future__ import annotations

import json
import re
import struct
from dataclasses import field, make_dataclass
from functools import cache
from importlib.resources import files
from types import SimpleNamespace

SCHEMA = json.loads(
    files('irobot.src.robots.sawyer')
    .joinpath('installation/wire_schema.json')
    .read_text()
)
MAX_MESSAGE_BYTES = 16 * 1024 * 1024
MAX_ARRAY_ITEMS = 1_000_000
NANOSECONDS_PER_SECOND = 1_000_000_000
FORMATS = {
    'bool': '?',
    'int8': 'b',
    'uint8': 'B',
    'byte': 'b',
    'char': 'B',
    'int16': 'h',
    'uint16': 'H',
    'int32': 'i',
    'uint32': 'I',
    'int64': 'q',
    'uint64': 'Q',
    'float32': 'f',
    'float64': 'd',
}


def field_name(name):
    """Use Python field names while preserving the original wire layout."""
    if name in ('secs', 'nsecs'):
        return {'secs': 'sec', 'nsecs': 'nanosec'}[name]
    return re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()


def split_type(name):
    if '[' not in name:
        return name, None
    base, length = name[:-1].split('[')
    return base, int(length) if length else -1


def default_value(name):
    base, length = split_type(name)
    if length is not None:
        return (
            []
            if length == -1
            else [default_value(base) for _ in range(length)]
        )
    if base == 'string':
        return ''
    if base in FORMATS:
        return False if base == 'bool' else 0
    return message_type(base)()


@cache
def message_type(name):
    spec = SCHEMA['messages'][name]
    fields = [
        (
            field_name(key),
            object,
            field(default_factory=lambda kind=kind: default_value(kind)),
        )
        for key, kind in spec['fields']
    ]
    fields.append(
        (
            'received_at',
            float,
            field(default=0.0, init=False, repr=False, compare=False),
        )
    )
    namespace = {
        'ros_type': name,
        'md5sum': spec['md5'],
        'definition': spec['definition'],
        **spec['constants'],
    }
    return make_dataclass(
        name.rsplit('/', 1)[-1].capitalize()
        if '/' not in name
        else name.rsplit('/', 1)[-1],
        fields,
        namespace=namespace,
        slots=True,
    )


@cache
def service_type(name):
    spec = SCHEMA['services'][name]
    return SimpleNamespace(
        ros_type=name,
        md5sum=spec['md5'],
        Request=message_type(spec['request']),
        Response=message_type(spec['response']),
    )


def encode_value(kind, value):
    base, length = split_type(kind)
    if length is not None:
        if length >= 0 and len(value) != length:
            message = f'{kind} requires exactly {length} items'
            raise ValueError(message)
        prefix = struct.pack('<I', len(value)) if length == -1 else b''
        if base in ('uint8', 'char'):
            return prefix + bytes(value)
        if len(value) > MAX_ARRAY_ITEMS:
            message = 'Wire array exceeds the item limit'
            raise ValueError(message)
        return prefix + b''.join(encode_value(base, item) for item in value)
    if base == 'string':
        data = value.encode('utf-8')
        return struct.pack('<I', len(data)) + data
    if base in FORMATS:
        return struct.pack('<' + FORMATS[base], value)
    if base == 'time' and not 0 <= value.nanosec < NANOSECONDS_PER_SECOND:
        message = 'Time nanoseconds must be in [0, 1000000000)'
        raise ValueError(message)
    return b''.join(
        encode_value(item_kind, getattr(value, field_name(key)))
        for key, item_kind in SCHEMA['messages'][base]['fields']
    )


def serialize(message):
    payload = encode_value(message.ros_type, message)
    if len(payload) > MAX_MESSAGE_BYTES:
        message = 'Wire message exceeds the byte limit'
        raise ValueError(message)
    return payload


class Reader:
    def __init__(self, payload):
        if len(payload) > MAX_MESSAGE_BYTES:
            message = 'Wire message exceeds the byte limit'
            raise ValueError(message)
        self.payload = memoryview(payload)
        self.offset = 0

    def take(self, count):
        end = self.offset + count
        if end > len(self.payload):
            message = 'Truncated wire message'
            raise ValueError(message)
        result = self.payload[self.offset : end]
        self.offset = end
        return result

    def integer(self):
        return struct.unpack('<I', self.take(4))[0]

    def read(self, kind):
        base, length = split_type(kind)
        if length is not None:
            count = self.integer() if length == -1 else length
            if base in ('uint8', 'char'):
                return bytes(self.take(count))
            if count > MAX_ARRAY_ITEMS:
                message = 'Wire array exceeds the item limit'
                raise ValueError(message)
            return [self.read(base) for _ in range(count)]
        if base == 'string':
            return bytes(self.take(self.integer())).decode('utf-8')
        if base in FORMATS:
            fmt = '<' + FORMATS[base]
            data = self.take(struct.calcsize(fmt))
            if base == 'bool' and data[0] not in (0, 1):
                message = 'Invalid wire boolean'
                raise ValueError(message)
            return struct.unpack(fmt, data)[0]
        values = {
            field_name(key): self.read(item_kind)
            for key, item_kind in SCHEMA['messages'][base]['fields']
        }
        if base == 'time' and values['nanosec'] >= NANOSECONDS_PER_SECOND:
            message = 'Invalid time nanoseconds'
            raise ValueError(message)
        return message_type(base)(**values)


def deserialize(kind, payload):
    reader = Reader(payload)
    message = reader.read(kind)
    if reader.offset != len(reader.payload):
        message = 'Trailing bytes in wire message'
        raise ValueError(message)
    return message


def from_dict(kind, values):
    """Rebuild a typed message from an asdict/JSON-compatible mapping."""
    name = kind if isinstance(kind, str) else kind.ros_type
    specification = SCHEMA['messages'][name]
    known = {field_name(key) for key, _ in specification['fields']}
    if set(values) - known - {'received_at'}:
        message = f'Unknown fields for {name}'
        raise ValueError(message)
    arguments = {}
    for original, item_kind in specification['fields']:
        key = field_name(original)
        if key not in values:
            continue
        value = values[key]
        base, length = split_type(item_kind)
        if base in SCHEMA['messages']:
            value = (
                from_dict(base, value)
                if length is None
                else [from_dict(base, item) for item in value]
            )
        arguments[key] = value
    result = message_type(name)(**arguments)
    serialize(result)
    return result


def __getattr__(name):
    """Expose packaged Intera types without generated ROS imports."""
    if name == 'Time':
        return message_type('time')
    for category, factory in (
        ('messages', message_type),
        ('services', service_type),
    ):
        matches = [
            key for key in SCHEMA[category] if key.rsplit('/', 1)[-1] == name
        ]
        if len(matches) == 1:
            return factory(matches[0])
    raise AttributeError(name)
