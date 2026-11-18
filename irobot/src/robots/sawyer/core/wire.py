"""Compatibility exports for the native wire protocol."""

from __future__ import annotations

from .communication import wire as _wire
from .communication.wire import (
    FORMATS,
    MAX_ARRAY_ITEMS,
    MAX_MESSAGE_BYTES,
    NANOSECONDS_PER_SECOND,
    SCHEMA,
    Reader,
    default_value,
    deserialize,
    encode_value,
    field_name,
    from_dict,
    message_type,
    serialize,
    service_type,
    split_type,
)

__all__ = [
    'FORMATS',
    'MAX_ARRAY_ITEMS',
    'MAX_MESSAGE_BYTES',
    'NANOSECONDS_PER_SECOND',
    'SCHEMA',
    'Reader',
    'default_value',
    'deserialize',
    'encode_value',
    'field_name',
    'from_dict',
    'message_type',
    'serialize',
    'service_type',
    'split_type',
]


def __getattr__(name):
    return getattr(_wire, name)
