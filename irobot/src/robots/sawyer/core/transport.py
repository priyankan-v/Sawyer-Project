"""Compatibility exports; implementation lives in communication.transport."""

from __future__ import annotations

from .communication.transport import (
    LOGGER,
    MAX_HEADER_BYTES,
    MAX_PENDING_MESSAGES,
    DirectTransport,
    Publisher,
    RpcServer,
    ServiceClient,
    Subscription,
    TimeoutTransport,
    close_socket,
    frame,
    receive_exact,
    receive_frame,
    receive_header,
    rpc,
    send_header,
)

__all__ = [
    'LOGGER',
    'MAX_HEADER_BYTES',
    'MAX_PENDING_MESSAGES',
    'DirectTransport',
    'Publisher',
    'RpcServer',
    'ServiceClient',
    'Subscription',
    'TimeoutTransport',
    'close_socket',
    'frame',
    'receive_exact',
    'receive_frame',
    'receive_header',
    'rpc',
    'send_header',
]
