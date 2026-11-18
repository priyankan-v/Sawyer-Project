"""Compatibility exports; implementation lives in hardware.io."""

from __future__ import annotations

from .hardware.io import (
    MAX_EVENTS,
    IODevice,
    validate_value,
)

__all__ = [
    'MAX_EVENTS',
    'IODevice',
    'validate_value',
]
