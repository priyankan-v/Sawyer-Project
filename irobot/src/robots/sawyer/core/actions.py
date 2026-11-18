"""Compatibility exports; implementation lives in communication.actions."""

from __future__ import annotations

from .communication.actions import (
    TERMINAL_STATES,
    ActionResult,
    NativeActionClient,
)

__all__ = [
    'TERMINAL_STATES',
    'ActionResult',
    'NativeActionClient',
]
