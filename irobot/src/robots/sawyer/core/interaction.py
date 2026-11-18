"""Compatibility exports; implementation lives in execution.interaction."""

from __future__ import annotations

from ..config import InteractionLimits
from .execution.interaction import InteractionSession
from .planning.native import (
    interaction_command,
    validate_command,
)

__all__ = [
    'InteractionLimits',
    'InteractionSession',
    'interaction_command',
    'run_interaction',
    'validate_command',
]


def run_interaction(  # noqa: PLR0913
    driver,
    message,
    duration,
    *,
    execute=False,
    limits=None,
    stream_options=None,
    cancel_event=None,
    on_step=None,
):
    return driver.run_interaction(
        message,
        duration,
        execute=execute,
        limits=limits,
        stream_options=stream_options,
        cancel_event=cancel_event,
        on_step=on_step,
    )
