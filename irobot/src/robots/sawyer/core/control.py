"""Compatibility exports; implementation lives in execution.streaming."""

from __future__ import annotations

from .execution.streaming import MODES, JointStream

__all__ = [
    'MODES',
    'JointStream',
    'stream_joint_commands',
]


def stream_joint_commands(  # noqa: PLR0913
    driver,
    mode,
    source,
    duration,
    *,
    options=None,
    execute=False,
    cancel_event=None,
    on_step=None,
):
    return driver.stream_joint_commands(
        mode,
        source,
        duration,
        options=options,
        execute=execute,
        cancel_event=cancel_event,
        on_step=on_step,
    )
