"""Compatibility exports; implementation lives in hardware.model."""

from __future__ import annotations

from .hardware.model import (
    effort_limits,
    urdf_joint_limits,
)

__all__ = [
    'effort_limits',
    'urdf_joint_limits',
]
