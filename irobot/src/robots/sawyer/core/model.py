"""
Read the controller's model and enforce its published joint effort limits.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from ..config import JOINT_NAMES


def urdf_joint_limits(description):
    root = ET.fromstring(description)  # noqa: S314
    result = {}
    for joint in root.findall('joint'):
        limit = joint.find('limit')
        if limit is None:
            continue
        values = {name: float(value) for name, value in limit.attrib.items()}
        if not all(math.isfinite(value) for value in values.values()):
            msg = 'URDF contains nonfinite joint limits'
            raise ValueError(msg)
        result[joint.attrib['name']] = values
    return result


def effort_limits(description):
    limits = urdf_joint_limits(description)
    values = tuple(limits[name]['effort'] for name in JOINT_NAMES)
    if any(value <= 0 for value in values):
        msg = 'URDF effort limits must be positive'
        raise ValueError(msg)
    return values
