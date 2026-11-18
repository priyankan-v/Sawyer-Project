"""Retained static and age-checked dynamic controller transforms."""

from __future__ import annotations

import time
from copy import deepcopy

from ..communication import wire
from ..types import CartesianPose


class TransformCache:
    def __init__(self, node, config):
        self.config = config
        self.dynamic, self.static = {}, {}
        node.observe(
            wire.TFMessage,
            config.tf_topic,
            lambda message: self.update(message, static=False),
        )
        node.observe(
            wire.TFMessage,
            config.tf_static_topic,
            lambda message: self.update(message, static=True),
        )

    def update(self, message, *, static):
        destination = self.static if static else self.dynamic
        for transform in message.transforms:
            parent, child = transform.header.frame_id, transform.child_frame_id
            if not parent or not child or parent == child:
                msg = 'Invalid TF parent/child frame'
                raise ValueError(msg)
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            CartesianPose(
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
                frame=parent,
            )
            destination[(parent, child)] = (transform, message.received_at)

    def snapshot(self, *, static=False):
        cache = self.static if static else self.dynamic
        now = time.monotonic()
        return wire.TFMessage(
            transforms=[
                deepcopy(transform)
                for transform, received_at in cache.values()
                if static or now - received_at <= self.config.feedback_timeout
            ]
        )
