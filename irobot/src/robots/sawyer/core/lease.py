"""Session identity, sequence and source-age checks for streamed commands."""

from __future__ import annotations

import threading
import time

from ..config import JOINT_NAMES, positive
from .execution.streaming import MODES

MAX_FUTURE_AGE = 0.01


class CommandLease:
    """One latest command for one explicitly accepted ROS 2 control session."""

    def __init__(self, session_id, mode, timeout, clock_ns):
        positive('lease timeout', timeout)
        self.session_id, self.mode, self.timeout = session_id, mode, timeout
        self.clock_ns = clock_ns
        self.lock = threading.Lock()
        self.sequence = -1
        self.command = None
        self.received_at = self.source_stamp = None
        self.error = None

    def accept(self, message):
        if message.session_id != self.session_id:
            return False
        with self.lock:
            if message.sequence <= self.sequence:
                return False
            try:
                self._validate(message)
            except ValueError as exc:
                self.error = str(exc)
                return False
            self.sequence = message.sequence
            self.source_stamp = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            self.received_at = time.monotonic()
            indices = [list(message.names).index(name) for name in JOINT_NAMES]
            fields = MODES[self.mode][1]
            self.command = {
                field: [getattr(message, field)[i] for i in indices]
                for field in fields
            }
        return True

    def _validate(self, message):
        mode, fields = MODES[self.mode]
        if message.mode != mode:
            msg = 'Command mode differs from the accepted session'
            raise ValueError(msg)
        if len(message.names) != len(JOINT_NAMES) or set(message.names) != set(
            JOINT_NAMES
        ):
            msg = 'A stream command must name each arm joint exactly once'
            raise ValueError(msg)
        for field in ('position', 'velocity', 'acceleration', 'effort'):
            if len(getattr(message, field)) != (
                len(JOINT_NAMES) if field in fields else 0
            ):
                msg = 'Command fields do not match the accepted mode'
                raise ValueError(msg)
        stamp = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        age = (self.clock_ns() - stamp) / 1e9
        if not stamp or age < -MAX_FUTURE_AGE or age > self.timeout:
            msg = 'Stream command source timestamp is stale or in the future'
            raise ValueError(msg)

    def get(self, _elapsed=0.0):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(self.error)
            if self.command is None:
                msg = 'No command for this session'
                raise RuntimeError(msg)
            source_age = (self.clock_ns() - self.source_stamp) / 1e9
            if (
                time.monotonic() - self.received_at > self.timeout
                or source_age > self.timeout
                or source_age < -MAX_FUTURE_AGE
            ):
                msg = 'Stream command lease expired; explicit restart required'
                raise TimeoutError(msg)
            return {
                field: list(values) for field, values in self.command.items()
            }
