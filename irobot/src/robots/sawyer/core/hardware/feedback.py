"""Measured joint and named endpoint feedback."""

from __future__ import annotations

import math
import time
from dataclasses import replace

from ...config import SawyerConfig
from ..types import CartesianPose, pose_error


def receipt_time(message):
    return getattr(message, 'received_at', 0.0) or time.monotonic()


class JointFeedback:
    """
    Cache position and velocity independently, with per-joint receipt times.

    A JointState message may contain any subset of joints, and either numeric
    array may be empty. Unrelated joints never refresh or invalidate required
    joints. Missing/invalid fields never refresh their last valid receipt time.
    """

    def __init__(self, names):
        self.names = tuple(names)
        if not self.names or len(set(self.names)) != len(self.names):
            msg = 'Required joint names must be nonempty and unique'
            raise ValueError(msg)
        self.position = {}
        self.velocity = {}
        self.position_at = {}
        self.velocity_at = {}
        self.effort, self.effort_at = {}, {}

    def update(self, names, positions, velocities, now, efforts=()):
        relevant = set(self.names).intersection(names)
        if not relevant:
            return False
        if len(set(names)) != len(names):
            msg = 'Duplicate names in arm JointState message'
            raise ValueError(msg)
        if not math.isfinite(now):
            msg = 'Joint feedback receipt time must be finite'
            raise ValueError(msg)
        updates = []
        for label, values, cache, times in (
            ('position', positions, self.position, self.position_at),
            ('velocity', velocities, self.velocity, self.velocity_at),
            ('effort', efforts, self.effort, self.effort_at),
        ):
            if len(values) == 0:
                continue
            if len(values) != len(names):
                msg = (
                    f'Arm JointState {label} length {len(values)} != names '
                    f'length {len(names)}'
                )
                raise ValueError(msg)
            selected = {
                name: float(value)
                for name, value in zip(names, values, strict=False)
                if name in relevant
            }
            if not all(math.isfinite(value) for value in selected.values()):
                msg = f'Non-finite arm JointState {label}'
                raise ValueError(msg)
            updates.append((selected, cache, times))
        # Validate the whole relevant message before committing either field.
        for selected, cache, times in updates:
            cache.update(selected)
            times.update({name: now for name in selected})
        return bool(updates)

    def snapshot(self):
        if any(
            name not in self.position or name not in self.velocity
            for name in self.names
        ):
            return None, None, 0.0
        return (
            tuple(self.position[name] for name in self.names),
            tuple(self.velocity[name] for name in self.names),
            min(*self.position_at.values(), *self.velocity_at.values()),
        )

    def fresh_positions(self, now, timeout):
        """
        Position hold needs fresh positions even if velocity feedback expired.
        """
        if any(
            name not in self.position_at
            or now - self.position_at[name] >= timeout
            for name in self.names
        ):
            return None
        return tuple(self.position[name] for name in self.names)

    def issues(self, now, timeout):
        issues = []
        for label, times in (
            ('position', self.position_at),
            ('velocity', self.velocity_at),
        ):
            missing = [name for name in self.names if name not in times]
            stale = [
                f'{name} ({now - times[name]:.3f}s old)'
                for name in self.names
                if name in times and now - times[name] >= timeout
            ]
            if missing:
                issues.append(f'missing {label}: {", ".join(missing)}')
            if stale:
                issues.append(
                    f'stale {label} (limit {timeout:.3f}s): {", ".join(stale)}'
                )
        return issues


def from_pose(pose, frame):
    return CartesianPose(
        (pose.position.x, pose.position.y, pose.position.z),
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
        frame,
    )


class EndpointFeedback:
    """
    Named endpoint cache; avoids guessing that an anonymous pose is a
    fingertip.
    """

    def __init__(self, fallback_frame, feedback_timeout, config=None):
        self.config = config or SawyerConfig()
        self.frame = fallback_frame
        self.feedback_timeout = feedback_timeout
        self.endpoint = None
        self.tips = {}
        self.raw_tips = {}
        self.last_error = None

    def on_endpoint(self, message):
        try:
            if not message.valid:
                msg = 'Endpoint pose has valid=false'
                raise ValueError(msg)  # noqa: TRY301
            self.endpoint = (
                from_pose(message.pose, message.header.frame_id or self.frame),
                receipt_time(message),
            )
        except ValueError as exc:
            self.endpoint = None
            self.last_error = str(exc)

    def on_tips(self, message):
        if len(message.names) != len(message.states) or len(
            set(message.names)
        ) != len(message.names):
            self.tips.clear()
            self.raw_tips.clear()
            self.last_error = 'Malformed named tip-state array'
            return
        now = receipt_time(message)
        self.tips = {
            name: value
            for name, value in self.tips.items()
            if name in message.names
        }
        self.raw_tips = {
            name: value
            for name, value in self.raw_tips.items()
            if name in message.names
        }
        for name, state in zip(message.names, message.states, strict=False):
            try:
                if not state.valid:
                    msg = f'Tip {name} has valid=false'
                    raise ValueError(msg)  # noqa: TRY301
                frame = (
                    state.header.frame_id
                    or message.header.frame_id
                    or self.frame
                )
                self.tips[name] = (from_pose(state.pose, frame), now)
                stamp = state.header.stamp
                if not stamp.sec and not stamp.nanosec:
                    stamp = message.header.stamp
                header = replace(state.header, frame_id=frame, stamp=stamp)
                normalized = replace(state, header=header)
                normalized.received_at = state.received_at
                self.raw_tips[name] = (normalized, now)
            except ValueError as exc:
                self.tips.pop(name, None)
                self.raw_tips.pop(name, None)
                self.last_error = str(exc)

    def read(self, tip):
        if tip not in self.tips:
            msg = (
                f'No valid pose for {tip} on /robot/limb/right/tip_states; '
                f'available tips: {sorted(self.tips)}. {self.last_error or ""}'
            )
            raise RuntimeError(msg)
        pose, received = self.tips[tip]
        if time.monotonic() - received >= self.feedback_timeout:
            msg = (
                f'Stale Cartesian pose for {tip}: '
                f'{time.monotonic() - received:.3f}s old'
            )
            raise RuntimeError(msg)
        return pose

    def select(self, requested=None):
        if requested:
            return requested, self.read(requested)
        if (
            self.endpoint is None
            or time.monotonic() - self.endpoint[1] >= self.feedback_timeout
        ):
            msg = 'Missing/stale /robot/limb/right/endpoint_state'
            raise RuntimeError(msg)
        matches = []
        for name in self.tips:
            try:
                position, angle = pose_error(self.read(name), self.endpoint[0])
            except (RuntimeError, ValueError):
                continue
            if (
                position <= self.config.tip_match_position_tolerance
                and angle <= self.config.tip_match_orientation_tolerance
            ):
                matches.append(name)
        if len(matches) != 1:
            msg = (
                f'Cannot uniquely identify endpoint tip (matches={matches}); '
                'inspect tip_states and select --tip NAME'
            )
            raise RuntimeError(msg)
        return matches[0], self.read(matches[0])
