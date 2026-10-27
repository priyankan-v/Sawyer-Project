"""Display ownership and clock leases, independent of ROS and process cleanup."""

import math
import uuid


CLOCK_TOPIC = '/sawyer/head_display/clock'
STATUS_TOPIC = '/sawyer/head_display/state'
RELEASE_TOPIC = '/sawyer/head_display/release'
DISPLAY_TOPIC = '/robot/head_display'


def clock_tag(session, owner):
    return f'clock:{session}:{owner}'


def parse_clock_tag(value):
    parts = value.split(':')
    if len(parts) != 3 or parts[0] != 'clock' or not parts[1] or not parts[2]:
        raise ValueError('Invalid clock ownership tag')
    return parts[1], parts[2]


class DisplayPolicy:
    def __init__(self, lease_seconds=2.5, session=None):
        if not math.isfinite(lease_seconds) or lease_seconds <= 0:
            raise ValueError('Clock lease must be positive and finite')
        self.session = session or uuid.uuid4().hex
        self.lease_seconds = lease_seconds
        self.bridge_active = False
        self.frame = self.owner = None
        self.received = 0.0

    def set_bridge(self, active):
        self.bridge_active = bool(active)
        if not active:
            self.frame = self.owner = None

    def accept_clock(self, session, owner, frame, now):
        if not self.bridge_active or session != self.session or not math.isfinite(now):
            return False
        self.frame, self.owner, self.received = frame, owner, now
        return True

    def release_clock(self, session, owner):
        if session == self.session and owner == self.owner:
            self.frame = self.owner = None

    def selection(self, now):
        if not self.bridge_active:
            return 'native', None
        if self.frame is not None and now-self.received < self.lease_seconds:
            return 'clock', self.frame
        return 'image', None
