"""
Collect controller diagnostics and input edges without command publishers.
"""

# Standalone read-only operator utility with explicit JSON output.
# ruff: noqa: T201
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path

from ..config import JOINT_NAMES, SawyerConfig, positive
from ..core import wire
from ..core.transport import DirectTransport

MAX_EVENTS = 256


class DiagnosticRecording:
    def __init__(self):
        self.report = {
            'date_utc': datetime.now(UTC).isoformat(),
            'counts': {},
            'assembly': None,
            'diagnostics': {},
            'inputs': {},
            'input_events': [],
            'dropped_events': 0,
            'joints': {},
        }

    def receive(self, label, message):
        report = self.report
        report['counts'][label] = report['counts'].get(label, 0) + 1
        if label == 'assembly':
            report['assembly'] = asdict(message)
        elif label == 'diagnostics':
            for item in message.status:
                report['diagnostics'][item.name] = asdict(item)
        elif label == 'joints':
            self.receive_joints(message)
        else:
            self.receive_input(label, message)

    def receive_joints(self, message):
        if len(message.name) != len(message.position):
            return
        for name, value in zip(message.name, message.position, strict=True):
            if name not in JOINT_NAMES or not math.isfinite(value):
                continue
            joints = self.report['joints']
            item = joints.setdefault(
                name, {'position': value, 'minimum': value, 'maximum': value}
            )
            item.update(
                position=value,
                minimum=min(item['minimum'], value),
                maximum=max(item['maximum'], value),
            )

    def receive_input(self, label, message):
        values = (
            {item.name: json.loads(item.data) for item in message.signals}
            if isinstance(message, wire.IODeviceStatus)
            else {'state': message.state}
            if isinstance(message, wire.DigitalIOState)
            else {'value': message.value}
        )
        cache = self.report['inputs'].setdefault(label, {})
        for name, value in values.items():
            if name in cache and cache[name] != value:
                events = self.report['input_events']
                events.append(
                    {
                        'device': label,
                        'name': name,
                        'previous': cache[name],
                        'value': value,
                        'received_at_monotonic': message.received_at,
                    }
                )
                if len(events) > MAX_EVENTS:
                    events.pop(0)
                    self.report['dropped_events'] += 1
            cache[name] = value


def record(config, duration=5.0):
    """Subscribe only; never enable, reset, move or change an IO setting."""
    positive('duration', duration)
    recording = DiagnosticRecording()
    node = DirectTransport(config)
    try:
        for label, kind, topic in (
            ('assembly', wire.RobotAssemblyState, config.robot_state_topic),
            ('diagnostics', wire.DiagnosticArray, config.diagnostics_topic),
            ('joints', wire.JointState, config.joint_state_topic),
            ('cuff', wire.IODeviceStatus, config.io_namespace + '/cuff/state'),
            (
                'navigator',
                wire.IODeviceStatus,
                config.io_namespace + '/navigator/state',
            ),
            (
                'upper_button',
                wire.DigitalIOState,
                '/robot/digital_io/right_upper_button/state',
            ),
            (
                'right_wheel',
                wire.AnalogIOState,
                '/robot/analog_io/right_wheel/state',
            ),
        ):
            node.observe(
                kind,
                topic,
                lambda message, label=label: recording.receive(label, message),
            )
        end = time.monotonic() + duration
        while time.monotonic() < end:
            node.spin_once(timeout_sec=0.01)
        recording.report['no_publishers_created'] = not bool(node.publishers)
        recording.report['transport_dropped_messages'] = node.dropped_messages
    finally:
        node.destroy_node()
    return recording.report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master', required=True)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    report = record(
        SawyerConfig(ros_master_uri=args.master, ros_ip=args.ip),
        args.duration,
    )
    args.report.write_text(
        json.dumps(report, indent=2, allow_nan=False) + '\n'
    )
    print(
        json.dumps(
            {
                'report': str(args.report),
                'diagnostics': len(report['diagnostics']),
                'input_events': len(report['input_events']),
                'no_publishers_created': report['no_publishers_created'],
            }
        )
    )


if __name__ == '__main__':
    main()
