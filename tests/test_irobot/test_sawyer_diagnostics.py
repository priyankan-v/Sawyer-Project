"""Read-only recording preserves sparse feedback and bounded input edges."""

from __future__ import annotations

from irobot.src.robots.sawyer.core import wire
from irobot.src.robots.sawyer.examples.diagnose_direct import (
    MAX_EVENTS,
    DiagnosticRecording,
)


def test_sparse_diagnostics_and_joint_fields_are_preserved():
    recording = DiagnosticRecording()
    for name in ('controller', 'joint_0'):
        recording.receive(
            'diagnostics',
            wire.DiagnosticArray(
                status=[wire.DiagnosticStatus(name=name, message='OK')]
            ),
        )
    for value in (0.1, 0.2):
        recording.receive(
            'joints',
            wire.JointState(
                name=['right_j0', 'head_pan'], position=[value, 0.0]
            ),
        )
    recording.receive(
        'joints', wire.JointState(name=['right_j0'], velocity=[0.0])
    )
    assert set(recording.report['diagnostics']) == {'controller', 'joint_0'}
    assert recording.report['joints'] == {
        'right_j0': {'position': 0.2, 'minimum': 0.1, 'maximum': 0.2}
    }


def test_change_only_input_cache_and_events_are_bounded():
    recording = DiagnosticRecording()
    recording.receive(
        'cuff',
        wire.IODeviceStatus(
            signals=[
                wire.IODataStatus(name='upper', data='[0]'),
                wire.IODataStatus(name='lower', data='[0]'),
            ]
        ),
    )
    recording.receive(
        'cuff',
        wire.IODeviceStatus(
            signals=[
                wire.IODataStatus(name='upper', data='[1]'),
            ]
        ),
    )
    assert recording.report['inputs']['cuff'] == {'upper': [1], 'lower': [0]}
    for value in range(MAX_EVENTS + 2):
        recording.receive('right_wheel', wire.AnalogIOState(value=value))
    assert len(recording.report['input_events']) == MAX_EVENTS
    assert recording.report['dropped_events'] > 0
    assert recording.report['input_events'][-1]['value'] == MAX_EVENTS + 1
