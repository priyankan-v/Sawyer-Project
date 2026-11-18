"""Read-only recording preserves sparse feedback and bounded input edges."""

from __future__ import annotations

import json
from io import StringIO

import pytest

from irobot.src.robots.sawyer.core.communication import wire
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


def test_partial_homing_feedback_preserves_other_joints():
    recording = DiagnosticRecording()
    recording.receive(
        'homing',
        wire.HomingState(
            name=['right_j0', 'head_pan'],
            state=[wire.HomingState.HOMED, wire.HomingState.NOT_HOMED],
        ),
    )
    recording.receive(
        'homing',
        wire.HomingState(name=['head_pan'], state=[wire.HomingState.HOMING]),
    )
    assert recording.report['homing'] == {
        'right_j0': wire.HomingState.HOMED,
        'head_pan': wire.HomingState.HOMING,
    }
    assert not recording.report['input_events']


def test_mismatched_homing_fields_do_not_modify_the_cache():
    recording = DiagnosticRecording()
    recording.receive(
        'homing',
        wire.HomingState(name=['right_j0'], state=[wire.HomingState.HOMED]),
    )
    with pytest.raises(ValueError, match='zip'):
        recording.receive(
            'homing',
            wire.HomingState(
                name=['right_j0', 'head_pan'],
                state=[wire.HomingState.HOMING],
            ),
        )
    assert recording.report['homing'] == {'right_j0': wire.HomingState.HOMED}


def test_trace_preserves_joint_fields_and_observed_command_order():
    trace = StringIO()
    recording = DiagnosticRecording(trace)
    feedback = wire.JointState(
        name=['right_j0'], position=[0.1], velocity=[0.2], effort=[0.3]
    )
    feedback.received_at = 10.0
    command = wire.Bool(data=True)
    command.received_at = 11.0
    recording.receive('joints', feedback)
    recording.receive('enable_command', command)
    entries = [json.loads(line) for line in trace.getvalue().splitlines()]
    assert [entry['sequence'] for entry in entries] == [0, 1]
    assert [entry['label'] for entry in entries] == [
        'joints',
        'enable_command',
    ]
    assert entries[0]['message']['velocity'] == [0.2]
    assert entries[0]['message']['effort'] == [0.3]
    assert entries[0]['received_at_monotonic'] == feedback.received_at
    assert entries[1]['received_at_monotonic'] == command.received_at
    assert recording.report['commands']['enable_command']['data'] is True
    assert recording.report['trace_records'] == len(entries)


def test_collision_feedback_is_preserved_without_becoming_an_input():
    recording = DiagnosticRecording()
    recording.receive(
        'collision', wire.CollisionDetectionState(collision_state=True)
    )
    assert recording.report['collision']['collision_state'] is True
    assert not recording.report['inputs']


def test_trace_write_failure_is_propagated():
    trace = StringIO()
    trace.close()
    recording = DiagnosticRecording(trace)
    with pytest.raises(ValueError, match='closed file'):
        recording.receive('stop_command', wire.Empty())
    assert recording.report['trace_records'] == 0
