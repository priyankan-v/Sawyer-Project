"""Wire-format checks against fixed ROS 1 layouts, without ROS imports."""

from __future__ import annotations

import struct

import pytest

from irobot.src.robots.sawyer.core import wire


def test_joint_state_matches_known_ros1_layout():
    kind = wire.message_type('sensor_msgs/JointState')
    assert kind.md5sum == '3066dcd76a6cfaef579bd0f34173e9fd'
    message = kind(
        name=['right_j0'], position=[0.25], velocity=[-0.5], effort=[]
    )
    expected = (
        struct.pack('<IIII', 0, 0, 0, 0)
        + struct.pack('<II', 1, 8)
        + b'right_j0'
        + struct.pack('<Id', 1, 0.25)
        + struct.pack('<Id', 1, -0.5)
        + struct.pack('<I', 0)
    )
    assert wire.serialize(message) == expected
    assert wire.deserialize(kind.ros_type, expected) == message


def test_all_packaged_schemas_round_trip():
    for name in wire.SCHEMA['messages']:
        payload = wire.serialize(wire.message_type(name)())
        assert wire.serialize(wire.deserialize(name, payload)) == payload


def test_wire_names_preserved_with_python_aliases():
    response = wire.SolvePositionFK.Response(
        is_valid=[True], in_collision=[False]
    )
    assert wire.serialize(response) == struct.pack('<IIBIB', 0, 1, 1, 1, 0)
    assert (
        wire.deserialize(response.ros_type, wire.serialize(response))
        == response
    )
    assert wire.SolvePositionIK.Request.SEED_USER == 1


@pytest.mark.parametrize('payload', [b'', b'\x01', b'\x00' * 9])
def test_truncated_or_trailing_payloads_fail(payload):
    with pytest.raises(ValueError, match='Truncated|Trailing'):
        wire.deserialize('std_msgs/Float64', payload)


def test_untrusted_array_length_is_bounded():
    with pytest.raises(ValueError, match='item limit'):
        wire.Reader(struct.pack('<I', 0xFFFFFFFF)).read('float64[]')


def test_invalid_boolean_and_timestamp_fail():
    with pytest.raises(ValueError, match='boolean'):
        wire.deserialize('std_msgs/Bool', b'\x02')
    with pytest.raises(ValueError, match='nanoseconds'):
        wire.deserialize('time', struct.pack('<II', 1, 1_000_000_000))


def test_fixed_arrays_require_exact_length():
    with pytest.raises(ValueError, match='exactly 2'):
        wire.encode_value('float64[2]', [1.0])


def test_unknown_fields_fail_instead_of_being_ignored():
    with pytest.raises(TypeError):
        wire.JointCommand(velocities=[0.1])
