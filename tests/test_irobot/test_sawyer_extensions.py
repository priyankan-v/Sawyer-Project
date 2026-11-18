"""Native action lifecycle, bounded control modes and metadata-driven I/O."""

# These fixtures inspect direct protocol state and use short deterministic
# clocks.
# ruff: noqa: SLF001, PLR2004
from __future__ import annotations

import json
import threading
import time
from dataclasses import replace
from unittest.mock import MagicMock, patch

import pytest

from irobot.src.robots.sawyer.core import wire
from irobot.src.robots.sawyer.core.actions import NativeActionClient
from irobot.src.robots.sawyer.core.interaction import (
    InteractionLimits,
    interaction_command,
    validate_command,
)
from irobot.src.robots.sawyer.core.io import IODevice, validate_value
from irobot.src.robots.sawyer.core.lease import CommandLease
from irobot.src.robots.sawyer.core.model import effort_limits
from tests.test_irobot.test_sawyer_control import Plant


def test_native_planning_previews_preserve_start_and_reject_mismatches():
    from irobot.src.robots.sawyer.core.motion import (
        joint_trajectory,
        validate_trajectory,
    )

    plant = Plant()
    target = [0.05] * 7
    with plant.running():
        preview = plant.robot.move_native_joint_path([target])
        goal = wire.from_dict(wire.MotionCommandGoal, preview['goal'])
        assert not preview['executed']
        assert goal.command == 'start'
        assert goal.trajectory.waypoints[0].joint_positions == list(plant.q)
        assert goal.trajectory.waypoints[-1].joint_positions == target
        assert not plant.messages

        trajectory = joint_trajectory(plant.node, [target])
        validate_trajectory(plant.node, trajectory)
        generated = plant.robot.execute_native_trajectory(
            trajectory, plan_only=True
        )
        assert generated['goal']['command'] == 'generate'
        assert not generated['executed']
        assert not plant.messages

        trajectory.waypoints[0].joint_positions = [0.1] * 7
        with pytest.raises(ValueError, match='measured start'):
            plant.robot.execute_native_trajectory(trajectory)


def test_native_cartesian_preview_uses_measured_tip_and_explicit_options():
    from irobot.src.robots.sawyer.core.motion import validate_trajectory

    plant = Plant()
    waypoint = wire.Waypoint(active_endpoint='right_hand')
    waypoint.pose.header.frame_id = 'base'
    waypoint.pose.pose.orientation.w = 1.0
    waypoint.options.max_joint_speed_ratio = 0.05
    for name in (
        'max_linear_speed',
        'max_linear_accel',
        'max_rotational_speed',
        'max_rotational_accel',
    ):
        setattr(waypoint.options, name, 0.1)
    trajectory = wire.Trajectory(
        joint_names=[f'right_j{i}' for i in range(7)], waypoints=[waypoint]
    )
    trajectory.trajectory_options.interpolation_type = (
        wire.TrajectoryOptions.CARTESIAN
    )
    with plant.running():
        validate_trajectory(plant.node, trajectory)
        assert not plant.robot.execute_native_trajectory(trajectory)[
            'executed'
        ]
        assert not plant.messages
        waypoint.pose.pose.position.x = 0.1
        with pytest.raises(ValueError, match='measured start'):
            plant.robot.execute_native_trajectory(trajectory)


def test_stream_modes_preview_bounds_and_exit():
    plant = Plant()
    plant.robot.config = plant.node.config = replace(
        plant.robot.config, allow_unvalidated_modes=True
    )
    plant.node.generation = 0
    plant.node.joint_effort_limits = (9.0,) * 7
    plant.node.master = MagicMock(
        return_value='<robot>'
        + ''.join(
            f'<joint name="right_j{i}"><limit effort="9"/></joint>'
            for i in range(7)
        )
        + '</robot>'
    )
    plant.node.publisher_for = MagicMock()
    publisher = plant.node.publisher_for.return_value
    publisher.get_subscription_count.return_value = 1
    sources = {
        'position': {'position': [0.0] * 7},
        'velocity': {'velocity': [0.01] * 7},
        'torque': {'effort': [0.05] * 7},
        'trajectory': {
            'position': [0.0] * 7,
            'velocity': [0.0] * 7,
            'acceleration': [0.0] * 7,
        },
    }
    with plant.running():
        for mode, source in sources.items():
            if 'position' in source:
                source['position'] = list(plant.node.joints)
            plant.messages.clear()
            report = plant.robot.stream_joint_commands(mode, source, 0.15)
            assert not report['executed']
            assert not plant.messages
            report = plant.robot.stream_joint_commands(
                mode, source, 0.15, execute=True
            )
            assert report['samples'] >= 10
            assert report['max_command_gap_s'] <= 0.02
            assert plant.messages[-1].mode == wire.JointCommand.POSITION_MODE
            assert not plant.node.streaming
        plant.messages.clear()
        with pytest.raises(ValueError, match='exceeds'):
            plant.robot.stream_joint_commands('torque', {'effort': [9] * 7}, 1)
        with pytest.raises(ValueError, match='exactly'):
            plant.robot.stream_joint_commands(
                'position', {'velocity': [0] * 7}, 1
            )
        assert not plant.messages


def test_stream_rejects_slow_callback_and_transport_restart_before_publish():
    plant = Plant()
    plant.node.generation = 0

    def slow_callback(*_args):
        plant.now += 0.15

    with plant.running(), pytest.raises(RuntimeError, match='stale|deadline'):
        plant.robot.stream_joint_commands(
            'velocity',
            {'velocity': [0.01] * 7},
            1,
            execute=True,
            on_step=slow_callback,
        )
    assert all(not any(m.velocity) for m in plant.messages if m.velocity)
    assert not plant.node.streaming


def action_peer(outcome='success'):
    node = MagicMock(generation=0)
    node.create_publisher.side_effect = lambda *_args: MagicMock()
    node.get_clock.return_value.now.return_value.to_msg.return_value = (
        wire.Time(sec=123)
    )
    client = NativeActionClient(
        node, '/motion', 'intera_motion_msgs/MotionCommand', server_timeout=0.1
    )
    client.goal_pub.get_subscription_count.return_value = 1
    client.cancel_pub.get_subscription_count.return_value = 1
    client.status_at = time.monotonic()
    sent = []

    def publish(message):
        sent.append(message)

    client.goal_pub.publish.side_effect = publish

    def finish(status, goal_id):
        client.on_result(
            wire.MotionCommandActionResult(
                status=wire.GoalStatus(
                    goal_id=wire.GoalID(id=goal_id), status=status
                ),
                result=wire.MotionCommandResult(
                    result=status == wire.GoalStatus.SUCCEEDED
                ),
            )
        )

    client.cancel_pub.publish.side_effect = lambda message: finish(
        wire.GoalStatus.PREEMPTED, message.id
    )

    def spin(**_kwargs):
        if sent:
            finish(wire.GoalStatus.SUCCEEDED, 'old-goal')
            assert client.result is None
            if outcome == 'success':
                finish(wire.GoalStatus.SUCCEEDED, client.active_id)
            elif outcome == 'reject':
                finish(wire.GoalStatus.REJECTED, client.active_id)
            elif outcome == 'restart':
                node.generation += 1
            else:
                time.sleep(0.002)

    node.spin_once.side_effect = spin
    return client, sent


@pytest.mark.parametrize('outcome', ['success', 'reject'])
def test_native_action_matches_ids_and_reports_rejection(outcome):
    client, sent = action_peer(outcome)
    result = client.run(wire.MotionCommandGoal(command='start'), timeout=0.1)
    assert result.succeeded == (outcome == 'success')
    assert result.goal_id == sent[0].goal_id.id
    assert sent[0].goal_id.stamp == wire.Time()
    assert not client.busy.locked()
    assert client.active_id is None


@pytest.mark.parametrize('outcome', ['restart', 'timeout'])
def test_native_action_failure_cancels_only_its_own_goal(outcome):
    client, sent = action_peer(outcome)
    with pytest.raises((TimeoutError, ConnectionError)):
        client.run(wire.MotionCommandGoal(command='start'), timeout=0.02)
    command = client.cancel_pub.publish.call_args.args[0]
    assert command.id == sent[0].goal_id.id
    assert command.id
    assert not client.busy.locked()


def test_cancelled_action_never_publishes():
    client, sent = action_peer()
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(RuntimeError, match='before execution'):
        client.run(wire.MotionCommandGoal(), timeout=1, cancel_event=cancel)
    assert not sent


def io_peer():
    node = MagicMock(dropped_messages=0)
    node.observe.return_value.connected.return_value = True
    node.get_clock.return_value.now.return_value.to_msg.return_value = (
        wire.Time(sec=12)
    )
    device = IODevice(node, '/io/test/device', 0.02)
    device.on_config(
        wire.IODeviceConfiguration(
            signals=[
                wire.IOComponentConfiguration(
                    name='light',
                    config=json.dumps({'props': {'format': {'type': 'bool'}}}),
                )
            ]
        )
    )
    message = wire.IODeviceStatus(
        signals=[
            wire.IODataStatus(
                name='light',
                data='[false]',
                format='{"type":"bool","role":"output"}',
                status=wire.IOStatus(tag='ready'),
            )
        ]
    )
    message.received_at = time.monotonic()
    device.on_state(message)
    return device, node, message


def test_io_types_ranges_input_only_and_independent_snapshot():
    assert validate_value(0.2, {'type': 'float', 'max': 1}, {})['data'] == [
        0.2
    ]
    for value in [True, float('nan'), 2]:
        with pytest.raises(ValueError, match='type|finite|exceeds'):
            validate_value(value, {'type': 'float', 'max': 1}, {})
    device, node, _ = io_peer()
    snapshot = device.snapshot()
    snapshot['signals']['light']['format']['role'] = 'input'
    assert device.signals['light']['format']['role'] == 'output'
    assert not device.write({'light': True})['executed']
    node.publisher_for.assert_not_called()
    device.signals['light']['format']['role'] = 'input'
    with pytest.raises(ValueError, match='read-only'):
        device.write({'light': True}, execute=True)


def test_io_acknowledgement_and_edges():
    device, node, message = io_peer()
    events = []
    device.register_callback('light', events.append)
    publisher = node.publisher_for.return_value
    publisher.get_subscription_count.return_value = 1

    def acknowledge(command):
        message.signals[0].data = '[true]'
        message.commands = [command.time]
        device.on_state(message)

    publisher.publish.side_effect = acknowledge
    report = device.write({'light': True}, execute=True)
    assert report['acknowledged']
    assert report['measured']['light']
    assert len(events) == 1
    assert events[0]['previous'] is False
    message.signals[0].data = '[false]'
    device.on_state(message)
    assert len(events) == 2
    assert events[1]['previous'] is True
    node.observe.return_value.connected.return_value = False
    with pytest.raises(ConnectionError):
        device.write({'light': True}, execute=True)


def test_effort_limits_must_be_published_for_all_seven_joints():
    with pytest.raises(KeyError):
        effort_limits('<robot/>')


def test_interaction_confirms_exit_and_reuses_one_observer():
    plant = Plant()
    plant.node.generation = 0
    observer = {}
    state = wire.InteractionControlState(interaction_control_active=False)
    state.received_at = plant.now
    publisher = MagicMock()
    publisher.get_subscription_count.return_value = 1
    plant.node.publisher_for = MagicMock(return_value=publisher)

    def observe(_kind, topic, callback):
        observer[topic] = callback
        callback(state)

    def publish(message):
        state.interaction_control_active = message.interaction_control_active
        state.received_at = plant.now
        observer[plant.robot.config.interaction_state_topic](state)

    plant.node.observe = MagicMock(side_effect=observe)
    publisher.publish.side_effect = publish
    with plant.running():
        message = interaction_command()
        assert not plant.robot.run_interaction(message, 0.1)['executed']
        plant.node.observe.assert_not_called()
        for _ in range(2):
            report = plant.robot.run_interaction(message, 0.1, execute=True)
            assert report['activation_confirmed']
            assert report['deactivation_confirmed']
            assert not state.interaction_control_active
        assert plant.node.observe.call_count == 1
        assert not plant.node.streaming


def test_interaction_rejects_wrench_and_invalid_frame_parameters():
    message = interaction_command()
    message.force_command[0] = 6.0
    with pytest.raises(ValueError, match='envelope'):
        validate_command(message, InteractionLimits())
    message = interaction_command()
    message.interaction_frame.orientation.w = 0.0
    with pytest.raises(ValueError, match='quaternion'):
        validate_command(message, InteractionLimits())


def test_transform_snapshots_omit_expired_dynamic_edges_and_keep_static():
    from irobot.src.robots.sawyer.core.transforms import TransformCache

    node = MagicMock()
    cache = TransformCache(node, Plant().robot.config)
    transform = wire.TransformStamped(child_frame_id='tool')
    transform.header.frame_id = 'base'
    transform.transform.rotation.w = 1.0
    message = wire.TFMessage(transforms=[transform])
    message.received_at = time.monotonic() - 1
    cache.update(message, static=False)
    cache.update(message, static=True)
    assert not cache.snapshot().transforms
    assert len(cache.snapshot(static=True).transforms) == 1
    transform.transform.rotation.w = 0.0
    with pytest.raises(ValueError, match='quaternion'):
        cache.update(message, static=False)


def test_command_lease_rejects_old_ids_sequences_timestamps_and_loss():
    from types import SimpleNamespace

    clock = [10_000_000_000]
    lease = CommandLease('current', 'velocity', 0.1, lambda: clock[0])
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=wire.Time(sec=10)),
        session_id='old',
        sequence=1,
        mode=2,
        names=list(reversed(Plant().node.feedback.names)),
        position=[],
        velocity=[0.01] * 7,
        acceleration=[],
        effort=[],
    )
    assert not lease.accept(message)
    message.session_id = 'current'
    assert lease.accept(message)
    assert not lease.accept(message)
    assert lease.get()['velocity'] == [0.01] * 7
    clock[0] += 200_000_000
    with pytest.raises(TimeoutError, match='lease expired'):
        lease.get()
    message.sequence = 2
    assert not lease.accept(message)
    with pytest.raises(RuntimeError, match='timestamp'):
        lease.get()


@pytest.mark.parametrize('drift', [False, True])
def test_enable_confirms_ready_and_latches_stop_on_drift(drift):
    from irobot.src.robots.sawyer.core.peripherals import SawyerDevices

    plant = Plant()
    devices = SawyerDevices(plant.robot)
    plant.node.state.enabled = plant.node.state.ready = False
    publisher = MagicMock()

    def publish(message):
        if isinstance(message, wire.Bool):
            plant.node.state.enabled = True
            plant.node.state.ready = not drift
            if drift:
                plant.q[0] = 0.03
        else:
            plant.node.state.enabled = False
            plant.node.state.stopped = True

    publisher.publish.side_effect = publish
    with (
        plant.running(),
        patch.object(devices, '_publisher', return_value=publisher),
    ):
        if drift:
            with pytest.raises(RuntimeError, match='enable envelope'):
                devices.set_enabled(True, execute=True)
            assert plant.node.state.stopped
            assert not plant.node.state.enabled
            assert any(
                isinstance(call.args[0], wire.Empty)
                for call in publisher.publish.call_args_list
            )
        else:
            assert devices.set_enabled(True, execute=True)['ready_confirmed']


def test_mode_exit_does_not_send_commands_after_controller_stop():
    plant = Plant()
    plant.node.streaming = True
    plant.node.state.stopped = True
    with plant.running():
        plant.node.stop()
    assert not plant.messages
    assert not plant.node.streaming


def test_unqualified_modes_require_explicit_opt_in_before_publication():
    plant = Plant()
    plant.node.generation = 0
    with (
        plant.running(),
        pytest.raises(RuntimeError, match='not hardware-qualified'),
    ):
        plant.robot.stream_joint_commands(
            'trajectory',
            {
                'position': [0.0] * 7,
                'velocity': [0.0] * 7,
                'acceleration': [0.0] * 7,
            },
            0.1,
            execute=True,
        )
    assert not plant.messages
