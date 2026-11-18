"""Peripheral acknowledgements, head refresh, and explicit execution checks."""

# Peripheral peer fixtures deliberately inspect direct-driver resources.
# ruff: noqa: SLF001, PLR2004
from __future__ import annotations

import json
import time
from unittest.mock import MagicMock

import pytest

from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.core.communication import wire
from irobot.src.robots.sawyer.core.hardware.peripherals import SawyerDevices


def camera_message(enabled, commands=()):
    return wire.IODeviceStatus(
        device=wire.IOComponentStatus(name='head_camera'),
        signals=[
            wire.IODataStatus(
                name='camera_streaming',
                format=json.dumps({'role': 'output', 'type': 'bool'}),
                data=json.dumps([enabled]),
                status=wire.IOStatus(tag='ready'),
            )
        ],
        commands=list(commands),
    )


@pytest.fixture
def peer():
    robot = SawyerController(
        SawyerConfig(enable_gripper=False, device_timeout=0.05)
    )
    robot._connected = True
    node = robot._node = MagicMock()
    node.state = wire.RobotAssemblyState(enabled=True, ready=True)
    node.state_at = time.monotonic()
    node.get_clock.return_value.now.return_value.to_msg.return_value = (
        wire.Time(sec=123, nanosec=456)
    )
    callbacks = {}

    def subscribe(kind, topic, callback):
        callbacks[topic] = callback
        message = (
            camera_message(False)
            if kind is wire.IODeviceStatus
            else wire.HeadState(pan=0.0, pan_mode=0)
        )
        message.received_at = time.monotonic()
        callback(message)

    node.create_subscription.side_effect = subscribe
    node.observe.side_effect = subscribe
    node.create_publisher.return_value.get_subscription_count.return_value = 1
    node.publisher_for.side_effect = node.create_publisher
    node.spin_until.side_effect = lambda _deadline: time.sleep(0.001)
    node.spin_once.side_effect = lambda **_kwargs: time.sleep(0.001)
    devices = SawyerDevices(robot)
    return devices, node, callbacks


def test_previews_send_no_commands_and_enable_never_resets_faults(peer):
    devices, node, _ = peer
    assert not devices.set_enabled(False)['executed']
    assert not devices.set_camera_streaming('head', True)['executed']
    node.create_publisher.assert_not_called()
    node.state.stopped = True
    with pytest.raises(RuntimeError, match='Reset faults explicitly'):
        devices.set_enabled(True, execute=True)
    node.create_publisher.assert_not_called()


def test_camera_needs_matching_timestamp_and_measured_signal(peer):
    devices, node, callbacks = peer
    publisher = node.create_publisher.return_value
    commands = []

    def acknowledge(message):
        commands.append(message)
        stamp = wire.Time(sec=1) if len(commands) == 1 else message.time
        callbacks[devices.config.camera_namespace + '/head_camera/state'](
            camera_message(True, [stamp])
        )

    publisher.publish.side_effect = acknowledge
    assert devices.set_camera_streaming('head', True, execute=True)['executed']
    assert len(commands) == 2
    assert commands[0].time == commands[1].time


def test_head_commands_refresh_until_measured_arrival(peer, monkeypatch):
    devices, node, callbacks = peer
    monkeypatch.setattr(devices, '_head_limits', lambda: (-1.0, 1.0))
    commands = []

    def respond(message):
        commands.append(message)
        if len(commands) >= 3:
            state = wire.HeadState(pan=message.target, pan_mode=1)
            state.received_at = time.monotonic()
            callbacks[devices.config.head_state_topic](state)

    node.create_publisher.return_value.publish.side_effect = respond
    assert devices.move_head(0.02, execute=True)['executed']
    assert len(commands) >= 3


def test_head_failure_requests_hold_and_restores_original_mode(
    peer, monkeypatch
):
    devices, node, _ = peer
    monkeypatch.setattr(devices, '_head_limits', lambda: (-1.0, 1.0))
    with pytest.raises(TimeoutError, match='Head did not reach'):
        devices.move_head(0.02, execute=True)
    command = node.create_publisher.return_value.publish.call_args.args[0]
    assert command.target == 0.0
    assert command.pan_mode == wire.HeadPanCommand.SET_PASSIVE_MODE


def test_display_validation_and_encoding_are_offline(peer):
    devices, node, _ = peer
    image = wire.Image(
        width=1, height=1, encoding='bgr8', step=3, data=b'\x00\x01\x02'
    )
    assert devices.display_image(image)['bytes'] > 3
    image.step = 4
    with pytest.raises(ValueError, match='valid RGB'):
        devices.display_image(image, execute=True)
    node.create_publisher.assert_not_called()


def test_head_interrupt_during_first_send_requests_hold(peer, monkeypatch):
    devices, node, _ = peer
    monkeypatch.setattr(devices, '_head_limits', lambda: (-1.0, 1.0))
    publisher = node.create_publisher.return_value
    publisher.publish.side_effect = [KeyboardInterrupt(), None]
    with pytest.raises(KeyboardInterrupt):
        devices.move_head(0.02, execute=True)
    hold = publisher.publish.call_args.args[0]
    assert hold.target == 0.0
    assert hold.pan_mode == wire.HeadPanCommand.SET_PASSIVE_MODE


def test_software_stop_sends_despite_stale_assembly_feedback(peer):
    devices, node, _ = peer
    node.state_at = time.monotonic() - 10
    published = []

    def stop(message):
        published.append(message)
        node.state = wire.RobotAssemblyState(stopped=True)
        node.state_at = time.monotonic()

    node.create_publisher.return_value.publish.side_effect = stop
    result = devices.software_stop(execute=True)
    assert result['executed']
    assert result['state']['stopped']
    assert isinstance(published[0], wire.Empty)
    node.cancel_requested.set.assert_called_once()


def test_software_stop_never_confirms_from_stale_stopped_state(peer):
    devices, node, _ = peer
    node.state = wire.RobotAssemblyState(stopped=True)
    node.state_at = time.monotonic() - 10
    with pytest.raises(TimeoutError, match='not confirmed'):
        devices.software_stop(execute=True)
    node.create_publisher.return_value.publish.assert_called()
