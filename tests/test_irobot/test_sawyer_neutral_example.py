"""
Neutral example routing and preview/execute behavior without ROS
initialization.
"""

# Numeric values are the expected CLI defaults and explicit overrides.
# ruff: noqa: PLR2004
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

import pytest

import main as entrypoint
from irobot.src.robots.sawyer.examples import move_neutral


def result(*, executed=False):
    plan = SimpleNamespace(
        start=(0.0,) * 7, end=(0.2,) * 7, duration=4.0, minimum_duration=3.0
    )
    execution = (
        SimpleNamespace(reference_duration=4.0, elapsed_duration=4.3)
        if executed
        else None
    )
    return SimpleNamespace(plan=plan, execution=execution)


def controller():
    robot = MagicMock()
    robot.__enter__.return_value = robot
    robot.move_to_neutral.return_value = result()
    robot.joint_positions = (0.2,) * 7
    return robot


def test_main_routes_to_the_sawyer_neutral_example():
    assert entrypoint.main is move_neutral.main


def test_default_preview_uses_only_public_arm_api(capsys):
    robot = controller()
    with patch.object(
        move_neutral, 'SawyerController', return_value=robot
    ) as factory:
        assert move_neutral.main([]) == 0
    config = factory.call_args.args[0]
    assert not config.enable_gripper
    assert not config.motion.execute
    assert config.motion.speed == 1.0
    assert config.motion.max_accel == 0.3
    robot.move_to_neutral.assert_called_once_with(duration=None)
    robot.open_gripper.assert_not_called()
    robot.__exit__.assert_called_once()
    assert 'Preview only' in capsys.readouterr().out


def test_explicit_lower_speed_remains_available():
    robot = controller()
    with patch.object(
        move_neutral, 'SawyerController', return_value=robot
    ) as factory:
        assert move_neutral.main(['--speed', '0.2']) == 0
    assert factory.call_args.args[0].motion.speed == 0.2
    robot.move_to_neutral.assert_called_once_with(duration=None)


def test_execution_replans_after_preview_and_reports_measurement(capsys):
    robot = controller()
    robot.move_to_neutral.side_effect = [result(), result(executed=True)]
    with patch.object(move_neutral, 'SawyerController', return_value=robot):
        assert move_neutral.main(['--execute', '--time', '4']) == 0
    assert robot.move_to_neutral.call_args_list == [
        call(duration=4.0),
        call(duration=4.0, execute=True),
    ]
    assert 'Neutral reached' in capsys.readouterr().out


@pytest.mark.parametrize(
    'args',
    [
        ['--speed', '0'],
        ['--speed', '2'],
        ['--time', 'nan'],
        ['--time', '-1'],
        ['--max-accel', '0'],
    ],
)
def test_invalid_settings_fail_before_connecting(args):
    with (
        patch.object(move_neutral, 'SawyerController') as factory,
        pytest.raises(SystemExit, match='2'),
    ):
        move_neutral.main(args)
    factory.assert_not_called()
