"""Public API lifecycle tests; no ROS initialization or robot connection."""

# Numeric regression bounds and controlled private fixtures are intentional.
# ruff: noqa: PLR2004, SLF001, PT009, SIM117
# Fault fixtures exercise nested cleanup and private resource lifecycles.
from __future__ import annotations

import subprocess
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from irobot import SawyerBase, SawyerConfig, SawyerController
from irobot.src.robots.sawyer.config import MotionOptions


class LifecycleTests(unittest.TestCase):
    def attach(self, robot):
        robot._transport = MagicMock()
        robot._node = MagicMock()
        robot._node.state = SimpleNamespace(
            enabled=True, ready=True, stopped=False, error=False
        )
        robot._gripper = MagicMock() if robot.config.enable_gripper else None

    def test_import_and_construction_do_not_load_ros(self):
        code = (
            'from irobot import SawyerConfig, SawyerBase, SawyerController; '
            'r = SawyerController(SawyerConfig()); '
            'assert not r.is_connected; '
            'assert not r.get_status()["connected"]; '
            'r.disconnect(); import sys; '
            'blocked = {"rclpy", "rospy", "intera_core_msgs", "cflib"}; '
            'assert not any(x.split(".")[0] in blocked for x in sys.modules)'
        )
        result = subprocess.run(  # noqa: S603
            [sys.executable, '-c', code],
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 0, result.stderr

    def test_connect_and_reconnect_do_not_command_or_duplicate_runtime(self):
        robot = SawyerController()
        with patch.object(
            robot, '_open_runtime', side_effect=lambda: self.attach(robot)
        ) as open_runtime:
            assert robot.connect()
            node = robot._node
            assert robot.connect()
            open_runtime.assert_called_once()
            node.command_pub.publish.assert_not_called()
            node.watchdog_pub.publish.assert_not_called()
            robot._gripper.perform.assert_not_called()
            node.wait_inputs.assert_any_call(False, timeout=unittest.mock.ANY)

    def test_failed_connect_cleans_partial_resources(self):
        robot = SawyerBase()

        def fail():
            self.attach(robot)
            self.transport = robot._transport
            robot._node.wait_inputs.side_effect = RuntimeError(
                'Missing feedback'
            )

        with patch.object(robot, '_open_runtime', side_effect=fail):
            with pytest.raises(RuntimeError, match='Missing feedback'):
                robot.connect()
        self.transport.destroy_node.assert_called_once()
        assert not robot.is_connected
        assert robot._node is None

    def test_context_manager_disconnects_after_failure(self):
        robot = SawyerBase(SawyerConfig(enable_gripper=False))
        with patch.object(
            robot, '_open_runtime', side_effect=lambda: self.attach(robot)
        ):
            with pytest.raises(ValueError, match='application'):  # noqa: PT012
                with robot:
                    assert robot.is_connected
                    msg = 'application failure'
                    raise ValueError(msg)
        assert not robot.is_connected
        robot.disconnect()

    def test_disconnected_commands_fail(self):
        robot = SawyerController()
        for action in (
            robot.move_to_neutral,
            robot.open_gripper,
            lambda: robot.move_relative(dz_cm=1),
            lambda: robot.move_to_joints([0.0] * 7),
        ):
            with pytest.raises(RuntimeError, match='disconnected'):
                action()

    def test_invalid_options_and_watchdog_widening_rejected(self):
        for settings in (
            {'feedback_timeout': 0.3},
            {'command_timeout': 1.0},
            {'domain_id': -1},
            {'neutral_joints': (0.0,) * 6},
            {'gripper_open_position': float('nan')},
        ):
            with (
                self.subTest(settings=settings),
                pytest.raises(ValueError, match='.'),
            ):
                SawyerConfig(**settings)
        for settings in (
            {'speed': 2},
            {'rate': 10},
            {'time': float('nan')},
            {'execute': 'yes'},
        ):
            with pytest.raises(ValueError, match='.'):
                MotionOptions(**settings)

    def test_preview_default_overrides_options_execution_flag(self):
        robot = SawyerController(SawyerConfig(tip='right_hand'))
        options = robot._motion_options(MotionOptions(execute=True, speed=0.1))
        assert not options.execute
        assert options.speed == 0.1
        assert options.tip == 'right_hand'
        assert robot._motion_options(execute=True).execute

    def test_simultaneous_operations_are_rejected(self):
        robot = SawyerController()
        with (
            robot._operation(),
            pytest.raises(RuntimeError, match='already running'),
        ):
            robot.connect()

    def test_gripper_methods_preview_by_default_and_disabled_is_explicit(self):
        robot = SawyerController()
        self.attach(robot)
        robot._connected = True
        robot.open_gripper()
        robot._gripper.perform.assert_called_once_with('open', execute=False)
        robot.close_gripper(execute=True)
        robot._gripper.perform.assert_called_with('close', execute=True)
        robot._gripper = None
        with pytest.raises(RuntimeError, match='disabled'):
            robot.open_gripper()


if __name__ == '__main__':
    unittest.main()


def test_readonly_validation_skips_planning_when_controller_is_stopped():
    from unittest.mock import MagicMock

    from irobot.src.robots.sawyer.examples.validate_direct import validate

    robot = MagicMock()
    robot.get_status.return_value = {
        'ready': False,
        'enabled': False,
        'stopped': True,
        'error': True,
    }
    robot.devices.robot_state.return_value = {'stopped': True}
    report = validate(robot)
    assert report['robot_state']['stopped']
    assert report['motion_validation'].startswith('skipped')
    robot.move_to_neutral.assert_not_called()
    robot.devices.set_enabled.assert_not_called()
