"""Saved posture validation and arm command ordering without a robot."""

from __future__ import annotations

import json
from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

import pytest

import sawyer_saved_pose
from irobot.src.robots.sawyer.config import JOINT_NAMES


@pytest.fixture
def pose_file(tmp_path):
    path = tmp_path / 'pose.json'
    path.write_text(json.dumps({'units': 'radians', 'joint_positions': dict.fromkeys(JOINT_NAMES, 0.2)}))
    return path


def controller():
    robot = MagicMock()
    robot.__enter__.return_value = robot
    robot.joint_positions = (0.2,) * 7
    result = SimpleNamespace(plan=SimpleNamespace(duration=4.0))
    robot.move_to_neutral.return_value = result
    robot.move_to_joints.return_value = result
    return robot


def test_capture_sends_no_motion_and_preserves_original(tmp_path):
    path = tmp_path / 'capture.json'
    robot = controller()
    with patch.object(sawyer_saved_pose, 'SawyerController', return_value=robot) as factory:
        assert sawyer_saved_pose.main(['save', str(path)]) == 0
        assert not factory.call_args.args[0].enable_gripper
    assert sawyer_saved_pose.load_pose(path) == robot.joint_positions
    assert json.loads(path.read_text())['captured_at_utc']
    robot.move_to_neutral.assert_not_called()
    robot.move_to_joints.assert_not_called()
    original = path.read_text()
    robot.joint_positions = (0.5,) * 7
    with pytest.raises(FileExistsError):
        sawyer_saved_pose.save_pose(robot, path)
    assert path.read_text() == original


def test_preview_sends_no_motion(pose_file, capsys):
    robot = controller()
    with patch.object(sawyer_saved_pose, 'SawyerController', return_value=robot):
        assert sawyer_saved_pose.main(['restore', str(pose_file)]) == 0
    assert robot.move_to_neutral.call_args_list == [call()]
    assert robot.move_to_joints.call_args_list == [call((0.2,) * 7)]
    assert 'Preview only' in capsys.readouterr().out


def test_execute_checks_target_then_moves_neutral_then_saved(pose_file, capsys):
    robot = controller()
    with patch.object(sawyer_saved_pose, 'SawyerController', return_value=robot):
        assert sawyer_saved_pose.main(['restore', str(pose_file), '--execute']) == 0
    moves = [entry for entry in robot.mock_calls if entry[0] in ('move_to_neutral', 'move_to_joints')]
    assert moves == [
        call.move_to_neutral(),
        call.move_to_joints((0.2,) * 7),
        call.move_to_neutral(execute=True),
        call.move_to_joints((0.2,) * 7, execute=True),
    ]
    assert 'Maximum joint error: 0.000000 rad' in capsys.readouterr().out


def test_invalid_target_is_rejected_before_neutral_motion(pose_file):
    robot = controller()
    robot.move_to_joints.side_effect = ValueError('Outside joint limits')
    with (
        patch.object(sawyer_saved_pose, 'SawyerController', return_value=robot),
        pytest.raises(ValueError, match='Outside joint limits'),
    ):
        sawyer_saved_pose.main(['restore', str(pose_file), '--execute'])
    robot.move_to_neutral.assert_called_once_with()


@pytest.mark.parametrize('bad_value', [True, '0.2', float('nan'), float('inf')])
def test_invalid_angles_fail_before_connect(pose_file, bad_value):
    state = json.loads(pose_file.read_text())
    state['joint_positions']['right_j0'] = bad_value
    pose_file.write_text(json.dumps(state))
    with patch.object(sawyer_saved_pose, 'SawyerController') as factory, pytest.raises(SystemExit, match='2'):
        sawyer_saved_pose.main(['restore', str(pose_file), '--execute'])
    factory.assert_not_called()


def test_named_joints_are_loaded_in_driver_order(pose_file):
    positions = {name: index / 10 for index, name in reversed(list(enumerate(JOINT_NAMES)))}
    pose_file.write_text(json.dumps({'units': 'radians', 'joint_positions': positions}))
    assert sawyer_saved_pose.load_pose(pose_file) == tuple(index / 10 for index in range(7))
