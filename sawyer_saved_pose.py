"""Save Sawyer's live arm posture, or restore it via neutral using the irobot API."""

# CLI output is intentional.
# ruff: noqa: T201
from __future__ import annotations

import argparse
import json
import math
from datetime import UTC, datetime
from pathlib import Path

from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.config import JOINT_NAMES, CartesianMotionOptions

DEFAULT_POSE = Path('.sawyer-runtime/saved_pose.json')


def load_pose(path):
    """Require seven named right-arm angles in radians before connecting."""
    state = json.loads(path.read_text())
    if not isinstance(state, dict) or state.get('units') != 'radians':
        msg = 'Pose must be a JSON object with units="radians"'
        raise ValueError(msg)
    joints = state.get('joint_positions')
    if not isinstance(joints, dict) or set(joints) != set(JOINT_NAMES):
        msg = 'Pose must contain exactly right_j0 through right_j6'
        raise ValueError(msg)
    target = tuple(joints[name] for name in JOINT_NAMES)
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in target):
        msg = 'Joint positions must be finite numbers in radians'
        raise ValueError(msg)
    return target


def save_pose(robot, path):
    state = {
        'captured_at_utc': datetime.now(UTC).isoformat(),
        'units': 'radians',
        'joint_positions': dict(zip(JOINT_NAMES, robot.joint_positions, strict=True)),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    # Preserve the original capture if this command is accidentally repeated.
    with path.open('x') as stream:
        json.dump(state, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(f'Saved live arm posture to {path}. No motion commands sent.')


def restore_pose(robot, target, *, execute=False):
    neutral = robot.move_to_neutral()
    robot.move_to_joints(target)  # Validate the saved target against live joint limits before motion.
    print('Saved target (radians):')
    for name, angle in zip(JOINT_NAMES, target, strict=True):
        print(f'  {name}: {angle:.6f}')
    print(f'Current → neutral reference: {neutral.plan.duration:.3f}s')
    if not execute:
        print('Preview only. Add --execute for current → neutral → saved posture.')
        return

    robot.move_to_neutral(execute=True)
    # The controller reads fresh feedback and plans this leg from the reached neutral pose.
    result = robot.move_to_joints(target, execute=True)
    error = max(abs(goal - actual) for goal, actual in zip(target, robot.joint_positions, strict=True))
    print(f'Neutral → saved reference: {result.plan.duration:.3f}s')
    print(f'Saved posture reached. Maximum joint error: {error:.6f} rad')


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    capture = commands.add_parser('save', help='Save the live arm position without moving')
    capture.add_argument('path', type=Path, nargs='?', default=DEFAULT_POSE)
    restore = commands.add_parser('restore', help='Move to neutral, then the saved arm position')
    restore.add_argument('path', type=Path, nargs='?', default=DEFAULT_POSE)
    restore.add_argument('--execute', action='store_true', help='Send motion commands; default: preview')
    restore.add_argument('--speed', type=float, default=0.1, help='Fraction of joint velocity limits (default: 0.1)')
    restore.add_argument('--max-accel', type=float, default=0.3, help='Acceleration cap in rad/s² (default: 0.3)')
    args = parser.parse_args(argv)
    try:
        target = load_pose(args.path) if args.command == 'restore' else None
        motion = CartesianMotionOptions(speed=getattr(args, 'speed', 0.1), max_accel=getattr(args, 'max_accel', 0.3))
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    with SawyerController(SawyerConfig(enable_gripper=False, motion=motion)) as robot:
        if args.command == 'save':
            save_pose(robot, args.path)
        else:
            restore_pose(robot, target, execute=args.execute)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
