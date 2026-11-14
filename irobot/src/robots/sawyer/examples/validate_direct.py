"""Read-only validation by default; --execute adds arm and gripper tests."""

# Standalone operator example with JSON output.
# ruff: noqa: T201
from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path

from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.config import CartesianMotionOptions


def validate(robot, *, execute=False):
    report = {'initial_status': robot.get_status(), 'execute': execute}
    if not report['initial_status']['ready']:
        if execute:
            msg = 'Robot is not ready; physical validation cannot start'
            raise RuntimeError(msg)
        report['motion_validation'] = 'skipped: controller is not ready'
        report['robot_state'] = robot.devices.robot_state()
        report['final_status'] = robot.get_status()
        return report
    joints = robot.joint_positions
    pose = robot.endpoint_pose
    report['pose'] = asdict(pose)
    report['fk'] = asdict(robot.forward_kinematics([joints])[0])
    report['ik'] = robot.inverse_kinematics(pose, seed=joints)
    preview = robot.move_to_neutral()
    report['neutral_duration_s'] = preview.plan.duration
    if execute:
        # FK checks controller-reported feasibility, not external obstacles.
        robot.forward_kinematics(
            [
                preview.plan.sample(preview.plan.duration * i / 200).position
                for i in range(201)
            ]
        )
        result = robot.move_to_neutral(execute=True)
        report['neutral_error_rad'] = max(
            abs(a - b)
            for a, b in zip(
                result.plan.end, robot.joint_positions, strict=True
            )
        )
        start = robot.endpoint_pose
        robot.move_relative(dz_cm=1, duration=3, execute=True)
        robot.move_relative(dz_cm=-1, duration=3, execute=True)
        report['cartesian_start'] = asdict(start)
        report['cartesian_final'] = asdict(robot.endpoint_pose)
        robot.calibrate_gripper(execute=True)
        robot.open_gripper(execute=True)
        robot.close_gripper(execute=True)
        robot.open_gripper(execute=True)
    report['gripper'] = robot.gripper_state
    report['final_status'] = robot.get_status()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master', required=True)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--tip', default=None)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument(
        '--execute',
        action='store_true',
        help=(
            'Run neutral, Cartesian, and empty-gripper tests. Requires '
            'a clear '
            'path, empty gripper, and operator with emergency-stop access.'
        ),
    )
    args = parser.parse_args()
    report = {'date_utc': datetime.now(UTC).isoformat()}
    config = SawyerConfig(
        ros_master_uri=args.master,
        ros_ip=args.ip,
        tip=args.tip,
        position_tolerance=0.002,
        motion=CartesianMotionOptions(
            speed=0.05, max_accel=0.1, max_error=0.08
        ),
    )
    try:
        with SawyerController(config) as robot:
            report.update(validate(robot, execute=args.execute))
        report['success'] = True
    except (OSError, RuntimeError, ValueError) as exc:
        report.update(success=False, error=str(exc))
        raise
    finally:
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
