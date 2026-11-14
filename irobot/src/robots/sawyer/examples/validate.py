"""
Preview or execute a bounded arm round trip and optional empty-gripper check.
"""

# Explicit validation-command output.
# ruff: noqa: T201
from __future__ import annotations

import argparse
import json
import math

from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.config import CartesianMotionOptions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--execute',
        action='store_true',
        help='Execute the 1 cm upward-and-return arm test',
    )
    parser.add_argument(
        '--gripper',
        action='store_true',
        help='Also calibrate/open/close/open the empty gripper',
    )
    parser.add_argument(
        '--no-gripper',
        action='store_true',
        help='Connect without electric-gripper feedback',
    )
    args = parser.parse_args()
    if args.gripper and args.no_gripper:
        parser.error('--gripper and --no-gripper cannot be combined')
    options = CartesianMotionOptions(speed=0.05, max_accel=0.1)
    report = {'executed': args.execute}
    with SawyerController(
        SawyerConfig(motion=options, enable_gripper=not args.no_gripper)
    ) as robot:
        report['initial_status'] = robot.get_status()
        start = robot.endpoint_pose
        report['neutral_preview_duration_s'] = (
            robot.move_to_neutral().plan.duration
        )
        outward = robot.move_relative(
            dz_cm=1, duration=3, execute=args.execute
        )
        report['outward_sampled_fk_error_m'] = outward.plan.max_position_error
        if args.execute:
            raised = robot.endpoint_pose
            report['outward_measured_error_m'] = math.dist(
                raised.position, outward.plan.line.end.position
            )
            offsets = tuple(
                100 * (target - current)
                for target, current in zip(
                    start.position, raised.position, strict=True
                )
            )
            robot.move_relative(*offsets, duration=3, execute=True)
            report['round_trip_error_m'] = math.dist(
                robot.endpoint_pose.position, start.position
            )
            if report['round_trip_error_m'] >= options.cartesian_tolerance:
                msg = (
                    'Round-trip error exceeds the configured Cartesian '
                    'arrival tolerance'
                )
                raise RuntimeError(msg)
        if args.gripper:
            state = robot.gripper_state
            if (
                state['is_gripping']
                or state['is_moving']
                or state['has_error']
            ):
                msg = (
                    'Gripper must be empty, stationary, and free of errors '
                    'for this test'
                )
                raise RuntimeError(msg)
            report['calibration'] = robot.calibrate_gripper(
                execute=args.execute
            )
            report['open'] = robot.open_gripper(execute=args.execute)
            report['close'] = robot.close_gripper(execute=args.execute)
            report['final_open'] = robot.open_gripper(execute=args.execute)
            report['gripper_state'] = robot.gripper_state
        report['final_status'] = robot.get_status()
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
