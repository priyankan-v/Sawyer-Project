"""
Preview or execute Sawyer's configured neutral pose through the irobot driver.
"""

# Command-line example output.
# ruff: noqa: T201
from __future__ import annotations

import argparse

from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.config import JOINT_NAMES, CartesianMotionOptions


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--time',
        type=float,
        help='Reference duration in seconds; default: minimum feasible',
    )
    parser.add_argument(
        '--speed',
        type=float,
        default=1.0,
        help='Fraction of each published joint velocity limit (default: 1.0)',
    )
    parser.add_argument(
        '--max-accel',
        type=float,
        default=0.3,
        help='Acceleration cap in rad/s² (default: 0.3)',
    )
    parser.add_argument(
        '--execute',
        action='store_true',
        help='Execute after printing the preview',
    )
    args = parser.parse_args(argv)
    try:
        options = CartesianMotionOptions(
            speed=args.speed, max_accel=args.max_accel, time=args.time
        )
    except ValueError as exc:
        parser.error(str(exc))

    # Neutral motion only needs the arm; do not depend on or command the
    # gripper.
    config = SawyerConfig(enable_gripper=False, motion=options)
    with SawyerController(config) as robot:
        preview = robot.move_to_neutral(duration=args.time)
        print('Joint       Current (rad)   Neutral (rad)')
        for name, start, target in zip(
            JOINT_NAMES, preview.plan.start, preview.plan.end, strict=True
        ):
            print(f'{name:10} {start:13.4f} {target:15.4f}')
        print(
            f'Reference: {preview.plan.duration:.3f}s; '
            f'minimum feasible: {preview.plan.minimum_duration:.3f}s'
        )
        if not args.execute:
            print('Preview only. Add --execute to move Sawyer to neutral.')
            return 0

        # Reacquire feedback and replan through the public API after terminal
        # output.
        result = robot.move_to_neutral(duration=args.time, execute=True)
        measured = robot.joint_positions
        error = max(
            abs(target - actual)
            for target, actual in zip(result.plan.end, measured, strict=True)
        )
        if result.execution is None:
            print(
                'Already within neutral tolerance. '
                f'Maximum joint error: {error:.6f} rad'
            )
        else:
            print(
                'Neutral reached. Reference: '
                f'{result.execution.reference_duration:.3f}s; '
                'tracking + settling: '
                f'{result.execution.elapsed_duration:.3f}s; '
                f'maximum joint error: {error:.6f} rad'
            )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
