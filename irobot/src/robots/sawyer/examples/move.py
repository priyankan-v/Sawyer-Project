"""Preview or explicitly execute a neutral/relative Cartesian move."""

# Command-line example output.
# ruff: noqa: T201
from __future__ import annotations

import argparse

from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.config import CartesianMotionOptions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('motion', choices=('neutral', 'relative'))
    parser.add_argument('--dx-cm', type=float, default=0.0)
    parser.add_argument('--dy-cm', type=float, default=0.0)
    parser.add_argument('--dz-cm', type=float, default=0.0)
    parser.add_argument('--time', type=float)
    parser.add_argument('--speed', type=float, default=0.1)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--no-gripper', action='store_true')
    args = parser.parse_args()
    config = SawyerConfig(enable_gripper=not args.no_gripper, motion=CartesianMotionOptions(speed=args.speed))
    with SawyerController(config) as robot:
        kwargs = {'duration': args.time, 'execute': args.execute}
        result = (
            robot.move_to_neutral(**kwargs)
            if args.motion == 'neutral'
            else robot.move_relative(args.dx_cm, args.dy_cm, args.dz_cm, **kwargs)
        )
        trajectory = result.plan if args.motion == 'neutral' else result.plan.trajectory
        print(f'Reference duration: {trajectory.duration:.3f}s; minimum: {trajectory.minimum_duration:.3f}s')
        print('Executed.' if result.execution is not None else 'Preview/no motion.')


if __name__ == '__main__':
    main()
