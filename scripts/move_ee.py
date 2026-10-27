#!/usr/bin/env python3
"""Move the EE along a straight Cartesian reference: z:+10 means +10 centimeters."""

from dataclasses import fields
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions

if __package__:
    from .arm_motion import add_motion_arguments, positive
    from .cartesian_motion import CartesianMotionNode, CartesianMotionOptions
    from .cartesian_path import parse_offsets
else:
    from arm_motion import add_motion_arguments, positive
    from cartesian_motion import CartesianMotionNode, CartesianMotionOptions
    from cartesian_path import parse_offsets


def build_parser():
    import argparse
    parser = add_motion_arguments(argparse.ArgumentParser(description=__doc__))
    parser.add_argument('offsets', nargs='+', metavar='AXIS:CM', help='Relative offsets, e.g. x:+5 y:-3 z:+10')
    parser.add_argument('--tip', help='Named IK tip; default: match the current endpoint to tip_states')
    parser.add_argument('--step-mm', type=positive, default=5.0, help='Maximum initial Cartesian sample spacing (mm)')
    parser.add_argument('--path-tolerance', type=positive, default=.001, help='FK interpolation tolerance (meters)')
    parser.add_argument('--max-cartesian-error', type=positive, default=.02, help='Abort EE tracking error (meters)')
    parser.add_argument('--max-samples', type=int, default=401, help='Maximum IK knots after refinement (>=11)')
    parser.add_argument('--ik-timeout', type=positive, default=2.0, help='IK/FK request timeout (seconds)')
    parser.add_argument('--planning-timeout', type=positive, default=60.0, help='Whole kinematics planning timeout (s)')
    return parser


def main():
    parser = build_parser()
    arguments = parser.parse_args()
    try:
        offset = parse_offsets(arguments.offsets)
        values = vars(arguments).copy()
        values['sample_step'] = arguments.step_mm / 1000
        names = {field.name for field in fields(CartesianMotionOptions)}
        options = CartesianMotionOptions(**{key: value for key, value in values.items() if key in names})
    except ValueError as exc:
        parser.error(str(exc))
    node = None
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = CartesianMotionNode(frame=options.frame)
        node.move_relative(*offset, duration=options.time, execute=options.execute, options=options)
        return 0
    except KeyboardInterrupt:
        print('Cartesian move interrupted; check that the robot has stopped.', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'Cartesian motion error: {exc}', file=sys.stderr)
        return 1
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
