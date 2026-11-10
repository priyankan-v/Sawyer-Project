"""Read Sawyer state through the public driver API; sends no robot commands."""

# Command-line example output.
# ruff: noqa: T201
from __future__ import annotations

import argparse
import pprint

from irobot import SawyerConfig, SawyerController


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-gripper', action='store_true')
    args = parser.parse_args()
    with SawyerController(SawyerConfig(enable_gripper=not args.no_gripper)) as robot:
        pprint.pp(robot.get_status())
        print('Joint positions (rad):', robot.joint_positions)
        print('Endpoint pose (m, quaternion xyzw):', robot.endpoint_pose)


if __name__ == '__main__':
    main()
