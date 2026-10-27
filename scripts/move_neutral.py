#!/usr/bin/env python3
"""Preview or execute a quintic neutral-pose trajectory with joint position PID."""

import argparse
import os
import sys
import time
import xmlrpc.client
import rclpy
from rclpy.signals import SignalHandlerOptions

if __package__:
    from .arm_motion import (ArmMotionNode, JointBounds, JOINT_NAMES, MotionOptions,
                             POSITION_TOLERANCE, VELOCITY_TOLERANCE,
                             add_motion_arguments, ordered_values, validate_motion_options)
    from .joint_feedback import JointFeedback
    from .motion_utils import vector
    from .pid_controller import JointPIDController
    from .quintic_trajectory import QuinticTrajectory
else:
    from arm_motion import (ArmMotionNode, JointBounds, JOINT_NAMES, MotionOptions,
                            POSITION_TOLERANCE, VELOCITY_TOLERANCE,
                            add_motion_arguments, ordered_values, validate_motion_options)
    from joint_feedback import JointFeedback
    from motion_utils import vector
    from pid_controller import JointPIDController
    from quintic_trajectory import QuinticTrajectory

class ParameterTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 5.0
        return connection


def read_neutral(master_uri):
    """Read the same named pose used by the ROS 1 SDK's move_to_neutral()."""
    with xmlrpc.client.ServerProxy(master_uri, transport=ParameterTransport()) as master:
        code, message, named = master.getParam("/ros2_move_neutral", "/named_poses/right")
    if code != 1:
        raise RuntimeError(f"Cannot read /named_poses/right: {message}")
    names = named["joint_names"]
    positions = named["poses"]["neutral"]
    if len(names) != 7 or len(positions) != 7 or set(names) != set(JOINT_NAMES):
        raise ValueError("Neutral pose must contain the seven unique right arm joints")
    angles = dict(zip(names, positions))
    return vector([angles[name] for name in JOINT_NAMES], 7, "neutral pose")


def plan_trajectory(current, target, bounds, options):
    bounds.check_position(current, "Current position", tolerance=1e-4)
    bounds.check_position(target, "Neutral position")
    velocity = tuple(options.speed * value for value in bounds.velocity)
    acceleration = tuple(min(value, options.max_accel) if options.max_accel is not None else value
                         for value in bounds.acceleration)
    return QuinticTrajectory(current, target, velocity, duration=options.time,
                             max_acceleration=acceleration)


class MoveNeutral(ArmMotionNode):
    def __init__(self):
        super().__init__("move_neutral")

    def track(self, trajectory, controller, options):
        return super().track(trajectory, controller, options, label="Neutral pose")

    def run(self, target, options):
        self.wait_inputs(options.execute)
        trajectory = plan_trajectory(self.joints, target, self.limits, options)
        print("Joint       Current (rad)   Neutral (rad)")
        for name, current, desired in zip(JOINT_NAMES, self.joints, target):
            print(f"{name:10} {current:13.4f} {desired:15.4f}")
        print(f"Quintic duration: {trajectory.duration:.6f} s; "
              f"minimum feasible: {trajectory.minimum_duration:.6f} s")
        print(f"Velocity bound requires {trajectory.velocity_duration:.6f} s; "
              f"acceleration bound requires {trajectory.acceleration_duration:.6f} s")
        if not options.execute:
            print("Preview only. Add --execute to stream PID velocity commands.")
            return
        # Printing can block: service feedback again before capturing the start.
        self.wait_inputs(True)
        self.check_ready()
        if max(map(abs, self.velocity)) >= VELOCITY_TOLERANCE:
            raise RuntimeError("Rest-to-rest planning requires the arm to be stationary before starting")
        if max(abs(a - b) for a, b in zip(self.joints, target)) < POSITION_TOLERANCE:
            print("Already within neutral pose tolerance; no motion requested.")
            return
        trajectory = plan_trajectory(self.joints, target, self.limits, options)
        controller = JointPIDController(
            7, options.kp, options.ki, options.kd, trajectory.max_velocity, trajectory.max_acceleration,
            position_lower=self.limits.lower, position_upper=self.limits.upper,
        )
        self.track(trajectory, controller, options)



def build_parser():
    return add_motion_arguments(argparse.ArgumentParser(description=__doc__))


def main():
    parser = build_parser()
    options = parser.parse_args()
    try:
        validate_motion_options(options)
    except ValueError as exc:
        parser.error(str(exc))
    node = None
    try:
        master_uri = os.environ.get("ROS_MASTER_URI")
        if not master_uri:
            raise RuntimeError("ROS_MASTER_URI is unset; source scripts/env.sh first")
        target = read_neutral(master_uri)
        # Keep Python's KeyboardInterrupt so the finally block can request a stop.
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = MoveNeutral()
        node.run(target, options)
        return 0
    except KeyboardInterrupt:
        print("Interrupted; stop requested if streaming. Check that the robot has stopped.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Neutral motion error: {exc}", file=sys.stderr)
        return 1
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    sys.exit(main())
