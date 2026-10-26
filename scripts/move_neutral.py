#!/usr/bin/env python3
"""Preview or execute Sawyer's configured neutral pose through the ROS 2 bridge."""

import argparse
import math
import os
import sys
import time
import xmlrpc.client

import rclpy
from action_msgs.msg import GoalStatus
from intera_core_msgs.msg import RobotAssemblyState
from intera_motion_msgs.action import MotionCommand
from intera_motion_msgs.msg import TrajectoryOptions, Waypoint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


JOINT_NAMES = [f"right_j{i}" for i in range(7)]


class ParameterTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 5.0
        return connection


def read_neutral(master_uri):
    """Read the same named pose used by the ROS 1 SDK's move_to_neutral()."""
    with xmlrpc.client.ServerProxy(
        master_uri, transport=ParameterTransport()
    ) as master:
        code, message, named = master.getParam("/ros2_move_neutral", "/named_poses/right")
    if code != 1:
        raise RuntimeError(f"Cannot read /named_poses/right: {message}")
    names = named["joint_names"]
    positions = named["poses"]["neutral"]
    if len(names) != 7 or len(positions) != 7 or set(names) != set(JOINT_NAMES):
        raise ValueError("Neutral pose must contain the seven unique right arm joints")
    angles = dict(zip(names, positions))
    target = [float(angles[name]) for name in JOINT_NAMES]
    if not all(math.isfinite(angle) for angle in target):
        raise ValueError("Neutral pose contains a non-finite joint angle")
    return target


def make_goal(current, target, speed):
    goal = MotionCommand.Goal()
    goal.command = MotionCommand.Goal.MOTION_START
    goal.trajectory.joint_names = JOINT_NAMES
    goal.trajectory.trajectory_options.interpolation_type = TrajectoryOptions.JOINT
    for positions in (current, target):
        waypoint = Waypoint()
        waypoint.joint_positions = list(positions)
        waypoint.active_endpoint = "right_hand"
        waypoint.options.max_joint_speed_ratio = speed
        waypoint.options.max_joint_accel = [0.3] * 7  # rad/s^2
        waypoint.options.joint_tolerances = [0.05] * 7
        goal.trajectory.waypoints.append(waypoint)
    return goal


class MoveNeutral(Node):
    def __init__(self):
        super().__init__("move_neutral")
        self.joints = None
        self.state = None
        self.joints_at = self.state_at = 0.0
        self.goal_handle = None
        self.create_subscription(
            JointState, "/robot/joint_states", self.on_joints, qos_profile_sensor_data
        )
        self.create_subscription(
            RobotAssemblyState, "/robot/state", self.on_state, qos_profile_sensor_data
        )
        self.client = ActionClient(self, MotionCommand, "/motion/motion_command")

    def on_joints(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values and math.isfinite(values[name]) for name in JOINT_NAMES):
            self.joints = [values[name] for name in JOINT_NAMES]
            self.joints_at = time.monotonic()

    def on_state(self, message):
        self.state = message
        self.state_at = time.monotonic()

    def fresh(self):
        now = time.monotonic()
        return now - self.joints_at < 1.0 and now - self.state_at < 1.0

    def check_ready(self):
        if not self.fresh():
            raise RuntimeError("Robot telemetry is missing or stale; check the bridge")
        if not self.state.enabled or not self.state.ready or self.state.stopped or self.state.error:
            raise RuntimeError("Robot must be enabled, ready, and free of stop/error conditions")

    def wait_for(self, future, timeout, monitor=False):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError("Timed out waiting for the motion action")
            rclpy.spin_once(self, timeout_sec=0.05)
            if monitor:
                self.check_ready()
        return future.result()

    def cancel(self):
        if self.goal_handle is None:
            return
        try:
            response = self.wait_for(self.goal_handle.cancel_goal_async(), 5.0)
            if not response.goals_canceling:
                self.get_logger().warning("Controller did not acknowledge cancellation")
        except Exception as exc:
            self.get_logger().warning(f"Could not confirm cancellation: {exc}")

    def run(self, target, execute, speed, timeout):
        if execute and not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/motion/motion_command is unavailable; check bridge and robot")
        deadline = time.monotonic() + 10.0
        while not self.fresh():
            if time.monotonic() >= deadline:
                raise RuntimeError("No fresh joint/robot state; start the bridge first")
            rclpy.spin_once(self, timeout_sec=0.1)
        print("Joint       Current (rad)   Neutral (rad)")
        for name, current, desired in zip(JOINT_NAMES, self.joints, target):
            print(f"{name:10} {current:13.4f} {desired:15.4f}")
        if not execute:
            print("Preview only. Use --execute to request this motion.")
            return
        self.check_ready()
        future = self.client.send_goal_async(make_goal(self.joints, target, speed))
        try:
            self.goal_handle = self.wait_for(future, 10.0)
        except BaseException:
            # A delayed acknowledgement must not leave an unobserved goal running.
            def cancel_late_goal(done):
                handle = done.result()
                if handle and handle.accepted:
                    handle.cancel_goal_async()
            future.add_done_callback(cancel_late_goal)
            until = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().warning("Goal acknowledgement failed; motion state is uncertain")
            raise
        if not self.goal_handle.accepted:
            raise RuntimeError("Motion goal was rejected")
        try:
            response = self.wait_for(self.goal_handle.get_result_async(), timeout, monitor=True)
            if response.status != GoalStatus.STATUS_SUCCEEDED or not response.result.result:
                raise RuntimeError(f"Motion failed: {response.result.error_id or response.status}")
        except BaseException:
            self.cancel()
            raise
        # Confirm arrival using fresh measured joint positions, not just goal acceptance.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.check_ready()
            if max(abs(a - b) for a, b in zip(self.joints, target)) < 0.02:
                print("Neutral pose reached (all joints within 0.02 rad).")
                return
        raise RuntimeError("Action finished but measured joints did not confirm neutral pose")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Send the motion; default is preview")
    parser.add_argument("--speed", type=float, default=0.1, help="Joint speed ratio (default: 0.1)")
    parser.add_argument("--timeout", type=float, default=90.0, help="Motion timeout in seconds")
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0.05 <= args.speed <= 1.0:
        parser.error("--speed must be between 0.05 and 1.0")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    node = None
    try:
        master_uri = os.environ.get("ROS_MASTER_URI")
        if not master_uri:
            raise RuntimeError("ROS_MASTER_URI is unset; source scripts/env.sh first")
        target = read_neutral(master_uri)
        # Keep Python's KeyboardInterrupt so the active goal can be canceled first.
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = MoveNeutral()
        node.run(target, args.execute, args.speed, args.timeout)
        return 0
    except KeyboardInterrupt:
        print("Interrupted; check that the robot has stopped.", file=sys.stderr)
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
