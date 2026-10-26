#!/usr/bin/env python3
"""Preview or execute a quintic neutral-pose trajectory with joint position PID."""

import argparse
from dataclasses import dataclass
import math
import os
import sys
import time
import xmlrpc.client

import rclpy
from intera_core_msgs.msg import CollisionDetectionState, JointCommand, JointLimits, RobotAssemblyState
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

if __package__:
    from .joint_feedback import JointFeedback
    from .motion_utils import vector
    from .pid_controller import JointPIDController
    from .quintic_trajectory import QuinticTrajectory
else:
    from joint_feedback import JointFeedback
    from motion_utils import vector
    from pid_controller import JointPIDController
    from quintic_trajectory import QuinticTrajectory


JOINT_NAMES = [f"right_j{i}" for i in range(7)]
COMMAND_TOPIC = "/robot/limb/right/joint_command"
WATCHDOG = 0.2
FEEDBACK_TIMEOUT = 0.2
POSITION_TOLERANCE = 0.02
VELOCITY_TOLERANCE = 0.03


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


def ordered_values(names, values, label):
    if len(names) != len(values) or len(set(names)) != len(names):
        raise ValueError(f"Malformed {label} names/values")
    values = dict(zip(names, values))
    if not all(name in values for name in JOINT_NAMES):
        raise ValueError(f"{label} is missing right arm joints")
    return vector([values[name] for name in JOINT_NAMES], 7, label)


@dataclass(frozen=True)
class JointBounds:
    lower: tuple
    upper: tuple
    velocity: tuple
    acceleration: tuple

    @classmethod
    def from_message(cls, message):
        bounds = cls(*(ordered_values(message.joint_names, getattr(message, field), field)
                       for field in ("position_lower", "position_upper", "velocity", "accel")))
        if any(a >= b for a, b in zip(bounds.lower, bounds.upper)):
            raise ValueError("Invalid joint position bounds")
        if any(value <= 0 for value in bounds.velocity + bounds.acceleration):
            raise ValueError("Joint velocity/acceleration limits must be positive")
        return bounds

    def check_position(self, positions, label, tolerance=0.0):
        positions = vector(positions, 7, label)
        for name, position, lower, upper in zip(JOINT_NAMES, positions, self.lower, self.upper):
            if not lower - tolerance <= position <= upper + tolerance:
                raise ValueError(f"{label}: {name}={position:.6f} outside [{lower}, {upper}]")


def plan_trajectory(current, target, bounds, options):
    bounds.check_position(current, "Current position", tolerance=1e-4)
    bounds.check_position(target, "Neutral position")
    velocity = tuple(options.speed * value for value in bounds.velocity)
    acceleration = tuple(min(value, options.max_accel) if options.max_accel is not None else value
                         for value in bounds.acceleration)
    return QuinticTrajectory(current, target, velocity, duration=options.time,
                             max_acceleration=acceleration)


class MoveNeutral(Node):
    def __init__(self):
        super().__init__("move_neutral")
        self.joints = self.velocity = self.state = self.limits = self.collision = None
        self.feedback = JointFeedback(JOINT_NAMES)
        self.last_feedback_error = self.last_limits_error = None
        self.joints_at = self.state_at = self.collision_at = 0.0
        self.streaming = False
        self.create_subscription(JointState, "/robot/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(RobotAssemblyState, "/robot/state", self.on_state, qos_profile_sensor_data)
        self.create_subscription(CollisionDetectionState, "/robot/limb/right/collision_detection_state",
                                 self.on_collision, qos_profile_sensor_data)
        limits_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointLimits, "/robot/joint_limits", self.on_limits, limits_qos)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 lifespan=Duration(seconds=WATCHDOG))
        self.command_pub = self.create_publisher(JointCommand, COMMAND_TOPIC, command_qos)
        self.watchdog_pub = self.create_publisher(Float64, COMMAND_TOPIC + "_timeout", 1)

    def on_joints(self, message):
        try:
            updated = self.feedback.update(message.name, message.position, message.velocity, time.monotonic())
        except ValueError as exc:
            self.last_feedback_error = str(exc)
            return
        if updated:
            self.joints, self.velocity, self.joints_at = self.feedback.snapshot()
            self.last_feedback_error = None

    def on_state(self, message):
        self.state, self.state_at = message, time.monotonic()

    def on_collision(self, message):
        self.collision, self.collision_at = message.collision_state, time.monotonic()

    def on_limits(self, message):
        try:
            self.limits = JointBounds.from_message(message)
            self.last_limits_error = None
        except ValueError as exc:
            self.limits = None
            self.last_limits_error = str(exc)
            self.get_logger().error(str(exc))

    def input_issues(self):
        now = time.monotonic()
        issues = []
        joint_issues = self.feedback.issues(now, FEEDBACK_TIMEOUT)
        if joint_issues:
            issues.append('/robot/joint_states: ' + '; '.join(joint_issues))
            if self.last_feedback_error:
                issues.append('last rejected arm sample: ' + self.last_feedback_error)
        for topic, value, stamp in (
            ('/robot/state', self.state, self.state_at),
            ('/robot/limb/right/collision_detection_state', self.collision, self.collision_at),
        ):
            if value is None:
                issues.append(f'{topic}: no sample received')
            elif now - stamp >= FEEDBACK_TIMEOUT:
                issues.append(f'{topic}: {now - stamp:.3f}s old (limit {FEEDBACK_TIMEOUT:.3f}s)')
        if self.limits is None:
            issues.append('/robot/joint_limits: ' + (self.last_limits_error or 'no valid sample received'))
        return issues

    def fresh(self):
        return not self.input_issues()

    def check_ready(self):
        issues = self.input_issues()
        if issues:
            raise RuntimeError("Missing/stale feedback: " + '; '.join(issues))
        if not self.state.enabled or not self.state.ready or self.state.stopped or self.state.error:
            raise RuntimeError(f"Robot not ready: enabled={self.state.enabled}, ready={self.state.ready}, "
                               f"stopped={self.state.stopped}, error={self.state.error}")
        if self.collision:
            raise RuntimeError("Robot reports a collision")
        self.limits.check_position(self.joints, "Measured position", tolerance=1e-4)

    def wait_inputs(self, execute):
        deadline = time.monotonic() + 10.0
        while not self.fresh() or (execute and not self.routes_connected()):
            if time.monotonic() >= deadline:
                issues = self.input_issues()
                if execute and not self.routes_connected():
                    issues.append('command/watchdog route absent; launch bridge with commands:=true')
                raise RuntimeError('Preflight timed out: ' + '; '.join(issues))
            rclpy.spin_once(self, timeout_sec=0.05)

    def routes_connected(self):
        return (self.command_pub.get_subscription_count() > 0
                and self.watchdog_pub.get_subscription_count() > 0)

    def publish_velocity(self, velocity):
        message = JointCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.mode = JointCommand.VELOCITY_MODE
        message.names = JOINT_NAMES
        message.velocity = list(vector(velocity, 7, "velocity command"))
        self.command_pub.publish(message)

    def spin_until(self, deadline):
        """Service callbacks for a real interval even when callbacks return immediately."""
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.01, deadline - time.monotonic())))

    def stop(self):
        """Request zero velocity, then measured-position hold as in SDK exit_control_mode.

        Delivery and physical stopping cannot be confirmed after a connection loss.
        The robot's command watchdog remains set to 0.2 s for velocity mode.
        """
        if not self.streaming:
            return
        try:
            for _ in range(5):
                self.publish_velocity((0.0,) * 7)
                self.spin_until(time.monotonic() + 0.02)
            positions = self.feedback.fresh_positions(time.monotonic(), FEEDBACK_TIMEOUT)
            if positions is not None:
                hold = JointCommand()
                hold.mode = JointCommand.POSITION_MODE
                hold.names = JOINT_NAMES
                hold.position = list(positions)
                for _ in range(3):
                    hold.header.stamp = self.get_clock().now().to_msg()
                    self.command_pub.publish(hold)
                    self.spin_until(time.monotonic() + 0.02)
        except Exception as exc:
            self.get_logger().warning(f"Stop command delivery could not be confirmed: {exc}")
        finally:
            self.streaming = False

    def track(self, trajectory, controller, options):
        period = 1.0 / options.rate
        timeout = options.timeout if options.timeout is not None else trajectory.duration + options.settle_time
        if timeout < trajectory.duration:
            raise ValueError("--timeout must be at least the planned motion duration")
        self.check_ready()
        controller.reset(self.velocity)
        self.watchdog_pub.publish(Float64(data=WATCHDOG))
        start = time.monotonic()
        previous, next_tick = start - period, start
        stable_since = None
        next_watchdog = start + 1.0
        self.streaming = True
        try:
            while True:
                now = time.monotonic()
                if now < next_tick:
                    rclpy.spin_once(self, timeout_sec=min(period, next_tick - now))
                    continue
                dt, elapsed = now - previous, now - start
                if dt > WATCHDOG / 2:
                    raise RuntimeError("Control loop missed its deadline (>0.1 s); stopping")
                if elapsed > timeout:
                    raise TimeoutError("Motion/settling timeout; neutral pose was not confirmed")
                self.check_ready()
                if not self.routes_connected():
                    raise RuntimeError("Bridge command subscription disappeared")
                # Do not keep using a plan after a live limits update.
                if any(a < b for a, b in zip(self.limits.velocity, trajectory.max_velocity)) or any(
                    a < b for a, b in zip(self.limits.acceleration, trajectory.max_acceleration)
                ):
                    raise RuntimeError("Robot limits changed during the move; replan required")
                self.limits.check_position(trajectory.end, "Neutral position")
                reference = trajectory.sample(elapsed)
                if max(abs(a - b) for a, b in zip(reference.position, self.joints)) > options.max_error:
                    raise RuntimeError("Joint tracking error exceeded --max-error")
                command = controller.update(reference.position, self.joints, reference.velocity, self.velocity, dt)
                self.publish_velocity(command)
                if now >= next_watchdog:
                    self.watchdog_pub.publish(Float64(data=WATCHDOG))
                    next_watchdog = now + 1.0
                arrived = (elapsed >= trajectory.duration
                           and max(abs(a - b) for a, b in zip(trajectory.end, self.joints)) < POSITION_TOLERANCE
                           and max(map(abs, self.velocity)) < VELOCITY_TOLERANCE
                           and max(map(abs, command)) < VELOCITY_TOLERANCE)
                stable_since = (now if stable_since is None else stable_since) if arrived else None
                if stable_since is not None and now - stable_since >= 0.2:
                    break
                previous, next_tick = now, now + period
        finally:
            self.stop()
        # Stop streaming before potentially blocking on terminal output.
        print(f"Neutral pose reached. Reference: {trajectory.duration:.3f} s; "
              f"tracking + settling: {elapsed:.3f} s")

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


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be positive and finite")
    return result


def nonnegative(value):
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise argparse.ArgumentTypeError("must be nonnegative and finite")
    return result


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Stream commands; default is preview")
    parser.add_argument("--time", type=positive, help="Reference motion duration in seconds; default: minimum feasible")
    parser.add_argument("--speed", type=positive, default=1.0, help="Scale robot velocity limits in (0,1] (default: 1)")
    parser.add_argument("--max-accel", type=positive, help="Optional cap on robot acceleration limits (rad/s^2)")
    parser.add_argument("--rate", type=positive, default=100.0, help="PID publishing frequency, 50..250 Hz (default: 100)")
    parser.add_argument("--kp", type=nonnegative, default=2.0, help="Position P gain (1/s); default: 2")
    parser.add_argument("--ki", type=nonnegative, default=0.1, help="Position I gain (1/s^2); default: 0.1")
    parser.add_argument("--kd", type=nonnegative, default=0.05, help="Velocity-error D gain; default: 0.05")
    parser.add_argument("--settle-time", type=positive, default=5.0, help="Extra settling allowance after trajectory (s)")
    parser.add_argument("--timeout", type=positive, help="Total tracking timeout (s); default: duration + settle-time")
    parser.add_argument("--max-error", type=positive, default=0.35, help="Abort tracking-error threshold (rad)")
    return parser


def main():
    parser = build_parser()
    options = parser.parse_args()
    if options.speed > 1:
        parser.error("--speed must be in (0,1]")
    if not 50 <= options.rate <= 250:
        parser.error("--rate must be between 50 and 250 Hz")
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
