"""Reusable ROS Cartesian move API built on the shared joint PID executor."""

from dataclasses import dataclass, replace
import math
import time

import rclpy
from intera_core_msgs.msg import EndpointState, EndpointStates
from rclpy.qos import qos_profile_sensor_data

if __package__:
    from .arm_motion import ArmMotionNode, JOINT_NAMES, MotionOptions, MotionResult, VELOCITY_TOLERANCE
    from .cartesian_path import pose_error
    from .cartesian_planner import CartesianPlan, CartesianPlanner
    from .pid_controller import JointPIDController
    from .ros_kinematics import EndpointFeedback, InteraKinematics
else:
    from arm_motion import ArmMotionNode, JOINT_NAMES, MotionOptions, MotionResult, VELOCITY_TOLERANCE
    from cartesian_path import pose_error
    from cartesian_planner import CartesianPlan, CartesianPlanner
    from pid_controller import JointPIDController
    from ros_kinematics import EndpointFeedback, InteraKinematics


@dataclass(frozen=True)
class CartesianMotionOptions(MotionOptions):
    tip: str | None = None
    frame: str = 'base'
    sample_step: float = .005
    path_tolerance: float = .001
    path_orientation_tolerance: float = .01
    max_joint_step: float = .15
    max_samples: int = 401
    cartesian_tolerance: float = .003
    goal_orientation_tolerance: float = .02
    max_cartesian_error: float = .02
    max_orientation_error: float = .1
    ik_timeout: float = 2.0
    planning_timeout: float = 60.0

    def __post_init__(self):
        super().__post_init__()
        for name in ('sample_step', 'path_tolerance', 'path_orientation_tolerance', 'max_joint_step',
                     'cartesian_tolerance', 'goal_orientation_tolerance', 'max_cartesian_error',
                     'max_orientation_error', 'ik_timeout', 'planning_timeout'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if not isinstance(self.max_samples, int) or self.max_samples < 11:
            raise ValueError('max_samples must be an integer >=11')
        if not isinstance(self.frame, str) or not self.frame.lstrip('/'):
            raise ValueError('A reference frame is required')


@dataclass(frozen=True)
class CartesianMoveResult:
    plan: CartesianPlan
    execution: MotionResult | None


class CartesianMotionNode(ArmMotionNode):
    def __init__(self, frame='base'):
        super().__init__('move_ee')
        self.pose_feedback = EndpointFeedback(frame)
        self.create_subscription(EndpointState, '/robot/limb/right/endpoint_state',
                                 self.pose_feedback.on_endpoint, qos_profile_sensor_data)
        self.create_subscription(EndpointStates, '/robot/limb/right/tip_states',
                                 self.pose_feedback.on_tips, qos_profile_sensor_data)

    def wait_pose(self, tip=None, timeout=10.0):
        deadline = time.monotonic() + timeout
        while True:
            try:
                return self.pose_feedback.select(tip)
            except RuntimeError as exc:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Cartesian pose unavailable: {exc}. Check endpoint/tip-state routes') from exc
                rclpy.spin_once(self, timeout_sec=.02)

    def planning_health(self, start_joints, start_pose, tip):
        issues = self.input_issues()
        if issues:
            # Polynomial fitting can occupy this thread; drain queued feedback
            # before treating old receipt times as a transport failure. No motion
            # is being commanded during planning. Execution retains the strict
            # per-step feedback watchdog in ArmMotionNode.
            self.wait_inputs(False, timeout=.5)
        if max(map(abs, self.velocity)) >= VELOCITY_TOLERANCE:
            raise RuntimeError('Cartesian rest-to-rest planning requires a stationary arm')
        if max(abs(a-b) for a, b in zip(self.joints, start_joints)) > .01:
            raise RuntimeError('Arm moved during Cartesian planning; rerun from the new pose')
        _, current_pose = self.wait_pose(tip, timeout=.5)
        position, orientation = pose_error(current_pose, start_pose)
        if position > .002 or orientation > .01:
            raise RuntimeError('Endpoint moved during Cartesian planning; rerun from the new pose')

    def move_relative(self, dx_cm: float = 0.0, dy_cm: float = 0.0, dz_cm: float = 0.0,
                      duration: float | None = None, execute: bool = False,
                      options: CartesianMotionOptions | None = None) -> CartesianMoveResult:
        """Preview/execute a relative XYZ translation in the pose reference frame.

        Offsets are centimeters; duration is seconds or None for minimum feasible
        uniform time scaling. Orientation is held. ROS must already be initialized;
        this method services callbacks internally and must not be called from a
        callback on the same single-threaded executor. No execution occurs without
        execute=True. The returned result contains the reusable plan and timing.
        """
        options = replace(options or CartesianMotionOptions(), time=duration, execute=execute)
        if options.frame.lstrip('/') != self.pose_feedback.frame.lstrip('/'):
            raise ValueError('Construct CartesianMotionNode with the same frame as options.frame')
        self.wait_inputs(execute)
        tip, start_pose = self.wait_pose(options.tip)
        start_joints = tuple(self.joints)
        self.planning_health(start_joints, start_pose, tip)
        velocity = tuple(options.speed * value for value in self.limits.velocity)
        acceleration = tuple(min(value, options.max_accel) if options.max_accel is not None else value
                             for value in self.limits.acceleration)
        kinematics = InteraKinematics(
            self, tip, start_pose.frame, options.ik_timeout, options.planning_timeout,
            health_check=lambda: self.planning_health(start_joints, start_pose, tip),
        )
        try:
            kinematics.wait_ready()
            planner = CartesianPlanner(
                kinematics.inverse, kinematics.forward, self.limits.lower, self.limits.upper,
                sample_step=options.sample_step, path_tolerance=options.path_tolerance,
                orientation_tolerance=options.path_orientation_tolerance,
                max_joint_step=options.max_joint_step, max_samples=options.max_samples,
                joint_names=JOINT_NAMES,
            )
            plan = planner.plan(start_pose, (dx_cm, dy_cm, dz_cm), start_joints,
                                velocity, acceleration, duration)
        finally:
            kinematics.close()
        print(f'Tip: {tip}; frame: {start_pose.frame}; orientation held fixed')
        print(f'Start XYZ (m): {plan.line.start.position}')
        print(f'Target XYZ (m): {plan.line.end.position}')
        print(f'Reference duration: {plan.trajectory.duration:.6f} s; '
              f'minimum for validated IK path: {plan.trajectory.minimum_duration:.6f} s')
        print(f'IK knots: {plan.samples}; checked FK deviation: '
              f'{plan.max_position_error:.6f} m / {plan.max_orientation_error:.6f} rad')
        if not execute:
            print('Preview only. Add --execute to send PID velocity commands.')
            return CartesianMoveResult(plan, None)
        self.wait_inputs(True)
        self.wait_pose(tip)
        self.check_ready()
        self.planning_health(start_joints, start_pose, tip)
        if plan.line.length == 0:
            print('Zero Cartesian offset; no motion requested.')
            return CartesianMoveResult(plan, None)
        controller = JointPIDController(
            7, options.kp, options.ki, options.kd, velocity, acceleration,
            position_lower=self.limits.lower, position_upper=self.limits.upper,
        )

        def monitor(elapsed, _joint_reference):
            fraction = elapsed / plan.trajectory.duration if plan.trajectory.duration else 1.0
            position, orientation = pose_error(self.pose_feedback.read(tip), plan.line.sample(fraction))
            if position > options.max_cartesian_error or orientation > options.max_orientation_error:
                raise RuntimeError(f'Cartesian tracking error: {position:.6f} m / {orientation:.6f} rad '
                                   'exceeds configured bounds')

        def at_goal():
            position, orientation = pose_error(self.pose_feedback.read(tip), plan.line.end)
            return position < options.cartesian_tolerance and orientation < options.goal_orientation_tolerance

        execution = self.track(plan.trajectory, controller, options, label='Cartesian target',
                               on_step=monitor, at_goal=at_goal)
        return CartesianMoveResult(plan, execution)
