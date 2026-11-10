"""ROS lifecycle and reusable interaction/execution helpers for pick and place."""

from contextlib import contextmanager
from dataclasses import fields, replace
import csv
import os
from pathlib import Path
import select
import subprocess
import sys
import time
from types import SimpleNamespace

import rclpy
from rclpy.signals import SignalHandlerOptions

if __package__:
    from .arm_motion import JOINT_NAMES, VELOCITY_TOLERANCE, add_motion_arguments, positive
    from .cartesian_motion import CartesianMotionNode, CartesianMotionOptions
    from .cartesian_path import pose_error
    from .cartesian_planner import CartesianPlanner
    from .gripper_control import GripperControl, OPEN_POSITION
    from .move_neutral import read_neutral
    from .pid_controller import JointPIDController
    from .ros_kinematics import InteraKinematics
    from .pick_place_plan import Waypoint, Segment, SegmentSequence, atomic_json, pose_data, read_pose
    from .pick_place_visualization import plot_environment, plot_python
else:
    from arm_motion import JOINT_NAMES, VELOCITY_TOLERANCE, add_motion_arguments, positive
    from cartesian_motion import CartesianMotionNode, CartesianMotionOptions
    from cartesian_path import pose_error
    from cartesian_planner import CartesianPlanner
    from gripper_control import GripperControl, OPEN_POSITION
    from move_neutral import read_neutral
    from pid_controller import JointPIDController
    from ros_kinematics import InteraKinematics
    from pick_place_plan import Waypoint, Segment, SegmentSequence, atomic_json, pose_data, read_pose
    from pick_place_visualization import plot_environment, plot_python


def add_arguments(parser, recording=False):
    add_motion_arguments(parser, include_time=False, execute_default=recording)
    parser.add_argument('--file', type=Path, default=Path('data/pick_place/task.json'), help='Saved recording and plan')
    parser.add_argument('--tip', help='Named IK tip (default: identify endpoint; replay uses the saved tip)')
    parser.add_argument('--step-mm', type=positive, default=5.0, help='Initial maximum Cartesian knot spacing (mm)')
    parser.add_argument('--path-tolerance', type=positive, default=.001, help='Sampled FK position tolerance (m)')
    parser.add_argument('--orientation-tolerance', type=positive, default=.01, help='Sampled FK orientation tolerance (rad)')
    parser.add_argument('--max-samples', type=int, default=401, help='Maximum IK knots per segment (>=11)')
    parser.add_argument('--ik-timeout', type=positive, default=2.0, help='Timeout per IK/FK call (s)')
    parser.add_argument('--planning-timeout', type=positive, default=300.0, help='Planning/revalidation budget per operation (s)')
    parser.add_argument('--gripper-timeout', type=positive, default=10.0, help='Gripper completion timeout (s)')
    parser.add_argument('--output-dir', type=Path, help='Plot/CSV directory (default: beside plan)')
    return parser


def motion_options(args, saved=None):
    values = vars(args).copy()
    values.update(sample_step=args.step_mm/1000, path_orientation_tolerance=args.orientation_tolerance)
    if saved:
        # Replay cannot loosen the Cartesian tolerances used to validate recording.
        values['path_tolerance'] = min(args.path_tolerance, saved['path_tolerance'])
        values['path_orientation_tolerance'] = min(args.orientation_tolerance, saved['path_orientation_tolerance'])
        values['tip'] = saved['tip']
    names = {f.name for f in fields(CartesianMotionOptions)}
    return CartesianMotionOptions(**{k: v for k, v in values.items() if k in names})


def limits_for(arm, options):
    return (tuple(options.speed*v for v in arm.limits.velocity),
            tuple(min(a, options.max_accel) if options.max_accel is not None else a
                  for a in arm.limits.acceleration))


def neutral_pose():
    uri = os.environ.get('ROS_MASTER_URI')
    if not uri:
        raise RuntimeError('ROS_MASTER_URI is unset; source scripts/env.sh')
    return read_neutral(uri)


class Session:
    def __init__(self, options, gripper_timeout=10):
        self.options = options
        self.arm = CartesianMotionNode(frame=options.frame)
        self.gripper = None
        self.tip = options.tip
        self.rows = []
        self.started = time.monotonic()
        self.gripper_timeout = gripper_timeout
        self.live_plot = None
        # Keep both nodes registered for the lifetime of the session. Every
        # nested rclpy.spin_once() call (IK, gripper, motion) then uses this same
        # executor without adding/removing nodes or discarding pending callbacks.
        self.executor = rclpy.get_global_executor()
        self.executor.add_node(self.arm)

    def close(self):
        try:
            self.arm.stop()
        finally:
            try:
                if self.gripper is not None:
                    self.executor.remove_node(self.gripper)
                    self.gripper.destroy_node()
            finally:
                self.executor.remove_node(self.arm)
                self.arm.destroy_node()

    def pump(self):
        if not rclpy.ok():
            raise KeyboardInterrupt
        # One constant-timeout spin services the complete registered graph.
        # Alternating .01/0 spins resets rclpy's ready-callback generator each
        # time and can permanently starve the later tip-state subscription.
        self.executor.spin_once(timeout_sec=.01)

    def wait_for_enter(self, prompt):
        """Canonical terminal input without blocking ROS callbacks or a worker thread."""
        print(prompt + ' [Enter] ', end='', flush=True)
        while True:
            self.pump()
            self.record_idle_measurement()
            readable, _, _ = select.select([sys.stdin], [], [], .01)
            if readable:
                line = sys.stdin.readline()
                if not line:
                    raise EOFError('Terminal input closed; workflow cancelled')
                if not line.strip():
                    return
                print('Press Enter with no text: ', end='', flush=True)

    def current_state(self):
        self.arm.wait_inputs(self.options.execute)
        self.arm.check_ready()
        self.tip, pose = self.arm.wait_pose(self.tip)
        if pose.frame != 'base':
            raise ValueError('Pick/place requires the robot base reference frame')
        if max(map(abs, self.arm.velocity)) >= VELOCITY_TOLERANCE:
            index = max(range(7), key=lambda i: abs(self.arm.velocity[i]))
            raise RuntimeError(f'Stationary arm required: {JOINT_NAMES[index]} velocity '
                               f'{self.arm.velocity[index]:.4f} rad/s exceeds {VELOCITY_TOLERANCE:.3f}')
        return pose, tuple(self.arm.joints)

    def capture_waypoint(self, name):
        while True:
            self.wait_for_enter(f'Hand-guide to {name}; release the cuff and press Enter to record')
            try:
                pose, joints = self.current_state()
            except RuntimeError as exc:
                print(f'Point not recorded: {exc}')
                continue
            print(f'Recorded {name}: XYZ={pose.position} m; quaternion XYZW={pose.orientation}')
            return Waypoint(name, pose, joints)

    def command_gripper(self, action):
        if not self.options.execute:
            raise RuntimeError('Gripper commands require execution mode')
        if self.gripper is None:
            self.gripper = GripperControl(SimpleNamespace(action='open', execute=True,
                                          timeout=self.gripper_timeout, open_position=OPEN_POSITION))
            self.executor.add_node(self.gripper)
        self.gripper.wait_for(lambda: self.gripper.gripper is not None, self.gripper_timeout,
                              'No gripper feedback; check bridge')
        self.gripper.perform(action)
        # Gripper.perform spins its own node; refresh the arm afterward.
        self.arm.wait_inputs(self.options.execute)

    @contextmanager
    def planner(self):
        pose, joints = self.current_state()
        kinematics = InteraKinematics(self.arm, self.tip, pose.frame,
            self.options.ik_timeout, self.options.planning_timeout,
            health_check=lambda: self.arm.planning_health(joints, pose, self.tip))
        try:
            kinematics.wait_ready()
            yield CartesianPlanner(kinematics.inverse, kinematics.forward,
                self.arm.limits.lower, self.arm.limits.upper, sample_step=self.options.sample_step,
                path_tolerance=self.options.path_tolerance,
                orientation_tolerance=self.options.path_orientation_tolerance,
                max_joint_step=self.options.max_joint_step, max_samples=self.options.max_samples,
                joint_names=JOINT_NAMES)
            self.arm.planning_health(joints, pose, self.tip)
        finally:
            kinematics.close()

    def wait_at_rest(self, segment, at_end=False):
        """Confirm fresh, stable feedback after control-mode transitions.

        Position hold can briefly change measured velocity after track() has
        finished. Wait for a continuous quiet interval; never disable the
        stationary check or proceed from a displaced starting pose.
        """
        trajectory, options = segment.trajectory, self.options
        target_joints = trajectory.end if at_end else trajectory.start
        if segment.line is not None:
            target_pose = segment.line.end if at_end else segment.line.start
        else:
            target_pose = segment.reference_poses[-1 if at_end else 0][1]
        self.arm.wait_inputs(options.execute)
        # At a motion boundary, explicitly reacquire current pose feedback before
        # checking the start. Execution itself retains the strict stale-data abort.
        self.tip, _ = self.arm.wait_pose(self.tip, timeout=min(1., options.settle_time))
        self.arm.check_ready()
        deadline = time.monotonic() + options.settle_time
        stable_since, last_stamp, fresh_samples = None, None, 0
        while True:
            # Force callbacks even when cached feedback is still nominally fresh.
            self.pump()
            self.arm.check_ready()
            self.arm.check_trajectory_limits(trajectory)
            pose = self.arm.pose_feedback.read(self.tip)
            joints = tuple(self.arm.joints)
            joint_error = max(abs(a-b) for a, b in zip(joints, target_joints))
            distance, angle = pose_error(pose, target_pose)
            self.record_measurement(segment.name, trajectory.duration if at_end else 0,
                pose, target_pose if segment.line is not None else None, target_joints, event='settling')
            if joint_error > .01 or distance > options.max_cartesian_error or angle > options.max_orientation_error:
                raise RuntimeError(f'{segment.name}: arm displaced while waiting at '
                    f'{"end" if at_end else "start"}: joint {joint_error:.4f} rad, '
                    f'EE {distance:.4f} m / {angle:.4f} rad')
            speed = max(map(abs, self.arm.velocity))
            now = time.monotonic()
            ready = (speed < VELOCITY_TOLERANCE and joint_error < (.005 if at_end else .01)
                     and distance < options.cartesian_tolerance and angle < options.goal_orientation_tolerance)
            if ready:
                if stable_since is None:
                    stable_since, fresh_samples = now, 0
                if self.arm.joints_at != last_stamp:
                    fresh_samples += 1
                if now-stable_since >= .2 and fresh_samples >= 2:
                    return
            else:
                stable_since, fresh_samples = None, 0
            last_stamp = self.arm.joints_at
            if now >= deadline:
                raise TimeoutError(f'{segment.name}: arm did not settle at '
                    f'{"end" if at_end else "start"} within {options.settle_time:.1f} s; '
                    f'maximum joint speed {speed:.4f} rad/s (limit {VELOCITY_TOLERANCE:.3f}), '
                    f'joint error {joint_error:.4f} rad, EE {distance:.4f} m / {angle:.4f} rad')

    def assert_start(self, segment):
        self.wait_at_rest(segment)

    def record_measurement(self, phase, elapsed, actual, desired, reference_joints, event='tracking'):
        row = dict(time_s=time.monotonic()-self.started, phase=phase, event=event,
            segment_time_s=elapsed, actual=pose_data(actual),
            desired=None if desired is None else pose_data(desired),
            joints=tuple(self.arm.joints), joint_velocity=tuple(self.arm.velocity),
            reference_joints=tuple(reference_joints))
        self.rows.append(row)
        viewer = getattr(self, 'live_plot', None)
        if viewer is not None:
            viewer.publish(row)

    def record_idle_measurement(self):
        """Keep measured EE feedback live while waiting at pick for Enter."""
        if not getattr(self, 'rows', None):
            return
        now = time.monotonic()
        if now < getattr(self, '_next_idle_sample', 0):
            return
        self._next_idle_sample = now+.05
        try:
            actual = self.arm.pose_feedback.read(self.tip)
        except RuntimeError:
            return  # Motion preflight still rejects stale feedback before commands.
        previous = self.rows[-1]
        desired = previous['desired']
        if desired is not None:
            desired = read_pose(desired)
        self.record_measurement(previous['phase'], previous['segment_time_s'], actual,
                                desired, previous['reference_joints'], event='waiting')

    def execute_segment(self, segment):
        return self.execute_segments((segment,))

    def execute_segments(self, segments):
        """Keep one PID/controller mode across the connected segment chain."""
        if not self.options.execute:
            raise RuntimeError('Execution requires --execute')
        options = self.options
        sequence = SegmentSequence(segments, options.settle_time, options.timeout)
        self.assert_start(sequence.segments[0])
        controller = JointPIDController(7, options.kp, options.ki, options.kd,
            sequence.max_velocity, sequence.max_acceleration,
            position_lower=self.arm.limits.lower, position_upper=self.arm.limits.upper)
        quiet_since, quiet_samples, previous_stamp = None, 0, None

        def monitor(elapsed, reference):
            nonlocal quiet_since, quiet_samples, previous_stamp
            sequence.check_timeout(elapsed)
            segment = sequence.active
            local_elapsed = sequence.segment_elapsed(elapsed)
            trajectory = segment.trajectory
            actual = self.arm.pose_feedback.read(self.tip)
            fraction = min(1.0, local_elapsed/trajectory.duration) if trajectory.duration else 1.0
            desired = segment.line.sample(fraction) if segment.line is not None else None
            self.record_measurement(segment.name, local_elapsed, actual, desired, reference.position)
            if desired is not None:
                distance, angle = pose_error(actual, desired)
                if distance > options.max_cartesian_error or angle > options.max_orientation_error:
                    raise RuntimeError(f'{segment.name}: Cartesian tracking error {distance:.4f} m / {angle:.4f} rad')
            if not sequence.is_last and local_elapsed >= trajectory.duration:
                target = segment.line.end if segment.line is not None else segment.reference_poses[-1][1]
                distance, angle = pose_error(actual, target)
                arrived = (max(abs(a-b) for a,b in zip(self.arm.joints, trajectory.end)) < .005
                    and max(map(abs, self.arm.velocity)) < VELOCITY_TOLERANCE
                    and max(map(abs, controller.previous_velocity)) < VELOCITY_TOLERANCE
                    and distance < options.cartesian_tolerance and angle < options.goal_orientation_tolerance)
                if arrived:
                    quiet_since = elapsed if quiet_since is None else quiet_since
                    if self.arm.joints_at != previous_stamp:
                        quiet_samples += 1
                    # Two fresh samples and a short quiet interval replace the
                    # previous stop/hold/reset and repeated 0.2 s waits at vias.
                    if quiet_samples >= 2 and elapsed-quiet_since >= .02:
                        sequence.advance(elapsed)
                        quiet_since, quiet_samples, previous_stamp = None, 0, None
                        return
                else:
                    quiet_since, quiet_samples = None, 0
                previous_stamp = self.arm.joints_at

        def at_goal():
            if not sequence.is_last:
                return False
            # Neutral uses joint-space timing; still require fresh Cartesian feedback.
            actual = self.arm.pose_feedback.read(self.tip)
            segment = sequence.active
            target = (segment.line.end if segment.line is not None else
                      segment.reference_poses[-1][1])
            distance, angle = pose_error(actual, target)
            return (distance < options.cartesian_tolerance and angle < options.goal_orientation_tolerance
                    and max(abs(a-b) for a, b in zip(self.arm.joints, sequence.end)) < .005)

        execution_started = time.monotonic()
        execution = self.arm.track(sequence, controller, replace(options, timeout=sequence.timeout),
                                   label=sequence.segments[-1].name,
                                   on_step=monitor, at_goal=at_goal, announce=False)
        # Confirm the position-mode handoff before accepting another motion or
        # releasing the object. track() already performs its normal joint stop.
        self.wait_at_rest(sequence.segments[-1], at_end=True)
        execution = replace(execution, elapsed_duration=time.monotonic()-execution_started,
                            final_joints=tuple(self.arm.joints))
        print(f'{sequence.segments[-1].name} reached and settled '
              f'({len(sequence.segments)} connected segments). Reference: {sequence.duration:.3f} s; '
              f'tracking + hold settling: {execution.elapsed_duration:.3f} s')
        return execution

    def wait_release_delay(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.pump()
            self.arm.check_ready()
            self.arm.pose_feedback.read(self.tip)


def initialize():
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)


def shutdown(session):
    try:
        if session is not None:
            session.close()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def plot_payload(points, segments, rows=()):
    return dict(waypoints=[dict(name=w.name, pose=pose_data(w.pose)) for w in points],
                   segments=[dict(name=s.name, poses=[pose_data(p) for _, p in s.reference_poses]) for s in segments],
                   measurements=rows)


def write_outputs(directory, stem, points, segments, rows=(), renderer=None):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    plot_data = directory / (stem + '.plot.json')
    atomic_json(plot_data, plot_payload(points, segments, rows))
    if rows:
        with (directory/(stem+'.csv')).open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'phase', 'segment_time_s', 'actual_x_m', 'actual_y_m', 'actual_z_m',
                'actual_qx', 'actual_qy', 'actual_qz', 'actual_qw',
                'desired_x_m', 'desired_y_m', 'desired_z_m', 'desired_qx', 'desired_qy', 'desired_qz', 'desired_qw']
                + list(JOINT_NAMES) + ['reference_'+j for j in JOINT_NAMES]
                + ['velocity_'+j+'_rad_s' for j in JOINT_NAMES] + ['event'])
            for row in rows:
                actual, desired = row['actual'], row['desired']
                desired_fields = ['']*7 if desired is None else list(desired['position'])+list(desired['orientation'])
                writer.writerow([row['time_s'], row['phase'], row['segment_time_s']]
                    + list(actual['position']) + list(actual['orientation']) + desired_fields
                    + list(row['joints']) + list(row['reference_joints'])
                    + list(row.get('joint_velocity', ['']*7)) + [row.get('event', 'tracking')])
    image = directory/(stem+'.png')
    subprocess.run([renderer or plot_python(), str(Path(__file__).with_name('pick_place_plot.py')),
                    str(plot_data), str(image)], check=True, env=plot_environment())
    print(f'3D plot: {image}; plot data: {plot_data}')
    if rows:
        print(f'Measured EE/joint data: {directory/(stem+".csv")}')
    return plot_data
