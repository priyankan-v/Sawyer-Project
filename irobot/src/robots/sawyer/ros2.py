"""
Optional Humble component using the direct Python driver.

Import this module only in a ROS 2 environment. Use a multithreaded executor
so stop/cancel requests remain serviceable during synchronous motion.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import replace

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .config import JOINT_NAMES, SawyerConfig
from .core.controller import SawyerController

NANOSECONDS_PER_SECOND = 1_000_000_000


def trajectory_points(trajectory):
    """Validate the accepted subset and reorder all joints canonically."""
    names = list(trajectory.joint_names)
    if len(names) != len(JOINT_NAMES) or set(names) != set(JOINT_NAMES):
        msg = 'Trajectory must name each right-arm joint once'
        raise ValueError(msg)
    if trajectory.header.stamp.sec or trajectory.header.stamp.nanosec:
        msg = 'Only immediate trajectories (zero stamp) supported'
        raise ValueError(msg)
    if not trajectory.points:
        msg = 'Trajectory needs at least one waypoint'
        raise ValueError(msg)
    indices = [names.index(name) for name in JOINT_NAMES]
    positions, times = [], []
    previous = 0.0
    for point in trajectory.points:
        if len(point.positions) != len(JOINT_NAMES):
            msg = 'Every waypoint needs seven positions'
            raise ValueError(msg)
        for values in (point.velocities, point.accelerations):
            if values and (
                len(values) != len(JOINT_NAMES)
                or any(value != 0 for value in values)
            ):
                msg = 'Only rest-to-rest waypoints supported'
                raise ValueError(msg)
        if point.effort:
            msg = 'Effort waypoints are not supported'
            raise ValueError(msg)
        target = tuple(point.positions[i] for i in indices)
        if not all(math.isfinite(value) for value in target):
            msg = 'Waypoint positions must be finite'
            raise ValueError(msg)
        stamp = point.time_from_start
        if stamp.sec < 0 or not 0 <= stamp.nanosec < NANOSECONDS_PER_SECOND:
            msg = 'Invalid waypoint duration'
            raise ValueError(msg)
        elapsed = stamp.sec + stamp.nanosec / 1e9
        if elapsed <= previous:
            msg = 'Times must strictly increase above zero'
            raise ValueError(msg)
        positions.append(target)
        times.append(elapsed)
        previous = elapsed
    return positions, times


class SawyerNode(Node):
    """
    One arm owner; rejected overlapping jobs are never queued/replayed.

    Construction and destruction do not initialize or shut down ROS context.
    The caller owns the connected driver and disconnects it after node close.
    """

    def __init__(
        self,
        driver,
        *,
        namespace='sawyer',
        context=None,
        cameras=False,
        interfaces=False,
    ):
        super().__init__('sawyer_driver', namespace=namespace, context=context)
        if not driver.is_connected:
            msg = 'Connect the direct driver before creating a node'
            raise ValueError(msg)
        self.driver = driver
        self.group = ReentrantCallbackGroup()
        self.job_lock = threading.Lock()
        self.software_stop_lock = threading.Lock()
        self.cache_lock = threading.Lock()
        self.driver_lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.worker = None
        self.active_handle = None
        self.closing = False
        self.cache = {
            'connected': True,
            'ready': False,
            'issues': ['Awaiting feedback'],
        }
        self.sample_time = None
        self.last_job = {'state': 'idle'}
        from .ros2_telemetry import SawyerTelemetry

        self.telemetry = SawyerTelemetry(
            self, driver, cameras=cameras, interfaces=interfaces
        )
        self.joints_pub = self.create_publisher(
            JointState, 'joint_states', qos_profile_sensor_data
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, 'endpoint_pose', qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(String, 'status', 1)
        self.command_services = []
        for name, operation in (
            ('move_to_neutral', self._neutral),
            (
                'reset_faults',
                lambda: driver.devices.reset_faults(execute=True),
            ),
            (
                'open_gripper',
                lambda: driver.open_gripper(
                    execute=True, cancel_event=self.cancel_event
                ),
            ),
            (
                'close_gripper',
                lambda: driver.close_gripper(
                    execute=True, cancel_event=self.cancel_event
                ),
            ),
            (
                'calibrate_gripper',
                lambda: driver.calibrate_gripper(
                    execute=True, cancel_event=self.cancel_event
                ),
            ),
        ):
            self.command_services.append(
                self.create_service(
                    Trigger,
                    name,
                    self._job_service(name, operation),
                    callback_group=self.group,
                )
            )
        self.command_services.append(
            self.create_service(
                SetBool, 'set_enabled', self._enable, callback_group=self.group
            )
        )
        self.command_services.append(
            self.create_service(
                Trigger, 'stop', self._stop, callback_group=self.group
            )
        )
        self.command_services.append(
            self.create_service(
                Trigger,
                'software_stop',
                self._software_stop,
                callback_group=self.group,
            )
        )
        self.action = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self.group,
        )
        self.timer = self.create_timer(0.05, self._publish)
        self.extended = None
        if interfaces:
            from .ros2_interfaces import ExtendedInterfaces

            self.extended = ExtendedInterfaces(self)

    def _enable(self, request, response):
        return self._job_service(
            'set_enabled',
            lambda: self.driver.devices.set_enabled(
                request.data, execute=True
            ),
        )(request, response)

    def _software_stop(self, _request, response):
        if not self.software_stop_lock.acquire(
            timeout=self.driver.config.device_timeout
        ):
            response.message = 'Controller stop already in progress'
            return response
        try:
            self.cancel_event.set()
            self.driver.stop()
            if not self.driver_lock.acquire(
                timeout=self.driver.config.device_timeout
            ):
                response.message = (
                    'Cancellation requested; controller stop unconfirmed'
                )
                return response
            try:
                report = self.driver.devices.software_stop(execute=True)
                response.success = True
                response.message = json.dumps(report)
            except (RuntimeError, ValueError, OSError) as exc:
                response.message = str(exc)
            finally:
                self.driver_lock.release()
            return response

        finally:
            self.software_stop_lock.release()

    def _reserve(self, name):
        if not self.software_stop_lock.acquire(blocking=False):
            return False
        try:
            if (
                self.closing
                or self.get_parameter('use_sim_time').value
                or not self.job_lock.acquire(blocking=False)
            ):
                return False
            self.cancel_event.clear()
            with self.cache_lock:
                self.last_job = {'operation': name, 'state': 'running'}
                self.cache.pop('reference', None)
            return True

        finally:
            self.software_stop_lock.release()

    def _finish(self, state, detail=''):
        with self.cache_lock:
            self.last_job.update(state=state, detail=detail)
            self.active_handle = None
        self.job_lock.release()

    def _neutral(self):
        self.driver.move_to_neutral(
            execute=True,
            cancel_event=self.cancel_event,
            on_step=self._feedback,
        )

    def _job_service(self, name, operation):
        def callback(_request, response):
            if not self._reserve(name):
                response.message = 'Busy or shutting down; request rejected'
                return response
            self.worker = threading.Thread(
                target=self._run_job, args=(operation,), daemon=True
            )
            self.worker.start()
            response.success = True
            response.message = (
                'Accepted; observe the status topic for completion'
            )
            return response

        return callback

    def _run_job(self, operation):
        try:
            if self.cancel_event.is_set():
                msg = 'Job cancelled before execution'
                raise RuntimeError(msg)  # noqa: TRY301
            with self.driver_lock:
                operation()
        except Exception as exc:  # noqa: BLE001
            state = 'cancelled' if self.cancel_event.is_set() else 'failed'
            self._finish(state, str(exc))
        else:
            self._finish('succeeded')

    def _stop(self, _request, response):
        self.cancel_event.set()
        self.driver.stop()
        response.success = True
        response.message = (
            'Cancellation requested; completion in the status topic'
        )
        return response

    def _goal(self, goal):
        try:
            trajectory_points(goal.trajectory)
            if goal.multi_dof_trajectory.points:
                msg = 'Multi-DOF trajectories are not supported'
                raise ValueError(msg)  # noqa: TRY301
            if (
                goal.path_tolerance
                or goal.goal_tolerance
                or goal.goal_time_tolerance.sec
                or goal.goal_time_tolerance.nanosec
            ):
                msg = 'Use configured driver tolerances'
                raise ValueError(msg)  # noqa: TRY301
            if self.get_parameter('use_sim_time').value:
                msg = 'Physical control requires use_sim_time=false'
                raise ValueError(msg)  # noqa: TRY301
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return GoalResponse.REJECT
        if not self._reserve('follow_joint_trajectory'):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, _handle):
        self.cancel_event.set()
        self.driver.stop()
        return CancelResponse.ACCEPT

    def _feedback(self, elapsed, reference, joints, velocities):
        # No ROS publication on the control thread: timer drains this cache.
        with self.cache_lock:
            self.cache.update(
                joints=joints,
                velocities=velocities,
                elapsed=elapsed,
                reference=reference,
                ready=True,
                issues=[],
            )
            self.sample_time = time.monotonic()

    def _execute(self, handle):
        result = FollowJointTrajectory.Result()
        self.active_handle = handle
        try:
            positions, times = trajectory_points(handle.request.trajectory)
            with self.driver_lock:
                self.driver.move_trajectory(
                    positions,
                    times,
                    execute=True,
                    cancel_event=self.cancel_event,
                    on_step=self._feedback,
                )
        except Exception as exc:  # noqa: BLE001
            result.error_string = str(exc)
            result.error_code = (
                result.INVALID_GOAL
                if isinstance(exc, ValueError)
                else result.PATH_TOLERANCE_VIOLATED
            )
            if handle.is_cancel_requested:
                handle.canceled()
                self._finish('cancelled', str(exc))
            else:
                handle.abort()
                self._finish('failed', str(exc))
        else:
            if handle.is_cancel_requested:
                handle.canceled()
                self._finish('cancelled')
            else:
                handle.succeed()
                self._finish('succeeded')
        return result

    def _poll_state(self):
        try:
            state = self.driver.get_status()
            with self.cache_lock:
                self.cache = state
                self.sample_time = time.monotonic()
            if not state['issues']:
                pose = self.driver.endpoint_pose
                message = PoseStamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = pose.frame
                (
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ) = pose.position
                (
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ) = pose.orientation
                self.pose_pub.publish(message)
        except (RuntimeError, ValueError, OSError) as exc:
            with self.cache_lock:
                self.cache.update(ready=False, issues=[str(exc)])

    def _publish(self):
        self.telemetry.publish()
        if self.extended is not None:
            self.extended.publish()
        busy = self.job_lock.locked()
        if not busy and self.driver_lock.acquire(blocking=False):
            try:
                self._poll_state()
            finally:
                self.driver_lock.release()
        with self.cache_lock:
            state = dict(self.cache)
            job = dict(self.last_job)
            receipt = self.sample_time
        age = None if receipt is None else time.monotonic() - receipt
        fresh = age is not None and age <= self.driver.config.feedback_timeout
        if (
            fresh
            and state.get('joint_feedback_fresh', not state.get('issues'))
            and state.get('joints') is not None
        ):
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = list(state['joints'])
            message.velocity = list(state['velocities'])
            message.effort = self.telemetry.joint_efforts(JOINT_NAMES)
            self.joints_pub.publish(message)
            handle, reference = self.active_handle, state.get('reference')
            if handle is not None and handle.is_active and reference:
                feedback = FollowJointTrajectory.Feedback()
                feedback.header = message.header
                feedback.joint_names = list(JOINT_NAMES)
                feedback.actual.positions = message.position
                feedback.actual.velocities = message.velocity
                feedback.desired.positions = list(reference.position)
                feedback.desired.velocities = list(reference.velocity)
                feedback.desired.accelerations = list(reference.acceleration)
                feedback.error.positions = [
                    desired - actual
                    for desired, actual in zip(
                        reference.position, message.position, strict=True
                    )
                ]
                elapsed_ns = int(state['elapsed'] * 1e9)
                for point in (
                    feedback.actual,
                    feedback.desired,
                    feedback.error,
                ):
                    point.time_from_start.sec = elapsed_ns // 1_000_000_000
                    point.time_from_start.nanosec = elapsed_ns % 1_000_000_000
                handle.publish_feedback(feedback)
        state.pop('reference', None)
        state.update(busy=busy, feedback_age_s=age, job=job)
        state['telemetry_errors'] = dict(self.telemetry.errors)
        with self.telemetry.lock:
            state['telemetry_received'] = dict(self.telemetry.received_counts)
        state['telemetry_published'] = dict(self.telemetry.published_counts)
        if not fresh:
            state.update(ready=False, issues=['Feedback is stale'])
        self.status_pub.publish(String(data=json.dumps(state)))

    def destroy_node(self):
        self.closing = True
        self.timer.cancel()
        self.cancel_event.set()
        self.driver.stop()
        if self.worker is not None:
            self.worker.join(timeout=self.driver.config.gripper_timeout + 1)
        self.action.destroy()
        if self.extended is not None:
            self.extended.close()
        with self.driver_lock:
            self.telemetry.close()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master', required=True)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--namespace', default='sawyer')
    parser.add_argument('--tip', default=None)
    parser.add_argument('--speed', default=0.05, type=float)
    parser.add_argument('--max-accel', default=0.1, type=float)
    parser.add_argument('--cameras', action='store_true')
    parser.add_argument('--interfaces', action='store_true')
    parser.add_argument('--allow-unvalidated-modes', action='store_true')
    args, ros_args = parser.parse_known_args()
    config = SawyerConfig(
        ros_master_uri=args.master,
        ros_ip=args.ip,
        tip=args.tip,
        allow_unvalidated_modes=args.allow_unvalidated_modes,
    )
    config = replace(
        config,
        motion=replace(
            config.motion, speed=args.speed, max_accel=args.max_accel
        ),
    )
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    driver = SawyerController(config)
    node, executor = None, MultiThreadedExecutor(num_threads=4)
    try:
        driver.connect()
        node = SawyerNode(
            driver,
            namespace=args.namespace,
            cameras=args.cameras,
            interfaces=args.interfaces,
        )
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        driver.stop()
    finally:
        if node is not None:
            node.closing = True
            node.timer.cancel()
            node.cancel_event.set()
        driver.stop()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if node is not None:
            with node.driver_lock:
                driver.disconnect()
        else:
            driver.disconnect()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
