"""Optional Humble tests. DDS integration is opt-in; no robot is accessed."""

from __future__ import annotations

import os
import threading
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('control_msgs.action')

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from irobot.src.robots.sawyer.config import JOINT_NAMES, SawyerConfig
from irobot.src.robots.sawyer.core.base import CartesianPose
from irobot.src.robots.sawyer.ros2 import SawyerNode, trajectory_points


def trajectory():
    return JointTrajectory(
        joint_names=list(reversed(JOINT_NAMES)),
        points=[JointTrajectoryPoint(positions=list(range(7)))],
    )


def test_trajectory_reorders_joints_and_rejects_unsupported_fields():
    goal = trajectory()
    goal.points[0].time_from_start.sec = 2
    positions, times = trajectory_points(goal)
    assert positions == [tuple(reversed(range(7)))]
    assert times == [2.0]
    goal.points[0].velocities = [0.1] * 7
    with pytest.raises(ValueError, match='rest-to-rest'):
        trajectory_points(goal)
    goal.points[0].velocities = []
    goal.joint_names = ['right_j0'] * 7
    with pytest.raises(ValueError, match='each right-arm joint'):
        trajectory_points(goal)


def test_trajectory_rejects_stale_schedule_and_nonfinite_values():
    goal = trajectory()
    with pytest.raises(ValueError, match='increase'):
        trajectory_points(goal)
    goal.points[0].time_from_start.sec = 1
    goal.points[0].positions[0] = float('nan')
    with pytest.raises(ValueError, match='finite'):
        trajectory_points(goal)
    goal.header.stamp.sec = 1
    with pytest.raises(ValueError, match='zero stamp'):
        trajectory_points(goal)


class LocalDriver:
    """Bounded fake operation for testing actual DDS request lifecycles."""

    def __init__(self):
        self.config = SawyerConfig()
        self.is_connected = True
        self._node = SimpleNamespace(
            observe=lambda *_args: None, remove_observer=lambda *_args: None
        )
        self.endpoint_pose = CartesianPose((0, 0, 0), (0, 0, 0, 1))
        self.calls = []
        self.operation_lock = threading.Lock()
        self.reading = threading.Event()
        self.release_read = threading.Event()

    def get_status(self):
        with self.operation_lock:
            self.reading.set()
            assert self.release_read.wait(5)
            return {
                'connected': True,
                'ready': True,
                'issues': [],
                'joints': (0.0,) * 7,
                'velocities': (0.0,) * 7,
            }

    def move_trajectory(
        self, positions, times, *, execute, cancel_event, on_step
    ):
        assert self.operation_lock.acquire(blocking=False), 'Concurrent access'
        try:
            self._track(positions, times, execute, cancel_event, on_step)
        finally:
            self.operation_lock.release()

    def _track(self, positions, times, execute, cancel_event, on_step):
        self.calls.append((positions, times, execute))
        reference = SimpleNamespace(
            position=(0.0,) * 7, velocity=(0.0,) * 7, acceleration=(0.0,) * 7
        )
        for step in range(100):
            if cancel_event.is_set():
                msg = 'Motion cancelled'
                raise RuntimeError(msg)
            on_step(step * 0.01, reference, (0.0,) * 7, (0.0,) * 7)
            time.sleep(0.01)

    def stop(self):
        pass


def completed(future):
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    assert done.wait(8), 'ROS future did not complete'
    return future.result()


@pytest.mark.skipif(
    os.environ.get('SAWYER_ROS2_TEST') != '1',
    reason='Set SAWYER_ROS2_TEST=1 for local DDS tests',
)
def test_real_dds_action_rejects_overlap_cancels_and_accepts_next_goal():
    context = Context()
    context.init(domain_id=98)
    driver = LocalDriver()
    server = SawyerNode(driver, context=context)
    client_node = Node('sawyer_test_client', context=context)
    client = ActionClient(
        client_node, FollowJointTrajectory, '/sawyer/follow_joint_trajectory'
    )
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=5)
        assert driver.reading.wait(5)
        goal = FollowJointTrajectory.Goal(trajectory=trajectory())
        goal.trajectory.points[0].time_from_start.sec = 2
        feedback = []
        first = completed(
            client.send_goal_async(goal, feedback_callback=feedback.append)
        )
        assert first.accepted
        assert not driver.calls
        driver.release_read.set()
        second = completed(client.send_goal_async(goal))
        assert not second.accepted
        time.sleep(0.15)
        cancellation = completed(first.cancel_goal_async())
        assert cancellation.goals_canceling
        result = completed(first.get_result_async())
        assert result.status == GoalStatus.STATUS_CANCELED
        assert feedback
        third = completed(client.send_goal_async(goal))
        assert third.accepted
        assert completed(third.get_result_async()).status == (
            GoalStatus.STATUS_SUCCEEDED
        )
        assert len(driver.calls) == 2  # noqa: PLR2004
    finally:
        driver.release_read.set()
        server.cancel_event.set()
        executor.shutdown()
        thread.join(timeout=3)
        client.destroy()
        server.destroy_node()
        assert context.ok()
        client_node.destroy_node()
        context.shutdown()


def test_software_stop_cancels_busy_job_and_waits_for_its_driver_lock():
    from unittest.mock import MagicMock

    from std_srvs.srv import Trigger

    driver = MagicMock()
    driver.config.device_timeout = 1.0
    driver.devices.software_stop.return_value = {
        'executed': True,
        'state': {'stopped': True},
    }
    node = SimpleNamespace(
        driver=driver,
        driver_lock=threading.Lock(),
        cancel_event=threading.Event(),
        job_lock=threading.Lock(),
        software_stop_lock=threading.Lock(),
    )
    running = threading.Event()

    def active_job():
        with node.driver_lock:
            running.set()
            assert node.cancel_event.wait(1)

    node.job_lock.acquire()
    worker = threading.Thread(target=active_job)
    worker.start()
    assert running.wait(1)
    try:
        response = SawyerNode._software_stop(  # noqa: SLF001
            node, Trigger.Request(), Trigger.Response()
        )
    finally:
        worker.join(timeout=1)
    assert response.success
    assert node.job_lock.locked()
    assert not node.driver_lock.locked()
    assert not node.software_stop_lock.locked()
    driver.stop.assert_called_once()
    driver.devices.software_stop.assert_called_once_with(execute=True)
    node.job_lock.release()
