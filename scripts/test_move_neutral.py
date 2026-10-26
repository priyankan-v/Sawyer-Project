"""Offline checks; these tests never initialize ROS or connect to a robot."""

import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState

import move_neutral as motion


class NeutralTests(unittest.TestCase):
    def read_pose(self, names, angles, code=1):
        proxy = MagicMock()
        proxy.__enter__.return_value.getParam.return_value = (
            code, "parameter response", {"joint_names": names, "poses": {"neutral": angles}}
        )
        with patch.object(motion.xmlrpc.client, "ServerProxy", return_value=proxy):
            return motion.read_neutral("http://unused:11311")

    def test_parameter_order_is_mapped_by_joint_name(self):
        self.assertEqual(self.read_pose(motion.JOINT_NAMES[::-1], list(range(7))),
                         [6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0])

    def test_missing_or_malformed_neutral_is_rejected(self):
        for names, angles in (
            (motion.JOINT_NAMES, [0.0] * 6),
            (["right_j0"] * 7, [0.0] * 7),
            (motion.JOINT_NAMES, [float("nan")] * 7),
        ):
            with self.subTest(names=names, angles=angles), self.assertRaises(ValueError):
                self.read_pose(names, angles)
        with self.assertRaises(RuntimeError):
            self.read_pose([], [], code=-1)

    def test_goal_uses_current_then_neutral_in_joint_space(self):
        current, target = [0.1] * 7, [0.2] * 7
        goal = motion.make_goal(current, target, 0.1)
        self.assertEqual(goal.command, "start")
        self.assertEqual(goal.trajectory.joint_names, motion.JOINT_NAMES)
        self.assertEqual(goal.trajectory.trajectory_options.interpolation_type, "JOINT")
        self.assertFalse(goal.trajectory.trajectory_options.interaction_control)
        self.assertEqual(len(goal.trajectory.waypoints), 2)
        for waypoint, expected in zip(goal.trajectory.waypoints, (current, target)):
            self.assertEqual(list(waypoint.joint_positions), expected)
            self.assertEqual(waypoint.options.max_joint_speed_ratio, 0.1)
            self.assertEqual(list(waypoint.options.max_joint_accel), [0.3] * 7)

    def test_incomplete_joint_messages_do_not_refresh_state(self):
        node = motion.MoveNeutral.__new__(motion.MoveNeutral)
        node.joints_at = 0.0
        node.on_joints(JointState(name=["right_j0"], position=[1.0]))
        self.assertEqual(node.joints_at, 0.0)
        node.on_joints(JointState(name=motion.JOINT_NAMES[::-1], position=list(map(float, range(7)))))
        self.assertEqual(node.joints, list(map(float, reversed(range(7)))))

    def test_stale_disabled_stopped_and_faulted_robots_are_rejected(self):
        node = motion.MoveNeutral.__new__(motion.MoveNeutral)
        node.joints_at = node.state_at = time.monotonic()
        node.state = SimpleNamespace(enabled=True, ready=True, stopped=False, error=False)
        node.check_ready()
        for field, value in (("enabled", False), ("ready", False), ("stopped", True), ("error", True)):
            original = getattr(node.state, field)
            setattr(node.state, field, value)
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                node.check_ready()
            setattr(node.state, field, original)
        node.joints_at = 0.0
        with self.assertRaises(RuntimeError):
            node.check_ready()

    def test_preview_does_not_submit_motion(self):
        node = motion.MoveNeutral.__new__(motion.MoveNeutral)
        node.joints_at = node.state_at = time.monotonic()
        node.joints = [0.0] * 7
        node.client = MagicMock()
        with patch("builtins.print"):
            node.run([0.1] * 7, execute=False, speed=0.1, timeout=90.0)
        node.client.send_goal_async.assert_not_called()

    def test_controller_failure_cancels_and_never_reports_success(self):
        node = motion.MoveNeutral.__new__(motion.MoveNeutral)
        node.joints_at = node.state_at = time.monotonic()
        node.joints = [0.0] * 7
        node.check_ready = MagicMock()
        node.client = MagicMock()
        handle = MagicMock(accepted=True)
        response = SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED,
            result=SimpleNamespace(result=False, error_id="PLANNED_MOTION_COLLISION"),
        )
        node.wait_for = MagicMock(side_effect=[handle, response])
        node.cancel = MagicMock()
        with patch("builtins.print"), self.assertRaisesRegex(RuntimeError, "PLANNED_MOTION_COLLISION"):
            node.run([0.1] * 7, execute=True, speed=0.1, timeout=90.0)
        node.cancel.assert_called_once()

    def test_timeout_and_interruption_request_cancellation(self):
        for failure in (TimeoutError("motion timeout"), KeyboardInterrupt()):
            with self.subTest(failure=type(failure).__name__):
                node = motion.MoveNeutral.__new__(motion.MoveNeutral)
                node.joints_at = node.state_at = time.monotonic()
                node.joints = [0.0] * 7
                node.check_ready = MagicMock()
                node.client = MagicMock()
                node.wait_for = MagicMock(side_effect=[MagicMock(accepted=True), failure])
                node.cancel = MagicMock()
                with patch("builtins.print"), self.assertRaises(type(failure)):
                    node.run([0.1] * 7, execute=True, speed=0.1, timeout=90.0)
                node.cancel.assert_called_once()

    def test_success_requires_measured_arrival(self):
        node = motion.MoveNeutral.__new__(motion.MoveNeutral)
        node.joints_at = node.state_at = time.monotonic()
        node.joints = [0.0] * 7
        node.check_ready = MagicMock()
        node.client = MagicMock()
        response = SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED, result=SimpleNamespace(result=True)
        )
        node.wait_for = MagicMock(side_effect=[MagicMock(accepted=True), response])
        target = [0.1] * 7
        def measured_arrival(*args, **kwargs):
            node.joints = target
        with patch.object(motion.rclpy, "spin_once", side_effect=measured_arrival), patch("builtins.print") as output:
            node.run(target, execute=True, speed=0.1, timeout=90.0)
        self.assertIn("Neutral pose reached", output.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
