"""Cartesian API and shared PID-loop simulation; no ROS init or robot connection."""

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from intera_core_msgs.msg import EndpointState, EndpointStates, JointCommand
from sensor_msgs.msg import JointState

from arm_motion import JOINT_NAMES
from cartesian_motion import CartesianMotionNode, CartesianMotionOptions
from cartesian_path import CartesianPose
import cartesian_motion as motion
import move_ee
from ros_kinematics import EndpointFeedback
from test_move_neutral import SimulatedRobot


class CartesianRobot(SimulatedRobot):
    def __init__(self):
        super().__init__()
        old_node = self.node
        self.node = CartesianMotionNode.__new__(CartesianMotionNode)
        self.node.__dict__.update(old_node.__dict__)
        self.node.pose_feedback = EndpointFeedback()
        self.pose_shift = 0.0
        self.drop_pose = False
        self.feed_pose()

    def feed_pose(self):
        state = EndpointState(valid=True)
        state.header.frame_id = 'base'
        state.pose.position.x = self.q[0] + self.pose_shift
        state.pose.position.y = self.q[1]
        state.pose.position.z = self.q[2]
        state.pose.orientation.w = 1.0
        with patch.object(motion.time, 'monotonic', return_value=self.now):
            self.node.pose_feedback.on_endpoint(state)
            self.node.pose_feedback.on_tips(EndpointStates(names=['right_hand'], states=[state]))

    def spin(self, node, timeout_sec):
        super().spin(node, timeout_sec)
        if not self.drop_pose:
            self.feed_pose()

    def kinematics(self):
        adapter = MagicMock()
        adapter.inverse.side_effect = lambda pose, seed: (*pose.position, *seed[3:])
        adapter.forward.side_effect = lambda qs: [CartesianPose(q[:3], (0,0,0,1)) for q in qs]
        return adapter


class CartesianMotionTests(unittest.TestCase):
    def test_preview_is_read_only_and_offsets_are_centimeters(self):
        robot = CartesianRobot()
        clock, spin = robot.context()
        adapter = robot.kinematics()
        with clock, spin, patch.object(motion, 'InteraKinematics', return_value=adapter), patch('builtins.print'):
            result = robot.node.move_relative(dz_cm=10, duration=2)
        self.assertEqual(result.plan.line.end.position, (0,0,.1))
        self.assertIsNone(result.execution)
        robot.node.command_pub.publish.assert_not_called()
        robot.node.watchdog_pub.publish.assert_not_called()
        adapter.close.assert_called_once()

    def test_combined_move_tracks_with_pid_and_measured_cartesian_goal(self):
        robot = CartesianRobot()
        clock, spin = robot.context()
        with clock, spin, patch.object(motion, 'InteraKinematics', return_value=robot.kinematics()), patch('builtins.print'):
            result = robot.node.move_relative(dx_cm=5, dy_cm=-3, dz_cm=10, duration=2, execute=True)
        self.assertEqual(result.execution.reference_duration, 2)
        self.assertLess(abs(robot.q[0]-.05), .003)
        self.assertLess(abs(robot.q[1]+.03), .003)
        self.assertLess(abs(robot.q[2]-.1), .003)
        self.assertEqual(robot.messages[-1].mode, JointCommand.POSITION_MODE)
        self.assertFalse(robot.node.streaming)

    def test_minimum_timing_and_zero_offset_no_op(self):
        for offset in (0, 10):
            robot = CartesianRobot()
            clock, spin = robot.context()
            with clock, spin, patch.object(motion, 'InteraKinematics', return_value=robot.kinematics()), patch('builtins.print'):
                result = robot.node.move_relative(dz_cm=offset, execute=True)
            if offset:
                self.assertGreater(result.plan.trajectory.duration, 0)
                self.assertAlmostEqual(result.plan.trajectory.duration, result.plan.trajectory.minimum_duration)
            else:
                self.assertIsNone(result.execution)
                robot.node.command_pub.publish.assert_not_called()
                robot.node.watchdog_pub.publish.assert_not_called()

    def test_short_duration_and_failed_ik_never_send_commands(self):
        for failed_ik in (False, True):
            robot = CartesianRobot()
            adapter = robot.kinematics()
            if failed_ik:
                adapter.inverse.side_effect = ValueError('unreachable')
            clock, spin = robot.context()
            with clock, spin, patch.object(motion, 'InteraKinematics', return_value=adapter):
                with self.assertRaises(ValueError):
                    robot.node.move_relative(dz_cm=10, duration=.01 if not failed_ik else 2, execute=True)
            robot.node.command_pub.publish.assert_not_called()
            robot.node.watchdog_pub.publish.assert_not_called()
            adapter.close.assert_called_once()

    def test_pose_change_after_planning_rejected_before_motion(self):
        robot = CartesianRobot()
        clock, spin = robot.context()
        changed = False
        def output(*args, **kwargs):
            nonlocal changed
            if not changed:
                robot.q[0] = .05
                robot.node.on_joints(JointState(name=JOINT_NAMES,
                                               position=robot.q, velocity=robot.velocity))
                robot.feed_pose()
                changed = True
        with clock, spin, patch.object(motion, 'InteraKinematics', return_value=robot.kinematics()), patch('builtins.print', side_effect=output):
            with self.assertRaisesRegex(RuntimeError, 'moved during Cartesian planning'):
                robot.node.move_relative(dz_cm=10, duration=2, execute=True)
        robot.node.command_pub.publish.assert_not_called()

    def test_stale_pose_and_cartesian_error_request_stop(self):
        for stale in (False, True):
            robot = CartesianRobot()
            clock, _ = robot.context()
            original = robot.spin
            def disturb(node, timeout_sec):
                if stale:
                    robot.drop_pose = True
                else:
                    robot.pose_shift = .1
                original(node, timeout_sec)
            with clock, patch.object(motion.rclpy, 'spin_once', side_effect=disturb), patch.object(motion, 'InteraKinematics', return_value=robot.kinematics()), patch('builtins.print'):
                with self.assertRaisesRegex(RuntimeError, 'Stale Cartesian' if stale else 'Cartesian tracking error'):
                    robot.node.move_relative(dz_cm=10, duration=2, execute=True)
            self.assertFalse(robot.node.streaming)
            zeros = [m for m in robot.messages if m.mode == JointCommand.VELOCITY_MODE and list(m.velocity) == [0.]*7]
            self.assertGreaterEqual(len(zeros), 5)

    def test_joint_arrival_without_cartesian_arrival_does_not_report_success(self):
        robot = CartesianRobot()
        clock, _ = robot.context()
        original = robot.spin
        def bias(node, timeout_sec):
            robot.pose_shift = .012  # Below tracking-abort bound, above final Cartesian tolerance.
            original(node, timeout_sec)
        options = CartesianMotionOptions(settle_time=.5)
        with clock, patch.object(motion.rclpy, 'spin_once', side_effect=bias), patch.object(motion, 'InteraKinematics', return_value=robot.kinematics()), patch('builtins.print'):
            with self.assertRaisesRegex(TimeoutError, 'Cartesian target was not confirmed'):
                robot.node.move_relative(dz_cm=10, duration=2, execute=True, options=options)
        self.assertFalse(robot.node.streaming)

    def test_cli_and_api_parameter_validation_before_ros_initialization(self):
        for arguments in (['q:10'], ['z:nan'], ['z:10', '--time', '0'], ['z:10', '--max-samples', '2']):
            with self.subTest(arguments=arguments), patch('sys.argv', ['move_ee.py', *arguments]), patch('sys.stderr'):
                with patch.object(move_ee.rclpy, 'init') as initialize, self.assertRaises(SystemExit):
                    move_ee.main()
            initialize.assert_not_called()
        for settings in (dict(speed=2), dict(path_tolerance=0), dict(max_samples=2), dict(frame='')):
            with self.assertRaises(ValueError):
                CartesianMotionOptions(**settings)

    def test_interior_joint_extrema_checked_before_streaming(self):
        robot = CartesianRobot()
        trajectory = SimpleNamespace(start=(0,)*7, end=(.2,)*7, min_position=(-1,)*7,
                                     max_position=(4,)*7, max_velocity=(1,)*7,
                                     max_acceleration=(1,)*7, duration=2)
        clock, spin = robot.context()
        with clock, spin, self.assertRaisesRegex(ValueError, 'Reference position maximum'):
            robot.node.track(trajectory, MagicMock(), CartesianMotionOptions())
        robot.node.command_pub.publish.assert_not_called()
        robot.node.watchdog_pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
