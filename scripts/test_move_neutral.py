"""Offline ROS-message and control-loop checks. No ROS init or robot connection."""

import copy
import math
import unittest
from unittest.mock import MagicMock, patch

from builtin_interfaces.msg import Time
from intera_core_msgs.msg import JointCommand, JointLimits, RobotAssemblyState
from sensor_msgs.msg import JointState

import move_neutral as motion


BOUNDS = motion.JointBounds((-3.0,) * 7, (3.0,) * 7, (1.0,) * 7, (1.0,) * 7)


class SimulatedRobot:
    """First-order joint velocity plant and a clock for the actual node loop."""

    def __init__(self):
        self.now = 100.0
        self.q = [0.0] * 7
        self.velocity = [0.0] * 7
        self.command = [0.0] * 7
        self.messages = []
        self.freeze = False
        self.stale = False
        self.interrupt = False
        self.delay = 0.0
        self.node = motion.MoveNeutral.__new__(motion.MoveNeutral)
        node = self.node
        node.joints, node.velocity = tuple(self.q), tuple(self.velocity)
        node.feedback = motion.JointFeedback(motion.JOINT_NAMES)
        node.feedback.update(motion.JOINT_NAMES, self.q, self.velocity, self.now)
        node.last_feedback_error = node.last_limits_error = None
        node.state = RobotAssemblyState(enabled=True, ready=True)
        node.limits, node.collision = BOUNDS, False
        node.joints_at = node.state_at = node.collision_at = self.now
        node.streaming = False
        node.command_pub, node.watchdog_pub = MagicMock(), MagicMock()
        node.command_pub.get_subscription_count.return_value = 1
        node.watchdog_pub.get_subscription_count.return_value = 1
        node.command_pub.publish.side_effect = self.publish
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        node.get_logger = MagicMock()

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))
        if message.mode == JointCommand.VELOCITY_MODE:
            self.command = list(message.velocity)
        else:
            self.command = [0.0] * 7

    def spin(self, node, timeout_sec):
        if self.interrupt and self.now > 100.1:
            self.interrupt = False
            raise KeyboardInterrupt()
        dt = max(timeout_sec, .00001) + self.delay
        self.now += dt
        if not self.freeze:
            for i in range(7):
                self.velocity[i] += (self.command[i] - self.velocity[i]) * min(1.0, dt / .04)
                self.q[i] += self.velocity[i] * dt
        if not self.stale:
            node.on_joints(JointState(name=motion.JOINT_NAMES, position=self.q, velocity=self.velocity))
            node.state_at = node.collision_at = self.now
        # Real /robot/joint_states also has a separate gripper publisher.
        node.on_joints(JointState(name=['right_gripper_l_finger_joint'], position=[0.], velocity=[0.]))

    def context(self):
        return (patch.object(motion.time, 'monotonic', side_effect=lambda: self.now),
                patch.object(motion.rclpy, 'spin_once', side_effect=self.spin))


class NeutralTests(unittest.TestCase):
    def options(self, *arguments):
        return motion.build_parser().parse_args(list(arguments))

    def read_pose(self, names, angles, code=1):
        proxy = MagicMock()
        proxy.__enter__.return_value.getParam.return_value = (
            code, 'response', {'joint_names': names, 'poses': {'neutral': angles}}
        )
        with patch.object(motion.xmlrpc.client, 'ServerProxy', return_value=proxy):
            return motion.read_neutral('http://unused:11311')

    def test_parameter_order_is_mapped_by_name(self):
        self.assertEqual(self.read_pose(motion.JOINT_NAMES[::-1], list(range(7))),
                         (6, 5, 4, 3, 2, 1, 0))

    def test_bad_neutral_is_rejected(self):
        for names, angles in ((motion.JOINT_NAMES, [0] * 6),
                              (['right_j0'] * 7, [0] * 7),
                              (motion.JOINT_NAMES, [float('nan')] * 7)):
            with self.subTest(names=names, angles=angles), self.assertRaises(ValueError):
                self.read_pose(names, angles)
        with self.assertRaises(RuntimeError):
            self.read_pose([], [], code=-1)

    def test_limits_are_reordered_and_validated(self):
        message = JointLimits(joint_names=motion.JOINT_NAMES[::-1], position_lower=[-3.] * 7,
                              position_upper=[3.] * 7, velocity=[float(i) for i in range(1, 8)],
                              accel=[2.] * 7)
        self.assertEqual(motion.JointBounds.from_message(message).velocity, (7, 6, 5, 4, 3, 2, 1))
        message.accel[0] = 0
        with self.assertRaises(ValueError):
            motion.JointBounds.from_message(message)

    def test_plan_uses_robot_limits_and_rejects_infeasible_time(self):
        options = self.options()
        path = motion.plan_trajectory([0] * 7, [1] * 7, BOUNDS, options)
        self.assertEqual(path.max_velocity, (1,) * 7)
        self.assertAlmostEqual(path.duration, math.sqrt(10 / math.sqrt(3)))
        options.time = .1
        with self.assertRaisesRegex(ValueError, 'minimum feasible'):
            motion.plan_trajectory([0] * 7, [1] * 7, BOUNDS, options)
        options.time = 5
        options.speed, options.max_accel = .5, .4
        path = motion.plan_trajectory([0] * 7, [1] * 7, BOUNDS, options)
        self.assertEqual(path.duration, 5)
        self.assertEqual(path.max_velocity, (.5,) * 7)
        self.assertEqual(path.max_acceleration, (.4,) * 7)
        with self.assertRaises(ValueError):
            motion.plan_trajectory([0] * 7, [4] * 7, BOUNDS, options)

    def test_missing_velocity_does_not_refresh_velocity(self):
        robot = SimulatedRobot()
        with patch.object(motion.time, 'monotonic', return_value=100.25):
            robot.node.on_joints(JointState(name=motion.JOINT_NAMES, position=[0.] * 7, velocity=[]))
            self.assertEqual(robot.node.joints_at, 100)
            self.assertFalse(robot.node.fresh())
            self.assertIn('stale velocity', '; '.join(robot.node.input_issues()))
        with patch.object(motion.time, 'monotonic', return_value=101):
            robot.node.on_joints(JointState(name=motion.JOINT_NAMES[::-1],
                                           position=list(map(float, range(7))), velocity=[0.] * 7))
        self.assertEqual(robot.node.joints, (6, 5, 4, 3, 2, 1, 0))

    def test_gripper_message_preserves_valid_arm_but_cannot_hide_stale_arm(self):
        robot = SimulatedRobot()
        message = JointState(name=['right_gripper_l_finger_joint'], position=[0.], velocity=[0.])
        with patch.object(motion.time, 'monotonic', return_value=100.01):
            robot.node.on_joints(message)
            self.assertEqual(robot.node.joints_at, 100)
            self.assertEqual(robot.node.joints, (0,) * 7)
            robot.node.check_ready()
        with patch.object(motion.time, 'monotonic', return_value=100.25):
            robot.node.on_joints(message)
            with self.assertRaisesRegex(RuntimeError, '/robot/joint_states: stale position'):
                robot.node.check_ready()

    def test_error_identifies_exact_stale_topic_and_age(self):
        robot = SimulatedRobot()
        robot.node.state_at = 99.7
        with patch.object(motion.time, 'monotonic', return_value=100):
            with self.assertRaisesRegex(RuntimeError, '/robot/state: 0.300s old'):
                robot.node.check_ready()

    def test_blocking_preview_print_is_followed_by_feedback_refresh(self):
        robot = SimulatedRobot()
        clock, spin = robot.context()
        delayed = False
        def slow_output(*args, **kwargs):
            nonlocal delayed
            if not delayed:
                robot.now += .3
                delayed = True
        with clock, spin, patch('builtins.print', side_effect=slow_output):
            robot.node.run([.2] * 7, self.options('--execute', '--time', '2'))
        self.assertFalse(robot.node.streaming)
        self.assertLess(max(abs(.2 - q) for q in robot.q), .02)

    def test_readiness_and_collision_checks(self):
        robot = SimulatedRobot()
        with patch.object(motion.time, 'monotonic', return_value=100):
            robot.node.check_ready()
            for field, value in (('enabled', False), ('ready', False), ('stopped', True), ('error', True)):
                original = getattr(robot.node.state, field)
                setattr(robot.node.state, field, value)
                with self.subTest(field=field), self.assertRaises(RuntimeError):
                    robot.node.check_ready()
                setattr(robot.node.state, field, original)
            robot.node.collision = True
            with self.assertRaisesRegex(RuntimeError, 'collision'):
                robot.node.check_ready()

    def test_preview_never_publishes_even_watchdog(self):
        robot = SimulatedRobot()
        clock, spin = robot.context()
        with clock, spin, patch('builtins.print'):
            robot.node.run([1.] * 7, self.options('--time', '5'))
        robot.node.command_pub.publish.assert_not_called()
        robot.node.watchdog_pub.publish.assert_not_called()

    def test_short_time_rejected_before_commands(self):
        robot = SimulatedRobot()
        clock, spin = robot.context()
        with clock, spin, self.assertRaises(ValueError):
            robot.node.run([1.] * 7, self.options('--execute', '--time', '.1'))
        self.assertFalse(robot.messages)
        robot.node.watchdog_pub.publish.assert_not_called()

    def test_running_arm_rejected_before_commands(self):
        robot = SimulatedRobot()
        robot.node.velocity = (.1,) * 7
        clock, spin = robot.context()
        with clock, spin, patch('builtins.print'), self.assertRaisesRegex(RuntimeError, 'stationary'):
            robot.node.run([1.] * 7, self.options('--execute'))
        self.assertFalse(robot.messages)

    def test_full_node_tracks_simulated_plant_and_exits_to_position_hold(self):
        robot = SimulatedRobot()
        clock, spin = robot.context()
        with clock, spin, patch('builtins.print') as output:
            robot.node.run([.2] * 7, self.options('--execute', '--time', '2'))
        self.assertTrue(any('Neutral pose reached' in str(call) for call in output.call_args_list))
        self.assertLess(max(abs(.2 - q) for q in robot.q), .02)
        self.assertTrue(any(message.mode == JointCommand.VELOCITY_MODE for message in robot.messages))
        self.assertEqual(robot.messages[-1].mode, JointCommand.POSITION_MODE)
        self.assertTrue(all(abs(v) <= 1 for message in robot.messages for v in message.velocity))
        self.assertFalse(robot.node.streaming)

    def test_faults_timeout_and_interruption_request_stop(self):
        for scenario, exception, pattern in (('stale', RuntimeError, 'stale'),
                                              ('delay', RuntimeError, 'deadline'),
                                              ('freeze', TimeoutError, 'timeout'),
                                              ('interrupt', KeyboardInterrupt, '')):
            with self.subTest(scenario=scenario):
                robot = SimulatedRobot()
                setattr(robot, scenario, .11 if scenario == 'delay' else True)
                clock, spin = robot.context()
                with clock, spin, patch('builtins.print'), self.assertRaisesRegex(exception, pattern):
                    robot.node.run([.2] * 7, self.options('--execute', '--time', '2', '--settle-time', '.4'))
                self.assertFalse(robot.node.streaming)
                zeros = [m for m in robot.messages if m.mode == JointCommand.VELOCITY_MODE
                         and list(m.velocity) == [0.] * 7]
                self.assertGreaterEqual(len(zeros), 5)
                if scenario == 'stale':
                    self.assertEqual(robot.messages[-1].mode, JointCommand.VELOCITY_MODE)

    def test_lost_route_and_excessive_tracking_error_stop(self):
        for lost_route in (True, False):
            robot = SimulatedRobot()
            clock, spin = robot.context()
            options = self.options('--execute', '--time', '2', '--max-error', '.01')
            original = robot.spin
            def failing_spin(node, timeout_sec):
                original(node, timeout_sec)
                if lost_route and robot.now > 100.1:
                    node.command_pub.get_subscription_count.return_value = 0
            robot.freeze = True
            with clock, patch.object(motion.rclpy, 'spin_once', side_effect=failing_spin), patch('builtins.print'):
                with self.assertRaisesRegex(RuntimeError, 'subscription' if lost_route else 'tracking error'):
                    robot.node.run([.2] * 7, options)
            self.assertFalse(robot.node.streaming)

    def test_stop_can_hold_fresh_positions_after_velocity_expires(self):
        robot = SimulatedRobot()
        robot.now += .3
        robot.node.streaming = True
        robot.stale = True
        original = robot.spin
        def position_only_spin(node, timeout_sec):
            original(node, timeout_sec)
            node.on_joints(JointState(name=motion.JOINT_NAMES, position=[0.] * 7))
        clock, _ = robot.context()
        with clock, patch.object(motion.rclpy, 'spin_once', side_effect=position_only_spin):
            robot.node.stop()
        self.assertEqual(robot.messages[-1].mode, JointCommand.POSITION_MODE)
        self.assertFalse(robot.node.streaming)

    def test_cli_time_and_numerical_validation(self):
        self.assertIsNone(self.options().time)
        self.assertEqual(self.options('--time', '8').time, 8)
        self.assertEqual(self.options().speed, 1)
        for argument, value in (('--time', '0'), ('--time', 'nan'), ('--kp', '-1'), ('--rate', 'inf')):
            with patch('sys.stderr'), self.assertRaises(SystemExit):
                self.options(argument, value)


if __name__ == '__main__':
    unittest.main()
