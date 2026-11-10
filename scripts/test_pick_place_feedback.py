"""Executor lifetime and long interactive waits: no ROS init or robot commands."""

import io
import unittest
from unittest.mock import MagicMock, call, patch

from intera_core_msgs.msg import EndpointState, EndpointStates

from cartesian_motion import CartesianMotionOptions
from ros_kinematics import EndpointFeedback
import pick_place_runtime as runtime


class ReadyCallbackExecutor:
    """Model rclpy's timeout-sensitive ready-callback iterator.

    Restarting the iterator always visits joint feedback first. Tip feedback is
    later in the ready batch, as with the real arm's subscription ordering.
    """
    def __init__(self, feedback):
        self.now = 100.
        self.feedback = feedback
        self.nodes = set()
        self.timeout = None
        self.index = 0
        self.tip_updates = 0
        self.worst_age = 0

    def add_node(self, node):
        self.nodes.add(node)

    def remove_node(self, node):
        self.nodes.remove(node)

    def feed_tip(self):
        state = EndpointState(valid=True)
        state.header.frame_id = 'base'
        state.pose.orientation.w = 1.
        self.feedback.on_tips(EndpointStates(names=['right_hand'], states=[state]))
        self.tip_updates += 1

    def spin_once(self, timeout_sec):
        if timeout_sec != self.timeout:
            self.index = 0
            self.timeout = timeout_sec
        self.now += .01
        if self.index == 3:
            self.feed_tip()
        self.index = (self.index+1) % 4
        self.worst_age = max(self.worst_age, self.now-self.feedback.tips['right_hand'][1])


class FeedbackLifecycleTests(unittest.TestCase):
    def test_two_nodes_remain_registered_and_pump_uses_one_executor(self):
        executor, arm, gripper = MagicMock(), MagicMock(), MagicMock()
        with patch.object(runtime, 'CartesianMotionNode', return_value=arm), \
                patch.object(runtime, 'GripperControl', return_value=gripper), \
                patch.object(runtime.rclpy, 'get_global_executor', return_value=executor), \
                patch.object(runtime.rclpy, 'spin_once') as temporary_spin, \
                patch.object(runtime.rclpy, 'ok', return_value=True):
            session = runtime.Session(CartesianMotionOptions(execute=True))
            session.command_gripper('open')
            session.command_gripper('close')
            for _ in range(10):
                session.pump()
            self.assertEqual(executor.add_node.call_args_list, [call(arm), call(gripper)])
            self.assertEqual(executor.spin_once.call_args_list, [call(timeout_sec=.01)]*10)
            temporary_spin.assert_not_called()
            executor.remove_node.assert_not_called()
            session.close()
            self.assertEqual(executor.remove_node.call_args_list, [call(gripper), call(arm)])
            arm.destroy_node.assert_called_once()
            gripper.destroy_node.assert_called_once()

    def test_arm_is_cleaned_up_even_if_gripper_cleanup_fails(self):
        executor, arm, gripper = MagicMock(), MagicMock(), MagicMock()
        gripper.destroy_node.side_effect = RuntimeError('cleanup failure')
        with patch.object(runtime, 'CartesianMotionNode', return_value=arm), \
                patch.object(runtime.rclpy, 'get_global_executor', return_value=executor):
            session = runtime.Session(CartesianMotionOptions())
            session.gripper = gripper
            with self.assertRaisesRegex(RuntimeError, 'cleanup failure'):
                session.close()
        arm.destroy_node.assert_called_once()
        self.assertEqual(executor.remove_node.call_args_list, [call(gripper), call(arm)])

    def test_25_second_enter_pause_keeps_tips_gripper_and_live_measurements_fresh(self):
        feedback = EndpointFeedback()
        executor = ReadyCallbackExecutor(feedback)
        arm, gripper, viewer = MagicMock(), MagicMock(), MagicMock()
        arm.pose_feedback = feedback
        arm.joints, arm.velocity = (0.,)*7, (0.,)*7
        terminal = io.StringIO('\n')
        def ready_input(*_args):
            return ([terminal], [], []) if executor.now >= 125 else ([], [], [])
        with patch.object(runtime.time, 'monotonic', side_effect=lambda: executor.now), \
                patch.object(runtime, 'CartesianMotionNode', return_value=arm), \
                patch.object(runtime, 'GripperControl', return_value=gripper), \
                patch.object(runtime.rclpy, 'get_global_executor', return_value=executor), \
                patch.object(runtime.rclpy, 'ok', return_value=True), \
                patch('sys.stdin', terminal), patch('select.select', side_effect=ready_input), \
                patch('builtins.print'):
            executor.feed_tip()
            session = runtime.Session(CartesianMotionOptions(execute=True, tip='right_hand'))
            session.command_gripper('open')  # Mocked IO, never a real command.
            session.live_plot = viewer
            pose = feedback.read('right_hand')
            session.record_measurement('approach_pick', 3., pose, pose, arm.joints)
            session.wait_for_enter('At pick')
            feedback.read('right_hand')
            self.assertLess(executor.worst_age, .05)
            self.assertGreater(executor.tip_updates, 500)
            self.assertGreater(len(session.rows), 400)
            self.assertEqual(set(executor.nodes), {arm, gripper})
            self.assertEqual(session.rows[-1]['event'], 'waiting')
            viewer.publish.assert_called()

    def test_changing_spin_timeouts_reproduces_starvation(self):
        feedback = EndpointFeedback()
        executor = ReadyCallbackExecutor(feedback)
        with patch.object(runtime.time, 'monotonic', side_effect=lambda: executor.now):
            executor.feed_tip()
            for _ in range(50):
                executor.spin_once(.01)
                executor.spin_once(0)
            self.assertEqual(executor.tip_updates, 1)
            with self.assertRaisesRegex(RuntimeError, 'Stale Cartesian pose'):
                feedback.read('right_hand')


if __name__ == '__main__':
    unittest.main()
