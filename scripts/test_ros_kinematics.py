"""Service messages and endpoint feedback checks with no ROS initialization."""

from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from intera_core_msgs.msg import EndpointState, EndpointStates
from intera_core_msgs.srv import SolvePositionIK, SolvePositionFK
from sensor_msgs.msg import JointState

from arm_motion import JOINT_NAMES
from cartesian_path import CartesianPose
import ros_kinematics as ros


def endpoint(position=(0,0,0), frame='', valid=True):
    result = EndpointState(valid=valid)
    result.pose.position.x, result.pose.position.y, result.pose.position.z = map(float, position)
    result.pose.orientation.w = 1.0
    result.header.frame_id = frame
    return result


class FeedbackTests(unittest.TestCase):
    def test_auto_tip_matches_legacy_endpoint_and_handles_empty_frame_headers(self):
        feedback = ros.EndpointFeedback()
        message = EndpointStates(names=['right_hand', 'other'], states=[endpoint(), endpoint((1,0,0))])
        feedback.on_endpoint(endpoint())
        feedback.on_tips(message)
        tip, pose = feedback.select()
        self.assertEqual(tip, 'right_hand')
        self.assertEqual(pose.frame, 'base')
        self.assertEqual(feedback.select('other')[0], 'other')

    def test_invalid_stale_and_ambiguous_tip_states_rejected(self):
        feedback = ros.EndpointFeedback()
        with patch.object(ros.time, 'monotonic', return_value=100):
            feedback.on_endpoint(endpoint())
            feedback.on_tips(EndpointStates(names=['a','b'], states=[endpoint(), endpoint()]))
            with self.assertRaisesRegex(RuntimeError, 'uniquely'):
                feedback.select()
        with patch.object(ros.time, 'monotonic', return_value=101):
            with self.assertRaisesRegex(RuntimeError, 'Stale'):
                feedback.read('a')
        feedback.on_tips(EndpointStates(names=['a'], states=[endpoint(valid=False)]))
        with self.assertRaisesRegex(RuntimeError, 'No valid pose'):
            feedback.read('a')
        feedback.on_endpoint(endpoint(valid=False))
        with self.assertRaisesRegex(RuntimeError, 'endpoint_state'):
            feedback.select()


class ServiceTests(unittest.TestCase):
    def adapter(self):
        adapter = ros.InteraKinematics.__new__(ros.InteraKinematics)
        adapter.node, adapter.ik, adapter.fk = MagicMock(), MagicMock(), MagicMock()
        adapter.node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=123)
        adapter.tip, adapter.frame = 'right_hand', 'base'
        adapter.request_timeout, adapter.deadline = 1.0, 200.0
        adapter.health_check = None
        return adapter

    def test_ik_uses_selected_tip_and_previous_user_seed(self):
        adapter = self.adapter()
        response = SolvePositionIK.Response(result_type=[1], joints=[JointState(
            name=JOINT_NAMES[::-1], position=list(map(float, range(7))))])
        adapter.call = MagicMock(return_value=response)
        result = adapter.inverse(CartesianPose((.4,.1,.2), (0,0,0,1)), [.1]*7)
        self.assertEqual(result, (6,5,4,3,2,1,0))
        request = adapter.call.call_args.args[1]
        self.assertEqual(request.tip_names, ['right_hand'])
        self.assertEqual(request.seed_mode, SolvePositionIK.Request.SEED_USER)
        self.assertEqual(list(request.seed_angles[0].position), [.1]*7)
        self.assertEqual(request.pose_stamp[0].header.frame_id, 'base')

    def test_invalid_ik_result_codes_are_not_used(self):
        for code, message in ((-1,'unreachable'), (-2,'collision'), (-3,'unknown endpoint'), (0,'IK rejected')):
            adapter = self.adapter()
            adapter.call = MagicMock(return_value=SolvePositionIK.Response(result_type=[code]))
            with self.subTest(code=code), self.assertRaisesRegex(ValueError, message):
                adapter.inverse(CartesianPose((0,0,0),(0,0,0,1)), [0.]*7)

    def test_fk_is_batched_and_rejects_collision_or_wrong_frame(self):
        adapter = self.adapter()
        def forward(client, request, label):
            poses = []
            for q in request.configuration:
                pose = PoseStamped()
                pose.header.frame_id = 'base'
                pose.pose.position.x = q.position[0]
                pose.pose.orientation.w = 1.0
                poses.append(pose)
            return SolvePositionFK.Response(pose_stamp=poses, is_valid=[True]*len(poses), in_collision=[False]*len(poses))
        adapter.call = MagicMock(side_effect=forward)
        self.assertEqual(len(adapter.forward([[.1]*7]*70)), 70)
        self.assertEqual(adapter.call.call_count, 3)
        adapter.call = MagicMock(return_value=SolvePositionFK.Response(
            pose_stamp=[PoseStamped()], is_valid=[True], in_collision=[True]))
        with self.assertRaisesRegex(ValueError, 'in_collision=True'):
            adapter.forward([[0.]*7])
        pose = PoseStamped()
        pose.header.frame_id, pose.pose.orientation.w = 'world', 1.0
        adapter.call = MagicMock(return_value=SolvePositionFK.Response(
            pose_stamp=[pose], is_valid=[True], in_collision=[False]))
        with self.assertRaisesRegex(ValueError, 'FK frame'):
            adapter.forward([[0.]*7])

    def test_request_timeout_and_health_failure_cancel_future(self):
        for health_failure in (False, True):
            adapter = self.adapter()
            future = MagicMock()
            future.done.return_value = False
            adapter.ik.call_async.return_value = future
            now = [100.0]
            if health_failure:
                adapter.health_check = MagicMock(side_effect=RuntimeError('arm moved'))
            def spin(*args, **kwargs):
                now[0] += .2
            with patch.object(ros.time, 'monotonic', side_effect=lambda: now[0]), patch.object(ros.rclpy, 'spin_once', side_effect=spin):
                with self.assertRaises(RuntimeError if health_failure else TimeoutError):
                    adapter.call(adapter.ik, SolvePositionIK.Request(), 'IK')
            future.cancel.assert_called_once()


if __name__ == '__main__':
    unittest.main()
