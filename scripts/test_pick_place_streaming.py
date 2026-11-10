"""Connected-reference timing, controller continuity and simulated hardware faults."""

import unittest
from unittest.mock import patch

from intera_core_msgs.msg import JointCommand
from cartesian_motion import CartesianMotionOptions
from cartesian_path import CartesianPose
from cartesian_planner import CartesianPlanner
from pick_place_plan import SegmentSequence, plan_task, retime_segment
from pick_place_runtime import Session
import pick_place_runtime as runtime
from test_cartesian_motion import CartesianRobot
from test_pick_place import waypoint, simulated_session_executor


def task():
    p = CartesianPlanner(lambda pose, seed: (*pose.position, *seed[3:]),
        lambda qs: [CartesianPose(q[:3], (0,0,0,1)) for q in qs], [-3]*7, [3]*7)
    points = [waypoint('pick',(0,0,0)), waypoint('middle_1',(.04,0,0)), waypoint('place',(.04,.04,0))]
    return points, plan_task(points, p, [1]*7, [1]*7, [0]*7)


def session_for(robot):
    session = Session.__new__(Session)
    session.arm, session.tip, session.gripper = robot.node, 'right_hand', None
    session.options = CartesianMotionOptions(execute=True)
    session.rows, session.started = [], robot.now
    session.executor = simulated_session_executor(robot.node)
    return session


class SequenceTests(unittest.TestCase):
    def test_endpoint_holds_until_feedback_gate_and_derivatives_are_continuous(self):
        _, segments = task()
        sequence = SegmentSequence(segments[:2], 5)
        end = segments[0].trajectory.duration
        with self.assertRaises(ValueError):
            sequence.advance(end/2)
        held = sequence.sample(end+.1)
        self.assertEqual(held.position, segments[0].trajectory.end)
        self.assertEqual(held.velocity, (0.,)*7)
        self.assertEqual(held.acceleration, (0.,)*7)
        sequence.advance(end+.1)
        start = sequence.sample(end+.1)
        self.assertEqual(start, held)
        self.assertTrue(sequence.is_last)

    def test_common_limits_retime_all_segments_and_timeout_is_per_segment(self):
        _, segments = task()
        slow = retime_segment(segments[1], [.1]*7, [.2]*7)
        sequence = SegmentSequence([segments[0], slow], .5)
        self.assertEqual(sequence.max_velocity, (.1,)*7)
        self.assertGreaterEqual(sequence.segments[0].trajectory.duration, segments[0].trajectory.duration)
        with self.assertRaisesRegex(TimeoutError, 'to_middle_1'):
            sequence.check_timeout(sequence.budgets[0]+.01)
        sequence.advance(sequence.active.trajectory.duration)
        sequence.check_timeout(sequence.segment_started+.1)
        with self.assertRaisesRegex(ValueError, 'timeout'):
            SegmentSequence(segments[:2], 5, timeout=.0001)

    def test_discontinuous_chain_is_rejected_before_commands(self):
        _, segments = task()
        with self.assertRaisesRegex(ValueError, 'discontinuous'):
            SegmentSequence([segments[0], segments[-1]], 5)


class ControllerStreamingTests(unittest.TestCase):
    def run_transfer(self, connected):
        robot = CartesianRobot()
        session = session_for(robot)
        _, segments = task()
        clock, spin = robot.context()
        with clock, spin, patch.object(runtime.rclpy, 'ok', return_value=True), patch('builtins.print'), \
                patch.object(runtime, 'JointPIDController', wraps=runtime.JointPIDController) as controllers:
            start = robot.now
            if connected:
                session.execute_segments(segments[:2])
            else:
                for segment in segments[:2]:
                    session.execute_segment(segment)
            duration = robot.now-start
        return robot, session, duration, controllers.call_count

    def test_one_controller_and_no_position_mode_switch_at_middle_point(self):
        robot, session, _, controllers = self.run_transfer(True)
        self.assertEqual(controllers, 1)
        modes = [m.mode for m in robot.messages]
        first_hold = modes.index(JointCommand.POSITION_MODE)
        self.assertTrue(all(mode == JointCommand.VELOCITY_MODE for mode in modes[:first_hold]))
        self.assertEqual(modes.count(JointCommand.POSITION_MODE), 3)
        self.assertLess(abs(robot.q[0]-.04), .005)
        self.assertLess(abs(robot.q[1]-.04), .005)
        rows = [r for r in session.rows if r['event'] == 'tracking']
        left = max(r['time_s'] for r in rows if r['phase'] == 'to_middle_1')
        right = min(r['time_s'] for r in rows if r['phase'] == 'to_place')
        self.assertLess(right-left, .04)
        first_next = next(r for r in rows if r['phase'] == 'to_place')
        self.assertLess(abs(first_next['actual']['position'][0]-.04), .003)
        self.assertLess(max(map(abs, first_next['joint_velocity'])), .03)
        self.assertFalse(robot.node.streaming)

    def test_connected_execution_reduces_overhead_without_raising_reference_limits(self):
        _, _, separate_time, separate_controllers = self.run_transfer(False)
        robot, _, connected_time, controllers = self.run_transfer(True)
        self.assertEqual(separate_controllers, 2)
        self.assertEqual(controllers, 1)
        self.assertGreater(separate_time-connected_time, .3)
        self.assertTrue(all(abs(v) <= 1+1e-8 for m in robot.messages for v in m.velocity))

    def test_tracking_failure_stops_before_advancing_to_next_line(self):
        robot = CartesianRobot()
        robot.freeze = True
        session = session_for(robot)
        _, segments = task()
        clock, spin = robot.context()
        with clock, spin, patch.object(runtime.rclpy, 'ok', return_value=True), patch('builtins.print'), \
                self.assertRaisesRegex(RuntimeError, 'Cartesian tracking error'):
            session.execute_segments(segments[:2])
        self.assertFalse(robot.node.streaming)
        self.assertNotIn('to_place', {r['phase'] for r in session.rows})


if __name__ == '__main__':
    unittest.main()
