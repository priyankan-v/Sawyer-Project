"""Offline pick/place planning, artifact integrity and sequencing regression tests."""

import json
import csv
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from cartesian_path import CartesianLine, CartesianPose, pose_error
from cartesian_planner import CartesianPlanner
from pick_place_plan import (Waypoint, Segment, atomic_json, recording_data, plan_task,
    segment_data, read_task, retime_segment, validate_segment, round_corners, replan_from_pick)


def forward(configurations):
    return [CartesianPose(q[:3], (0, 0, math.sin(q[3]/2), math.cos(q[3]/2))) for q in configurations]


def inverse(pose, seed):
    q = pose.orientation
    return (*pose.position, 2*math.atan2(q[2], q[3]), *seed[4:])


def planner():
    return CartesianPlanner(inverse, forward, [-3]*7, [3]*7, sample_step=.04)


def waypoint(name, xyz, yaw=0):
    joints = (*xyz, yaw, 0, 0, 0)
    return Waypoint(name, forward([joints])[0], joints)


def simulated_session_executor(node):
    """Drive the existing simulated ROS plant through Session's executor API."""
    from types import SimpleNamespace
    import rclpy
    return SimpleNamespace(spin_once=lambda timeout_sec: rclpy.spin_once(node, timeout_sec=timeout_sec))


class PickPlacePlanTests(unittest.TestCase):
    def task(self, middle=True):
        points = [waypoint('pick', (0, 0, 0))]
        if middle:
            points.append(waypoint('middle_1', (.04, -.02, .03), .15))
        points.append(waypoint('place', (.10, 0, .02), .3))
        segments = plan_task(points, planner(), [1]*7, [1]*7, [0]*7)
        data = recording_data(points, 'right_hand', {'tip': 'right_hand'}, {})
        data.update(status='planned', segments=[segment_data(s) for s in segments])
        return points, segments, data

    def test_rotation_only_and_antipodal_orientation(self):
        start = waypoint('pick', (0,0,0)).pose
        end = waypoint('place', (0,0,0), .8).pose
        plan = planner().plan_pose(start, end, [0]*7, [1]*7, [1]*7)
        self.assertGreater(plan.trajectory.duration, 0)
        self.assertGreater(plan.samples, 11)
        for fraction in np.linspace(0, 1, 43):
            actual = forward([plan.trajectory.path.sample(float(fraction)).position])[0]
            distance, angle = pose_error(actual, plan.line.sample(float(fraction)))
            self.assertLess(distance, 1e-10)
            self.assertLess(angle, 1e-6)
        opposite = CartesianPose(end.position, tuple(-q for q in end.orientation))
        a, b = CartesianLine.between(start, end), CartesianLine.between(start, opposite)
        self.assertLess(pose_error(a.sample(.4), b.sample(.4))[1], 1e-7)

    def test_shortest_rotation_across_quaternion_sign_boundary(self):
        start = waypoint('pick', (0,0,0), math.radians(170)).pose
        end = waypoint('place', (0,0,0), math.radians(-170)).pose
        line = CartesianLine.between(start, end)
        self.assertAlmostEqual(line.angle, math.radians(20))
        self.assertAlmostEqual(abs(line.sample(.5).orientation[2]), 1)

    def test_joint_constrained_approach_reaches_saved_pick_branch(self):
        target = (.1, .02, .03, .2, .25, -.1, .1)
        plan = planner().plan_pose(forward([[0]*7])[0], forward([target])[0],
                                   [0]*7, [1]*7, [1]*7, end_joints=target)
        self.assertEqual(plan.trajectory.end, target)
        # Contradictory terminal joints must be rejected by FK validation.
        with self.assertRaises(ValueError):
            planner().plan_pose(forward([[0]*7])[0], forward([target])[0],
                                [0]*7, [1]*7, [1]*7, end_joints=[0]*7)

    def test_full_plan_round_trip_and_limits(self):
        for middle in (False, True):
            points, segments, data = self.task(middle)
            self.assertEqual(len(segments), len(points)+1)
            lift = segments[-2].line
            self.assertAlmostEqual(lift.end.position[2]-lift.start.position[2], .2)
            self.assertEqual(lift.end.orientation, lift.start.orientation)
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'task.json'
                atomic_json(path, data)
                _, restored_points, restored = read_task(path)
            self.assertEqual(len(restored_points), len(points))
            for original, loaded in zip(segments, restored):
                validate_segment(loaded, planner())
                self.assertAlmostEqual(original.trajectory.duration, loaded.trajectory.duration, places=7)
                for fraction in np.linspace(0, 1, 29):
                    sample = loaded.trajectory.sample(float(fraction)*loaded.trajectory.duration)
                    self.assertTrue(all(abs(v) <= 1+1e-8 for v in sample.velocity))
                    self.assertTrue(all(abs(a) <= 1+1e-7 for a in sample.acceleration))
                for fraction in (0, 1):
                    sample = loaded.trajectory.path.sample(fraction)
                    self.assertEqual(sample.velocity, (0.,)*7)
                    self.assertEqual(sample.acceleration, (0.,)*7)

    def test_lower_replay_limits_retime_and_do_not_speed_up_recording(self):
        _, segments, _ = self.task()
        original = segments[0]
        slower = retime_segment(original, [.2]*7, [.3]*7)
        self.assertGreater(slower.trajectory.duration, original.trajectory.duration)
        faster = retime_segment(slower, [2]*7, [2]*7)
        self.assertEqual(faster.trajectory.max_velocity, slower.trajectory.max_velocity)
        self.assertEqual(faster.trajectory.duration, slower.trajectory.duration)

    def test_alternate_pick_posture_keeps_geometry_limits_timings_and_joint_continuity(self):
        points, saved, _ = self.task()
        pick_joints = (*points[0].joints[:4], 1.2, 0, 0)
        adapted = replan_from_pick(saved, planner(), pick_joints)
        previous = pick_joints
        for original, new in zip(saved, adapted):
            self.assertEqual(new.trajectory.start, previous)
            self.assertGreaterEqual(new.trajectory.duration, original.trajectory.duration)
            self.assertEqual(new.trajectory.max_velocity, original.trajectory.max_velocity)
            self.assertEqual(new.trajectory.max_acceleration, original.trajectory.max_acceleration)
            if original.line is not None:
                for u in (0, .25, .5, .75, 1):
                    self.assertLess(max(pose_error(original.line.sample(u), new.line.sample(u))), 1e-7)
                self.assertEqual(new.trajectory.end[4], 1.2)
            validate_segment(new, planner())
            previous = new.trajectory.end
        self.assertEqual(adapted[-1].trajectory.end, saved[-1].trajectory.end)
        self.assertEqual(saved[0].trajectory.start, points[0].joints)
        self.assertEqual(saved[0].trajectory.end[4], 0)

    def test_adapted_transfer_failure_identifies_stage_and_does_not_skip_validation(self):
        points, saved, _ = self.task()
        p = planner()
        p.inverse = MagicMock(side_effect=ValueError('unreachable pose'))
        with self.assertRaisesRegex(ValueError, 'to_middle_1.*unreachable pose'):
            replan_from_pick(saved, p, (*points[0].joints[:4], .8, 0, 0))

    def test_corrupt_incomplete_discontinuous_and_wrong_lift_artifacts_rejected(self):
        _, _, data = self.task(False)
        mutations = [lambda d: d.update(status='recorded'),
                     lambda d: d['segments'][0]['positions'][0].__setitem__(0, .01),
                     lambda d: d['segments'][-2]['line']['end']['position'].__setitem__(2, .99),
                     lambda d: d['segments'][0].update(duration_s=.00001),
                     lambda d: d['segments'][0]['positions'][3].__setitem__(0, float('nan')),
                     lambda d: d.update(joint_names=list(reversed(d['joint_names'])))]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'task.json'
            for mutation in mutations:
                copy = json.loads(json.dumps(data))
                mutation(copy)
                path.write_text(json.dumps(copy))
                with self.assertRaises(ValueError):
                    read_task(path)

    def test_changed_kinematics_are_rejected_and_rounding_is_explicitly_deferred(self):
        _, segments, _ = self.task(False)
        changed = planner()
        changed.forward = lambda qs: [CartesianPose((q[0]+.01, q[1], q[2]), forward([q])[0].orientation) for q in qs]
        with self.assertRaisesRegex(ValueError, 'FK tolerances'):
            validate_segment(segments[0], changed)
        with self.assertRaises(NotImplementedError):
            round_corners()


class PickPlaceWorkflowTests(unittest.TestCase):
    def test_replay_from_different_redundant_posture_has_no_joint_jump(self):
        """Regression: near pick in XYZ, very different elbow/wrist angles.

        The old approach forced recorded joints into every seed, rather than
        continuing from the current posture and reconnecting the Cartesian task.
        """
        from test_cartesian_motion import CartesianRobot
        from cartesian_motion import CartesianMotionOptions
        from pick_place_runtime import Session
        from replay_pick_place import prepare_replay, replay_sequence
        import pick_place_runtime as runtime
        robot = CartesianRobot()
        robot.q[4] = 1.2
        session = Session.__new__(Session)
        session.arm, session.tip = robot.node, 'right_hand'
        session.executor = simulated_session_executor(robot.node)
        session.options = CartesianMotionOptions(execute=True)
        session.rows, session.started, session.gripper = [], robot.now, None
        session.command_gripper, session.wait_for_enter = MagicMock(), MagicMock()
        points = [waypoint('pick', (.001,0,0)), waypoint('place', (.05,.01,0))]
        p = CartesianPlanner(lambda pose, seed: (*pose.position, *seed[3:]),
            lambda qs: [CartesianPose(q[:3], (0,0,0,1)) for q in qs], [-3]*7, [3]*7)
        saved = plan_task(points, p, [1]*7, [1]*7, [0]*7)
        clock, spin = robot.context()
        with clock, spin, patch.object(runtime, 'InteraKinematics', return_value=robot.kinematics()), \
                patch.object(runtime.rclpy, 'ok', return_value=True), \
                patch('replay_pick_place.neutral_pose', return_value=(0,)*7), patch('builtins.print'):
            robot.spin(robot.node, 0)
            approach, adapted = prepare_replay(session, points, saved)
            robot.node.command_pub.publish.assert_not_called()
            self.assertEqual(approach.trajectory.end[4], 1.2)
            self.assertEqual(adapted[0].trajectory.start, approach.trajectory.end)
            self.assertEqual(saved[0].trajectory.start[4], 0)
            replay_sequence(session, approach, adapted)
        self.assertLess(max(abs(q) for q in robot.q), .005)
        # All transfer samples stay on the approached redundant posture.
        self.assertTrue(all(abs(r['reference_joints'][4]-1.2) < 1e-8
                            for r in session.rows if r['phase'] in ('to_place', 'lift')))
        self.assertFalse(robot.node.streaming)

    def test_full_replay_in_simulated_velocity_plant_and_preview_does_not_publish(self):
        from test_cartesian_motion import CartesianRobot
        from cartesian_motion import CartesianMotionOptions
        from pick_place_runtime import Session
        from replay_pick_place import prepare_replay, replay_sequence
        import pick_place_runtime as runtime
        for execute in (False, True):
            robot = CartesianRobot()
            session = Session.__new__(Session)
            session.arm, session.tip = robot.node, 'right_hand'
            session.executor = simulated_session_executor(robot.node)
            session.options = CartesianMotionOptions(execute=execute)
            session.rows, session.started, session.gripper = [], robot.now, None
            session.command_gripper = MagicMock()
            session.wait_for_enter = MagicMock()
            points = [waypoint('pick', (.02,0,0)), waypoint('place', (.06,.01,0))]
            p = CartesianPlanner(lambda pose, seed: (*pose.position, *seed[3:]),
                lambda qs: [CartesianPose(q[:3], (0,0,0,1)) for q in qs], [-3]*7, [3]*7)
            segments = plan_task(points, p, [1]*7, [1]*7, [0]*7)
            clock, spin = robot.context()
            with clock, spin, patch.object(runtime, 'InteraKinematics', return_value=robot.kinematics()), \
                    patch.object(runtime.rclpy, 'ok', return_value=True), \
                    patch('replay_pick_place.neutral_pose', return_value=(0,)*7), patch('builtins.print'):
                approach, checked = prepare_replay(session, points, segments)
                robot.node.command_pub.publish.assert_not_called()
                if execute:
                    replay_sequence(session, approach, checked)
            if execute:
                self.assertEqual(set(r['phase'] for r in session.rows),
                                 {'approach_pick', 'to_place', 'lift', 'neutral'})
                self.assertLess(max(abs(q) for q in robot.q), .005)
                last_transfer = max(r['time_s'] for r in session.rows if r['phase'] == 'to_place')
                first_lift = min(r['time_s'] for r in session.rows if r['phase'] == 'lift')
                self.assertGreaterEqual(first_lift-last_transfer, 2)
            else:
                session.command_gripper.assert_not_called()
                robot.node.command_pub.publish.assert_not_called()

    def test_enter_wait_services_ros_and_closed_input_cancels(self):
        from pick_place_runtime import Session
        from io import StringIO
        session = Session.__new__(Session)
        session.pump = MagicMock()
        for input_text, expected_error in (('\n', None), ('', EOFError)):
            terminal = StringIO(input_text)
            with patch('sys.stdin', terminal), patch('select.select', return_value=([terminal], [], [])), \
                    patch('builtins.print'):
                if expected_error:
                    with self.assertRaises(expected_error):
                        session.wait_for_enter('Record')
                else:
                    session.wait_for_enter('Record')
            session.pump.assert_called()

    def test_plot_and_csv_export(self):
        from pick_place_runtime import write_outputs
        from pick_place_plan import pose_data
        points, segments, _ = PickPlacePlanTests().task(False)
        row = dict(time_s=1, phase='to_place', segment_time_s=.2,
                   actual=pose_data(points[0].pose), desired=pose_data(points[1].pose),
                   joints=points[0].joints, reference_joints=points[1].joints)
        with tempfile.TemporaryDirectory() as directory, patch('builtins.print'):
            write_outputs(directory, 'test', points, segments, [row], renderer='/usr/bin/python3')
            image = Path(directory)/'test.png'
            self.assertTrue(image.read_bytes().startswith(b'\x89PNG'))
            self.assertGreater(image.stat().st_size, 10000)
            with (Path(directory)/'test.csv').open() as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(len(rows[0]), len(rows[1]))
            self.assertIn('actual_z_m', rows[0])

    def test_record_opens_before_first_prompt_and_finishes_with_release(self):
        from record_pick_place import record_sequence
        pick, middle, place = waypoint('pick', (0,0,0)), waypoint('middle_1', (.02,0,0)), waypoint('place', (.04,0,0))
        session = MagicMock()
        session.capture_waypoint.side_effect = [pick, middle, place]
        session.current_state.side_effect = [(pick.pose, pick.joints), (place.pose, place.joints)]
        with patch('builtins.print'):
            points = record_sequence(session, 1)
        actions = [call[0] for call in session.method_calls]
        self.assertEqual(actions[0], 'command_gripper')
        self.assertEqual(session.method_calls[0].args, ('open',))
        self.assertEqual(session.method_calls[-1].args, ('open',))
        self.assertEqual(session.wait_for_enter.call_count, 2)
        self.assertEqual(points, (pick, middle, place))

    def test_replay_order_and_release_delay(self):
        from replay_pick_place import replay_sequence
        from pick_place_runtime import Session
        session = MagicMock(spec=Session)
        with patch('builtins.print'):
            replay_sequence(session, 'approach', ['transfer1', 'transfer2', 'lift', 'neutral'])
        calls = session.method_calls
        self.assertEqual(calls[0].args, ('open',))
        self.assertEqual(calls[1].args, ('approach',))
        self.assertEqual(calls[2][0], 'wait_for_enter')
        self.assertEqual(calls[4].args, ('close',))
        self.assertEqual(calls[5][0], 'execute_segments')
        self.assertEqual(calls[5].args, (['transfer1', 'transfer2'],))
        self.assertEqual(calls[6].args, ('open',))
        self.assertEqual(calls[7].args, (2.0,))
        self.assertEqual(calls[8].args, (['lift', 'neutral'],))

    def test_failed_transfer_does_not_release_or_return_to_neutral(self):
        from replay_pick_place import replay_sequence
        from pick_place_runtime import Session
        session = MagicMock(spec=Session)
        session.execute_segments.side_effect = RuntimeError('tracking failure')
        with self.assertRaises(RuntimeError):
            replay_sequence(session, 'approach', ['transfer', 'lift', 'neutral'])
        self.assertEqual(session.command_gripper.call_args_list[-1].args, ('close',))
        session.wait_release_delay.assert_not_called()

    def test_pid_execution_logs_actual_feedback_and_stops_on_pose_error(self):
        from test_cartesian_motion import CartesianRobot
        from cartesian_motion import CartesianMotionOptions
        from pick_place_runtime import Session
        for disturb in (False, True):
            robot = CartesianRobot()
            session = Session.__new__(Session)
            session.arm, session.tip = robot.node, 'right_hand'
            session.executor = simulated_session_executor(robot.node)
            session.options = CartesianMotionOptions(execute=True)
            session.rows, session.started = [], 0
            p = CartesianPlanner(lambda pose, seed: (*pose.position, *seed[3:]),
                lambda qs: [CartesianPose(q[:3], (0,0,0,1)) for q in qs], [-3]*7, [3]*7)
            plan = p.plan(CartesianPose((0,0,0), (0,0,0,1)), (4,0,0), [0]*7, [1]*7, [1]*7, 2)
            segment = Segment('to_place', plan.trajectory, plan.line)
            clock, _ = robot.context()
            original = robot.spin
            def spin(node, timeout_sec):
                if disturb and robot.node.streaming:
                    robot.pose_shift = .1
                original(node, timeout_sec)
            import cartesian_motion
            with clock, patch.object(cartesian_motion.rclpy, 'spin_once', side_effect=spin), \
                    patch.object(cartesian_motion.rclpy, 'ok', return_value=True), patch('builtins.print'):
                if disturb:
                    with self.assertRaisesRegex(RuntimeError, 'Cartesian tracking error'):
                        session.execute_segment(segment)
                else:
                    session.execute_segment(segment)
            self.assertTrue(session.rows)
            self.assertIn('actual', session.rows[0])
            self.assertFalse(robot.node.streaming)

    def test_position_hold_velocity_transient_settles_before_next_segment(self):
        from test_cartesian_motion import CartesianRobot
        from cartesian_motion import CartesianMotionOptions
        from pick_place_runtime import Session
        from intera_core_msgs.msg import JointCommand
        import pick_place_runtime as runtime
        robot = CartesianRobot()
        session = Session.__new__(Session)
        session.arm, session.tip, session.gripper = robot.node, 'right_hand', None
        session.executor = simulated_session_executor(robot.node)
        session.options = CartesianMotionOptions(execute=True)
        session.rows, session.started = [], robot.now
        p = CartesianPlanner(lambda pose, seed: (*pose.position, *seed[3:]),
            lambda qs: [CartesianPose(q[:3], (0,0,0,1)) for q in qs], [-3]*7, [3]*7)
        points = [waypoint('pick', (0,0,0)), waypoint('middle_1', (.02,0,0)), waypoint('place', (.04,0,0))]
        segments = plan_task(points, p, [1]*7, [1]*7, [0]*7)
        kick_until = [0]
        def publish(message):
            robot.publish(message)
            if message.mode == JointCommand.POSITION_MODE:
                kick_until[0] = robot.now+.15
        original_spin = robot.spin
        def spin(node, timeout_sec):
            original_spin(node, timeout_sec)
            if robot.now < kick_until[0]:
                robot.node.velocity = (0.,0.,0.,0.,.08,0.,0.)
        robot.node.command_pub.publish.side_effect = publish
        clock, _ = robot.context()
        with clock, patch.object(runtime.rclpy, 'spin_once', side_effect=spin), \
                patch.object(runtime.rclpy, 'ok', return_value=True), patch('builtins.print'):
            session.execute_segment(segments[0])
            session.execute_segment(segments[1])
        self.assertLess(abs(robot.q[0]-.04), .005)
        self.assertTrue(any(r['event'] == 'settling' and r['joint_velocity'][4] == .08 for r in session.rows))
        self.assertFalse(robot.node.streaming)

    def test_transition_rejects_persistent_motion_displacement_and_stale_feedback(self):
        from test_cartesian_motion import CartesianRobot
        from cartesian_motion import CartesianMotionOptions
        from pick_place_runtime import Session
        import pick_place_runtime as runtime
        for failure, exception, message in [('moving', TimeoutError, 'did not settle'),
                                             ('displaced', RuntimeError, 'displaced'),
                                             ('stale', RuntimeError, 'Stale Cartesian')]:
            robot = CartesianRobot()
            session = Session.__new__(Session)
            session.arm, session.tip, session.gripper = robot.node, 'right_hand', None
            session.executor = simulated_session_executor(robot.node)
            session.options = CartesianMotionOptions(execute=True, settle_time=.4)
            session.rows, session.started = [], robot.now
            p = CartesianPlanner(lambda pose, seed: (*pose.position, *seed[3:]),
                lambda qs: [CartesianPose(q[:3], (0,0,0,1)) for q in qs], [-3]*7, [3]*7)
            plan = p.plan(waypoint('pick',(0,0,0)).pose, (4,0,0), [0]*7, [1]*7, [1]*7)
            segment = Segment('to_middle_2', plan.trajectory, plan.line)
            original = robot.spin
            def spin(node, timeout_sec):
                if failure == 'stale':
                    robot.drop_pose = True
                if failure == 'displaced':
                    robot.q[0] = .02
                original(node, timeout_sec)
                if failure == 'moving':
                    robot.node.velocity = (.08,)*7
            clock, _ = robot.context()
            with clock, patch.object(runtime.rclpy, 'spin_once', side_effect=spin), \
                    patch.object(runtime.rclpy, 'ok', return_value=True), self.assertRaisesRegex(exception, message):
                session.assert_start(segment)
            robot.node.command_pub.publish.assert_not_called()

    def test_after_enter_reacquires_queued_pose_before_checking_start(self):
        from test_cartesian_motion import CartesianRobot
        from cartesian_motion import CartesianMotionOptions
        from pick_place_runtime import Session
        import pick_place_runtime as runtime
        robot = CartesianRobot()
        session = Session.__new__(Session)
        session.arm, session.tip, session.gripper = robot.node, 'right_hand', None
        session.executor = simulated_session_executor(robot.node)
        session.options = CartesianMotionOptions(execute=True)
        session.rows, session.started = [], robot.now
        p = CartesianPlanner(lambda pose, seed: (*pose.position, *seed[3:]),
            lambda qs: [CartesianPose(q[:3], (0,0,0,1)) for q in qs], [-3]*7, [3]*7)
        plan = p.plan(waypoint('pick',(0,0,0)).pose, (4,0,0), [0]*7, [1]*7, [1]*7)
        segment = Segment('to_middle_1', plan.trajectory, plan.line)
        pose, _ = robot.node.pose_feedback.tips['right_hand']
        robot.node.pose_feedback.tips['right_hand'] = pose, robot.now-20
        original, callbacks = robot.spin, [0]
        def spin(node, timeout_sec):
            callbacks[0] += 1
            # A real executor may service joints/state/IO before the queued tip.
            robot.drop_pose = callbacks[0] < 6
            original(node, timeout_sec)
        clock, _ = robot.context()
        with clock, patch.object(runtime.rclpy, 'spin_once', side_effect=spin), \
                patch.object(runtime.rclpy, 'ok', return_value=True):
            session.assert_start(segment)
            robot.node.pose_feedback.read('right_hand')
        self.assertGreaterEqual(callbacks[0], 6)
        self.assertLess(robot.now-session.started, 1)
        robot.node.command_pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
