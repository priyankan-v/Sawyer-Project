"""Packaged controller with a virtual plant; never initialize ROS or use hardware."""

# Numeric regression bounds and controlled private fixtures are intentional.
# ruff: noqa: PLR2004, SLF001, PT009, SIM117
# Fault fixtures deliberately exercise nested cleanup and private resource lifecycles.
from __future__ import annotations

import copy
import json
import threading
import unittest
from contextlib import contextmanager
from unittest.mock import MagicMock, patch

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('intera_core_msgs')

from builtin_interfaces.msg import Time
from intera_core_msgs.msg import (
    EndpointState,
    EndpointStates,
    IOComponentStatus,
    IODataStatus,
    IODeviceStatus,
    IOStatus,
    JointCommand,
    RobotAssemblyState,
)
from sensor_msgs.msg import JointState

from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.config import JOINT_NAMES
from irobot.src.robots.sawyer.core.base import (
    CartesianPose,
    EndpointFeedback,
    InteraKinematics,
    JointBounds,
    JointFeedback,
    SawyerGripper,
    _BridgeClient,
)


def gripper_message(calibrated=True):  # noqa: FBT002
    signals = []
    for name, value, role in (
        ('has_error', False, 'input'),
        ('is_calibrated', calibrated, 'input'),
        ('is_moving', False, 'input'),
        ('is_gripping', False, 'input'),
        ('position_response_m', 0.0, 'input'),
        ('position_m', 0.0, 'output'),
        ('go', True, 'output'),
        ('calibrate', False, 'output'),
    ):
        kind = 'bool' if type(value) is bool else 'float'
        signals.append(
            IODataStatus(
                name=name,
                format=json.dumps({'role': role, 'type': kind}),
                data=json.dumps([value]),
                status=IOStatus(tag='ready'),
            )
        )
    return IODeviceStatus(device=IOComponentStatus(name='right_gripper', status=IOStatus(tag='ready')), signals=signals)


class Plant:
    def __init__(self):
        self.now = 100.0
        self.q, self.velocity, self.command = [0.0] * 7, [0.0] * 7, [0.0] * 7
        self.messages = []
        self.stale = self.drop_pose = self.interrupt = self.cancel = self.collide = False
        self.pose_shift = 0.0
        self.robot = SawyerController(SawyerConfig(enable_gripper=False, neutral_joints=(0.2,) * 7))
        node = self.node = _BridgeClient.__new__(_BridgeClient)
        self.robot._node, self.robot._connected = node, True
        node.config = self.robot.config
        node.joints, node.velocity = tuple(self.q), tuple(self.velocity)
        node.feedback = JointFeedback(JOINT_NAMES)
        node.feedback.update(JOINT_NAMES, self.q, self.velocity, self.now)
        node.last_feedback_error = node.last_limits_error = None
        node.state = RobotAssemblyState(enabled=True, ready=True)
        node.limits = JointBounds((-3.0,) * 7, (3.0,) * 7, (1.0,) * 7, (1.0,) * 7)
        node.collision, node.streaming = False, False
        node.joints_at = node.state_at = node.collision_at = self.now
        node.cancel_requested = threading.Event()
        node.command_pub, node.watchdog_pub = MagicMock(), MagicMock()
        node.command_pub.get_subscription_count.return_value = 1
        node.watchdog_pub.get_subscription_count.return_value = 1
        node.command_pub.publish.side_effect = self.publish
        node.get_clock = MagicMock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=123)
        node.get_logger = MagicMock()
        node.pose_feedback = EndpointFeedback('base', 0.2)
        self.feed_pose()

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))
        self.command = list(message.velocity) if message.mode == JointCommand.VELOCITY_MODE else [0.0] * 7

    def feed_pose(self):
        state = EndpointState(valid=True)
        state.pose.position.x = self.q[0] + self.pose_shift
        state.pose.position.y, state.pose.position.z = self.q[1:3]
        state.pose.orientation.w = 1.0
        with patch('time.monotonic', return_value=self.now):
            self.node.pose_feedback.on_endpoint(state)
            self.node.pose_feedback.on_tips(EndpointStates(names=['right_hand'], states=[state]))

    def spin(self, timeout_sec):
        if self.now > 100.1:
            if self.interrupt:
                self.interrupt = False
                raise KeyboardInterrupt
            if self.cancel:
                self.robot.stop()
            if self.collide:
                self.node.collision = True
        dt = max(timeout_sec, 0.00001)
        self.now += dt
        for i in range(7):
            self.velocity[i] += (self.command[i] - self.velocity[i]) * min(1.0, dt / 0.04)
            self.q[i] += self.velocity[i] * dt
        if not (self.stale and self.node.streaming):
            self.node.on_joints(JointState(name=list(JOINT_NAMES), position=self.q, velocity=self.velocity))
            self.node.state_at = self.node.collision_at = self.now
        # An unrelated gripper sample must never refresh stale arm feedback.
        self.node.on_joints(JointState(name=['finger'], position=[0.0], velocity=[0.0]))
        if not self.drop_pose:
            self.feed_pose()

    @contextmanager
    def running(self):
        with (
            patch('time.monotonic', side_effect=lambda: self.now),
            patch.object(self.node, 'spin_once', side_effect=self.spin),
        ):
            yield

    def kinematics(self):
        adapter = MagicMock()
        adapter.inverse.side_effect = lambda pose, seed: (*pose.position, *seed[3:])
        adapter.forward.side_effect = lambda qs: [CartesianPose(q[:3], (0, 0, 0, 1)) for q in qs]
        return adapter


class ArmTests(unittest.TestCase):
    def test_neutral_preview_and_infeasible_execution_never_publish(self):
        plant = Plant()
        with plant.running():
            result = plant.robot.move_to_neutral(duration=2)
            assert result.execution is None
            with pytest.raises(ValueError, match='minimum feasible'):
                plant.robot.move_to_neutral(duration=0.01, execute=True)
        assert not plant.messages
        plant.node.watchdog_pub.publish.assert_not_called()

    def test_neutral_execution_tracks_then_holds_position(self):
        plant = Plant()
        with plant.running():
            result = plant.robot.move_to_neutral(duration=2, execute=True)
        assert result.execution.reference_duration == 2
        assert max(abs(q - 0.2) for q in plant.q) < 0.02
        assert plant.messages[-1].mode == JointCommand.POSITION_MODE
        assert not plant.node.streaming

    def test_faults_interruptions_and_cancellation_stop_streaming(self):
        for fault in ('stale', 'interrupt', 'cancel', 'collide'):
            plant = Plant()
            setattr(plant, fault, True)
            with self.subTest(fault=fault), plant.running():
                with pytest.raises((RuntimeError, KeyboardInterrupt)):
                    plant.robot.move_to_neutral(duration=2, execute=True)
            assert not plant.node.streaming
            zeros = [
                m for m in plant.messages if m.mode == JointCommand.VELOCITY_MODE and list(m.velocity) == [0.0] * 7
            ]
            assert len(zeros) >= 5

    def test_cancellation_during_planning_never_starts_commands(self):
        from irobot.src.robots.sawyer.core.controller import QuinticTrajectory

        plant = Plant()

        def plan_and_cancel(*args, **kwargs):
            plan = QuinticTrajectory(*args, **kwargs)
            plant.robot.stop()
            return plan

        with (
            plant.running(),
            patch('irobot.src.robots.sawyer.core.controller.QuinticTrajectory', side_effect=plan_and_cancel),
            pytest.raises(RuntimeError, match='cancelled'),
        ):
            plant.robot.move_to_neutral(duration=2, execute=True)
        assert not plant.messages
        plant.node.watchdog_pub.publish.assert_not_called()

    def test_disabled_or_missing_command_routes_never_start(self):
        plant = Plant()
        plant.node.state.enabled = False
        with plant.running(), pytest.raises(RuntimeError, match='not ready'):
            plant.robot.move_to_neutral(duration=2, execute=True)
        assert not plant.messages
        plant = Plant()
        plant.node.command_pub.get_subscription_count.return_value = 0
        with plant.running(), pytest.raises(RuntimeError, match='command/watchdog route absent'):
            plant.robot.move_to_neutral(duration=2, execute=True)
        assert not plant.messages

    def test_neutral_parameter_is_reordered_and_missing_master_is_actionable(self):
        robot = SawyerController()
        proxy = MagicMock()
        proxy.__enter__.return_value.getParam.return_value = (
            1,
            '',
            {'joint_names': list(JOINT_NAMES[::-1]), 'poses': {'neutral': list(range(7))}},
        )
        with (
            patch.dict('os.environ', {'ROS_MASTER_URI': 'http://robot:11311'}),
            patch('xmlrpc.client.ServerProxy', return_value=proxy),
        ):
            assert robot._neutral_pose() == (6, 5, 4, 3, 2, 1, 0)
        with patch.dict('os.environ', {}, clear=True), pytest.raises(RuntimeError, match='ROS_MASTER_URI'):
            robot._neutral_pose()


class CartesianTests(unittest.TestCase):
    def test_public_cartesian_preview_then_execution(self):
        plant = Plant()
        with (
            plant.running(),
            patch('irobot.src.robots.sawyer.core.controller.InteraKinematics', return_value=plant.kinematics()),
        ):
            preview = plant.robot.move_relative(dz_cm=10, duration=2)
            assert preview.plan.line.end.position == (0, 0, 0.1)
            assert not plant.messages
            result = plant.robot.move_relative(dz_cm=10, duration=2, execute=True)
        assert result.execution is not None
        assert abs(plant.q[2] - 0.1) < 0.003
        assert plant.messages[-1].mode == JointCommand.POSITION_MODE

    def test_stale_pose_stops_execution(self):
        plant = Plant()
        original = plant.spin

        def lose_pose(timeout_sec):
            if plant.node.streaming:
                plant.drop_pose = True
            original(timeout_sec)

        with (
            plant.running(),
            patch.object(plant.node, 'spin_once', side_effect=lose_pose),
            patch('irobot.src.robots.sawyer.core.controller.InteraKinematics', return_value=plant.kinematics()),
        ):
            with pytest.raises(RuntimeError, match='Stale Cartesian'):
                plant.robot.move_relative(dz_cm=10, duration=2, execute=True)
        assert not plant.node.streaming

    def test_kinematics_uses_same_node_and_configured_services(self):
        node = MagicMock()
        node.config = SawyerConfig()
        adapter = InteraKinematics(node, 'right_hand', 'base')
        calls = node.create_client.call_args_list
        assert calls[0].args[1] == node.config.ik_service
        assert calls[1].args[1] == node.config.fk_service
        adapter.close()
        assert node.destroy_client.call_count == 2


class GripperTests(unittest.TestCase):
    def gripper(self, calibrated=True):  # noqa: FBT002
        node = MagicMock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=123)
        gripper = SawyerGripper(node, SawyerConfig())
        gripper.publisher.get_subscription_count.return_value = 1
        gripper.on_state(gripper_message(calibrated=calibrated))
        return gripper

    def test_preview_never_publishes_and_uncalibrated_execution_rejected(self):
        gripper = self.gripper(calibrated=False)
        result = gripper.perform('open')
        assert not result['executed']
        with pytest.raises(RuntimeError, match='not calibrated'):
            gripper.perform('open', execute=True)
        gripper.publisher.publish.assert_not_called()

    def test_execution_waits_for_ack_and_measured_completion(self):
        gripper = self.gripper()

        def acknowledge(command):
            message = gripper_message()
            message.commands = [command.time]
            values = json.loads(command.args)['signals']
            for entry in message.signals:
                if entry.name == 'position_response_m':
                    entry.data = json.dumps(values['position_m']['data'])
            gripper.on_state(message)

        gripper.publisher.publish.side_effect = acknowledge
        assert gripper.perform('open', execute=True)['executed']
        gripper.publisher.publish.assert_called_once()

    def test_failure_requests_stop(self):
        gripper = self.gripper()
        with (
            patch.object(gripper, 'send', side_effect=TimeoutError('No ack')),
            patch.object(gripper, 'request_stop') as stop,
            pytest.raises(TimeoutError),
        ):
            gripper.perform('open', execute=True)
        stop.assert_called_once()


if __name__ == '__main__':
    unittest.main()


class CalibrationReadinessTests(unittest.TestCase):
    def test_busy_device_is_not_completed_even_when_calibrated(self):
        node = MagicMock()
        item = SawyerGripper(node, SawyerConfig())
        message = gripper_message(calibrated=True)
        message.device.status.tag = 'busy'
        item.on_state(message)
        assert not item.completed('calibrate', 0.0)
        assert not item.completed('close', 0.0)
        message.device.status.tag = 'ready'
        item.on_state(message)
        assert item.completed('calibrate', 0.0)
        assert item.completed('close', 0.0)


class GripperSettlingTests(unittest.TestCase):
    def test_completion_must_remain_stable_after_late_feedback(self):
        node = MagicMock()
        item = SawyerGripper(node, SawyerConfig(gripper_timeout=1.0))
        now = [0.0]
        node.spin_once.side_effect = lambda **_: now.__setitem__(0, now[0] + 0.05)
        item.completed = lambda *_: not 0.05 <= now[0] < 0.1
        with patch('time.monotonic', side_effect=lambda: now[0]):
            item.wait_completion('calibrate', 0.0)
        assert now[0] >= 0.3
