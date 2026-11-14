"""
Explicit localhost-only DDS check with synthetic state, never a real robot.

Run with ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=231 after activating the runtime.
This file is intentionally outside unittest's test_*.py discovery pattern.
"""

# Explicit synthetic-check command output.
# ruff: noqa: T201
from __future__ import annotations

import os
import threading
import time


def main():
    if (
        os.environ.get('ROS_LOCALHOST_ONLY') != '1'
        or os.environ.get('ROS_DOMAIN_ID') != '231'
    ):
        msg = (
            'Use ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=231 '
            'for this synthetic check'
        )
        raise RuntimeError(msg)
    from intera_core_msgs.msg import (
        CollisionDetectionState,
        EndpointState,
        EndpointStates,
        JointCommand,
        JointLimits,
        RobotAssemblyState,
    )
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from sensor_msgs.msg import JointState

    from irobot import SawyerConfig, SawyerController

    context = Context()
    context.init(args=[], domain_id=231)
    node = Node('synthetic_sawyer_feedback', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    names = [f'right_j{i}' for i in range(7)]
    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    tip = EndpointState(valid=True)
    tip.header.frame_id = 'base'
    tip.pose.orientation.w = 1.0
    samples = [
        (
            '/robot/joint_states',
            JointState(name=names, position=[0.0] * 7, velocity=[0.0] * 7),
            qos_profile_sensor_data,
        ),
        (
            '/robot/state',
            RobotAssemblyState(enabled=False, ready=False),
            qos_profile_sensor_data,
        ),
        (
            '/robot/limb/right/collision_detection_state',
            CollisionDetectionState(collision_state=False),
            qos_profile_sensor_data,
        ),
        (
            '/robot/joint_limits',
            JointLimits(
                joint_names=names,
                position_lower=[-3.0] * 7,
                position_upper=[3.0] * 7,
                velocity=[1.0] * 7,
                accel=[1.0] * 7,
            ),
            latched,
        ),
        ('/robot/limb/right/endpoint_state', tip, qos_profile_sensor_data),
        (
            '/robot/limb/right/tip_states',
            EndpointStates(names=['right_hand'], states=[tip]),
            qos_profile_sensor_data,
        ),
    ]
    publishers = [
        (node.create_publisher(type(message), topic, qos), message)
        for topic, message, qos in samples
    ]
    commands = []
    node.create_subscription(
        JointCommand, '/robot/limb/right/joint_command', commands.append, 1
    )
    node.create_timer(
        0.01, lambda: [pub.publish(message) for pub, message in publishers]
    )
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    robot = SawyerController(
        SawyerConfig(domain_id=231, enable_gripper=False, connect_timeout=5.0)
    )
    try:
        assert robot.connect()
        assert robot.joint_positions == (0.0,) * 7
        assert robot.endpoint_pose.frame == 'base'
        status = robot.get_status()
        assert status['connected'], status
        assert not status['ready'], status
        assert not status['enabled'], status
        robot.disconnect()
        assert (
            context.ok()
        ), 'Driver shut down the simulator/application context'
        time.sleep(0.05)
        assert not commands, 'Read-only connection published arm commands'
        print(
            'PASS: synthetic DDS feedback, read-only connection, '
            'and independent context shutdown.'
        )
    finally:
        robot.disconnect()
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        context.shutdown()


if __name__ == '__main__':
    main()
