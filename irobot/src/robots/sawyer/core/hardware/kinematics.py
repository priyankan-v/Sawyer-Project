"""Robot-side inverse and forward kinematics queries."""

from __future__ import annotations

import time

from ...config import JOINT_NAMES
from ..types import ordered_values
from .feedback import from_pose


class InteraKinematics:
    """
    Seeded IK and batched FK. No motion commands are sent by this adapter.
    """

    def __init__(  # noqa: PLR0913
        self,
        node,
        tip,
        frame,
        request_timeout=None,
        planning_timeout=None,
        health_check=None,
    ):
        self.node, self.tip, self.frame = node, tip, frame
        self.request_timeout = (
            node.config.motion.ik_timeout
            if request_timeout is None
            else request_timeout
        )
        self.deadline = time.monotonic() + (
            node.config.motion.planning_timeout
            if planning_timeout is None
            else planning_timeout
        )
        self.health_check = health_check
        from ..communication.wire import SolvePositionFK, SolvePositionIK

        self.ik = node.create_client(SolvePositionIK, node.config.ik_service)
        self.fk = node.create_client(SolvePositionFK, node.config.fk_service)

    def close(self):
        self.node.destroy_client(self.ik)
        self.node.destroy_client(self.fk)

    def wait_ready(self, timeout=None):
        deadline = min(
            time.monotonic()
            + (
                self.node.config.connect_timeout
                if timeout is None
                else timeout
            ),
            self.deadline,
        )
        for client, name in (
            (self.ik, self.node.config.ik_service),
            (self.fk, self.node.config.fk_service),
        ):
            while not client.wait_for_service(
                timeout_sec=self.node.config.input_poll_timeout
            ):
                if time.monotonic() >= deadline:
                    msg = f'Kinematics service unavailable: {name}'
                    raise RuntimeError(msg)
                self.node.spin_once(
                    timeout_sec=self.node.config.callback_slice
                )

    def call(self, client, request, label):
        if time.monotonic() >= self.deadline:
            msg = 'Cartesian planning exceeded its planning timeout'
            raise TimeoutError(msg)
        future = client.call_async(request)
        deadline = min(self.deadline, time.monotonic() + self.request_timeout)
        try:
            while not future.done():
                if time.monotonic() >= deadline:
                    msg = f'{label} response timed out'
                    raise TimeoutError(msg)  # noqa: TRY301
                self.node.spin_once(
                    timeout_sec=self.node.config.callback_slice
                )
                if self.health_check:
                    self.health_check()
            response = future.result()
            if response is None:
                msg = f'{label} returned no response'
                raise RuntimeError(msg)  # noqa: TRY301
            return response  # noqa: TRY300
        except BaseException:
            future.cancel()
            raise

    def inverse(self, pose, seed):
        from ..communication.wire import (
            JointState,
            PoseStamped,
            SolvePositionIK,
        )

        if pose.frame != self.frame:
            msg = 'IK pose frame changed during planning'
            raise ValueError(msg)
        target = PoseStamped()
        target.header.frame_id = pose.frame
        target.header.stamp = self.node.get_clock().now().to_msg()
        (
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ) = pose.position
        (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ) = pose.orientation
        request = SolvePositionIK.Request()
        request.pose_stamp = [target]
        request.tip_names = [self.tip]
        request.seed_mode = SolvePositionIK.Request.SEED_USER
        request.seed_angles = [
            JointState(name=list(JOINT_NAMES), position=list(seed))
        ]
        request.use_nullspace_goal = [False]
        response = self.call(self.ik, request, 'IK')
        if len(response.result_type) != 1:
            msg = 'IK returned a malformed response'
            raise ValueError(msg)
        code = int(response.result_type[0])
        if code <= 0:
            reasons = {
                -1: 'unreachable pose',
                -2: 'self collision',
                -3: 'unknown endpoint',
            }
            msg = (
                f'IK rejected position {pose.position}: '
                f'{reasons.get(code, code)}'
            )
            raise ValueError(msg)
        if len(response.joints) != 1:
            msg = 'IK returned a malformed joint solution'
            raise ValueError(msg)
        return ordered_values(
            response.joints[0].name, response.joints[0].position, 'IK solution'
        )

    def forward_results(self, configurations):
        from ..communication.wire import JointState, SolvePositionFK

        poses = []
        for offset in range(0, len(configurations), 32):
            batch = configurations[offset : offset + 32]
            request = SolvePositionFK.Request()
            request.configuration = [
                JointState(name=list(JOINT_NAMES), position=list(values))
                for values in batch
            ]
            request.tip_names = [self.tip] * len(batch)
            response = self.call(self.fk, request, 'FK')
            if any(
                len(field) != len(batch)
                for field in (
                    response.pose_stamp,
                    response.is_valid,
                    response.in_collision,
                )
            ):
                msg = 'FK returned a malformed response'
                raise ValueError(msg)
            for stamped, valid, collision in zip(
                response.pose_stamp,
                response.is_valid,
                response.in_collision,
                strict=False,
            ):
                if not valid:
                    msg = (
                        f'FK rejected path configuration: valid={valid}, '
                        f'in_collision={collision}'
                    )
                    raise ValueError(msg)
                pose = from_pose(
                    stamped.pose, stamped.header.frame_id or self.frame
                )
                if pose.frame != self.frame:
                    msg = (
                        f'FK frame {pose.frame} does not match requested '
                        f'frame {self.frame}'
                    )
                    raise ValueError(msg)
                poses.append((pose, bool(collision)))
        return poses

    def forward(self, configurations):
        results = self.forward_results(configurations)
        if any(collision for _, collision in results):
            msg = 'FK rejected a path configuration in collision'
            raise ValueError(msg)
        return [pose for pose, _ in results]
