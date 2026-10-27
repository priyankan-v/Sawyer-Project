"""ROS adapters for Intera pose feedback and read-only IK/FK services."""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from intera_core_msgs.srv import SolvePositionFK, SolvePositionIK
from sensor_msgs.msg import JointState

if __package__:
    from .arm_motion import FEEDBACK_TIMEOUT, JOINT_NAMES, ordered_values
    from .cartesian_path import CartesianPose, pose_error
else:
    from arm_motion import FEEDBACK_TIMEOUT, JOINT_NAMES, ordered_values
    from cartesian_path import CartesianPose, pose_error


IK_SERVICE = '/ExternalTools/right/PositionKinematicsNode/IKService'
FK_SERVICE = '/ExternalTools/right/PositionKinematicsNode/FKService'


def from_pose(pose, frame):
    return CartesianPose((pose.position.x, pose.position.y, pose.position.z),
                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), frame)


class EndpointFeedback:
    """Named endpoint cache; avoids guessing that an anonymous pose is a fingertip."""

    def __init__(self, fallback_frame='base'):
        self.frame = fallback_frame
        self.endpoint = None
        self.tips = {}
        self.last_error = None

    def on_endpoint(self, message):
        try:
            if not message.valid:
                raise ValueError('Endpoint pose has valid=false')
            self.endpoint = (from_pose(message.pose, message.header.frame_id or self.frame), time.monotonic())
        except ValueError as exc:
            self.endpoint = None
            self.last_error = str(exc)

    def on_tips(self, message):
        if len(message.names) != len(message.states) or len(set(message.names)) != len(message.names):
            self.tips.clear()
            self.last_error = 'Malformed named tip-state array'
            return
        now = time.monotonic()
        for name, state in zip(message.names, message.states):
            try:
                if not state.valid:
                    raise ValueError(f'Tip {name} has valid=false')
                frame = state.header.frame_id or message.header.frame_id or self.frame
                self.tips[name] = (from_pose(state.pose, frame), now)
            except ValueError as exc:
                self.tips.pop(name, None)
                self.last_error = str(exc)

    def read(self, tip):
        if tip not in self.tips:
            raise RuntimeError(f'No valid pose for {tip} on /robot/limb/right/tip_states; '
                               f'available tips: {sorted(self.tips)}. {self.last_error or ""}')
        pose, received = self.tips[tip]
        if time.monotonic() - received >= FEEDBACK_TIMEOUT:
            raise RuntimeError(f'Stale Cartesian pose for {tip}: {time.monotonic() - received:.3f}s old')
        return pose

    def select(self, requested=None):
        if requested:
            return requested, self.read(requested)
        if self.endpoint is None or time.monotonic() - self.endpoint[1] >= FEEDBACK_TIMEOUT:
            raise RuntimeError('Missing/stale /robot/limb/right/endpoint_state')
        matches = []
        for name in self.tips:
            try:
                position, angle = pose_error(self.read(name), self.endpoint[0])
            except (RuntimeError, ValueError):
                continue
            if position <= 1e-4 and angle <= 1e-3:
                matches.append(name)
        if len(matches) != 1:
            raise RuntimeError(f'Cannot uniquely identify endpoint tip (matches={matches}); '
                               'inspect tip_states and select --tip NAME')
        return matches[0], self.read(matches[0])


class InteraKinematics:
    """Seeded IK and batched FK. No motion commands are sent by this adapter."""

    def __init__(self, node, tip, frame, request_timeout=2.0, planning_timeout=60.0, health_check=None):
        self.node, self.tip, self.frame = node, tip, frame
        self.request_timeout = request_timeout
        self.deadline = time.monotonic() + planning_timeout
        self.health_check = health_check
        self.ik = node.create_client(SolvePositionIK, IK_SERVICE)
        self.fk = node.create_client(SolvePositionFK, FK_SERVICE)

    def close(self):
        self.node.destroy_client(self.ik)
        self.node.destroy_client(self.fk)

    def wait_ready(self, timeout=5.0):
        deadline = min(time.monotonic() + timeout, self.deadline)
        for client, name in ((self.ik, IK_SERVICE), (self.fk, FK_SERVICE)):
            while not client.wait_for_service(timeout_sec=.05):
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'Kinematics service unavailable: {name}; check bridge configuration')
                rclpy.spin_once(self.node, timeout_sec=.01)

    def call(self, client, request, label):
        if time.monotonic() >= self.deadline:
            raise TimeoutError('Cartesian planning exceeded its planning timeout')
        future = client.call_async(request)
        deadline = min(self.deadline, time.monotonic() + self.request_timeout)
        try:
            while not future.done():
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'{label} response timed out')
                rclpy.spin_once(self.node, timeout_sec=.01)
                if self.health_check:
                    self.health_check()
            response = future.result()
            if response is None:
                raise RuntimeError(f'{label} returned no response')
            return response
        except BaseException:
            future.cancel()
            raise

    def inverse(self, pose, seed):
        if pose.frame != self.frame:
            raise ValueError('IK pose frame changed during planning')
        target = PoseStamped()
        target.header.frame_id = pose.frame
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = pose.position
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = pose.orientation
        request = SolvePositionIK.Request()
        request.pose_stamp = [target]
        request.tip_names = [self.tip]
        request.seed_mode = SolvePositionIK.Request.SEED_USER
        request.seed_angles = [JointState(name=JOINT_NAMES, position=list(seed))]
        request.use_nullspace_goal = [False]
        response = self.call(self.ik, request, 'IK')
        if len(response.result_type) != 1:
            raise ValueError('IK returned a malformed response')
        code = int(response.result_type[0])
        if code <= 0:
            reasons = {-1: 'unreachable pose', -2: 'self collision', -3: 'unknown endpoint'}
            raise ValueError(f'IK rejected position {pose.position}: {reasons.get(code, code)}')
        if len(response.joints) != 1:
            raise ValueError('IK returned a malformed joint solution')
        return ordered_values(response.joints[0].name, response.joints[0].position, 'IK solution')

    def forward(self, configurations):
        poses = []
        for offset in range(0, len(configurations), 32):
            batch = configurations[offset:offset + 32]
            request = SolvePositionFK.Request()
            request.configuration = [JointState(name=JOINT_NAMES, position=list(values)) for values in batch]
            request.tip_names = [self.tip] * len(batch)
            response = self.call(self.fk, request, 'FK')
            if any(len(field) != len(batch) for field in
                   (response.pose_stamp, response.is_valid, response.in_collision)):
                raise ValueError('FK returned a malformed response')
            for stamped, valid, collision in zip(response.pose_stamp, response.is_valid, response.in_collision):
                if not valid or collision:
                    raise ValueError(f'FK rejected path configuration: valid={valid}, in_collision={collision}')
                pose = from_pose(stamped.pose, stamped.header.frame_id or self.frame)
                if pose.frame != self.frame:
                    raise ValueError(f'FK frame {pose.frame} does not match requested frame {self.frame}')
                poses.append(pose)
        return poses
