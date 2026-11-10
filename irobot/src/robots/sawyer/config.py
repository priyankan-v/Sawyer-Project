"""Configuration and options shared by the Sawyer driver; no ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from urllib.parse import urlparse

JOINT_NAMES = tuple(f'right_j{i}' for i in range(7))
MIN_CONTROL_RATE = 50
MAX_CONTROL_RATE = 250
MIN_IK_SAMPLES = 11
MAX_WATCHDOG = 0.2
MAX_GRIPPER_OPEN_POSITION = 0.041667
MAX_ROS_DOMAIN = 232
QUATERNION_MIN_NORM = 1e-12
QUATERNION_NORM_TOLERANCE = 0.05
ANGULAR_SAMPLE_STEP = math.pi / 36


def positive(name, value):
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        msg = f'{name} must be positive and finite'
        raise ValueError(msg)


@dataclass(frozen=True)
class MotionOptions:
    execute: bool = False
    time: float | None = None
    speed: float = 1.0
    max_accel: float | None = None
    rate: float = 100.0
    kp: float = 2.0
    ki: float = 0.1
    kd: float = 0.05
    settle_time: float = 5.0
    timeout: float | None = None
    max_error: float = 0.35

    def __post_init__(self):
        if type(self.execute) is not bool:
            msg = 'execute must be a boolean'
            raise ValueError(msg)
        for name in ('speed', 'rate', 'settle_time', 'max_error'):
            positive(name, getattr(self, name))
        for name in ('kp', 'ki', 'kd'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                msg = f'{name} must be nonnegative and finite'
                raise ValueError(msg)
        for name in ('time', 'max_accel', 'timeout'):
            value = getattr(self, name)
            if value is not None:
                positive(name, value)
        if self.speed > 1:
            msg = 'speed must be in (0, 1]'
            raise ValueError(msg)
        if not MIN_CONTROL_RATE <= self.rate <= MAX_CONTROL_RATE:
            msg = 'rate must be between 50 and 250 Hz'
            raise ValueError(msg)


@dataclass(frozen=True)
class CartesianMotionOptions(MotionOptions):
    tip: str | None = None
    frame: str = 'base'
    angular_step: float = ANGULAR_SAMPLE_STEP
    sample_step: float = 0.005
    path_tolerance: float = 0.001
    path_orientation_tolerance: float = 0.01
    max_joint_step: float = 0.15
    max_samples: int = 401
    cartesian_tolerance: float = 0.003
    goal_orientation_tolerance: float = 0.02
    max_cartesian_error: float = 0.02
    max_orientation_error: float = 0.1
    ik_timeout: float = 2.0
    planning_timeout: float = 60.0

    def __post_init__(self):
        super().__post_init__()
        for name in (
            'angular_step',
            'sample_step',
            'path_tolerance',
            'path_orientation_tolerance',
            'max_joint_step',
            'cartesian_tolerance',
            'goal_orientation_tolerance',
            'max_cartesian_error',
            'max_orientation_error',
            'ik_timeout',
            'planning_timeout',
        ):
            positive(name, getattr(self, name))
        if type(self.max_samples) is not int or self.max_samples < MIN_IK_SAMPLES:
            msg = 'max_samples must be an integer >= 11'
            raise ValueError(msg)
        if not self.frame or not self.frame.lstrip('/'):
            msg = 'A reference frame is required'
            raise ValueError(msg)


@dataclass(frozen=True)
class SawyerConfig:
    """
    Right-arm bridge client configuration. Construction sends no commands.

    ROS_MASTER_URI is needed only for neutral-pose lookup unless neutral_joints
    is provided. ROS_IP is a bridge-process setting, not a DDS client setting.
    """

    node_name: str = 'irobot_sawyer'
    connect_timeout: float = 10.0
    callback_timeout: float = 0.02
    callback_slice: float = 0.01
    input_poll_timeout: float = 0.05
    planning_refresh_timeout: float = 0.5
    planning_joint_tolerance: float = 0.01
    planning_position_tolerance: float = 0.002
    planning_orientation_tolerance: float = 0.01
    tip_match_position_tolerance: float = 0.0001
    tip_match_orientation_tolerance: float = 0.001
    watchdog_refresh_interval: float = 1.0
    stop_zero_samples: int = 5
    stop_hold_samples: int = 3
    stop_publish_interval: float = 0.02
    gripper_ack_retry_interval: float = 0.1
    gripper_position_tolerance: float = 0.002
    gripper_settle_confirmation: float = 0.2
    feedback_timeout: float = 0.2
    command_timeout: float = 0.2
    position_tolerance: float = 0.02
    velocity_tolerance: float = 0.03
    settle_confirmation: float = 0.2
    frame: str = 'base'
    tip: str | None = None
    domain_id: int | None = None
    ros_master_uri: str | None = None
    neutral_joints: tuple[float, ...] | None = None
    neutral_parameter: str = '/named_poses/right'
    parameter_timeout: float = 5.0
    enable_gripper: bool = True
    gripper_open_position: float = 0.041667
    gripper_timeout: float = 10.0
    joint_state_topic: str = '/robot/joint_states'
    robot_state_topic: str = '/robot/state'
    collision_topic: str = '/robot/limb/right/collision_detection_state'
    joint_limits_topic: str = '/robot/joint_limits'
    joint_command_topic: str = '/robot/limb/right/joint_command'
    command_timeout_topic: str = '/robot/limb/right/joint_command_timeout'
    endpoint_topic: str = '/robot/limb/right/endpoint_state'
    tip_states_topic: str = '/robot/limb/right/tip_states'
    ik_service: str = '/ExternalTools/right/PositionKinematicsNode/IKService'
    fk_service: str = '/ExternalTools/right/PositionKinematicsNode/FKService'
    gripper_namespace: str = '/io/end_effector/right_gripper'
    motion: CartesianMotionOptions = field(default_factory=CartesianMotionOptions)

    def __post_init__(self):
        for name in (
            'connect_timeout',
            'feedback_timeout',
            'command_timeout',
            'position_tolerance',
            'velocity_tolerance',
            'settle_confirmation',
            'parameter_timeout',
            'gripper_timeout',
            'callback_timeout',
            'callback_slice',
            'input_poll_timeout',
            'planning_refresh_timeout',
            'planning_joint_tolerance',
            'planning_position_tolerance',
            'planning_orientation_tolerance',
            'tip_match_position_tolerance',
            'tip_match_orientation_tolerance',
            'watchdog_refresh_interval',
            'stop_publish_interval',
            'gripper_ack_retry_interval',
            'gripper_position_tolerance',
            'gripper_settle_confirmation',
        ):
            positive(name, getattr(self, name))
        for name in ('stop_zero_samples', 'stop_hold_samples'):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                msg = f'{name} must be a positive integer'
                raise ValueError(msg)
        if self.feedback_timeout > MAX_WATCHDOG or self.command_timeout > MAX_WATCHDOG:
            msg = 'Feedback and command timeouts must be <= 0.2 seconds'
            raise ValueError(msg)
        if type(self.enable_gripper) is not bool:
            msg = 'enable_gripper must be a boolean'
            raise ValueError(msg)
        if not 0 <= self.gripper_open_position <= MAX_GRIPPER_OPEN_POSITION:
            msg = 'gripper_open_position must be between 0 and 0.041667 meters'
            raise ValueError(msg)
        if not self.frame or not self.frame.lstrip('/'):
            msg = 'A reference frame is required'
            raise ValueError(msg)
        if self.domain_id is not None and (
            type(self.domain_id) is not int or not 0 <= self.domain_id <= MAX_ROS_DOMAIN
        ):
            msg = 'domain_id must be an integer between 0 and 232'
            raise ValueError(msg)
        if self.ros_master_uri is not None:
            parsed = urlparse(self.ros_master_uri)
            if parsed.scheme != 'http' or not parsed.hostname:
                msg = 'ros_master_uri must be an http:// ROS 1 master URL'
                raise ValueError(msg)
        if self.neutral_joints is not None and (
            len(self.neutral_joints) != len(JOINT_NAMES) or not all(math.isfinite(q) for q in self.neutral_joints)
        ):
            msg = 'neutral_joints must contain seven finite angles in radians'
            raise ValueError(msg)
        if not isinstance(self.motion, CartesianMotionOptions):
            msg = 'motion must be CartesianMotionOptions'
            raise TypeError(msg)

    def bridge_configuration(self) -> dict:
        """Generate bridge routes from the same hardware settings as the client."""

        def telemetry(topic, message, *, latched=False, depth=1):
            qos = {'history': 'keep_last', 'depth': depth, 'reliability': 'reliable' if latched else 'best_effort'}
            if latched:
                qos['durability'] = 'transient_local'
            return {
                'topic': topic,
                'type': message,
                'queue_size': depth,
                'direction': '1to2',
                'enabled': True,
                'qos': qos,
            }

        topics = [
            telemetry(self.joint_state_topic, 'sensor_msgs/msg/JointState', depth=5),
            telemetry(self.robot_state_topic, 'intera_core_msgs/msg/RobotAssemblyState'),
            telemetry(self.collision_topic, 'intera_core_msgs/msg/CollisionDetectionState'),
            telemetry(self.joint_limits_topic, 'intera_core_msgs/msg/JointLimits', latched=True),
            telemetry(self.endpoint_topic, 'intera_core_msgs/msg/EndpointState', depth=5),
            telemetry(self.tip_states_topic, 'intera_core_msgs/msg/EndpointStates'),
        ]
        seconds = int(self.command_timeout)
        lifespan = {'secs': seconds, 'nsecs': round((self.command_timeout - seconds) * 1e9)}
        for topic, message in (
            (self.joint_command_topic, 'intera_core_msgs/msg/JointCommand'),
            (self.command_timeout_topic, 'std_msgs/msg/Float64'),
        ):
            qos = {'history': 'keep_last', 'depth': 1, 'reliability': 'reliable'}
            if topic == self.joint_command_topic:
                qos['lifespan'] = lifespan
            topics.append(
                {'topic': topic, 'type': message, 'queue_size': 1, 'direction': '2to1', 'enabled': False, 'qos': qos}
            )
        if self.enable_gripper:
            topics.append(
                telemetry(self.gripper_namespace + '/state', 'intera_core_msgs/msg/IODeviceStatus', latched=True)
            )
            topics.append(
                {
                    'topic': self.gripper_namespace + '/command',
                    'type': 'intera_core_msgs/msg/IOComponentCommand',
                    'queue_size': 1,
                    'direction': '2to1',
                    'enabled': True,
                    'qos': {'history': 'keep_last', 'depth': 1, 'reliability': 'reliable'},
                }
            )
        services = [
            {'service': name, 'type': message, 'package': ''}
            for name, message in (
                (self.ik_service, 'intera_core_msgs/SolvePositionIK'),
                (self.fk_service, 'intera_core_msgs/SolvePositionFK'),
            )
        ]
        return {'topics': topics, 'services_1_to_2': [], 'services_2_to_1': services, 'actions': []}
