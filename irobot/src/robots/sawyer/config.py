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


def vector(values, size, name, *, positive=False, nonnegative=False):
    items = (
        (values,) * size if isinstance(values, int | float) else tuple(values)
    )
    if any(isinstance(value, bool) for value in items):
        msg = f'{name} requires numeric values rather than booleans'
        raise ValueError(msg)
    result = tuple(float(value) for value in items)
    if len(result) != size or not all(
        math.isfinite(value) for value in result
    ):
        msg = f'{name} must contain {size} finite values'
        raise ValueError(msg)
    if positive and any(value <= 0 for value in result):
        msg = f'{name} must be positive'
        raise ValueError(msg)
    if nonnegative and any(value < 0 for value in result):
        msg = f'{name} must be nonnegative'
        raise ValueError(msg)
    return result


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
        if (
            type(self.max_samples) is not int
            or self.max_samples < MIN_IK_SAMPLES
        ):
            msg = 'max_samples must be an integer >= 11'
            raise ValueError(msg)
        if not self.frame or not self.frame.lstrip('/'):
            msg = 'A reference frame is required'
            raise ValueError(msg)


@dataclass(frozen=True)
class StreamOptions:
    """Application envelopes; these do not replace controller safeguards."""

    rate: float = 100.0
    max_velocity: float = 0.1
    max_acceleration: float = 0.5
    max_displacement: float = 0.1
    effort_fraction: float = 0.05
    position_speed: float = 0.05

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            positive(name, getattr(self, name))
        if not MIN_CONTROL_RATE <= self.rate <= MAX_CONTROL_RATE:
            msg = 'rate must be between 50 and 250 Hz'
            raise ValueError(msg)
        if self.effort_fraction > 1 or self.position_speed > 1:
            msg = 'Effort fraction and position speed must be at most one'
            raise ValueError(msg)


@dataclass(frozen=True)
class SawyerConfig:
    """
    Direct right-arm connection settings. Construction sends no commands.

    ros_ip must advertise an address reachable by Sawyer. ROS 2 is optional.
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
    domain_id: int | None = 0
    ros_master_uri: str | None = 'http://169.254.121.3:11311'
    ros_ip: str | None = '169.254.121.2'
    transport_timeout: float = 0.2
    transport_retry_interval: float = 0.2
    transport_idle_timeout: float = 5.0
    registration_refresh_interval: float = 2.0
    neutral_joints: tuple[float, ...] | None = None
    neutral_parameter: str = '/named_poses/right'
    parameter_timeout: float = 5.0
    enable_gripper: bool = True
    allow_unvalidated_modes: bool = False
    gripper_open_position: float = 0.041667
    gripper_timeout: float = 10.0
    joint_state_topic: str = '/robot/joint_states'
    robot_state_topic: str = '/robot/state'
    homing_state_topic: str = '/robot/homing_states'
    enable_topic: str = '/robot/set_super_enable'
    reset_topic: str = '/robot/set_super_reset'
    software_stop_topic: str = '/robot/set_super_stop'
    version_parameter: str = (
        '/manifest/robot_software/version/HLR_VERSION_STRING'
    )
    robot_description_parameter: str = '/robot_description'
    head_state_topic: str = '/robot/head/head_state'
    head_command_topic: str = '/robot/head/command_head_pan'
    head_display_topic: str = '/robot/head_display'
    camera_namespace: str = '/io/internal_camera'
    io_namespace: str = '/io/robot'
    device_timeout: float = 5.0
    enable_max_displacement: float = 0.02
    head_position_tolerance: float = 0.01
    collision_topic: str = '/robot/limb/right/collision_detection_state'
    joint_limits_topic: str = '/robot/joint_limits'
    joint_command_topic: str = '/robot/limb/right/joint_command'
    command_timeout_topic: str = '/robot/limb/right/joint_command_timeout'
    speed_ratio_topic: str = '/robot/limb/right/set_speed_ratio'
    interaction_command_topic: str = (
        '/robot/limb/right/interaction_control_command'
    )
    interaction_state_topic: str = (
        '/robot/limb/right/interaction_controller/state'
    )
    diagnostics_topic: str = '/diagnostics'
    tf_topic: str = '/tf'
    tf_static_topic: str = '/tf_static'
    native_motion_action: str = '/motion/motion_command'
    calibration_action: str = '/calibration_command'
    action_server_timeout: float = 2.0
    endpoint_topic: str = '/robot/limb/right/endpoint_state'
    tip_states_topic: str = '/robot/limb/right/tip_states'
    ik_service: str = '/ExternalTools/right/PositionKinematicsNode/IKService'
    fk_service: str = '/ExternalTools/right/PositionKinematicsNode/FKService'
    gripper_namespace: str = '/io/end_effector/right_gripper'
    motion: CartesianMotionOptions = field(
        default_factory=CartesianMotionOptions
    )

    def __post_init__(self):
        for name in (
            'transport_timeout',
            'device_timeout',
            'enable_max_displacement',
            'head_position_tolerance',
            'transport_retry_interval',
            'transport_idle_timeout',
            'registration_refresh_interval',
            'action_server_timeout',
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
        if (
            self.feedback_timeout > MAX_WATCHDOG
            or self.command_timeout > MAX_WATCHDOG
        ):
            msg = 'Feedback and command timeouts must be <= 0.2 seconds'
            raise ValueError(msg)
        if type(self.enable_gripper) is not bool:
            msg = 'enable_gripper must be a boolean'
            raise ValueError(msg)
        if type(self.allow_unvalidated_modes) is not bool:
            msg = 'allow_unvalidated_modes must be a boolean'
            raise ValueError(msg)
        if not 0 <= self.gripper_open_position <= MAX_GRIPPER_OPEN_POSITION:
            msg = 'gripper_open_position must be between 0 and 0.041667 meters'
            raise ValueError(msg)
        if not self.frame or not self.frame.lstrip('/'):
            msg = 'A reference frame is required'
            raise ValueError(msg)
        self._validate_connection()
        if self.neutral_joints is not None and (
            len(self.neutral_joints) != len(JOINT_NAMES)
            or not all(math.isfinite(q) for q in self.neutral_joints)
        ):
            msg = 'neutral_joints must contain seven finite angles in radians'
            raise ValueError(msg)
        if not isinstance(self.motion, CartesianMotionOptions):
            msg = 'motion must be CartesianMotionOptions'
            raise TypeError(msg)

    def _validate_connection(self):
        if self.domain_id is not None and (
            type(self.domain_id) is not int
            or not 0 <= self.domain_id <= MAX_ROS_DOMAIN
        ):
            msg = 'domain_id must be an integer between 0 and 232'
            raise ValueError(msg)
        if self.ros_master_uri is not None:
            parsed = urlparse(self.ros_master_uri)
            if parsed.scheme != 'http' or not parsed.hostname:
                msg = 'ros_master_uri must be an http:// ROS 1 master URL'
                raise ValueError(msg)


@dataclass(frozen=True)
class InteractionLimits:
    """Application bounds, not manufacturer-certified Cartesian limits."""

    stiffness: tuple = (1300.0, 1300.0, 1300.0, 30.0, 30.0, 30.0)
    damping: tuple = (100.0,) * 6
    nullspace_stiffness: tuple = (10.0,) * 7
    wrench: tuple = (5.0, 5.0, 5.0, 1.0, 1.0, 1.0)
    translation: float = 0.01
    rotation: float = 0.1

    def __post_init__(self):
        for name, length in (
            ('stiffness', 6),
            ('damping', 6),
            ('nullspace_stiffness', 7),
            ('wrench', 6),
        ):
            object.__setattr__(
                self,
                name,
                vector(getattr(self, name), length, name, positive=True),
            )
        positive('translation', self.translation)
        positive('rotation', self.rotation)
