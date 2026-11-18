"""Account for every pinned public SDK method without importing ROS."""

# Standalone development tool; coverage data is intentionally explicit.
# ruff: noqa: INP001
from __future__ import annotations

import argparse
import json
from pathlib import Path

DEFAULT = {
    'availability': 'available',
    'software': 'implemented',
    'hardware': 'not_recorded',
    'python_api': [],
    'ros2_endpoints': [],
    'controller_interface': [],
    'units': 'as declared by the schema',
    'limits': 'published controller metadata; finite values',
    'side_effects': 'read/preview is inert; writes need explicit execution',
    'cancellation': 'not applicable to reads',
    'schema_source': 'installation/wire_schema.json and SDK_INVENTORY.json',
    'evidence': [],
    'notes': '',
}


def feature(**fields):
    unknown = fields.keys() - DEFAULT.keys()
    if unknown:
        raise ValueError(str(unknown))
    return DEFAULT | fields


FEATURES = {
    'identity': feature(
        python_api=['devices.inventory', 'get_parameter', 'check_version'],
        ros2_endpoints=['inventory (std_srvs/Trigger)', 'read_parameter'],
        controller_interface=['ROS master parameters/graph'],
        units='strings/configuration JSON',
        hardware='not_recorded',
        evidence=[
            'tests/test_irobot/test_sawyer_transport.py',
        ],
    ),
    'robot_state': feature(
        python_api=['get_status', 'devices.robot_state'],
        ros2_endpoints=['status', 'robot_state'],
        controller_interface=[
            '/robot/state: intera_core_msgs/RobotAssemblyState'
        ],
        units='booleans and vendor stop-source enums',
        hardware='not_recorded',
    ),
    'enable': feature(
        python_api=['devices.set_enabled'],
        ros2_endpoints=['set_enabled'],
        controller_interface=['/robot/set_super_enable: std_msgs/Bool'],
        units='boolean',
        hardware='not_recorded',
        limits=(
            'ready confirmation; fresh positions; 0.02 rad enable envelope '
            'by default'
        ),
        cancellation='enable failure requests a latched software stop',
        notes=(
            'Enabling requires fresh feedback and preserves the '
            'displacement guard.'
        ),
        evidence=[
            'tests/test_irobot/test_sawyer_peripherals.py',
        ],
    ),
    'stop_reset': feature(
        python_api=['devices.software_stop', 'devices.reset_faults'],
        ros2_endpoints=['software_stop', 'reset_faults'],
        controller_interface=[
            '/robot/set_super_stop',
            '/robot/set_super_reset',
        ],
        units='explicit Empty commands',
        hardware='not_recorded',
        limits='reset requires stopped state and released physical E-stop',
        notes=(
            'Stop and reset are explicit operations; reset is never '
            'automatic.'
        ),
        evidence=[
            'tests/test_irobot/test_sawyer_peripherals.py',
            'tests/test_irobot/test_sawyer_ros2.py',
        ],
    ),
    'joints': feature(
        python_api=['joint_positions', 'joint_velocities', 'joint_efforts'],
        ros2_endpoints=['joint_states'],
        controller_interface=['/robot/joint_states: sensor_msgs/JointState'],
        units='radians, rad/s, Nm',
        hardware='not_recorded',
        limits=(
            'seven canonical arm joints; per-field monotonic receipt '
            'freshness'
        ),
    ),
    'limits': feature(
        python_api=[
            'joint_limits',
            'joint_effort_limits',
            'robot_description',
        ],
        ros2_endpoints=['joint_limits', 'read_parameter'],
        controller_interface=['/robot/joint_limits', '/robot_description'],
        units='rad, rad/s, rad/s^2, Nm',
        hardware='not_recorded',
        limits='torque uses the minimum of published and URDF effort bounds',
    ),
    'endpoint': feature(
        python_api=[
            'endpoint_pose',
            'endpoint_velocity',
            'endpoint_effort',
            'get_tip_state',
            'tip_names',
        ],
        ros2_endpoints=[
            'endpoint_pose',
            'endpoint_twist',
            'endpoint_wrench',
            'endpoint_states',
        ],
        controller_interface=[
            '/robot/limb/right/tip_states',
            '/robot/limb/right/endpoint_state',
        ],
        units=(
            'm, quaternion xyzw, m/s, rad/s, N, Nm; declared reference '
            'frame'
        ),
        hardware='not_recorded',
        limits=(
            'valid named tip and current data; no invented tool-frame '
            'fallback'
        ),
    ),
    'joint_motion': feature(
        python_api=['move_to_neutral', 'move_to_joints', 'move_trajectory'],
        ros2_endpoints=['move_to_neutral', 'follow_joint_trajectory'],
        controller_interface=['/robot/limb/right/joint_command'],
        units='rad and seconds',
        hardware='not_recorded',
        limits=(
            'published position/velocity/acceleration bounds, fresh '
            'readiness/collision feedback, tracking error and loop deadline'
        ),
        cancellation=(
            'cancel event or stop(); zero velocity then fresh '
            'measured-position hold while enabled'
        ),
        notes=(
            'Neutral velocity ceilings and acceleration bounds remain '
            'configurable.'
        ),
        evidence=[
            'tests/test_irobot/test_sawyer_control.py',
            'tests/test_irobot/test_sawyer_neutral_example.py',
            'tests/test_irobot/test_sawyer_transport.py',
        ],
    ),
    'joint_stream': feature(
        python_api=[
            'stream_joint_commands (position/velocity)',
            'StreamOptions',
        ],
        ros2_endpoints=['joint_stream', 'stream_command'],
        controller_interface=[
            'JointCommand POSITION_MODE/VELOCITY_MODE',
            'joint_command_timeout',
        ],
        units='rad, rad/s, seconds',
        hardware='not_recorded',
        limits=(
            'finite session, displacement/speed bounds, measured feedback, '
            'command gap < half watchdog'
        ),
        cancellation=(
            'session cancellation, ROS 2 source-age/sequence lease, '
            'robot-side watchdog'
        ),
    ),
    'torque': feature(
        python_api=['stream_joint_commands (torque)'],
        ros2_endpoints=['joint_stream (TORQUE)'],
        controller_interface=['JointCommand TORQUE_MODE'],
        units='Nm',
        software='experimental',
        hardware='not_recorded',
        limits=(
            'published/URDF effort caps, effort fraction, measured '
            'displacement and speed; allow_unvalidated_modes required'
        ),
        cancellation=(
            'direct measured-position exit; no command after '
            'stopped/disabled state'
        ),
        notes=('Experimental execution requires explicit opt-in.'),
    ),
    'advanced_trajectory': feature(
        python_api=['stream_joint_commands (trajectory)'],
        ros2_endpoints=['joint_stream (TRAJECTORY)'],
        controller_interface=['JointCommand TRAJECTORY_MODE'],
        units='rad, rad/s, rad/s^2',
        software='experimental',
        hardware='not_recorded',
        limits=(
            'reference continuity, published limits, measured envelope; '
            'allow_unvalidated_modes required'
        ),
        notes=('Experimental execution requires explicit opt-in.'),
        cancellation='direct measured-position hold',
    ),
    'kinematics': feature(
        python_api=[
            'forward_kinematics',
            'inverse_kinematics',
            'configuration_collisions',
            'move_to_pose',
            'move_relative',
        ],
        ros2_endpoints=[
            'forward_kinematics',
            'inverse_kinematics',
            'check_joint_collision',
            'move_cartesian',
        ],
        controller_interface=['SolvePositionFK', 'SolvePositionIK'],
        units='m, quaternion xyzw, rad; move_relative offsets are cm',
        hardware='not_recorded',
        limits=(
            'known frame, joint bounds, seeded IK, collision validity; '
            'Cartesian movement requires ready feedback'
        ),
        cancellation='motion stop/cancel; bounded model-service calls',
        notes=('Model queries do not execute a Cartesian movement.'),
    ),
    'native_motion': feature(
        python_api=['move_native_joint_path', 'execute_native_trajectory'],
        ros2_endpoints=['native_motion'],
        controller_interface=[
            '/motion/motion_command: intera_motion_msgs/MotionCommandAction'
        ],
        units='typed native trajectory; rad, m, seconds',
        hardware='not_recorded',
        limits=(
            'joint order/bounds, first-waypoint continuity, frame/numeric '
            'validation; interaction requires experimental opt-in'
        ),
        cancellation=(
            'cancel this goal ID only; heartbeat/result timeout; no goal '
            'replay'
        ),
        notes=(
            'Planning does not silently initialize control by moving the '
            'arm.'
        ),
    ),
    'interaction': feature(
        python_api=[
            'run_interaction',
            'interaction_command',
            'InteractionLimits',
        ],
        ros2_endpoints=['interaction'],
        controller_interface=[
            'InteractionControlCommand',
            'InteractionControlState',
        ],
        units='N/m, Nm/rad, Ns/m, Nms/rad, N, Nm',
        software='experimental',
        hardware='not_recorded',
        limits=(
            'application stiffness/damping/wrench and displacement '
            'envelopes; compliant/force variants require '
            'allow_unvalidated_modes'
        ),
        cancellation=(
            'confirmed inactive state then position hold while enabled; '
            'process-loss deactivation not proven'
        ),
        notes=('Experimental execution requires explicit opt-in.'),
    ),
    'gripper_read': feature(
        python_api=['gripper_state', 'devices.read_io'],
        ros2_endpoints=['gripper/state', 'gripper/config'],
        controller_interface=['/io/end_effector/right_gripper/state'],
        units='m, m/s, kg, N; negative force_response_n means unavailable',
        hardware='not_recorded',
        limits='change-only state plus socket/receipt metadata',
    ),
    'gripper_motion': feature(
        python_api=[
            'move_gripper',
            'open_gripper',
            'close_gripper',
            'stop_gripper',
            'calibrate_gripper',
        ],
        ros2_endpoints=[
            'gripper_command',
            'open_gripper',
            'close_gripper',
            'stop_gripper',
            'calibrate_gripper',
        ],
        controller_interface=[
            '/io/end_effector/right_gripper/command: IOComponentCommand'
        ],
        units='finger separation m',
        hardware='not_recorded',
        limits=(
            '0..configured opening; explicit calibration; valid/idle '
            'device; command acknowledgement plus settling/grasp state'
        ),
        cancellation='cancel event requests go=false; no automatic opening',
    ),
    'gripper_settings': feature(
        python_api=['devices.configure_gripper'],
        ros2_endpoints=['gripper/set_parameters'],
        controller_interface=[
            'speed_mps',
            'dead_zone_m',
            'right_gripper_tip_object_kg',
        ],
        units='m/s, m, kg',
        hardware='not_recorded',
        limits='published format/ranges and idle/error-free device',
        notes=('Acknowledgement and measured settings are separate results.'),
    ),
    'gripper_recovery': feature(
        python_api=['start_gripper', 'reboot_gripper'],
        ros2_endpoints=['start_gripper', 'reboot_gripper'],
        controller_interface=['IO go/position_m or reboot'],
        hardware='not_recorded',
        limits=(
            'idle device; start uses explicit reported target; reboot '
            'removes calibration'
        ),
        cancellation='cancel requests go=false; calibration never automatic',
        notes=('Reboot and recalibration are separate explicit operations.'),
    ),
    'fixed_force': feature(
        availability='deprecated_fixed',
        software='not_applicable',
        hardware='not_applicable',
        python_api=['gripper_state (force feedback only)'],
        ros2_endpoints=['gripper_command max_effort must be zero'],
        controller_interface=['SDK set_holding_force is deprecated/fixed'],
        units='N',
        notes=(
            'The SDK does not provide adjustable holding force on this '
            'gripper. Unavailable effort is NaN in GripperCommand results.'
        ),
    ),
    'io_read': feature(
        python_api=['devices.read_io', 'devices.io_state', 'poll'],
        ros2_endpoints=['device state/config topics', 'io_health'],
        controller_interface=['IODeviceStatus', 'IODeviceConfiguration'],
        hardware='not_recorded',
        units='published signal/port format and dimensions',
        limits=(
            'connection state and update age are distinct from sensor '
            'freshness'
        ),
    ),
    'io_write': feature(
        python_api=['devices.write_io'],
        ros2_endpoints=['device set_parameters'],
        controller_interface=['IOComponentCommand'],
        hardware='not_recorded',
        units='published format/units',
        limits=(
            'type, dimensions, ranges, role and matching command '
            'timestamp; motion signals use explicit gripper APIs'
        ),
        cancellation='bounded acknowledgement and cancel event',
        notes=('Acknowledgement and measured values are reported separately.'),
        evidence=[
            'tests/test_irobot/test_sawyer_extensions.py',
        ],
    ),
    'io_callbacks': feature(
        python_api=[
            'devices.register_io_callback',
            'devices.deregister_io_callback',
            'devices.io_events',
            'poll',
        ],
        ros2_endpoints=['io_events', 'device state topics'],
        controller_interface=['cuff/navigator IODeviceStatus'],
        hardware='not_recorded',
        limits=(
            'bounded edge queues with drop counters; callbacks must be '
            'nonblocking'
        ),
        notes=('Callbacks run while polling; event queues are bounded.'),
    ),
    'digital': feature(
        python_api=['devices.digital_state', 'devices.set_digital_output'],
        ros2_endpoints=[
            'digital_io/right_hand_camera_power',
            'digital_outputs/set_parameters',
        ],
        controller_interface=[
            'DigitalIOState',
            '/robot/digital_io/command: DigitalOutputCommand',
        ],
        units='boolean state; is_input_only is enforced',
        hardware='not_recorded',
        notes=(
            'Writes require published device permissions and matching '
            'acknowledgements.'
        ),
    ),
    'lights': feature(
        python_api=['devices.set_light', 'devices.read_io'],
        ros2_endpoints=['robot_io/set_parameters', 'robot_io/state'],
        controller_interface=['/io/robot/robot: six SDK light signals'],
        units='boolean',
        hardware='not_recorded',
        limits=(
            'only the six SDK-defined lights override IO role=input; '
            'DigitalIO input-only permissions remain enforced'
        ),
        notes=(
            'The public SDK documents these writes despite direction '
            'metadata.'
        ),
    ),
    'camera_frames': feature(
        python_api=[
            'devices.camera_frame',
            'devices.register_camera_callback',
            'devices.deregister_camera_callback',
        ],
        ros2_endpoints=[
            'head/image',
            'hand/image',
            'head/camera_info',
            'hand/camera_info',
        ],
        controller_interface=['sensor_msgs/Image', 'sensor_msgs/CameraInfo'],
        units='native pixels/encoding and calibrated camera model',
        hardware='not_recorded',
        limits=(
            '16 MiB wire payload; advancing frame identities; raw CDR path '
            'and Fast DDS camera profile'
        ),
        notes=('Reading a camera never implicitly starts its stream.'),
    ),
    'camera_settings': feature(
        python_api=[
            'devices.camera_state',
            'devices.configure_camera',
            'devices.set_camera_streaming',
        ],
        ros2_endpoints=[
            'head/streaming',
            'hand/streaming',
            'camera set_parameters',
        ],
        controller_interface=['camera IOComponentCommand/config/state'],
        units=(
            'published exposure/gain/offset units; boolean '
            'flip/mirror/streaming'
        ),
        hardware='not_recorded',
        limits=(
            'only discovered configured writable signals; metadata '
            'type/range validation'
        ),
        notes=('Settings require published metadata and explicit execution.'),
    ),
    'display': feature(
        python_api=['devices.display_image'],
        ros2_endpoints=['display_image'],
        controller_interface=['/robot/head_display: sensor_msgs/Image'],
        units='rgb8/bgr8 image',
        hardware='not_recorded',
        notes=('Image publication is an explicit operation.'),
    ),
    'head_pan': feature(
        python_api=['devices.head_state', 'devices.move_head'],
        controller_interface=['HeadState', 'HeadPanCommand'],
        units='rad',
        hardware='not_recorded',
        software='implemented_not_qualified',
        notes=('Head-pan testing is deferred by user request.'),
    ),
    'clicksmart': feature(
        availability='discovery_only',
        software='not_applicable',
        hardware='not_applicable',
        python_api=['devices.inventory (discovery only)'],
        ros2_endpoints=['inventory'],
        controller_interface=['no ClickSmart tool attached'],
        notes=(
            'Discovery is available; activation and persistent '
            'configuration are not implemented.'
        ),
    ),
    'records': feature(
        availability='data_model',
        hardware='not_applicable',
        python_api=[
            'core.wire typed dataclasses',
            'wire.from_dict',
            'dataclasses.asdict',
        ],
        ros2_endpoints=['typed native_motion/interaction action fields'],
        controller_interface=[
            'public message fields through validated execution methods'
        ],
        notes=(
            'SDK setter/getter/builder helpers are represented by typed '
            'fields; constructing or editing data never executes it.'
        ),
        evidence=[
            'tests/test_irobot/test_sawyer_wire.py',
            'tests/test_irobot/test_sawyer_extended_ros2.py',
        ],
    ),
    'application': feature(
        availability='application_helper',
        software='outside_hardware_driver',
        hardware='not_applicable',
        python_api=[
            (
                'Python JSON/CSV/file/logging utilities; '
                'application-generated joint targets'
            )
        ],
        notes=(
            'File export, presets and RandomWalk are application '
            'utilities, not additional controller access. They are '
            'accounted for without duplicating the SDK helper library.'
        ),
    ),
    'cache': feature(
        availability='sdk_internal_helper',
        hardware='not_applicable',
        python_api=['devices.read_io', 'poll', 'core.io.IODevice lifecycle'],
        notes=(
            'SDK cache/command-builder helpers are replaced by automatic '
            'direct subscription caching and typed acknowledged writes; '
            'they are not separate hardware actions.'
        ),
        evidence=[
            'tests/test_irobot/test_sawyer_transport.py',
            'tests/test_irobot/test_sawyer_extensions.py',
        ],
    ),
}

CLASS_FEATURE = {
    'Cameras': 'camera_settings',
    'SimpleClickSmartGripper': 'clicksmart',
    'Cuff': 'io_read',
    'DigitalIO': 'digital',
    'Gripper': 'gripper_read',
    'GripperFactory': 'identity',
    'Head': 'head_pan',
    'HeadDisplay': 'display',
    'JointLimits': 'limits',
    'Lights': 'lights',
    'Limb': 'joints',
    'Navigator': 'io_read',
    'RobotEnable': 'robot_state',
    'RobotParams': 'identity',
    'IOCommand': 'cache',
    'SetCommand': 'cache',
    'IOInterface': 'cache',
    'IODeviceInterface': 'io_read',
    'InteractionOptions': 'records',
    'InteractionPublisher': 'interaction',
    'MotionControllerActionClient': 'native_motion',
    'MotionTrajectory': 'records',
    'MotionWaypoint': 'records',
    'MotionWaypointOptions': 'records',
    'RandomWalk': 'application',
}
OVERRIDES = {
    'camera_frames': ['Cameras.set_callback'],
    'identity': [
        'Cameras.list_cameras',
        'Cameras.verify_camera_exists',
        'Limb.joint_names',
        'RobotEnable.version_check',
    ],
    'io_callbacks': [
        'Cuff.register_callback',
        'Cuff.deregister_callback',
        'Navigator.register_callback',
        'Navigator.deregister_callback',
        'IODeviceInterface.register_callback',
        'IODeviceInterface.deregister_callback',
    ],
    'io_write': [
        'IODeviceInterface.set_signal_value',
        'IODeviceInterface.set_port_value',
    ],
    'gripper_motion': [
        'Gripper.stop',
        'Gripper.open',
        'Gripper.close',
        'Gripper.calibrate',
        'Gripper.set_position',
    ],
    'gripper_recovery': ['Gripper.reboot', 'Gripper.start'],
    'gripper_settings': [
        'Gripper.set_velocity',
        'Gripper.set_cmd_velocity',
        'Gripper.set_object_weight',
        'Gripper.set_dead_zone',
    ],
    'fixed_force': ['Gripper.set_holding_force'],
    'endpoint': [
        'Limb.endpoint_pose',
        'Limb.endpoint_velocity',
        'Limb.endpoint_effort',
        'Limb.tip_state',
    ],
    'robot_state': ['Limb.has_collided'],
    'joint_motion': ['Limb.move_to_neutral', 'Limb.move_to_joint_positions'],
    'joint_stream': [
        'Limb.set_command_timeout',
        'Limb.exit_control_mode',
        'Limb.set_joint_position_speed',
        'Limb.set_joint_positions',
        'Limb.set_joint_velocities',
    ],
    'torque': ['Limb.set_joint_torques'],
    'advanced_trajectory': ['Limb.set_joint_trajectory'],
    'kinematics': [
        'Limb.ik_request',
        'Limb.fk_request',
        'Limb.joint_angles_to_cartesian_pose',
        'Limb.fk_request_in_collision',
    ],
    'enable': ['RobotEnable.enable', 'RobotEnable.disable'],
    'stop_reset': ['RobotEnable.reset', 'RobotEnable.stop'],
    'native_motion': [
        'MotionTrajectory.stop_trajectory',
        'MotionTrajectory.send_trajectory',
        'MotionTrajectory.get_state',
        'MotionTrajectory.wait_for_result',
    ],
    'application': [
        'RobotParams.log_message',
        'Navigator.button_string_lookup',
    ],
}


def build(inventory):
    overrides = {
        name: feature_name
        for feature_name, names in OVERRIDES.items()
        for name in names
    }
    operations = []
    for item in inventory['operations']:
        _, class_name, method = item['operation'].rsplit('.', 2)
        if class_name not in CLASS_FEATURE:
            raise ValueError(class_name)
        name = overrides.get(
            class_name + '.' + method, CLASS_FEATURE[class_name]
        )
        if method.endswith(('_yaml_file', '_csv_file')) or method in (
            'to_string',
        ):
            name = 'application'
        operations.append(
            item
            | {
                'id': item['operation'] + '@' + str(item['line']),
                'feature': name,
                **FEATURES[name],
            }
        )
    return {
        'schema_version': 1,
        'target': 'Intera 5.3.1 / SDK 5.3.0 / standard electric gripper',
        'full_sdk_qualified': False,
        'release_state': ('unqualified: hardware records are not retained'),
        'scope': (
            'Supported public hardware operations. Helper methods are '
            'classified separately; advertised private types are not '
            'assumed supported.'
        ),
        'source_files_sha256': inventory['files'],
        'operation_count': len(operations),
        'operations': operations,
        'additional_capabilities': {
            'arm_calibration': {
                'python_api': 'calibrate_arm',
                'ros2_endpoint': 'calibrate_arm',
                'availability': 'requires compatible tool configuration',
                'hardware': 'not_recorded',
            },
            'tf': {
                'python_api': 'devices.transforms / robot_description',
                'ros2_endpoints': ['tf', 'tf_static', 'read_parameter'],
                'hardware': 'not_recorded',
            },
            'diagnostics': {
                'python_api': 'devices.diagnostics',
                'ros2_endpoint': 'diagnostics',
                'hardware': 'not_recorded',
            },
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).parent.parent
    parser.add_argument(
        '--inventory', type=Path, default=root / 'SDK_INVENTORY.json'
    )
    parser.add_argument(
        '--output', type=Path, default=root / 'CAPABILITY_LEDGER.json'
    )
    args = parser.parse_args()
    args.output.write_text(
        json.dumps(build(json.loads(args.inventory.read_text())), indent=2)
        + '\n'
    )


if __name__ == '__main__':
    main()
