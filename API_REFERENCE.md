# Existing API reference

This reference describes the APIs implemented in the current working tree.
It covers both robot drivers, Sawyer configuration and result types, ROS 2
services/actions/topics, optional ros_sugar adapters, and the custom interface
schemas. Source links are relative to this repository.

The application-facing Python APIs are documented first. The final inventory
lists supporting implementation callables; those internal modules are not a
promise of a stable application interface. Installation scripts, example CLI
arguments, test fixtures, and APIs inherited from third-party libraries are
outside this reference. Implementation availability does not establish physical
hardware qualification; see [Sawyer validation](irobot/src/robots/sawyer/VALIDATION.md).

## Contents

- [Architecture and common contracts](#architecture-and-common-contracts)
- [Public imports](#public-imports)
- [Sawyer Python API](#sawyer-python-api)
- [Sawyer configuration and data types](#sawyer-configuration-and-data-types)
- [Sawyer ROS 2 API](#sawyer-ros-2-api)
- [ros_sugar adapters](#ros_sugar-adapters)
- [Crazyflie Python API](#crazyflie-python-api)
- [Custom ROS 2 schemas](#custom-ros-2-schemas)
- [Native Intera protocol](#native-intera-protocol)
- [Supporting Python callable inventory](#supporting-python-callable-inventory)

## Architecture and common contracts

```text
Python application ───────────────────────┐
ROS 2 client → SawyerNode ────────────────┤
ros_sugar application → SawyerComponent ─┤
                                         ↓
                                 SawyerController
                                         ↓ inherits
                                    SawyerBase
                                         ↓
                              DirectTransport + wire
                                         ↓
                              Sawyer Intera controller
```

Sawyer uses XML-RPC discovery/parameters and native TCPROS messages, services,
and actions without a workstation ROS 1 runtime. ROS 2 is an optional adapter
around the same backend. Crazyflie delegates hardware communication to `cflib`.

Sawyer contracts:

- Joint vectors contain seven values in `right_j0` through `right_j6` order.
- Joint units are radians, radians/second, radians/second², and Nm. Cartesian
  position is metres; orientation is an `(x, y, z, w)` quaternion. The
  `move_relative()` convenience API takes centimetres.
- Driver construction is offline. `connect()` does not enable, reset, calibrate,
  start cameras, or move the robot. A connection may be open while disabled or
  stopped; `get_status()` reports readiness separately.
- APIs with `execute=False` validate/preview by default. Preview can require live
  state, limits, or model services. Set the method's `execute=True` explicitly
  to request execution; its keyword overrides an option object's execute field.
- Operations are synchronous unless exposed through an adapter. One driver
  operation owns the session at a time; overlapping calls are rejected.
- `stop()` requests cancellation. `devices.software_stop(execute=True)` separately
  latches the controller stop and waits for fresh stopped-state confirmation.
- `cancel_event` is a threading-event-like object supporting `is_set()`.
  External cancellation events are not cleared by the driver. Callbacks run
  during polling/control and must be fast and nonblocking.
- Invalid arguments generally raise `ValueError` or `TypeError`; missing/stale
  state, busy ownership, and readiness failures raise `RuntimeError`; bounded
  waits may raise `TimeoutError`; transport failures may raise connection,
  socket, or XML-RPC errors. Native action outcome dictionaries also expose
  controller success/failure, so inspect those results.
- Torque and trajectory streaming, plus force/compliant interaction execution,
  require `SawyerConfig(allow_unvalidated_modes=True)`. Native interaction
  trajectory execution has the same opt-in. This flag does not qualify hardware.

## Public imports

```python
from irobot import (
    SawyerBase, SawyerConfig, SawyerController,
    CrazyflieBase, CrazyflieConfig, CrazyflieController,
)
from irobot.src.robots.sawyer.config import (
    JOINT_NAMES, MotionOptions, CartesianMotionOptions, StreamOptions,
)
from irobot.src.robots.sawyer.core.base import CartesianPose
from irobot.src.robots.sawyer.core.interaction import (
    InteractionLimits, interaction_command,
)
from irobot.src.robots.sawyer.core import wire
```

Only the six driver/configuration classes above are exported at `irobot`'s top
level. Crazyflie imports are lazy; importing Sawyer does not load `cflib`.

## Sawyer Python API

### `SawyerBase`

Source: [irobot/src/robots/sawyer/core/base.py](irobot/src/robots/sawyer/core/base.py). SawyerController inherits all these members. Supports `with SawyerController(config) as robot:`; context entry connects and exit disconnects.

```python
SawyerBase(config: SawyerConfig | None=None)
```

#### `SawyerBase.devices`

```python
SawyerBase.devices
```

Read-only property returning the session-owned SawyerDevices helper. Requires an open connection. Use `robot.devices.<method>()`.

#### `SawyerBase.capabilities`

```python
SawyerBase.capabilities
```

Read-only property returning the bundled versioned capability ledger dictionary; available offline. Hardware readiness and qualification are separate.

#### `SawyerBase.check_version`

```python
SawyerBase.check_version(compatible=('5.3.1',))
```

Check the selected public-SDK baseline without changing the robot.

#### `SawyerBase.connect`

```python
SawyerBase.connect() -> bool
```

Open the direct transport and await usable robot state; return True or raise on failure. Repeated connection does not recreate the runtime. Does not actuate. Failed setup releases partial resources.

#### `SawyerBase.is_connected`

```python
SawyerBase.is_connected
```

Read-only property indicating an open owned session; it does not by itself establish feedback freshness or readiness.

#### `SawyerBase.joint_positions`

```python
SawyerBase.joint_positions
```

Read-only property returning a tuple of seven fresh measured joint positions in radians.

#### `SawyerBase.joint_velocities`

```python
SawyerBase.joint_velocities
```

Read-only property returning a tuple of seven fresh measured joint velocities in rad/s.

#### `SawyerBase.joint_efforts`

```python
SawyerBase.joint_efforts
```

Read-only property returning seven measured joint efforts in Nm, with per-joint freshness checks.

#### `SawyerBase.joint_limits`

```python
SawyerBase.joint_limits
```

Read-only property returning a dictionary with lower, upper, velocity, and acceleration vectors from published limits.

#### `SawyerBase.get_parameter`

```python
SawyerBase.get_parameter(name)
```

Read the named native ROS parameter through the master and return its decoded Python value. Requires connection; this is not a ROS 2 parameter lookup.

#### `SawyerBase.robot_description`

```python
SawyerBase.robot_description
```

Read-only property returning the configured robot-description parameter, normally the URDF XML string.

#### `SawyerBase.joint_effort_limits`

```python
SawyerBase.joint_effort_limits
```

Read-only property returning seven conservative effort limits: the minimum of URDF and published limits for each joint.

#### `SawyerBase.endpoint_state`

```python
SawyerBase.endpoint_state
```

Read-only property returning a copied native EndpointState for the configured named tip, including pose, twist, wrench, header, and related fields.

#### `SawyerBase.get_tip_state`

```python
SawyerBase.get_tip_state(tip=None)
```

Return a copied complete native EndpointState for the selected tip. None selects config.tip. Requires fresh, frame-qualified endpoint feedback.

#### `SawyerBase.tip_names`

```python
SawyerBase.tip_names
```

Read-only property returning a tuple of named tips with feedback inside feedback_timeout.

#### `SawyerBase.poll`

```python
SawyerBase.poll(timeout=0.02)
```

Dispatch feedback and registered callbacks for the requested nonnegative interval without issuing commands. Useful when the application is otherwise idle.

#### `SawyerBase.endpoint_velocity`

```python
SawyerBase.endpoint_velocity
```

Read-only property returning {'linear': (x, y, z), 'angular': (x, y, z), 'frame': name}; units m/s and rad/s.

#### `SawyerBase.endpoint_effort`

```python
SawyerBase.endpoint_effort
```

Read-only property returning {'force': (x, y, z), 'torque': (x, y, z), 'frame': name}; units N and Nm.

#### `SawyerBase.endpoint_pose`

```python
SawyerBase.endpoint_pose
```

Read-only property returning CartesianPose(position, orientation, frame) for the selected fresh tip.

#### `SawyerBase.forward_kinematics`

```python
SawyerBase.forward_kinematics(configurations, *, tip=None)
```

Accept an iterable of seven-joint configurations and return CartesianPose values. Uses the controller FK service and validates inputs and results. No movement.

#### `SawyerBase.configuration_collisions`

```python
SawyerBase.configuration_collisions(configurations, *, tip=None)
```

Accept batches of seven-joint configurations and return a tuple of controller collision flags. No movement; this is not general external-obstacle avoidance.

#### `SawyerBase.inverse_kinematics`

```python
SawyerBase.inverse_kinematics(pose, *, seed=None, tip=None)
```

Accept CartesianPose and an optional seven-joint seed; return a validated seven-joint solution. With seed=None, use current measured joints. No movement.

#### `SawyerBase.gripper_state`

```python
SawyerBase.gripper_state
```

Return the latest standard Intera gripper signals without commanding it.

#### `SawyerBase.get_status`

```python
SawyerBase.get_status() -> dict
```

Return a status dictionary. Disconnected: connected=False, ready=False, issues=['Disconnected']. Connected: also includes enabled, stopped, error, homed, collision, joints, velocities, and joint_feedback_fresh. This call dispatches pending feedback.

#### `SawyerBase.disconnect`

```python
SawyerBase.disconnect()
```

Stop/release the owned runtime, sockets, subscriptions, and helpers. Return None. Also called when leaving the context manager.

### `SawyerController`

Source: [irobot/src/robots/sawyer/core/controller.py](irobot/src/robots/sawyer/core/controller.py). Inherits SawyerBase(config=None); all motion methods below are synchronous.

#### `SawyerController.run_interaction`

```python
SawyerController.run_interaction(message, duration, *, execute=False, limits=None, stream_options=None, cancel_event=None, on_step=None)
```

Accept a wire.InteractionControlCommand and positive finite duration. limits defaults to InteractionLimits(); stream_options supplies joint envelopes. Preview returns validation details; execution returns executed, activation_confirmed, deactivation_confirmed, samples, and duration_s. Always requests deactivation during cleanup.

#### `SawyerController.stream_joint_commands`

```python
SawyerController.stream_joint_commands(mode, source, duration, *, options=None, execute=False, cancel_event=None, on_step=None)
```

Run a finite position, velocity, torque, or trajectory command session. source is a constant command dictionary or source(elapsed_seconds) returning one. options defaults to StreamOptions(). Preview validates the initial sample and returns executed, command, duration_s, and envelope. Execution returns executed, mode, samples, duration_s, max_command_gap_s, and final joints. See the stream contract below.

#### `SawyerController.move_native_joint_path`

```python
SawyerController.move_native_joint_path(positions, *, execute=False, plan_only=False, speed=0.05, max_acceleration=0.1, timeout=60.0, cancel_event=None, on_feedback=None)
```

Build a typed native joint trajectory from positions and use the Intera motion action server. speed is a velocity ratio; max_acceleration is a joint acceleration cap in rad/s², clipped to each published joint limit. execute=False returns a goal preview. execute=True with plan_only=True asks Intera to generate a plan without running motion. Executed results include succeeded, status, and result.

#### `SawyerController.execute_native_trajectory`

```python
SawyerController.execute_native_trajectory(trajectory, *, execute=False, plan_only=False, timeout=60.0, cancel_event=None, on_feedback=None)
```

Validate a wire.Trajectory (native joint/Cartesian path) and use the Intera motion action. execute=False returns executed=False and goal. execute=True sends start, or generate when plan_only=True. Return an action outcome dictionary; inspect succeeded and the controller result.

#### `SawyerController.calibrate_arm`

```python
SawyerController.calibrate_arm(*, execute=False, timeout=120.0, cancel_event=None, on_feedback=None)
```

Explicit native arm calibration/homing. Preview reports executed=False, available, and the gripper-removal precondition. Execution requires the gripper removed/configured out and a ready controller. Return a native action outcome dictionary.

#### `SawyerController.move_to_joints`

```python
SawyerController.move_to_joints(joints, *, duration=None, execute=False, options=None, cancel_event=None, on_step=None)
```

Plan from current measured joints to seven target angles. duration=None derives timing from limits. Return JointMoveResult(plan, execution); execution is None for preview or a no-op, otherwise MotionResult. Uses a synchronized quintic reference and PID velocity tracking.

#### `SawyerController.move_to_neutral`

```python
SawyerController.move_to_neutral(*, duration=None, execute=False, options=None, cancel_event=None, on_step=None)
```

Resolve config.neutral_joints, or read the named neutral pose from config.neutral_parameter; then use the same planner/tracker as move_to_joints. Return JointMoveResult.

#### `SawyerController.move_trajectory`

```python
SawyerController.move_trajectory(positions, times, *, execute=False, options=None, cancel_event=None, on_step=None)
```

Accept positions as a sequence of seven-joint waypoints and matching times in seconds from start. Times must strictly increase above zero; every segment ends at rest. Return JointMoveResult.

#### `SawyerController.move_relative`

```python
SawyerController.move_relative(dx_cm=0.0, dy_cm=0.0, dz_cm=0.0, *, duration=None, execute=False, options=None)
```

Plan a straight Cartesian displacement in the base frame with fixed orientation. dx_cm/dy_cm/dz_cm are centimetres. Return CartesianMoveResult(plan, execution).

#### `SawyerController.move_to_pose`

```python
SawyerController.move_to_pose(pose, *, duration=None, execute=False, options=None)
```

Accept a CartesianPose in the configured frame; plan a straight Cartesian line with shortest-path quaternion interpolation, seeded IK, and sampled FK validation. Return CartesianMoveResult. This signature has no external cancel_event/on_step keywords; stop() requests cancellation.

#### `SawyerController.open_gripper`

```python
SawyerController.open_gripper(*, execute=False, cancel_event=None)
```

Preview or open to config.gripper_open_position. Return a command/result dictionary; execution matches command acknowledgement and measured completion.

#### `SawyerController.move_gripper`

```python
SawyerController.move_gripper(position, *, execute=False, cancel_event=None)
```

Preview or command finger separation position in metres, between zero and config.gripper_open_position. Wait for acknowledgement and stable measured completion on execution. Return a dictionary.

#### `SawyerController.close_gripper`

```python
SawyerController.close_gripper(*, execute=False, cancel_event=None)
```

Preview or close the electric gripper. Completion may represent gripping an object rather than reaching zero separation. Return a command/result dictionary.

#### `SawyerController.calibrate_gripper`

```python
SawyerController.calibrate_gripper(*, execute=False, cancel_event=None)
```

Explicitly preview/calibrate the electric gripper. Connection never calibrates automatically. Return a command/result dictionary.

#### `SawyerController.stop_gripper`

```python
SawyerController.stop_gripper(*, execute=False)
```

Explicit gripper stop command, previewed by default. Return a command/result dictionary.

#### `SawyerController.start_gripper`

```python
SawyerController.start_gripper(*, execute=False, cancel_event=None)
```

Resume the currently reported gripper position target explicitly; does not invent a new target. Return a command/result dictionary.

#### `SawyerController.reboot_gripper`

```python
SawyerController.reboot_gripper(*, execute=False, cancel_event=None)
```

Power-cycle an idle gripper explicitly. Subsequent calibration must be requested separately. Return a command/result dictionary.

#### `SawyerController.stop`

```python
SawyerController.stop()
```

Request cancellation of the active arm operation and its ordinary mode-specific stop cleanup. Does not latch the controller software stop or reset/enable the robot. Return None.

### Motion callbacks and stream command contract

For move_to_joints, move_to_neutral, and move_trajectory,
`on_step(elapsed, reference, joints, velocities)` receives the elapsed seconds,
a TrajectorySample, and measured joint/velocity tuples. Streaming and interaction
use `on_step(elapsed, joints, velocities)`. Native actions use
`on_feedback(native_feedback_message)`.

| Stream mode | Exactly these command keys | Values |
| --- | --- | --- |
| `position` | `position` | Seven angles in radians |
| `velocity` | `velocity` | Seven velocities in rad/s |
| `torque` | `effort` | Seven torques in Nm |
| `trajectory` | `position`, `velocity`, `acceleration` | Seven values per field |

Streams are bounded by duration, rate, displacement, limits, watchdog, and fresh
feedback. A preview checks only the initial source sample; execution validates
every sample. Commands are not retained for later replay.

```python
from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.core.base import CartesianPose

config = SawyerConfig(ros_master_uri='http://ROBOT_IP:11311', ros_ip='PC_IP')
with SawyerController(config) as robot:
    status = robot.get_status()
    joints = robot.joint_positions
    preview = robot.move_to_joints(joints)
    pose = robot.endpoint_pose
    target = CartesianPose(pose.position, pose.orientation, pose.frame)
    cartesian_preview = robot.move_to_pose(target)
```

### Device API access

Obtain the owned helper as `devices = robot.devices`; do not create a second
hardware owner. Camera selectors are exactly `'head'` and `'hand'`.

### `SawyerDevices`

Source: [irobot/src/robots/sawyer/core/peripherals.py](irobot/src/robots/sawyer/core/peripherals.py).

```python
SawyerDevices(driver)
```

#### `SawyerDevices.transforms`

```python
SawyerDevices.transforms(*, static=False)
```

Return a wire.TFMessage containing known TransformStamped edges. static=False omits expired dynamic edges; static=True returns retained static transforms.

#### `SawyerDevices.read_io`

```python
SawyerDevices.read_io(namespace)
```

Return a merged IO snapshot for a native namespace, including namespace, device_status, connected, last_message_age_s, dropped_events, transport_dropped_messages, signals, ports, and configuration. Signal/port entries include value, format, status, and received_at.

#### `SawyerDevices.write_io`

```python
SawyerDevices.write_io(namespace, values, *, category='signals', execute=False, cancel_event=None)
```

Write a nonempty name-to-value mapping to category='signals' or 'ports'. Validate published type, dimensions, permissions, status, and bounds. Preview returns the command; execution requires acknowledgement and matching measured values. Gripper motion signals must use explicit gripper methods.

#### `SawyerDevices.register_io_callback`

```python
SawyerDevices.register_io_callback(namespace, name, callback)
```

Register callback(event) for a known signal in a namespace; return an integer identifier. Events include category, name, previous, value, and received_at.

#### `SawyerDevices.deregister_io_callback`

```python
SawyerDevices.deregister_io_callback(namespace, identifier)
```

Remove a registered IO callback by its namespace and identifier. Return None.

#### `SawyerDevices.io_events`

```python
SawyerDevices.io_events(namespace)
```

Return and drain the bounded list of IO change events for this namespace. Drop counts remain available in read_io().

#### `SawyerDevices.diagnostics`

```python
SawyerDevices.diagnostics()
```

Return the latest native DiagnosticArray received on the configured diagnostics topic. This convenience read does not aggregate historical statuses.

#### `SawyerDevices.configure_gripper`

```python
SawyerDevices.configure_gripper(*, speed=None, dead_zone=None, object_mass=None, execute=False)
```

Set any supplied speed (m/s), dead_zone (m), or object_mass (kg) using the native speed_mps, dead_zone_m, and right_gripper_tip_object_kg signals. Requires idle, error-free gripper state. Return an IO preview/result dictionary.

#### `SawyerDevices.configure_camera`

```python
SawyerDevices.configure_camera(camera, settings, *, execute=False)
```

Write a settings mapping for head or hand. Accepted keys: exposure, gain, brightness, flip, mirror, window_x, window_y, resolution, strobe, embedded_data. Values follow published IO metadata. Return an IO preview/result dictionary.

#### `SawyerDevices.set_light`

```python
SawyerDevices.set_light(name, enabled, *, execute=False)
```

Set one documented light signal using a boolean: head_red_light, head_green_light, head_blue_light, right_hand_red_light, right_hand_green_light, or right_hand_blue_light. Return an IO preview/result dictionary.

#### `SawyerDevices.node`

```python
SawyerDevices.node
```

Property exposing the connected internal _ArmSession. This is an implementation access point; application code should prefer the explicit device methods.

#### `SawyerDevices.inventory`

```python
SawyerDevices.inventory()
```

Return version, native topic_types, graph, wire_messages, and wire_services using robot discovery and the bundled schema.

#### `SawyerDevices.robot_state`

```python
SawyerDevices.robot_state()
```

Return a dictionary of the fresh RobotAssemblyState fields, including enabled/ready/stopped/error/homed information.

#### `SawyerDevices.set_enabled`

```python
SawyerDevices.set_enabled(enabled, *, execute=False)
```

Explicitly enable or disable. Enabling rejects stopped/error states and bounds displacement while confirming readiness. Preview/result dictionaries indicate executed, already_complete where relevant, and resulting state.

#### `SawyerDevices.reset_faults`

```python
SawyerDevices.reset_faults(*, execute=False)
```

Explicitly preview/reset controller faults; does not implicitly enable. Return a command/result dictionary with measured state confirmation on execution.

#### `SawyerDevices.software_stop`

```python
SawyerDevices.software_stop(*, execute=False)
```

Latch the controller stopped state explicitly and require fresh confirmation. Separate from stop() cancellation. Return a preview/result dictionary.

#### `SawyerDevices.head_state`

```python
SawyerDevices.head_state()
```

Return fresh native HeadState feedback for head pan.

#### `SawyerDevices.move_head`

```python
SawyerDevices.move_head(angle, *, speed=0.1, execute=False)
```

Preview or command pan angle in radians, using a positive speed ratio no greater than one. Validate URDF limits and confirm measured arrival. Return a command/result dictionary. Physical head-pan qualification remains deferred.

#### `SawyerDevices.camera_state`

```python
SawyerDevices.camera_state(camera)
```

Return the latest native IODeviceStatus for the selected head/hand camera.

#### `SawyerDevices.register_camera_callback`

```python
SawyerDevices.register_camera_callback(camera, callback)
```

Register callback(native_wire_image) for head/hand image frames and return an integer identifier. Does not start the camera stream. Callbacks run during poll/control.

#### `SawyerDevices.deregister_camera_callback`

```python
SawyerDevices.deregister_camera_callback(identifier)
```

Remove a registered camera callback by its identifier. Return None.

#### `SawyerDevices.digital_state`

```python
SawyerDevices.digital_state(name)
```

Read native DigitalIOState for a published component name (a single name, without slashes).

#### `SawyerDevices.set_digital_output`

```python
SawyerDevices.set_digital_output(name, value, *, execute=False)
```

Validate a boolean value and published output permission, then preview/write the named digital output. Execution confirms fresh measured output state. Return executed/measured information; already_complete may be True.

#### `SawyerDevices.io_state`

```python
SawyerDevices.io_state(device)
```

Return raw IODeviceStatus for one of 'cuff', 'navigator', 'robot', or 'right_end_of_arm'; preserves vector and change-only signals.

#### `SawyerDevices.set_camera_streaming`

```python
SawyerDevices.set_camera_streaming(camera, enabled, *, execute=False)
```

Explicitly preview/start/stop head or hand camera streaming. Execution requires a matching acknowledgement and observed camera_streaming state. Return a dictionary with executed.

#### `SawyerDevices.camera_frame`

```python
SawyerDevices.camera_frame(camera)
```

Return (wire.Image, wire.CameraInfo) for advancing current frames. Requires the camera already streaming; this read never starts it.

#### `SawyerDevices.display_image`

```python
SawyerDevices.display_image(image, *, execute=False)
```

Accept a valid wire.Image with rgb8 or bgr8 encoding. Preview returns serialized byte count. Execution publishes to the head display and returns executed=True, delivery_confirmed=False; publication has no display acknowledgement.

## Sawyer configuration and data types

Configuration classes are frozen dataclasses. Values below are source defaults;
explicit arguments override them. SawyerConfig.motion uses a fresh
CartesianMotionOptions object. Options accept keyword construction and validate
finite bounds. MotionOptions.rate and StreamOptions.rate must be 50–250 Hz;
speed ratios are in (0, 1]. CartesianMotionOptions inherits all MotionOptions
fields in addition to its own table. The method's execute keyword controls
whether motion runs.

### `MotionOptions` fields

Source: [irobot/src/robots/sawyer/config.py](irobot/src/robots/sawyer/config.py#L28).

| Field | Type | Default |
| --- | --- | --- |
| `execute` | `bool` | `False` |
| `time` | `float \| None` | `None` |
| `speed` | `float` | `1.0` |
| `max_accel` | `float \| None` | `None` |
| `rate` | `float` | `100.0` |
| `kp` | `float` | `2.0` |
| `ki` | `float` | `0.1` |
| `kd` | `float` | `0.05` |
| `settle_time` | `float` | `5.0` |
| `timeout` | `float \| None` | `None` |
| `max_error` | `float` | `0.35` |

### `CartesianMotionOptions` fields

Source: [irobot/src/robots/sawyer/config.py](irobot/src/robots/sawyer/config.py#L65).

| Field | Type | Default |
| --- | --- | --- |
| `tip` | `str \| None` | `None` |
| `frame` | `str` | `'base'` |
| `angular_step` | `float` | `ANGULAR_SAMPLE_STEP` |
| `sample_step` | `float` | `0.005` |
| `path_tolerance` | `float` | `0.001` |
| `path_orientation_tolerance` | `float` | `0.01` |
| `max_joint_step` | `float` | `0.15` |
| `max_samples` | `int` | `401` |
| `cartesian_tolerance` | `float` | `0.003` |
| `goal_orientation_tolerance` | `float` | `0.02` |
| `max_cartesian_error` | `float` | `0.02` |
| `max_orientation_error` | `float` | `0.1` |
| `ik_timeout` | `float` | `2.0` |
| `planning_timeout` | `float` | `60.0` |

### `StreamOptions` fields

Source: [irobot/src/robots/sawyer/config.py](irobot/src/robots/sawyer/config.py#L109).

| Field | Type | Default |
| --- | --- | --- |
| `rate` | `float` | `100.0` |
| `max_velocity` | `float` | `0.1` |
| `max_acceleration` | `float` | `0.5` |
| `max_displacement` | `float` | `0.1` |
| `effort_fraction` | `float` | `0.05` |
| `position_speed` | `float` | `0.05` |

### `SawyerConfig` fields

Source: [irobot/src/robots/sawyer/config.py](irobot/src/robots/sawyer/config.py#L131).

| Field | Type | Default |
| --- | --- | --- |
| `node_name` | `str` | `'irobot_sawyer'` |
| `connect_timeout` | `float` | `10.0` |
| `callback_timeout` | `float` | `0.02` |
| `callback_slice` | `float` | `0.01` |
| `input_poll_timeout` | `float` | `0.05` |
| `planning_refresh_timeout` | `float` | `0.5` |
| `planning_joint_tolerance` | `float` | `0.01` |
| `planning_position_tolerance` | `float` | `0.002` |
| `planning_orientation_tolerance` | `float` | `0.01` |
| `tip_match_position_tolerance` | `float` | `0.0001` |
| `tip_match_orientation_tolerance` | `float` | `0.001` |
| `watchdog_refresh_interval` | `float` | `1.0` |
| `stop_zero_samples` | `int` | `5` |
| `stop_hold_samples` | `int` | `3` |
| `stop_publish_interval` | `float` | `0.02` |
| `gripper_ack_retry_interval` | `float` | `0.1` |
| `gripper_position_tolerance` | `float` | `0.002` |
| `gripper_settle_confirmation` | `float` | `0.2` |
| `feedback_timeout` | `float` | `0.2` |
| `command_timeout` | `float` | `0.2` |
| `position_tolerance` | `float` | `0.02` |
| `velocity_tolerance` | `float` | `0.03` |
| `settle_confirmation` | `float` | `0.2` |
| `frame` | `str` | `'base'` |
| `tip` | `str \| None` | `None` |
| `domain_id` | `int \| None` | `0` |
| `ros_master_uri` | `str \| None` | `'http://169.254.121.3:11311'` |
| `ros_ip` | `str \| None` | `'169.254.121.2'` |
| `transport_timeout` | `float` | `0.2` |
| `transport_retry_interval` | `float` | `0.2` |
| `transport_idle_timeout` | `float` | `5.0` |
| `registration_refresh_interval` | `float` | `2.0` |
| `neutral_joints` | `tuple[float, ...] \| None` | `None` |
| `neutral_parameter` | `str` | `'/named_poses/right'` |
| `parameter_timeout` | `float` | `5.0` |
| `enable_gripper` | `bool` | `True` |
| `allow_unvalidated_modes` | `bool` | `False` |
| `gripper_open_position` | `float` | `0.041667` |
| `gripper_timeout` | `float` | `10.0` |
| `joint_state_topic` | `str` | `'/robot/joint_states'` |
| `robot_state_topic` | `str` | `'/robot/state'` |
| `homing_state_topic` | `str` | `'/robot/homing_states'` |
| `enable_topic` | `str` | `'/robot/set_super_enable'` |
| `reset_topic` | `str` | `'/robot/set_super_reset'` |
| `software_stop_topic` | `str` | `'/robot/set_super_stop'` |
| `version_parameter` | `str` | `'/manifest/robot_software/version/HLR_VERSION_STRING'` |
| `robot_description_parameter` | `str` | `'/robot_description'` |
| `head_state_topic` | `str` | `'/robot/head/head_state'` |
| `head_command_topic` | `str` | `'/robot/head/command_head_pan'` |
| `head_display_topic` | `str` | `'/robot/head_display'` |
| `camera_namespace` | `str` | `'/io/internal_camera'` |
| `io_namespace` | `str` | `'/io/robot'` |
| `device_timeout` | `float` | `5.0` |
| `enable_max_displacement` | `float` | `0.02` |
| `head_position_tolerance` | `float` | `0.01` |
| `collision_topic` | `str` | `'/robot/limb/right/collision_detection_state'` |
| `joint_limits_topic` | `str` | `'/robot/joint_limits'` |
| `joint_command_topic` | `str` | `'/robot/limb/right/joint_command'` |
| `command_timeout_topic` | `str` | `'/robot/limb/right/joint_command_timeout'` |
| `speed_ratio_topic` | `str` | `'/robot/limb/right/set_speed_ratio'` |
| `interaction_command_topic` | `str` | `'/robot/limb/right/interaction_control_command'` |
| `interaction_state_topic` | `str` | `'/robot/limb/right/interaction_controller/state'` |
| `diagnostics_topic` | `str` | `'/diagnostics'` |
| `tf_topic` | `str` | `'/tf'` |
| `tf_static_topic` | `str` | `'/tf_static'` |
| `native_motion_action` | `str` | `'/motion/motion_command'` |
| `calibration_action` | `str` | `'/calibration_command'` |
| `action_server_timeout` | `float` | `2.0` |
| `endpoint_topic` | `str` | `'/robot/limb/right/endpoint_state'` |
| `tip_states_topic` | `str` | `'/robot/limb/right/tip_states'` |
| `ik_service` | `str` | `'/ExternalTools/right/PositionKinematicsNode/IKService'` |
| `fk_service` | `str` | `'/ExternalTools/right/PositionKinematicsNode/FKService'` |
| `gripper_namespace` | `str` | `'/io/end_effector/right_gripper'` |
| `motion` | `CartesianMotionOptions` | `field(default_factory=CartesianMotionOptions)` |

### `InteractionLimits` fields

Source: [irobot/src/robots/sawyer/core/interaction.py](irobot/src/robots/sawyer/core/interaction.py#L18). Application envelopes for interaction; vectors are XYZ/rotation axes or seven joints as appropriate, not controller-certified limits.

| Field | Type | Default |
| --- | --- | --- |
| `stiffness` | `tuple` | `(1300.0, 1300.0, 1300.0, 30.0, 30.0, 30.0)` |
| `damping` | `tuple` | `(100.0,) * 6` |
| `nullspace_stiffness` | `tuple` | `(10.0,) * 7` |
| `wrench` | `tuple` | `(5.0, 5.0, 5.0, 1.0, 1.0, 1.0)` |
| `translation` | `float` | `0.01` |
| `rotation` | `float` | `0.1` |

### Geometry, plans, and results

Import these from the linked defining modules. Required fields have no default.
CartesianPose normalizes approximately unit quaternions, rejects invalid values,
and requires a nonempty frame. Geometry must match the selected configured frame;
these motion calls do not perform arbitrary TF frame conversion.

### `CartesianPose` fields

Source: [irobot/src/robots/sawyer/core/base.py](irobot/src/robots/sawyer/core/base.py#L50).

| Field | Type | Default |
| --- | --- | --- |
| `position` | `tuple` | `required` |
| `orientation` | `tuple` | `required` |
| `frame` | `str` | `'base'` |

### `MotionResult` fields

Source: [irobot/src/robots/sawyer/core/base.py](irobot/src/robots/sawyer/core/base.py#L200).

| Field | Type | Default |
| --- | --- | --- |
| `reference_duration` | `float` | `required` |
| `elapsed_duration` | `float` | `required` |
| `final_joints` | `tuple` | `required` |

### `JointBounds` fields

Source: [irobot/src/robots/sawyer/core/base.py](irobot/src/robots/sawyer/core/base.py#L218).

| Field | Type | Default |
| --- | --- | --- |
| `lower` | `tuple` | `required` |
| `upper` | `tuple` | `required` |
| `velocity` | `tuple` | `required` |
| `acceleration` | `tuple` | `required` |

### `TrajectorySample` fields

Source: [irobot/src/robots/sawyer/core/controller.py](irobot/src/robots/sawyer/core/controller.py#L40).

| Field | Type | Default |
| --- | --- | --- |
| `position` | `tuple` | `required` |
| `velocity` | `tuple` | `required` |
| `acceleration` | `tuple` | `required` |
| `jerk` | `tuple` | `required` |

### `CartesianPlan` fields

Source: [irobot/src/robots/sawyer/core/controller.py](irobot/src/robots/sawyer/core/controller.py#L645).

| Field | Type | Default |
| --- | --- | --- |
| `line` | `CartesianLine` | `required` |
| `trajectory` | `TimedJointPath` | `required` |
| `samples` | `int` | `required` |
| `max_position_error` | `float` | `required` |
| `max_orientation_error` | `float` | `required` |

### `CartesianMoveResult` fields

Source: [irobot/src/robots/sawyer/core/controller.py](irobot/src/robots/sawyer/core/controller.py#L921).

| Field | Type | Default |
| --- | --- | --- |
| `plan` | `CartesianPlan` | `required` |
| `execution` | `MotionResult \| None` | `required` |

### `JointMoveResult` fields

Source: [irobot/src/robots/sawyer/core/controller.py](irobot/src/robots/sawyer/core/controller.py#L927).

| Field | Type | Default |
| --- | --- | --- |
| `plan` | `object` | `required` |
| `execution` | `object \| None` | `required` |

### `ActionResult` fields

Source: [irobot/src/robots/sawyer/core/actions.py](irobot/src/robots/sawyer/core/actions.py#L17). Internal native-action result wrapper; succeeded is a property checking status == SUCCEEDED.

| Field | Type | Default |
| --- | --- | --- |
| `goal_id` | `str` | `required` |
| `status` | `int` | `required` |
| `text` | `str` | `required` |
| `result` | `object` | `required` |

### Interaction command helper

```python
interaction_command(*, endpoint='right_hand')
```

Defined in core/interaction.py. Returns a typed wire.InteractionControlCommand
with active interaction, maximum impedance flags, zero requested force, base
frame, and an identity interaction-frame quaternion. Call
`robot.run_interaction(message, duration, ...)` to preview or run it.

## Sawyer ROS 2 API

Source: [ros2.py](irobot/src/robots/sawyer/ros2.py),
[ros2_interfaces.py](irobot/src/robots/sawyer/ros2_interfaces.py), and
[ros2_telemetry.py](irobot/src/robots/sawyer/ros2_telemetry.py).

```python
SawyerNode(driver, *, namespace='sawyer', context=None,
           cameras=False, interfaces=False)
```

The caller connects driver before node construction and owns ROS context setup,
shutdown, and final driver disconnection. destroy_node() closes node-owned
resources. `interfaces=True` requires the explicitly built
`irobot_sawyer_interfaces` package. `cameras=True` exposes camera telemetry;
it does not start camera streaming. All names below are relative to the node
namespace, normally `/sawyer`. Framework-provided ROS 2 discovery/parameter
endpoints are not custom driver commands.

### Standard services

| Name | Type | Behavior |
| --- | --- | --- |
| `move_to_neutral` | `std_srvs/srv/Trigger` | Submit neutral motion |
| `open_gripper` | `std_srvs/srv/Trigger` | Submit gripper open |
| `close_gripper` | `std_srvs/srv/Trigger` | Submit gripper close |
| `calibrate_gripper` | `std_srvs/srv/Trigger` | Submit gripper calibration |
| `reset_faults` | `std_srvs/srv/Trigger` | Submit controller reset |
| `set_enabled` | `std_srvs/srv/SetBool` | Submit enable/disable from request.data |
| `stop` | `std_srvs/srv/Trigger` | Request active operation cancellation |
| `software_stop` | `std_srvs/srv/Trigger` | Cancel, wait boundedly for ownership, and confirm latched stop |

Job services return success=True for acceptance, not completion. Observe
`status` for succeeded/failed/cancelled state. Busy/shutting-down requests are
rejected rather than queued. software_stop separately waits for confirmation
and returns failure when confirmation is unavailable.

### Standard trajectory action

| Name | Type | Backend |
| --- | --- | --- |
| `follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | driver.move_trajectory(execute=True) |

Supported goal subset: all seven distinct arm joint names, immediate zero header
timestamp, strictly increasing positive waypoint times, position waypoints, and
omitted or zero velocities/accelerations. Effort, custom
path/goal tolerances, nonzero goal-time tolerance, multi-DOF paths, and simulated
time are rejected. Feedback includes desired/actual positions and velocities,
desired acceleration, position error, and elapsed time. Cancellation requests
driver stop; final results distinguish success, cancellation, and abort. A
finite goal continues if the client disappears unless cancelled.

### Extended actions (`interfaces=True`)

| Name | Type | Implementation and restrictions |
| --- | --- | --- |
| `joint_stream` | `irobot_sawyer_interfaces/action/JointStream` | Finite leased command stream; mode constants 1–4; complete positive envelope values required |
| `native_motion` | `irobot_sawyer_interfaces/action/MotionCommand` | Typed Intera trajectory; accepts command=start or generate; cancel the action to stop |
| `calibrate_arm` | `irobot_sawyer_interfaces/action/CalibrationCommand` | Only command=start accepted; gripper removal precondition enforced |
| `interaction` | `irobot_sawyer_interfaces/action/InteractionSession` | Timed typed interaction with activation/deactivation results |
| `move_cartesian` | `irobot_sawyer_interfaces/action/MoveCartesian` | PoseStamped target; duration=0 selects automatic timing; positive speed/max_acceleration required |
| `gripper_command` | `control_msgs/action/GripperCommand` | Position in metres within configured range; max_effort must be 0 (fixed hardware force) |

The MoveCartesian schema declares actual-pose feedback, but the current handler
does not populate/publish it. GripperCommand populates the final result from
measured position, force, gripping state, and position tolerance; its handler
does not publish intermediate action feedback. ROS message numeric fields
default to zero, not the positive Python option defaults; fill goal values
explicitly where validation requires positive values.

For JointStream, `stream_command` packets need the action session ID (the goal
UUID bytes encoded as hex), a strictly increasing sequence, a current ROS 2
source timestamp, the seven names, matching mode, and exactly the mode's fields.
Feedback reports session_id after streaming starts. The server waits for an
initial command before starting its streaming feedback, so the initial packet
must use the accepted goal UUID. lease_timeout must not exceed half the hardware
command_timeout. Expiry aborts the session; late packets do not resume it.

### Extended services (`interfaces=True`)

| Name | Type | Request/result contract |
| --- | --- | --- |
| `forward_kinematics` | `irobot_sawyer_interfaces/srv/ForwardKinematics` | Flattened batches of seven positions, optional tip; returns poses |
| `inverse_kinematics` | `irobot_sawyer_interfaces/srv/InverseKinematics` | PoseStamped, seven-value seed, optional tip; returns seven angles |
| `check_joint_collision` | `irobot_sawyer_interfaces/srv/CheckJointCollision` | Flattened batches of seven angles, optional tip; returns collision flags |
| `read_parameter` | `irobot_sawyer_interfaces/srv/ReadParameter` | Native parameter name; returns JSON text in json_value |
| `inventory` | `std_srvs/srv/Trigger` | Discovery dictionary serialized as JSON in response.message |
| `capabilities` | `std_srvs/srv/Trigger` | Bundled capability ledger JSON in response.message |
| `stop_gripper` | `std_srvs/srv/Trigger` | Submit gripper stop job |
| `start_gripper` | `std_srvs/srv/Trigger` | Submit gripper resume job |
| `reboot_gripper` | `std_srvs/srv/Trigger` | Submit gripper reboot job |
| `head/streaming`, `hand/streaming` | `std_srvs/srv/SetBool` | Explicitly start/stop the selected camera |
| `digital_outputs/set_parameters` | `rcl_interfaces/srv/SetParameters` | Named boolean digital outputs; per-item results |
| `{device}/set_parameters` | `rcl_interfaces/srv/SetParameters` | IO signal writes with per-item results |
| `{device}/ports/set_parameters` | `rcl_interfaces/srv/SetParameters` | IO port writes with per-item results |

Writable device labels are gripper, head_camera, hand_camera, and robot_io.
Object/array IO values use a JSON string parameter. The adapter checks published
IO metadata, acknowledgement, and measured state. Dedicated gripper motion
methods own position/go/calibration/reboot writes. Per-item writes may partially
succeed; inspect every SetParametersResult. Typed query services return success
and detail; queries can fail while the driver is busy.

### Published topics

| Relative name | ROS 2 type | Availability |
| --- | --- | --- |
| `joint_states` | `sensor_msgs/msg/JointState` | Standard; canonical fresh arm state |
| `endpoint_pose` | `geometry_msgs/msg/PoseStamped` | Standard; measured endpoint |
| `endpoint_twist` | `geometry_msgs/msg/TwistStamped` | Standard |
| `endpoint_wrench` | `geometry_msgs/msg/WrenchStamped` | Standard |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Standard |
| `tf`, `tf_static` | `tf2_msgs/msg/TFMessage` | Standard; tf_static retained |
| `status` | `std_msgs/msg/String` | Standard; JSON status/job information |
| `robot_state` | `irobot_sawyer_interfaces/msg/RobotAssemblyState` | interfaces=True |
| `endpoint_states` | `irobot_sawyer_interfaces/msg/EndpointStates` | interfaces=True |
| `joint_limits` | `irobot_sawyer_interfaces/msg/JointLimits` | interfaces=True |
| `{device}/state` | `irobot_sawyer_interfaces/msg/IODeviceStatus` | interfaces=True |
| `{device}/config` | `irobot_sawyer_interfaces/msg/IODeviceConfiguration` | interfaces=True |
| `digital_io/right_hand_camera_power` | `irobot_sawyer_interfaces/msg/DigitalIOState` | interfaces=True |
| `io_events`, `io_health` | `std_msgs/msg/String` | interfaces=True; JSON |
| `head/image`, `hand/image` | `sensor_msgs/msg/Image` | cameras=True; source already streaming |
| `head/camera_info`, `hand/camera_info` | `sensor_msgs/msg/CameraInfo` | cameras=True |

Telemetry device labels are gripper, head_camera, hand_camera, robot_io, cuff,
and navigator. Cuff/navigator writes are not exposed. Most live telemetry uses
sensor-data QoS; static TF uses retained durability. Publication/conversion runs
outside hardware control callbacks. ROS 2 message timestamps use host receipt
or publication time because controller and workstation clocks are not synchronized.

status JSON includes robot readiness/state fields, busy, feedback_age_s, job,
telemetry_errors, telemetry_received, and telemetry_published. Job state can be
idle, running, succeeded, failed, or cancelled. A successful service submission
is not a measured motion result.

### Subscribed command topics (`interfaces=True`)

| Relative name | ROS 2 type | Behavior |
| --- | --- | --- |
| `stream_command` | `irobot_sawyer_interfaces/msg/StreamCommand` | Only applies to the matching active leased JointStream goal |
| `display_image` | `sensor_msgs/msg/Image` | Convert and explicitly publish a valid RGB/BGR head display image |

### Image and conversion helpers

```python
# irobot.src.robots.sawyer.ros2_images
serialize_image(image, stamp)  # wire.Image + sec/nanosec stamp -> XCDR1 bytes
deserialize_image(payload)    # little-endian XCDR1 bytes -> wire.Image

# irobot.src.robots.sawyer.ros2_telemetry
to_ros(message)               # native wire message -> corresponding ROS 2 type
to_wire(message, kind)        # ROS 2 message -> specified native wire type
```

Raw image helpers support efficient rclpy raw=True consumers. Conversion omits
ROS 1 Header.seq and local received_at metadata and maps Intera types to the
custom ROS 2 package.

## ros_sugar adapters

### `SawyerComponent`

Source: [irobot/src/robots/sawyer/ros_sugar_component.py](irobot/src/robots/sawyer/ros_sugar_component.py). Optional application component inheriting ros_sugar BaseComponent. Requires activation before submission and rejects overlapping tasks. The lifecycle execution step stores a completed result or an error dictionary in last_result. It is an alternative hardware owner to SawyerNode.

```python
SawyerComponent(*, component_name='sawyer_application', sawyer_config=None, **kwargs)
```

#### `SawyerComponent.custom_on_configure`

```python
SawyerComponent.custom_on_configure()
```

Connect the owned SawyerController and create a single-worker executor. Return None.

#### `SawyerComponent.custom_on_activate`

```python
SawyerComponent.custom_on_activate()
```

Allow task submissions without starting any movement. Return None.

#### `SawyerComponent.move_to_neutral`

```python
SawyerComponent.move_to_neutral(*, execute=False, duration=None)
```

Submit a neutral task to the single worker. Preview is the default. Returns None immediately; completion is exposed through last_result.

#### `SawyerComponent.open_gripper`

```python
SawyerComponent.open_gripper(*, execute=False)
```

Submit a gripper-open task; preview by default. Returns None immediately.

#### `SawyerComponent.close_gripper`

```python
SawyerComponent.close_gripper(*, execute=False)
```

Submit a gripper-close task; preview by default. Returns None immediately.

#### `SawyerComponent.cancel_motion`

```python
SawyerComponent.cancel_motion()
```

Set the component cancellation event and request driver stop. Return None.

#### `SawyerComponent.custom_on_deactivate`

```python
SawyerComponent.custom_on_deactivate()
```

Disable submissions, request cancellation, and wait boundedly for the current task.

#### `SawyerComponent.custom_on_cleanup`

```python
SawyerComponent.custom_on_cleanup()
```

Deactivate, shut down the worker, and disconnect the driver.

#### `SawyerComponent.custom_on_shutdown`

```python
SawyerComponent.custom_on_shutdown()
```

Perform component cleanup.

#### `SawyerComponent.custom_on_error`

```python
SawyerComponent.custom_on_error()
```

Request cancellation and driver stop.

### CrazyflieDemo

Source: [crazyflie_ros_component.py](irobot/src/robots/crazyflie/examples/crazyflie_ros_component.py).

```python
CrazyflieDemo(*, component_name: str, config: CrazyflieConfig, **kwargs)
```

Example ros_sugar BaseComponent. Owns a CrazyflieController and a cflib
PositionHlCommander; its internal execute-once hook flies a fixed four-waypoint
path and lands. It defines no additional general-purpose public command methods.

## Crazyflie Python API

CrazyflieController inherits CrazyflieBase and its LighthouseLogging mixin.
Unlike Sawyer, CrazyflieBase construction initializes cflib radio drivers and
calls connect(); the connection callback also sends an arming request and starts
Lighthouse logging. These APIs do not implement Sawyer's execute=False preview
contract. State fields reflect the latest logged sample rather than Sawyer's
per-field freshness model.

### `CrazyflieBase`

Source: [irobot/src/robots/crazyflie/core/base.py](irobot/src/robots/crazyflie/core/base.py).

```python
CrazyflieBase(config: CrazyflieConfig | None=None) -> None
```

#### `CrazyflieBase.connect`

```python
CrazyflieBase.connect() -> bool
```

Open the configured cflib radio link, attach callbacks, and wait up to connect_timeout. Return the connection boolean; exceptions are logged and return False.

#### `CrazyflieBase.disconnect`

```python
CrazyflieBase.disconnect() -> None
```

Close the cflib link and clear is_connected. Return None.

#### `CrazyflieBase.arm`

```python
CrazyflieBase.arm() -> None
```

Send an explicit arming request when connected. Return None.

#### `CrazyflieBase.disarm`

```python
CrazyflieBase.disarm() -> None
```

Send an explicit disarming request when connected. Return None.

#### `CrazyflieBase.get_status`

```python
CrazyflieBase.get_status() -> dict[str, Any]
```

Return a dictionary with uri, connected, and a wall-clock timestamp.

### Crazyflie state attributes

| Attribute | Meaning |
| --- | --- |
| `config` | CrazyflieConfig instance |
| `cf` | Underlying cflib Crazyflie object, or None |
| `is_connected` | Connection boolean attribute, not a method |
| `current_x`, `current_y`, `current_z` | Latest Lighthouse position, metres |
| `current_vx`, `current_vy` | Latest velocity, m/s |
| `current_yaw` | Latest stateEstimate.yaw value in the SDK's units |
| `state_ready` | True after a Lighthouse state callback has run |

LighthouseLogging._publish_state() is an intentionally empty subclass extension
hook called after each state update. It is not a public command API.

### `CrazyflieController`

Source: [irobot/src/robots/crazyflie/core/controller.py](irobot/src/robots/crazyflie/core/controller.py). Inherits CrazyflieBase(config=None); construction connects automatically. These methods send commands directly and do not return trajectory/result objects.

#### `CrazyflieController.send_setpoint`

```python
CrazyflieController.send_setpoint(roll: float, pitch: float, yaw_rate: float, thrust: int) -> None
```

Delegate raw roll, pitch, yaw-rate, and thrust to cflib.commander.send_setpoint. Preserve SDK units. Logs a warning when disconnected; return None.

#### `CrazyflieController.send_velocity_setpoint`

```python
CrazyflieController.send_velocity_setpoint(vx: float, vy: float, yaw_rate: float, z_hold: float) -> None
```

Send vx/vy in m/s at fixed z_hold metres using send_hover_setpoint; yaw_rate follows SDK units. Logs a warning when disconnected; return None.

#### `CrazyflieController.ramp_motors`

```python
CrazyflieController.ramp_motors() -> None
```

Run the built-in blocking thrust-ramp demonstration, then send zero thrust. No preview. Return None.

#### `CrazyflieController.fly_to`

```python
CrazyflieController.fly_to(x: float, y: float, z: float, yaw: float=0.0, duration: float=2.0) -> None
```

Linearly interpolate from latest measured XYZ to the target in metres using position setpoints at approximately 20 Hz, then send the endpoint. duration is seconds; yaw follows SDK units. Return None.

#### `CrazyflieController.fly_from_to`

```python
CrazyflieController.fly_from_to(start: tuple[float, float, float], end: tuple[float, float, float], travel_time: float=6.0, hover_time: float=3.0) -> None
```

Run the blocking built-in takeoff, hover, start-to-end flight, hover, and landing sequence. start/end are XYZ tuples in metres; travel_time/hover_time are seconds. Return None.

#### `CrazyflieController.takeoff`

```python
CrazyflieController.takeoff(z_hold: float=0.5, duration: float=3.0) -> None
```

Repeatedly send a hover-height setpoint at approximately 10 Hz for duration seconds. z_hold is metres. Return None.

#### `CrazyflieController.land`

```python
CrazyflieController.land(z_hold: float=0.5, duration: float=2.5) -> None
```

Ramp requested hover height down toward 0.05 m over duration seconds, then send zero thrust. Return None.

### `CrazyflieConfig` fields

Source: [irobot/src/robots/crazyflie/config.py](irobot/src/robots/crazyflie/config.py#L7). Mutable dataclass; cflib is required for this driver.

| Field | Type | Default |
| --- | --- | --- |
| `uri` | `str` | `'radio://0/80/2M/E7E7E7E781'` |
| `log_rate_ms` | `int` | `50` |
| `connect_timeout` | `float` | `10.0` |
| `hover_z` | `float` | `0.3` |

## Custom ROS 2 schemas

The following definitions are copied from the current interface source files.
They define the full request/result/feedback fields and constants, including
supporting trajectory/IO messages. For services, `---` separates request and
response. For actions, it separates goal, result, and feedback. Message types
alone do not create endpoints; use the endpoint tables above to find which are
actually exposed. Source definitions require an explicit ROS 2 interface build.

### Services

#### `CheckJointCollision` (srv)

Source: [ros2/irobot_sawyer_interfaces/srv/CheckJointCollision.srv](ros2/irobot_sawyer_interfaces/srv/CheckJointCollision.srv).

```text
float64[] positions
string tip
---
bool success
string detail
bool[] in_collision
```

#### `ForwardKinematics` (srv)

Source: [ros2/irobot_sawyer_interfaces/srv/ForwardKinematics.srv](ros2/irobot_sawyer_interfaces/srv/ForwardKinematics.srv).

```text
float64[] positions
string tip
---
bool success
string detail
geometry_msgs/PoseStamped[] poses
```

#### `InverseKinematics` (srv)

Source: [ros2/irobot_sawyer_interfaces/srv/InverseKinematics.srv](ros2/irobot_sawyer_interfaces/srv/InverseKinematics.srv).

```text
geometry_msgs/PoseStamped pose
float64[7] seed
string tip
---
bool success
string detail
float64[7] position
```

#### `ReadParameter` (srv)

Source: [ros2/irobot_sawyer_interfaces/srv/ReadParameter.srv](ros2/irobot_sawyer_interfaces/srv/ReadParameter.srv).

```text
string name
---
bool success
string detail
string json_value
```

### Actions

#### `CalibrationCommand` (action)

Source: [ros2/irobot_sawyer_interfaces/action/CalibrationCommand.action](ros2/irobot_sawyer_interfaces/action/CalibrationCommand.action).

```text
# Generated from the pinned public Intera schema.
string CALIBRATION_START=start
string CALIBRATION_STOP=stop
string command
---
# Generated from the pinned public Intera schema.
string SUCCESS=Success
string STOPPED=Stopped
string GENERIC_FAILURE=calibrationFailure
string INCOMPLETE=incomplete
string GRIPPER_ON=cannotCalibrateWithGripper
string TORQUES_EXCEEDED_THRESHOLD=torquesExceededThreshold
string PLANNER_FAILURE=plannerFailure
bool result
string status_id
---
# Generated from the pinned public Intera schema.
string current_state
uint32 number_of_poses
uint32 current_pose_number
```

#### `InteractionSession` (action)

Source: [ros2/irobot_sawyer_interfaces/action/InteractionSession.action](ros2/irobot_sawyer_interfaces/action/InteractionSession.action).

```text
irobot_sawyer_interfaces/InteractionControlCommand command
float64 duration
---
bool success
bool activation_confirmed
bool deactivation_confirmed
string detail
---
float64 elapsed
float64[7] position
float64[7] velocity
```

#### `JointStream` (action)

Source: [ros2/irobot_sawyer_interfaces/action/JointStream.action](ros2/irobot_sawyer_interfaces/action/JointStream.action).

```text
uint8 POSITION=1
uint8 VELOCITY=2
uint8 TORQUE=3
uint8 TRAJECTORY=4
uint8 mode
float64 duration
float64 rate
float64 max_velocity
float64 max_acceleration
float64 max_displacement
float64 effort_fraction
float64 position_speed
float64 lease_timeout
---
bool success
string detail
uint64 samples
float64 max_command_gap
---
string session_id
float64 elapsed
float64[7] position
float64[7] velocity
```

#### `MotionCommand` (action)

Source: [ros2/irobot_sawyer_interfaces/action/MotionCommand.action](ros2/irobot_sawyer_interfaces/action/MotionCommand.action).

```text
# Generated from the pinned public Intera schema.
string MOTION_START=start
string MOTION_STOP=stop
string MOTION_GENERATE=generate
string command
irobot_sawyer_interfaces/Trajectory trajectory
---
# Generated from the pinned public Intera schema.
string FAILED_TO_PARAMETERIZE=FAILED_TO_PARAMETERIZE
string PLANNED_MOTION_COLLISION=PLANNED_MOTION_COLLISION
string INVALID_TRAJECTORY_MESSAGE=INVALID_TRAJECTORY_MESSAGE
string ENDPOINT_DOES_NOT_EXIST=ENDPOINT_DOES_NOT_EXIST
string CARTESIAN_INTERPOLATION_FAILED=CARTESIAN_INTERPOLATION_FAILED
string FINAL_POSE_NOT_WITHIN_TOLERANCE=FINAL_POSE_NOT_WITHIN_TOLERANCE
string CONTROLLER_NOT_FOLLOWING=CONTROLLER_NOT_FOLLOWING
string ZERO_G_ACTIVATED_DURING_TRAJECTORY=ZERO_G_ACTIVATED_DURING_TRAJECTORY
string PLANNED_JOINT_ACCEL_LIMIT=PLANNED_JOINT_ACCEL_LIMIT
int32 HAVE_NOT_REACHED_FIRST_WAYPOINT=-1
int32 GENERAL_TRAJECTORY_FAILURE=-2
bool result
string error_id
irobot_sawyer_interfaces/TrajectoryAnalysis trajectory_analysis
int32 last_successful_waypoint
---
# Generated from the pinned public Intera schema.
irobot_sawyer_interfaces/MotionStatus status
```

#### `MoveCartesian` (action)

Source: [ros2/irobot_sawyer_interfaces/action/MoveCartesian.action](ros2/irobot_sawyer_interfaces/action/MoveCartesian.action).

```text
geometry_msgs/PoseStamped target
float64 duration
float64 speed
float64 max_acceleration
---
bool success
string detail
---
geometry_msgs/PoseStamped actual
```

### Messages

#### `CalibrationCommandFeedback` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/CalibrationCommandFeedback.msg](ros2/irobot_sawyer_interfaces/msg/CalibrationCommandFeedback.msg).

```text
# Generated from the pinned public Intera schema.
string current_state
uint32 number_of_poses
uint32 current_pose_number
```

#### `CalibrationCommandGoal` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/CalibrationCommandGoal.msg](ros2/irobot_sawyer_interfaces/msg/CalibrationCommandGoal.msg).

```text
# Generated from the pinned public Intera schema.
string CALIBRATION_START=start
string CALIBRATION_STOP=stop
string command
```

#### `CalibrationCommandResult` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/CalibrationCommandResult.msg](ros2/irobot_sawyer_interfaces/msg/CalibrationCommandResult.msg).

```text
# Generated from the pinned public Intera schema.
string SUCCESS=Success
string STOPPED=Stopped
string GENERIC_FAILURE=calibrationFailure
string INCOMPLETE=incomplete
string GRIPPER_ON=cannotCalibrateWithGripper
string TORQUES_EXCEEDED_THRESHOLD=torquesExceededThreshold
string PLANNER_FAILURE=plannerFailure
bool result
string status_id
```

#### `DigitalIOState` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/DigitalIOState.msg](ros2/irobot_sawyer_interfaces/msg/DigitalIOState.msg).

```text
# Generated from the pinned public Intera schema.
int8 OFF = 0
int8 ON  = 1
int8 PRESSED = 1
int8 UNPRESSED = 0
int8 state
bool is_input_only
```

#### `EndpointState` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/EndpointState.msg](ros2/irobot_sawyer_interfaces/msg/EndpointState.msg).

```text
# Generated from the pinned public Intera schema.
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Wrench wrench
bool valid
```

#### `EndpointStates` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/EndpointStates.msg](ros2/irobot_sawyer_interfaces/msg/EndpointStates.msg).

```text
# Generated from the pinned public Intera schema.
std_msgs/Header header
string[] names
irobot_sawyer_interfaces/EndpointState[] states
```

#### `IOComponentConfiguration` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/IOComponentConfiguration.msg](ros2/irobot_sawyer_interfaces/msg/IOComponentConfiguration.msg).

```text
# Generated from the pinned public Intera schema.
string name
string config
```

#### `IOComponentStatus` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/IOComponentStatus.msg](ros2/irobot_sawyer_interfaces/msg/IOComponentStatus.msg).

```text
# Generated from the pinned public Intera schema.
string name
irobot_sawyer_interfaces/IOStatus status
```

#### `IODataStatus` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/IODataStatus.msg](ros2/irobot_sawyer_interfaces/msg/IODataStatus.msg).

```text
# Generated from the pinned public Intera schema.
string name
string format
string data
irobot_sawyer_interfaces/IOStatus status
```

#### `IODeviceConfiguration` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/IODeviceConfiguration.msg](ros2/irobot_sawyer_interfaces/msg/IODeviceConfiguration.msg).

```text
# Generated from the pinned public Intera schema.
builtin_interfaces/Time time
string commanded
string upgraded
irobot_sawyer_interfaces/IOComponentConfiguration device
irobot_sawyer_interfaces/IOComponentConfiguration[] ports
irobot_sawyer_interfaces/IOComponentConfiguration[] signals
```

#### `IODeviceStatus` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/IODeviceStatus.msg](ros2/irobot_sawyer_interfaces/msg/IODeviceStatus.msg).

```text
# Generated from the pinned public Intera schema.
builtin_interfaces/Time time
irobot_sawyer_interfaces/IOComponentStatus device
irobot_sawyer_interfaces/IODataStatus[] ports
irobot_sawyer_interfaces/IODataStatus[] signals
builtin_interfaces/Time[] commands
string[] responses
```

#### `IOStatus` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/IOStatus.msg](ros2/irobot_sawyer_interfaces/msg/IOStatus.msg).

```text
# Generated from the pinned public Intera schema.
string DOWN      = down
string READY     = ready
string BUSY      = busy
string UNREADY   = unready
string ERROR     = error
string tag
string id
string detail
```

#### `InteractionControlCommand` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/InteractionControlCommand.msg](ros2/irobot_sawyer_interfaces/msg/InteractionControlCommand.msg).

```text
# Generated from the pinned public Intera schema.
uint8 IMPEDANCE_MODE=1
uint8 FORCE_MODE=2
uint8 IMPEDANCE_WITH_FORCE_LIMIT_MODE=3
uint8 FORCE_WITH_MOTION_LIMIT_MODE=4
std_msgs/Header header
bool interaction_control_active
float64[] k_impedance
bool[] max_impedance
float64[] d_impedance
float64[] k_nullspace
float64[] force_command
geometry_msgs/Pose interaction_frame
string endpoint_name
bool in_endpoint_frame
bool disable_damping_in_force_control
bool disable_reference_resetting
uint8[] interaction_control_mode
bool rotations_for_constrained_zero_g
```

#### `InteractionControlState` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/InteractionControlState.msg](ros2/irobot_sawyer_interfaces/msg/InteractionControlState.msg).

```text
# Generated from the pinned public Intera schema.
std_msgs/Header header
bool interaction_control_active
float64[] k_impedance
float64[] d_impedance
float64[] endpoint_force_command
string endpoint_name
bool in_endpoint_frame
bool disable_damping_in_force_control
bool disable_reference_resetting
bool rotations_for_constrained_zero_g
```

#### `JointLimits` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/JointLimits.msg](ros2/irobot_sawyer_interfaces/msg/JointLimits.msg).

```text
# Generated from the pinned public Intera schema.
string[] joint_names
float64[] position_lower
float64[] position_upper
float64[] velocity
float64[] accel
float64[] effort
```

#### `MotionCommandFeedback` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/MotionCommandFeedback.msg](ros2/irobot_sawyer_interfaces/msg/MotionCommandFeedback.msg).

```text
# Generated from the pinned public Intera schema.
irobot_sawyer_interfaces/MotionStatus status
```

#### `MotionCommandGoal` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/MotionCommandGoal.msg](ros2/irobot_sawyer_interfaces/msg/MotionCommandGoal.msg).

```text
# Generated from the pinned public Intera schema.
string MOTION_START=start
string MOTION_STOP=stop
string MOTION_GENERATE=generate
string command
irobot_sawyer_interfaces/Trajectory trajectory
```

#### `MotionCommandResult` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/MotionCommandResult.msg](ros2/irobot_sawyer_interfaces/msg/MotionCommandResult.msg).

```text
# Generated from the pinned public Intera schema.
string FAILED_TO_PARAMETERIZE=FAILED_TO_PARAMETERIZE
string PLANNED_MOTION_COLLISION=PLANNED_MOTION_COLLISION
string INVALID_TRAJECTORY_MESSAGE=INVALID_TRAJECTORY_MESSAGE
string ENDPOINT_DOES_NOT_EXIST=ENDPOINT_DOES_NOT_EXIST
string CARTESIAN_INTERPOLATION_FAILED=CARTESIAN_INTERPOLATION_FAILED
string FINAL_POSE_NOT_WITHIN_TOLERANCE=FINAL_POSE_NOT_WITHIN_TOLERANCE
string CONTROLLER_NOT_FOLLOWING=CONTROLLER_NOT_FOLLOWING
string ZERO_G_ACTIVATED_DURING_TRAJECTORY=ZERO_G_ACTIVATED_DURING_TRAJECTORY
string PLANNED_JOINT_ACCEL_LIMIT=PLANNED_JOINT_ACCEL_LIMIT
int32 HAVE_NOT_REACHED_FIRST_WAYPOINT=-1
int32 GENERAL_TRAJECTORY_FAILURE=-2
bool result
string error_id
irobot_sawyer_interfaces/TrajectoryAnalysis trajectory_analysis
int32 last_successful_waypoint
```

#### `MotionStatus` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/MotionStatus.msg](ros2/irobot_sawyer_interfaces/msg/MotionStatus.msg).

```text
# Generated from the pinned public Intera schema.
string MOTION_IDLE=idle
string MOTION_PENDING=pending
string MOTION_RUNNING=running
string MOTION_STOPPING=stopping
string MOTION_DONE=done
string MOTION_PREEMPTED=preempted
string MOTION_ERROR=error
std_msgs/Header header
string motion_status
string current_trajectory
uint32 current_waypoint
uint32 motion_request
```

#### `RobotAssemblyState` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/RobotAssemblyState.msg](ros2/irobot_sawyer_interfaces/msg/RobotAssemblyState.msg).

```text
# Generated from the pinned public Intera schema.
uint8   ESTOP_BUTTON_UNPRESSED = 0
uint8   ESTOP_BUTTON_PRESSED   = 1
uint8   ESTOP_BUTTON_UNKNOWN   = 2
uint8   ESTOP_BUTTON_RELEASED  = 3
uint8  ESTOP_SOURCE_NONE      = 0
uint8  ESTOP_SOURCE_USER      = 1
uint8  ESTOP_SOURCE_UNKNOWN   = 2
uint8  ESTOP_SOURCE_FAULT     = 3
uint8  ESTOP_SOURCE_ENGINE    = 4
bool homed
bool ready
bool enabled
bool stopped
bool error
bool low_voltage
uint8 estop_button
uint8 estop_source
```

#### `StreamCommand` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/StreamCommand.msg](ros2/irobot_sawyer_interfaces/msg/StreamCommand.msg).

```text
std_msgs/Header header
string session_id
uint64 sequence
uint8 mode
string[] names
float64[] position
float64[] velocity
float64[] acceleration
float64[] effort
```

#### `TrackingOptions` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/TrackingOptions.msg](ros2/irobot_sawyer_interfaces/msg/TrackingOptions.msg).

```text
# Generated from the pinned public Intera schema.
bool use_min_time_rate
float64 min_time_rate
bool use_max_time_rate
float64 max_time_rate
float64[] goal_joint_tolerance
bool use_settling_time_at_goal
float64 settling_time_at_goal
```

#### `Trajectory` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/Trajectory.msg](ros2/irobot_sawyer_interfaces/msg/Trajectory.msg).

```text
# Generated from the pinned public Intera schema.
string label
string[] joint_names
irobot_sawyer_interfaces/Waypoint[] waypoints
irobot_sawyer_interfaces/TrajectoryOptions trajectory_options
```

#### `TrajectoryAnalysis` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/TrajectoryAnalysis.msg](ros2/irobot_sawyer_interfaces/msg/TrajectoryAnalysis.msg).

```text
# Generated from the pinned public Intera schema.
float64 planned_duration
float64 measured_duration
float64[] min_angle_command
float64[] max_angle_command
float64[] peak_speed_command
float64[] peak_accel_command
float64[] peak_jerk_command
float64 min_time_rate
float64 max_time_rate
float64[] max_position_error
float64[] max_velocity_error
```

#### `TrajectoryOptions` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/TrajectoryOptions.msg](ros2/irobot_sawyer_interfaces/msg/TrajectoryOptions.msg).

```text
# Generated from the pinned public Intera schema.
string CARTESIAN=CARTESIAN
string JOINT=JOINT
string interpolation_type
bool interaction_control
irobot_sawyer_interfaces/InteractionControlCommand interaction_params
bool nso_start_offset_allowed
bool nso_check_end_offset
irobot_sawyer_interfaces/TrackingOptions tracking_options
builtin_interfaces/Time end_time
float64 path_interpolation_step
```

#### `Waypoint` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/Waypoint.msg](ros2/irobot_sawyer_interfaces/msg/Waypoint.msg).

```text
# Generated from the pinned public Intera schema.
float64[] joint_positions
string active_endpoint
geometry_msgs/PoseStamped pose
irobot_sawyer_interfaces/WaypointOptions options
```

#### `WaypointOptions` (msg)

Source: [ros2/irobot_sawyer_interfaces/msg/WaypointOptions.msg](ros2/irobot_sawyer_interfaces/msg/WaypointOptions.msg).

```text
# Generated from the pinned public Intera schema.
string label
float64 max_joint_speed_ratio
float64[] joint_tolerances
float64[] max_joint_accel
float64 max_linear_speed
float64 max_linear_accel
float64 max_rotational_speed
float64 max_rotational_accel
float64 corner_distance
```

## Native Intera protocol

Native routes are distinct from ROS 2 routes. They are configured by SawyerConfig
and used by the Python backend. The complete direction/type/handler map is in
[TOPIC_MAP.md](irobot/src/robots/sawyer/TOPIC_MAP.md).

| Operation family | Default route |
| --- | --- |
| Master discovery and parameter calls | http://169.254.121.3:11311 |
| Arm command publisher | /robot/limb/right/joint_command |
| Command watchdog | /robot/limb/right/joint_command_timeout |
| Speed ratio | /robot/limb/right/set_speed_ratio |
| Joint/assembly feedback | /robot/joint_states, /robot/state |
| Homing observation used by diagnostics | /robot/homing_states |
| Collision and limits | /robot/limb/right/collision_detection_state, /robot/joint_limits |
| Endpoint/tip feedback | /robot/limb/right/endpoint_state, /robot/limb/right/tip_states |
| IK service | /ExternalTools/right/PositionKinematicsNode/IKService |
| FK service | /ExternalTools/right/PositionKinematicsNode/FKService |
| Native motion action | /motion/motion_command |
| Native calibration action | /calibration_command |
| Interaction command/state | /robot/limb/right/interaction_control_command, /robot/limb/right/interaction_controller/state |
| Enable/reset/stop | /robot/set_super_enable, /robot/set_super_reset, /robot/set_super_stop |
| Gripper IO | /io/end_effector/right_gripper/{state,config,command} |
| Head camera IO/images | /io/internal_camera/head_camera/{state,config,command,image_rect_color,camera_info} |
| Hand camera IO/images | /io/internal_camera/right_hand_camera/{state,config,command,image_rect,camera_info} |
| Robot IO | /io/robot/{robot,cuff,navigator,right_end_of_arm}/state; metadata/commands where supported |
| Head pan/display | /robot/head/head_state, /robot/head/command_head_pan, /robot/head_display |
| Digital IO | /robot/digital_io/{name}/state, /robot/digital_io/command |
| Diagnostics/transforms | /diagnostics, /tf, /tf_static |

Native action clients use identified goal/status/feedback/result/cancel topics.
Cancellation targets only the owned goal. The transport checks ROS type/MD5
handshakes, bounded frame sizes, socket deadlines, registration health, and
command ownership. Callback dispatch is serviced by poll/control, while socket
readers and registration maintenance run on background threads.

### Wire API

Source: [wire.py](irobot/src/robots/sawyer/core/wire.py).

| Callable | Contract |
| --- | --- |
| `message_type(name)` | Build/cache a dataclass for a schema type such as sensor_msgs/JointState |
| `service_type(name)` | Return ros_type, md5sum, Request, and Response for a schema service |
| `serialize(message)` | Encode a native message to ROS 1 binary bytes |
| `deserialize(kind, payload)` | Decode bytes for a schema type name |
| `from_dict(kind, values)` | Construct a wire message recursively from field dictionaries |

wire module attributes resolve schema classes such as wire.Image, wire.Trajectory,
and wire.InteractionControlCommand. These are independent of ROS 2 generated
classes. Field names are normalized to snake_case (time uses sec/nanosec), and
messages include a local received_at field that is not serialized. The complete
bundled schema, including fields, constants, type names, and MD5s, is
[wire_schema.json](irobot/src/robots/sawyer/installation/wire_schema.json).

## Supporting Python callable inventory

These are implementation-level entry points, not additional hardware endpoints.
The inventory includes locally defined non-underscore functions and methods,
plus explicit constructors, outside the main classes already documented above.
It includes planning, feedback, transport, leases, conversion, and adapter
handlers. Private underscore helpers and inherited external methods are omitted.
Use the linked implementation when extending the driver; prefer the public
controller/device APIs for applications.

### `irobot/src/robots/sawyer/core/actions.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `ActionResult.succeeded()` | [line 24](irobot/src/robots/sawyer/core/actions.py#L24) | Implementation helper; see linked source for behavior. |
| `NativeActionClient(node, namespace, action_type, *, server_timeout=2.0)` | [line 37](irobot/src/robots/sawyer/core/actions.py#L37) | Implementation helper; see linked source for behavior. |
| `NativeActionClient.on_status(message)` | [line 64](irobot/src/robots/sawyer/core/actions.py#L64) | Implementation helper; see linked source for behavior. |
| `NativeActionClient.on_result(message)` | [line 72](irobot/src/robots/sawyer/core/actions.py#L72) | Implementation helper; see linked source for behavior. |
| `NativeActionClient.on_feedback(message)` | [line 85](irobot/src/robots/sawyer/core/actions.py#L85) | Implementation helper; see linked source for behavior. |
| `NativeActionClient.wait_ready(timeout)` | [line 89](irobot/src/robots/sawyer/core/actions.py#L89) | Implementation helper; see linked source for behavior. |
| `NativeActionClient.cancel()` | [line 104](irobot/src/robots/sawyer/core/actions.py#L104) | Cancel this goal only; an empty ID must never cancel other clients. |
| `NativeActionClient.run(goal, *, timeout, cancel_event=None, on_feedback=None, health_check=None, cancel_timeout=2.0)` | [line 166](irobot/src/robots/sawyer/core/actions.py#L166) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/core/base.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `receipt_time(message)` | [line 23](irobot/src/robots/sawyer/core/base.py#L23) | Implementation helper; see linked source for behavior. |
| `vector(values, size, name, *, positive=False, nonnegative=False)` | [line 27](irobot/src/robots/sawyer/core/base.py#L27) | Implementation helper; see linked source for behavior. |
| `pose_error(actual, desired)` | [line 78](irobot/src/robots/sawyer/core/base.py#L78) | Implementation helper; see linked source for behavior. |
| `JointFeedback(names)` | [line 104](irobot/src/robots/sawyer/core/base.py#L104) | Implementation helper; see linked source for behavior. |
| `JointFeedback.update(names, positions, velocities, now, efforts=())` | [line 115](irobot/src/robots/sawyer/core/base.py#L115) | Implementation helper; see linked source for behavior. |
| `JointFeedback.snapshot()` | [line 154](irobot/src/robots/sawyer/core/base.py#L154) | Implementation helper; see linked source for behavior. |
| `JointFeedback.fresh_positions(now, timeout)` | [line 166](irobot/src/robots/sawyer/core/base.py#L166) | Position hold needs fresh positions even if velocity feedback expired. |
| `JointFeedback.issues(now, timeout)` | [line 178](irobot/src/robots/sawyer/core/base.py#L178) | Implementation helper; see linked source for behavior. |
| `ordered_values(names, values, label)` | [line 206](irobot/src/robots/sawyer/core/base.py#L206) | Implementation helper; see linked source for behavior. |
| `JointBounds.from_message(message)` | [line 225](irobot/src/robots/sawyer/core/base.py#L225) | Implementation helper; see linked source for behavior. |
| `JointBounds.check_position(positions, label, tolerance=0.0)` | [line 249](irobot/src/robots/sawyer/core/base.py#L249) | Implementation helper; see linked source for behavior. |
| `from_pose(pose, frame)` | [line 262](irobot/src/robots/sawyer/core/base.py#L262) | Implementation helper; see linked source for behavior. |
| `EndpointFeedback(fallback_frame, feedback_timeout, config=None)` | [line 281](irobot/src/robots/sawyer/core/base.py#L281) | Implementation helper; see linked source for behavior. |
| `EndpointFeedback.on_endpoint(message)` | [line 290](irobot/src/robots/sawyer/core/base.py#L290) | Implementation helper; see linked source for behavior. |
| `EndpointFeedback.on_tips(message)` | [line 303](irobot/src/robots/sawyer/core/base.py#L303) | Implementation helper; see linked source for behavior. |
| `EndpointFeedback.read(tip)` | [line 345](irobot/src/robots/sawyer/core/base.py#L345) | Implementation helper; see linked source for behavior. |
| `EndpointFeedback.select(requested=None)` | [line 361](irobot/src/robots/sawyer/core/base.py#L361) | Implementation helper; see linked source for behavior. |
| `InteraKinematics(node, tip, frame, request_timeout=None, planning_timeout=None, health_check=None)` | [line 395](irobot/src/robots/sawyer/core/base.py#L395) | Implementation helper; see linked source for behavior. |
| `InteraKinematics.close()` | [line 421](irobot/src/robots/sawyer/core/base.py#L421) | Implementation helper; see linked source for behavior. |
| `InteraKinematics.wait_ready(timeout=None)` | [line 425](irobot/src/robots/sawyer/core/base.py#L425) | Implementation helper; see linked source for behavior. |
| `InteraKinematics.call(client, request, label)` | [line 449](irobot/src/robots/sawyer/core/base.py#L449) | Implementation helper; see linked source for behavior. |
| `InteraKinematics.inverse(pose, seed)` | [line 474](irobot/src/robots/sawyer/core/base.py#L474) | Implementation helper; see linked source for behavior. |
| `InteraKinematics.forward_results(configurations)` | [line 525](irobot/src/robots/sawyer/core/base.py#L525) | Implementation helper; see linked source for behavior. |
| `InteraKinematics.forward(configurations)` | [line 572](irobot/src/robots/sawyer/core/base.py#L572) | Implementation helper; see linked source for behavior. |
| `read_signals(message)` | [line 588](irobot/src/robots/sawyer/core/base.py#L588) | Implementation helper; see linked source for behavior. |
| `signal_value(signals, name, kind=None)` | [line 608](irobot/src/robots/sawyer/core/base.py#L608) | Implementation helper; see linked source for behavior. |
| `command_args(signals, values)` | [line 635](irobot/src/robots/sawyer/core/base.py#L635) | Implementation helper; see linked source for behavior. |
| `time_key(stamp)` | [line 651](irobot/src/robots/sawyer/core/base.py#L651) | Implementation helper; see linked source for behavior. |
| `SawyerGripper(node, config)` | [line 656](irobot/src/robots/sawyer/core/base.py#L656) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.on_state(message)` | [line 671](irobot/src/robots/sawyer/core/base.py#L671) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.wait_for(predicate, timeout, description)` | [line 681](irobot/src/robots/sawyer/core/base.py#L681) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.wait_for_state(timeout)` | [line 692](irobot/src/robots/sawyer/core/base.py#L692) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.require_ready(action)` | [line 699](irobot/src/robots/sawyer/core/base.py#L699) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.make_message(values)` | [line 733](irobot/src/robots/sawyer/core/base.py#L733) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.send(values)` | [line 742](irobot/src/robots/sawyer/core/base.py#L742) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.completed(action, target)` | [line 774](irobot/src/robots/sawyer/core/base.py#L774) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.wait_completion(action, target)` | [line 823](irobot/src/robots/sawyer/core/base.py#L823) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.request_stop()` | [line 844](irobot/src/robots/sawyer/core/base.py#L844) | Implementation helper; see linked source for behavior. |
| `SawyerGripper.perform(action, *, execute=False, cancel_event=None, position=None)` | [line 855](irobot/src/robots/sawyer/core/base.py#L855) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/core/control.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `JointStream(node, mode, options)` | [line 32](irobot/src/robots/sawyer/core/control.py#L32) | Implementation helper; see linked source for behavior. |
| `JointStream.torque_limits()` | [line 50](irobot/src/robots/sawyer/core/control.py#L50) | Implementation helper; see linked source for behavior. |
| `JointStream.command(values)` | [line 61](irobot/src/robots/sawyer/core/control.py#L61) | Implementation helper; see linked source for behavior. |
| `JointStream.check_displacement(positions)` | [line 102](irobot/src/robots/sawyer/core/control.py#L102) | Implementation helper; see linked source for behavior. |
| `JointStream.check_health(cancel_event)` | [line 110](irobot/src/robots/sawyer/core/control.py#L110) | Implementation helper; see linked source for behavior. |
| `JointStream.prepare(cancel_event)` | [line 128](irobot/src/robots/sawyer/core/control.py#L128) | Implementation helper; see linked source for behavior. |
| `JointStream.run(source, duration, *, cancel_event=None, on_step=None)` | [line 154](irobot/src/robots/sawyer/core/control.py#L154) | Implementation helper; see linked source for behavior. |
| `stream_joint_commands(driver, mode, source, duration, *, options=None, execute=False, cancel_event=None, on_step=None)` | [line 242](irobot/src/robots/sawyer/core/control.py#L242) | Preview or execute through the driver's single operation owner. |

### `irobot/src/robots/sawyer/core/controller.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `QuinticTrajectory(start, end, max_velocity, duration=None, max_acceleration=None)` | [line 60](irobot/src/robots/sawyer/core/controller.py#L60) | Implementation helper; see linked source for behavior. |
| `QuinticTrajectory.sample(elapsed)` | [line 121](irobot/src/robots/sawyer/core/controller.py#L121) | Implementation helper; see linked source for behavior. |
| `JointSequence(start, positions, times, velocity, acceleration)` | [line 149](irobot/src/robots/sawyer/core/controller.py#L149) | Implementation helper; see linked source for behavior. |
| `JointSequence.sample(elapsed)` | [line 179](irobot/src/robots/sawyer/core/controller.py#L179) | Implementation helper; see linked source for behavior. |
| `JointPIDController(size, kp, ki, kd, max_velocity, max_acceleration, integral_limit=0.25, position_lower=None, position_upper=None)` | [line 201](irobot/src/robots/sawyer/core/controller.py#L201) | Implementation helper; see linked source for behavior. |
| `JointPIDController.reset(velocity=None)` | [line 252](irobot/src/robots/sawyer/core/controller.py#L252) | Implementation helper; see linked source for behavior. |
| `JointPIDController.update(reference_position, measured_position, reference_velocity, measured_velocity, dt)` | [line 270](irobot/src/robots/sawyer/core/controller.py#L270) | Implementation helper; see linked source for behavior. |
| `CartesianLine(start, offset_cm, end_orientation=None)` | [line 344](irobot/src/robots/sawyer/core/controller.py#L344) | Implementation helper; see linked source for behavior. |
| `CartesianLine.between(start, end)` | [line 360](irobot/src/robots/sawyer/core/controller.py#L360) | Implementation helper; see linked source for behavior. |
| `CartesianLine.sample(normalized_time)` | [line 373](irobot/src/robots/sawyer/core/controller.py#L373) | Implementation helper; see linked source for behavior. |
| `slerp(start, end, fraction)` | [line 385](irobot/src/robots/sawyer/core/controller.py#L385) | Shortest rotation between unit quaternions, including antipodal inputs. |
| `extrema(coefficients)` | [line 411](irobot/src/robots/sawyer/core/controller.py#L411) | Polynomial extrema over [0,1], including both endpoints. |
| `JointPath(fractions, positions)` | [line 441](irobot/src/robots/sawyer/core/controller.py#L441) | Implementation helper; see linked source for behavior. |
| `JointPath.sample(fraction)` | [line 529](irobot/src/robots/sawyer/core/controller.py#L529) | Implementation helper; see linked source for behavior. |
| `JointPath.validation_fractions()` | [line 559](irobot/src/robots/sawyer/core/controller.py#L559) | Implementation helper; see linked source for behavior. |
| `TimedJointPath(path, max_velocity, max_acceleration, duration=None)` | [line 583](irobot/src/robots/sawyer/core/controller.py#L583) | Implementation helper; see linked source for behavior. |
| `TimedJointPath.sample(elapsed)` | [line 628](irobot/src/robots/sawyer/core/controller.py#L628) | Implementation helper; see linked source for behavior. |
| `CartesianPlanner(inverse, forward, lower, upper, *, sample_step=CartesianMotionOptions.sample_step, path_tolerance=CartesianMotionOptions.path_tolerance, orientation_tolerance=DEFAULT_ORIENTATION_TOLERANCE, max_joint_step=CartesianMotionOptions.max_joint_step, max_samples=CartesianMotionOptions.max_samples, joint_names=None, angular_step=CartesianMotionOptions.angular_step)` | [line 667](irobot/src/robots/sawyer/core/controller.py#L667) | Implementation helper; see linked source for behavior. |
| `CartesianPlanner.checked_joints(values, label)` | [line 717](irobot/src/robots/sawyer/core/controller.py#L717) | Implementation helper; see linked source for behavior. |
| `CartesianPlanner.plan(start_pose, offset_cm, start_joints, max_velocity, max_acceleration, duration=None)` | [line 730](irobot/src/robots/sawyer/core/controller.py#L730) | Implementation helper; see linked source for behavior. |
| `CartesianPlanner.plan_pose(start_pose, end_pose, start_joints, max_velocity, max_acceleration, duration=None, end_joints=None)` | [line 744](irobot/src/robots/sawyer/core/controller.py#L744) | Straight XYZ and shortest SLERP, optionally meeting a specific joint branch. A joint-constrained endpoint biases IK seeds toward that posture and is still subject to continuity, FK and position-limit checks. A branch that cannot connect continuously is rejected rather than jumped to. |
| `CartesianPlanner.plan_line(line, start_joints, max_velocity, max_acceleration, duration=None, end_joints=None)` | [line 772](irobot/src/robots/sawyer/core/controller.py#L772) | Implementation helper; see linked source for behavior. |
| `ParameterTransport(timeout)` | [line 933](irobot/src/robots/sawyer/core/controller.py#L933) | Implementation helper; see linked source for behavior. |
| `ParameterTransport.make_connection(host)` | [line 937](irobot/src/robots/sawyer/core/controller.py#L937) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/core/interaction.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `interaction_command(*, endpoint='right_hand')` | [line 44](irobot/src/robots/sawyer/core/interaction.py#L44) | SDK-compatible maximum-impedance, zero-force starting configuration. |
| `validate_command(message, limits)` | [line 63](irobot/src/robots/sawyer/core/interaction.py#L63) | Implementation helper; see linked source for behavior. |
| `InteractionSession(node, limits, stream_options)` | [line 116](irobot/src/robots/sawyer/core/interaction.py#L116) | Implementation helper; see linked source for behavior. |
| `InteractionSession.on_state(message)` | [line 130](irobot/src/robots/sawyer/core/interaction.py#L130) | Implementation helper; see linked source for behavior. |
| `InteractionSession.state_ready()` | [line 133](irobot/src/robots/sawyer/core/interaction.py#L133) | Implementation helper; see linked source for behavior. |
| `InteractionSession.wait_ready()` | [line 139](irobot/src/robots/sawyer/core/interaction.py#L139) | Implementation helper; see linked source for behavior. |
| `InteractionSession.deactivate()` | [line 154](irobot/src/robots/sawyer/core/interaction.py#L154) | Implementation helper; see linked source for behavior. |
| `InteractionSession.run(message, duration, *, cancel_event=None, on_step=None)` | [line 172](irobot/src/robots/sawyer/core/interaction.py#L172) | Implementation helper; see linked source for behavior. |
| `run_interaction(driver, message, duration, *, execute=False, limits=None, stream_options=None, cancel_event=None, on_step=None)` | [line 235](irobot/src/robots/sawyer/core/interaction.py#L235) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/core/io.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `validate_value(value, formatting, configuration)` | [line 17](irobot/src/robots/sawyer/core/io.py#L17) | Validate scalar/vector JSON values and the device's published ranges. |
| `IODevice(node, namespace, timeout, *, writable_inputs=())` | [line 68](irobot/src/robots/sawyer/core/io.py#L68) | Implementation helper; see linked source for behavior. |
| `IODevice.on_config(message)` | [line 88](irobot/src/robots/sawyer/core/io.py#L88) | Implementation helper; see linked source for behavior. |
| `IODevice.on_state(message)` | [line 91](irobot/src/robots/sawyer/core/io.py#L91) | Implementation helper; see linked source for behavior. |
| `IODevice.wait(predicate, description, cancel_event=None)` | [line 134](irobot/src/robots/sawyer/core/io.py#L134) | Implementation helper; see linked source for behavior. |
| `IODevice.wait_ready()` | [line 144](irobot/src/robots/sawyer/core/io.py#L144) | Implementation helper; see linked source for behavior. |
| `IODevice.snapshot()` | [line 150](irobot/src/robots/sawyer/core/io.py#L150) | Implementation helper; see linked source for behavior. |
| `IODevice.register_callback(name, callback)` | [line 164](irobot/src/robots/sawyer/core/io.py#L164) | Implementation helper; see linked source for behavior. |
| `IODevice.deregister_callback(identifier)` | [line 173](irobot/src/robots/sawyer/core/io.py#L173) | Implementation helper; see linked source for behavior. |
| `IODevice.make_command(values, category='signals')` | [line 176](irobot/src/robots/sawyer/core/io.py#L176) | Implementation helper; see linked source for behavior. |
| `IODevice.write(values, *, category='signals', execute=False, cancel_event=None)` | [line 212](irobot/src/robots/sawyer/core/io.py#L212) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/core/lease.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `CommandLease(session_id, mode, timeout, clock_ns)` | [line 17](irobot/src/robots/sawyer/core/lease.py#L17) | Implementation helper; see linked source for behavior. |
| `CommandLease.accept(message)` | [line 27](irobot/src/robots/sawyer/core/lease.py#L27) | Implementation helper; see linked source for behavior. |
| `CommandLease.get(_elapsed=0.0)` | [line 77](irobot/src/robots/sawyer/core/lease.py#L77) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/core/model.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `urdf_joint_limits(description)` | [line 13](irobot/src/robots/sawyer/core/model.py#L13) | Implementation helper; see linked source for behavior. |
| `effort_limits(description)` | [line 28](irobot/src/robots/sawyer/core/model.py#L28) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/core/motion.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `check_finite(value)` | [line 16](irobot/src/robots/sawyer/core/motion.py#L16) | Implementation helper; see linked source for behavior. |
| `validate_tracking(tracking)` | [line 28](irobot/src/robots/sawyer/core/motion.py#L28) | Implementation helper; see linked source for behavior. |
| `validate_waypoint_options(node, options)` | [line 49](irobot/src/robots/sawyer/core/motion.py#L49) | Implementation helper; see linked source for behavior. |
| `validate_cartesian_waypoint(node, waypoint, *, initial)` | [line 73](irobot/src/robots/sawyer/core/motion.py#L73) | Implementation helper; see linked source for behavior. |
| `validate_trajectory(node, trajectory)` | [line 106](irobot/src/robots/sawyer/core/motion.py#L106) | Check supported joint/Cartesian native paths before any action goal. |
| `run_trajectory(driver, trajectory, *, execute=False, plan_only=False, timeout=60.0, cancel_event=None, on_feedback=None)` | [line 155](irobot/src/robots/sawyer/core/motion.py#L155) | Implementation helper; see linked source for behavior. |
| `joint_trajectory(node, positions, *, speed=0.05, max_acceleration=0.1)` | [line 198](irobot/src/robots/sawyer/core/motion.py#L198) | Build a joint path including the measured start, in SDK joint order. |
| `MotionActions(node, config)` | [line 235](irobot/src/robots/sawyer/core/motion.py#L235) | Implementation helper; see linked source for behavior. |
| `MotionActions.client(name)` | [line 239](irobot/src/robots/sawyer/core/motion.py#L239) | Implementation helper; see linked source for behavior. |
| `MotionActions.check_health()` | [line 259](irobot/src/robots/sawyer/core/motion.py#L259) | Implementation helper; see linked source for behavior. |
| `MotionActions.run(goal, *, name='motion', timeout=60.0, cancel_event=None, on_feedback=None)` | [line 265](irobot/src/robots/sawyer/core/motion.py#L265) | Implementation helper; see linked source for behavior. |
| `run_joint_path(driver, positions, *, execute=False, plan_only=False, speed=0.05, max_acceleration=0.1, timeout=60.0, cancel_event=None, on_feedback=None)` | [line 286](irobot/src/robots/sawyer/core/motion.py#L286) | Preview, generate on the controller, or explicitly execute a joint path. |

### `irobot/src/robots/sawyer/core/transforms.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `TransformCache(node, config)` | [line 13](irobot/src/robots/sawyer/core/transforms.py#L13) | Implementation helper; see linked source for behavior. |
| `TransformCache.update(message, *, static)` | [line 27](irobot/src/robots/sawyer/core/transforms.py#L27) | Implementation helper; see linked source for behavior. |
| `TransformCache.snapshot(*, static=False)` | [line 43](irobot/src/robots/sawyer/core/transforms.py#L43) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/core/transport.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `receive_exact(connection, count, deadline)` | [line 31](irobot/src/robots/sawyer/core/transport.py#L31) | Implementation helper; see linked source for behavior. |
| `receive_frame(connection, deadline, limit=MAX_MESSAGE_BYTES)` | [line 47](irobot/src/robots/sawyer/core/transport.py#L47) | Implementation helper; see linked source for behavior. |
| `frame(payload)` | [line 55](irobot/src/robots/sawyer/core/transport.py#L55) | Implementation helper; see linked source for behavior. |
| `send_header(connection, values)` | [line 59](irobot/src/robots/sawyer/core/transport.py#L59) | Implementation helper; see linked source for behavior. |
| `receive_header(connection, deadline)` | [line 69](irobot/src/robots/sawyer/core/transport.py#L69) | Implementation helper; see linked source for behavior. |
| `TimeoutTransport(timeout)` | [line 95](irobot/src/robots/sawyer/core/transport.py#L95) | Implementation helper; see linked source for behavior. |
| `TimeoutTransport.make_connection(host)` | [line 99](irobot/src/robots/sawyer/core/transport.py#L99) | Implementation helper; see linked source for behavior. |
| `rpc(uri, method, *args, timeout)` | [line 105](irobot/src/robots/sawyer/core/transport.py#L105) | Implementation helper; see linked source for behavior. |
| `Publisher(transport, kind, topic)` | [line 122](irobot/src/robots/sawyer/core/transport.py#L122) | Implementation helper; see linked source for behavior. |
| `Publisher.get_subscription_count()` | [line 127](irobot/src/robots/sawyer/core/transport.py#L127) | Implementation helper; see linked source for behavior. |
| `Publisher.publish(message)` | [line 131](irobot/src/robots/sawyer/core/transport.py#L131) | Implementation helper; see linked source for behavior. |
| `Publisher.remove(connection)` | [line 153](irobot/src/robots/sawyer/core/transport.py#L153) | Implementation helper; see linked source for behavior. |
| `Publisher.close()` | [line 158](irobot/src/robots/sawyer/core/transport.py#L158) | Implementation helper; see linked source for behavior. |
| `close_socket(connection)` | [line 165](irobot/src/robots/sawyer/core/transport.py#L165) | Implementation helper; see linked source for behavior. |
| `Subscription(transport, kind, topic, callback)` | [line 173](irobot/src/robots/sawyer/core/transport.py#L173) | Implementation helper; see linked source for behavior. |
| `Subscription.callback(message)` | [line 180](irobot/src/robots/sawyer/core/transport.py#L180) | Implementation helper; see linked source for behavior. |
| `Subscription.dispatch(callbacks, message)` | [line 184](irobot/src/robots/sawyer/core/transport.py#L184) | Implementation helper; see linked source for behavior. |
| `Subscription.add_observer(callback)` | [line 189](irobot/src/robots/sawyer/core/transport.py#L189) | Implementation helper; see linked source for behavior. |
| `Subscription.retain(key, message)` | [line 201](irobot/src/robots/sawyer/core/transport.py#L201) | Keep feedback for late observers, preserving local receipt times. |
| `Subscription.connected()` | [line 238](irobot/src/robots/sawyer/core/transport.py#L238) | Implementation helper; see linked source for behavior. |
| `Subscription.update(publishers)` | [line 245](irobot/src/robots/sawyer/core/transport.py#L245) | Implementation helper; see linked source for behavior. |
| `Subscription.receive(uri, stopped)` | [line 262](irobot/src/robots/sawyer/core/transport.py#L262) | Implementation helper; see linked source for behavior. |
| `Subscription.receive_stream(uri, stopped)` | [line 284](irobot/src/robots/sawyer/core/transport.py#L284) | Implementation helper; see linked source for behavior. |
| `Subscription.close()` | [line 358](irobot/src/robots/sawyer/core/transport.py#L358) | Implementation helper; see linked source for behavior. |
| `ServiceClient(transport, kind, name)` | [line 363](irobot/src/robots/sawyer/core/transport.py#L363) | Implementation helper; see linked source for behavior. |
| `ServiceClient.wait_for_service(timeout_sec)` | [line 366](irobot/src/robots/sawyer/core/transport.py#L366) | Implementation helper; see linked source for behavior. |
| `ServiceClient.call_async(request)` | [line 374](irobot/src/robots/sawyer/core/transport.py#L374) | Implementation helper; see linked source for behavior. |
| `ServiceClient.call(request)` | [line 377](irobot/src/robots/sawyer/core/transport.py#L377) | Implementation helper; see linked source for behavior. |
| `DirectTransport(config)` | [line 421](irobot/src/robots/sawyer/core/transport.py#L421) | Implementation helper; see linked source for behavior. |
| `DirectTransport.master(method, *args)` | [line 489](irobot/src/robots/sawyer/core/transport.py#L489) | Implementation helper; see linked source for behavior. |
| `DirectTransport.start_thread(target, *args, start=True, **kwargs)` | [line 494](irobot/src/robots/sawyer/core/transport.py#L494) | Implementation helper; see linked source for behavior. |
| `DirectTransport.track_socket(connection)` | [line 503](irobot/src/robots/sawyer/core/transport.py#L503) | Implementation helper; see linked source for behavior. |
| `DirectTransport.untrack_socket(connection)` | [line 510](irobot/src/robots/sawyer/core/transport.py#L510) | Implementation helper; see linked source for behavior. |
| `DirectTransport.create_subscription(kind, topic, callback)` | [line 514](irobot/src/robots/sawyer/core/transport.py#L514) | Implementation helper; see linked source for behavior. |
| `DirectTransport.create_publisher(kind, topic)` | [line 527](irobot/src/robots/sawyer/core/transport.py#L527) | Implementation helper; see linked source for behavior. |
| `DirectTransport.observe(kind, topic, callback)` | [line 537](irobot/src/robots/sawyer/core/transport.py#L537) | Share one wire subscription with additional in-process consumers. |
| `DirectTransport.enqueue(key, callback, message)` | [line 549](irobot/src/robots/sawyer/core/transport.py#L549) | Implementation helper; see linked source for behavior. |
| `DirectTransport.publisher_for(kind, topic)` | [line 557](irobot/src/robots/sawyer/core/transport.py#L557) | Implementation helper; see linked source for behavior. |
| `DirectTransport.remove_observer(topic, callback)` | [line 567](irobot/src/robots/sawyer/core/transport.py#L567) | Implementation helper; see linked source for behavior. |
| `DirectTransport.refresh_registration()` | [line 576](irobot/src/robots/sawyer/core/transport.py#L576) | Restore graph registration; never publish or replay a command. |
| `DirectTransport.check_command_owner()` | [line 611](irobot/src/robots/sawyer/core/transport.py#L611) | Best-effort graph check; ROS 1 provides no exclusive writer lease. |
| `DirectTransport.maintain_registration()` | [line 625](irobot/src/robots/sawyer/core/transport.py#L625) | Implementation helper; see linked source for behavior. |
| `DirectTransport.get_publications(_caller)` | [line 635](irobot/src/robots/sawyer/core/transport.py#L635) | Implementation helper; see linked source for behavior. |
| `DirectTransport.get_subscriptions(_caller)` | [line 643](irobot/src/robots/sawyer/core/transport.py#L643) | Implementation helper; see linked source for behavior. |
| `DirectTransport.get_bus_info(_caller)` | [line 654](irobot/src/robots/sawyer/core/transport.py#L654) | Implementation helper; see linked source for behavior. |
| `DirectTransport.create_client(kind, name)` | [line 690](irobot/src/robots/sawyer/core/transport.py#L690) | Implementation helper; see linked source for behavior. |
| `DirectTransport.destroy_client(_client)` | [line 693](irobot/src/robots/sawyer/core/transport.py#L693) | Implementation helper; see linked source for behavior. |
| `DirectTransport.publisher_update(_caller, topic, publishers)` | [line 697](irobot/src/robots/sawyer/core/transport.py#L697) | Implementation helper; see linked source for behavior. |
| `DirectTransport.request_topic(_caller, topic, protocols)` | [line 703](irobot/src/robots/sawyer/core/transport.py#L703) | Implementation helper; see linked source for behavior. |
| `DirectTransport.remote_shutdown(_caller, _reason)` | [line 710](irobot/src/robots/sawyer/core/transport.py#L710) | Implementation helper; see linked source for behavior. |
| `DirectTransport.accept()` | [line 714](irobot/src/robots/sawyer/core/transport.py#L714) | Implementation helper; see linked source for behavior. |
| `DirectTransport.serve_subscriber(connection)` | [line 729](irobot/src/robots/sawyer/core/transport.py#L729) | Implementation helper; see linked source for behavior. |
| `DirectTransport.spin_once(timeout_sec=0.02)` | [line 773](irobot/src/robots/sawyer/core/transport.py#L773) | Implementation helper; see linked source for behavior. |
| `DirectTransport.get_logger()` | [line 785](irobot/src/robots/sawyer/core/transport.py#L785) | Implementation helper; see linked source for behavior. |
| `DirectTransport.get_clock()` | [line 789](irobot/src/robots/sawyer/core/transport.py#L789) | Implementation helper; see linked source for behavior. |
| `DirectTransport.destroy_node()` | [line 796](irobot/src/robots/sawyer/core/transport.py#L796) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/core/wire.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `field_name(name)` | [line 36](irobot/src/robots/sawyer/core/wire.py#L36) | Use Python field names while preserving the original wire layout. |
| `split_type(name)` | [line 43](irobot/src/robots/sawyer/core/wire.py#L43) | Implementation helper; see linked source for behavior. |
| `default_value(name)` | [line 50](irobot/src/robots/sawyer/core/wire.py#L50) | Implementation helper; see linked source for behavior. |
| `message_type(name)` | [line 66](irobot/src/robots/sawyer/core/wire.py#L66) | Implementation helper; see linked source for behavior. |
| `service_type(name)` | [line 100](irobot/src/robots/sawyer/core/wire.py#L100) | Implementation helper; see linked source for behavior. |
| `encode_value(kind, value)` | [line 110](irobot/src/robots/sawyer/core/wire.py#L110) | Implementation helper; see linked source for behavior. |
| `serialize(message)` | [line 137](irobot/src/robots/sawyer/core/wire.py#L137) | Implementation helper; see linked source for behavior. |
| `Reader(payload)` | [line 146](irobot/src/robots/sawyer/core/wire.py#L146) | Implementation helper; see linked source for behavior. |
| `Reader.take(count)` | [line 153](irobot/src/robots/sawyer/core/wire.py#L153) | Implementation helper; see linked source for behavior. |
| `Reader.integer()` | [line 162](irobot/src/robots/sawyer/core/wire.py#L162) | Implementation helper; see linked source for behavior. |
| `Reader.read(kind)` | [line 165](irobot/src/robots/sawyer/core/wire.py#L165) | Implementation helper; see linked source for behavior. |
| `deserialize(kind, payload)` | [line 194](irobot/src/robots/sawyer/core/wire.py#L194) | Implementation helper; see linked source for behavior. |
| `from_dict(kind, values)` | [line 203](irobot/src/robots/sawyer/core/wire.py#L203) | Rebuild a typed message from an asdict/JSON-compatible mapping. |

### `irobot/src/robots/sawyer/ros2.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `trajectory_points(trajectory)` | [line 36](irobot/src/robots/sawyer/ros2.py#L36) | Validate the accepted subset and reorder all joints canonically. |
| `SawyerNode(driver, *, namespace='sawyer', context=None, cameras=False, interfaces=False)` | [line 91](irobot/src/robots/sawyer/ros2.py#L91) | Implementation helper; see linked source for behavior. |
| `SawyerNode.destroy_node()` | [line 476](irobot/src/robots/sawyer/ros2.py#L476) | Implementation helper; see linked source for behavior. |
| `main()` | [line 491](irobot/src/robots/sawyer/ros2.py#L491) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/ros2_interfaces.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `cartesian_pose(message)` | [line 48](irobot/src/robots/sawyer/ros2_interfaces.py#L48) | Implementation helper; see linked source for behavior. |
| `pose_message(pose)` | [line 62](irobot/src/robots/sawyer/ros2_interfaces.py#L62) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces(node)` | [line 78](irobot/src/robots/sawyer/ros2_interfaces.py#L78) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.goal_callback(name, validate)` | [line 221](irobot/src/robots/sawyer/ros2_interfaces.py#L221) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.execute_callback(kind, operation)` | [line 236](irobot/src/robots/sawyer/ros2_interfaces.py#L236) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.cache_feedback(handle, message)` | [line 286](irobot/src/robots/sawyer/ros2_interfaces.py#L286) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.publish()` | [line 290](irobot/src/robots/sawyer/ros2_interfaces.py#L290) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.validate_stream(request)` | [line 298](irobot/src/robots/sawyer/ros2_interfaces.py#L298) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.stream_options(request)` | [line 311](irobot/src/robots/sawyer/ros2_interfaces.py#L311) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.receive_command(message)` | [line 319](irobot/src/robots/sawyer/ros2_interfaces.py#L319) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.stream(handle, result)` | [line 324](irobot/src/robots/sawyer/ros2_interfaces.py#L324) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.validate_native(request)` | [line 371](irobot/src/robots/sawyer/ros2_interfaces.py#L371) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.native(handle, result)` | [line 376](irobot/src/robots/sawyer/ros2_interfaces.py#L376) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.validate_calibration(request)` | [line 397](irobot/src/robots/sawyer/ros2_interfaces.py#L397) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.calibration(handle, result)` | [line 404](irobot/src/robots/sawyer/ros2_interfaces.py#L404) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.validate_interaction(request)` | [line 425](irobot/src/robots/sawyer/ros2_interfaces.py#L425) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.interaction(handle, result)` | [line 434](irobot/src/robots/sawyer/ros2_interfaces.py#L434) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.validate_cartesian(request)` | [line 460](irobot/src/robots/sawyer/ros2_interfaces.py#L460) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.cartesian(handle, result)` | [line 468](irobot/src/robots/sawyer/ros2_interfaces.py#L468) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.validate_gripper(request)` | [line 481](irobot/src/robots/sawyer/ros2_interfaces.py#L481) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.gripper(handle, result)` | [line 494](irobot/src/robots/sawyer/ros2_interfaces.py#L494) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.query(response, operation, *, error_field='detail')` | [line 511](irobot/src/robots/sawyer/ros2_interfaces.py#L511) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.forward(request, response)` | [line 529](irobot/src/robots/sawyer/ros2_interfaces.py#L529) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.inverse(request, response)` | [line 549](irobot/src/robots/sawyer/ros2_interfaces.py#L549) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.collisions(request, response)` | [line 561](irobot/src/robots/sawyer/ros2_interfaces.py#L561) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.read_parameter(request, response)` | [line 580](irobot/src/robots/sawyer/ros2_interfaces.py#L580) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.inventory(_request, response)` | [line 588](irobot/src/robots/sawyer/ros2_interfaces.py#L588) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.capabilities(_request, response)` | [line 594](irobot/src/robots/sawyer/ros2_interfaces.py#L594) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.digital_outputs(request, response)` | [line 599](irobot/src/robots/sawyer/ros2_interfaces.py#L599) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.camera_callback(camera)` | [line 631](irobot/src/robots/sawyer/ros2_interfaces.py#L631) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.display(message)` | [line 642](irobot/src/robots/sawyer/ros2_interfaces.py#L642) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.io_callback(namespace, *, category='signals')` | [line 656](irobot/src/robots/sawyer/ros2_interfaces.py#L656) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.parameter_value(value)` | [line 709](irobot/src/robots/sawyer/ros2_interfaces.py#L709) | Implementation helper; see linked source for behavior. |
| `ExtendedInterfaces.close()` | [line 730](irobot/src/robots/sawyer/ros2_interfaces.py#L730) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/ros2_telemetry.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `to_ros(message)` | [line 26](irobot/src/robots/sawyer/ros2_telemetry.py#L26) | Copy standard data, omitting ROS 1 Header.seq and local receipt metadata. |
| `to_wire(message, kind)` | [line 70](irobot/src/robots/sawyer/ros2_telemetry.py#L70) | Convert a typed ROS 2 request into the driver's independent types. |
| `SawyerTelemetry(node, driver, *, cameras=False, interfaces=False)` | [line 105](irobot/src/robots/sawyer/ros2_telemetry.py#L105) | Implementation helper; see linked source for behavior. |
| `SawyerTelemetry.observe(source, target, kind, ros_kind)` | [line 226](irobot/src/robots/sawyer/ros2_telemetry.py#L226) | Implementation helper; see linked source for behavior. |
| `SawyerTelemetry.receive(target, message)` | [line 248](irobot/src/robots/sawyer/ros2_telemetry.py#L248) | Implementation helper; see linked source for behavior. |
| `SawyerTelemetry.merge_io_state(target, message)` | [line 273](irobot/src/robots/sawyer/ros2_telemetry.py#L273) | Implementation helper; see linked source for behavior. |
| `SawyerTelemetry.receive_joints(message)` | [line 302](irobot/src/robots/sawyer/ros2_telemetry.py#L302) | Implementation helper; see linked source for behavior. |
| `SawyerTelemetry.joint_efforts(names)` | [line 315](irobot/src/robots/sawyer/ros2_telemetry.py#L315) | Implementation helper; see linked source for behavior. |
| `SawyerTelemetry.publish()` | [line 326](irobot/src/robots/sawyer/ros2_telemetry.py#L326) | Implementation helper; see linked source for behavior. |
| `SawyerTelemetry.publish_io_health(events, receipts, now)` | [line 381](irobot/src/robots/sawyer/ros2_telemetry.py#L381) | Implementation helper; see linked source for behavior. |
| `SawyerTelemetry.close()` | [line 400](irobot/src/robots/sawyer/ros2_telemetry.py#L400) | Implementation helper; see linked source for behavior. |
| `SawyerTelemetry.publish_endpoint(message, stamp)` | [line 405](irobot/src/robots/sawyer/ros2_telemetry.py#L405) | Implementation helper; see linked source for behavior. |

### `irobot/src/robots/sawyer/ros2_images.py`

| Signature | Source | Purpose |
| --- | --- | --- |
| `serialize_image(image, stamp)` | [line 12](irobot/src/robots/sawyer/ros2_images.py#L12) | Encode the standard XCDR1 layout without converting every pixel to Python. rclpy Publisher.publish accepts serialized bytes. Metadata stays identical to sensor_msgs/Image; cameras retain their full resolution and encoding. |
| `deserialize_image(payload)` | [line 45](irobot/src/robots/sawyer/ros2_images.py#L45) | Read a serialized Image efficiently for rclpy raw=True consumers. |
