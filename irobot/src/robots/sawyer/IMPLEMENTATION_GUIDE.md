# Sawyer implementation guide: files, functions and unfinished work

Snapshot: November 13, 2026, branch `sawyer_py`.

This describes the Sawyer implementation in the current working tree, including
its internal helpers, integration files, examples, installation assets, optional
ROS 2 definitions and tests. It does not inventory unrelated Crazyflie code,
downloaded dependencies, generated build trees, or ignored local test scripts.
Named Python functions/methods, including nested helpers and properties, are
listed below. Lambdas, dataclass-generated methods and generated ROS type-support
functions are represented by their owning file/class rather than individual rows.
Source links include definition line numbers for this snapshot.

**Implementation is ahead of physical qualification.** The last recorded physical
state was disabled, software-stopped and in error after unexpected enable movement.
That observation is historical, not a live robot-status query. Head pan is deferred.
Use [unfinished work](UNFINISHED_WORK.md), the [original checklist](DIRECT_DRIVER_PLAN.md)
and [validation evidence](VALIDATION.md) before interpreting an API as qualified.

## Read the architecture first

```text
Your application (for example, move to neutral)
    ↓
SawyerController       task API, preview, planning and bounded execution
    ↓ inherits from
SawyerBase             connection, feedback, model queries, operation ownership
    ↓ uses
DirectTransport + wire XML-RPC discovery and native Intera TCPROS messages
    ↓
Existing Intera controller on Sawyer

Optional ROS 2 SawyerNode ───────→ the same SawyerController
Optional ros_sugar application ─→ its owned SawyerController
```

The irobot template is the `config.py` / `core/base.py` / `core/controller.py`
organization and public imports. Your application chooses tasks and when to run
them. The Sawyer implementation supplies the reusable hardware access under that
application. ROS 2 is optional. The workstation needs no ROS 1 runtime or separate
bridge process; the existing robot-side Intera software still speaks its native
protocol, and the driver implements that protocol directly. This does not replace
Sawyer firmware or provide undocumented private-controller access.

For `robot.move_to_neutral()`, the controller reads the named neutral target and
fresh measured joints, validates bounds, and returns a plan. For explicit
`execute=True`, it reserves the operation owner, tracks that plan under readiness,
feedback, deadline and cancellation checks, then performs mode-specific cleanup.
`SawyerBase` owns the session and measured state; transport/wire code carries the
commands. ROS 2 clients reach the same path through `SawyerNode`.

Construction/imports are offline. Connection does not automatically enable, reset,
calibrate or start cameras. One driver owns hardware commands; overlapping jobs
are rejected. `stop()` requests operation cancellation; `devices.software_stop()`
explicitly latches a controller stop and requires fresh confirmation. Software
checks are not a replacement for the controller's physical safety system.

## Driver, adapters, examples and development functions

Each file states its responsibility and design reason. Function tables include
public APIs and implementation helpers; an underscore indicates an internal API.
Property getters appear as functions because they execute read/validation logic.


### `__init__.py`

[__init__.py](__init__.py)

**What:** Exports SawyerBase, SawyerConfig and SawyerController. **Why:** Give callers stable driver imports without transport knowledge.

No explicitly defined Python functions.


### `config.py`

[config.py](config.py)

**What:** Defines connection settings, units, defaults and operation envelopes. **Why:** Applications and both adapters must agree on one validated configuration.

| Class / data type | Responsibility |
| --- | --- |
| [`MotionOptions`](config.py#L28) | Immutable preview/execution timing, rate, PID and arrival settings. |
| [`CartesianMotionOptions`](config.py#L65) | MotionOptions plus Cartesian sampling, IK/FK and pose tolerances. |
| [`StreamOptions`](config.py#L109) | Finite command-session mode, duration, source timing and motion envelopes. |
| [`SawyerConfig`](config.py#L131) | Offline connection, device and default operation configuration. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`positive`](config.py#L21) | Reject booleans, nonfinite numbers and nonpositive values. | Prevent invalid timing and limit calculations. |
| [`MotionOptions.__post_init__`](config.py#L41) | Validate motion timing, PID gains, rate and tolerances. | Reject invalid requests before execution. |
| [`CartesianMotionOptions.__post_init__`](config.py#L81) | Validate Cartesian sampling, continuity and arrival limits. | Bound IK interpolation and pose tracking. |
| [`StreamOptions.__post_init__`](config.py#L119) | Validate finite stream duration, mode, watchdog and displacement bounds. | Keep streamed commands explicitly bounded. |
| [`SawyerConfig.__post_init__`](config.py#L220) | Validate robot settings, joint names, option types and device limits. | Keep construction offline while catching configuration errors early. |
| [`SawyerConfig._validate_connection`](config.py#L288) | Validate master URI, advertised PC address, timeouts and runtime settings. | Make invalid network configuration actionable before opening sockets. |


### `core/__init__.py`

[core/__init__.py](core/__init__.py)

**What:** Marks the internal driver package. **Why:** Keep internal modules importable without startup actions.

No explicitly defined Python functions.


### `core/actions.py`

[core/actions.py](core/actions.py)

**What:** Implements the native ROS action protocol over direct transport. **Why:** Use Intera action servers without a workstation ROS 1 runtime.

| Class / data type | Responsibility |
| --- | --- |
| [`ActionResult`](core/actions.py#L17) | Terminal action status plus typed result payload. |
| [`NativeActionClient`](core/actions.py#L28) | One identified native action goal and its feedback/result subscriptions. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`ActionResult.succeeded`](core/actions.py#L24) | Report whether the terminal action status is SUCCEEDED. | Do not confuse a received result with successful execution. |
| [`NativeActionClient.__init__`](core/actions.py#L37) | Initialize NativeActionClient state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`NativeActionClient.on_status`](core/actions.py#L64) | Match status updates to the active goal and retain heartbeat timing. | Detect server loss without accepting other clients' goals. |
| [`NativeActionClient.on_result`](core/actions.py#L72) | Capture only the active goal's terminal result. | Prevent unrelated result messages from completing this operation. |
| [`NativeActionClient.on_feedback`](core/actions.py#L85) | Deliver feedback belonging to the active goal. | Preserve per-request progress identity. |
| [`NativeActionClient.wait_ready`](core/actions.py#L89) | Await action routes within a deadline. | Reject unavailable action servers before sending a goal. |
| [`NativeActionClient.cancel`](core/actions.py#L104) | Cancel only this client's nonempty goal ID. | Never issue a global cancel for other applications. |
| [`NativeActionClient._wait_cancel`](core/actions.py#L112) | Poll boundedly for cancellation acknowledgement. | Avoid unbounded shutdown waits. |
| [`NativeActionClient._start`](core/actions.py#L118) | Create a unique goal identity and publish the request. | Prevent goal replay and identity collisions. |
| [`NativeActionClient._await_result`](core/actions.py#L132) | Poll results with health, cancellation, heartbeat and deadline guards. | Bound native operation lifetime. |
| [`NativeActionClient.run`](core/actions.py#L166) | Own one goal from route readiness through result or cancellation cleanup. | Give motion and calibration one consistent action implementation. |


### `core/base.py`

[core/base.py](core/base.py)

**What:** Owns connection lifecycle, shared feedback, readiness, kinematics and gripper protocol. **Why:** Separate connection and measured state from task-level movement.

| Class / data type | Responsibility |
| --- | --- |
| [`CartesianPose`](core/base.py#L50) | Frame-qualified position in metres and xyzw quaternion. |
| [`JointFeedback`](core/base.py#L95) | Per-joint, per-field measured feedback with independent receipt ages. |
| [`MotionResult`](core/base.py#L200) | Measured tracking outcome and execution statistics. |
| [`JointBounds`](core/base.py#L218) | Canonical joint position, velocity and acceleration limits. |
| [`EndpointFeedback`](core/base.py#L275) | Named endpoint observations with frame and freshness metadata. |
| [`InteraKinematics`](core/base.py#L390) | Shared direct FK/IK service clients. |
| [`Signal`](core/base.py#L581) | Decoded IO value and its published type metadata. |
| [`SawyerGripper`](core/base.py#L655) | Electric-gripper protocol, merged state, acknowledgement and settling logic. |
| [`_ArmSession`](core/base.py#L925) | One transport, arm state cache, command routes and bounded tracking loop. |
| [`SawyerBase`](core/base.py#L1466) | Connection ownership and public read-only robot state/model queries. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`receipt_time`](core/base.py#L23) | Return the local monotonic receipt time of feedback. | Detect stale data despite the controller clock mismatch. |
| [`vector`](core/base.py#L27) | Normalize a finite numeric vector of the required length. | Give planners and commands consistent validated inputs. |
| [`CartesianPose.__post_init__`](core/base.py#L55) | Validate position, quaternion and reference frame. | Reject malformed Cartesian targets before IK or motion. |
| [`pose_error`](core/base.py#L78) | Calculate translational and quaternion angular distance. | Measure actual arrival without quaternion-sign ambiguity. |
| [`JointFeedback.__init__`](core/base.py#L104) | Initialize JointFeedback state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`JointFeedback.update`](core/base.py#L115) | Merge relevant joint fields with independent receipt times. | Partial messages must not erase or refresh unrelated joint data. |
| [`JointFeedback.snapshot`](core/base.py#L154) | Return canonical position and velocity snapshots. | Present seven joints in a stable order. |
| [`JointFeedback.fresh_positions`](core/base.py#L166) | Require recent positions independently of velocities. | Allow measured-position hold when velocity feedback has expired. |
| [`JointFeedback.issues`](core/base.py#L178) | Report missing, invalid or stale required joint fields. | Keep readiness based on measured feedback. |
| [`ordered_values`](core/base.py#L206) | Reorder named message values into canonical right-arm order. | Avoid commanding or interpreting the wrong joint. |
| [`JointBounds.from_message`](core/base.py#L225) | Build position, velocity and acceleration bounds from published limits. | Use controller limits rather than guessed values. |
| [`JointBounds.check_position`](core/base.py#L249) | Reject joint positions outside the available bounds. | Validate starts and targets before control. |
| [`from_pose`](core/base.py#L262) | Convert a wire pose and its declared frame into CartesianPose. | Keep geometry independent of ROS runtime types. |
| [`EndpointFeedback.__init__`](core/base.py#L281) | Initialize EndpointFeedback state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`EndpointFeedback.on_endpoint`](core/base.py#L290) | Cache endpoint feedback and receipt metadata. | Make measured endpoint data available to the shared session. |
| [`EndpointFeedback.on_tips`](core/base.py#L303) | Merge explicitly named tip states and parent-header metadata. | Avoid assuming an anonymous pose belongs to the requested tool. |
| [`EndpointFeedback.read`](core/base.py#L345) | Read the selected fresh endpoint pose. | Prevent planning from stale or mismatched feedback. |
| [`EndpointFeedback.select`](core/base.py#L361) | Resolve a named tip and its current state. | Preserve tool and frame identity. |
| [`InteraKinematics.__init__`](core/base.py#L395) | Initialize InteraKinematics state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`InteraKinematics.close`](core/base.py#L421) | Release FK/IK client resources. | Keep connection teardown deterministic. |
| [`InteraKinematics.wait_ready`](core/base.py#L425) | Wait boundedly for the configured kinematics services. | Report unavailable services before planning. |
| [`InteraKinematics.call`](core/base.py#L449) | Issue a bounded service call and check its result. | Keep planning failures distinct from usable solutions. |
| [`InteraKinematics.inverse`](core/base.py#L474) | Request seeded IK for a validated Cartesian pose. | Preserve solution continuity without moving hardware. |
| [`InteraKinematics.forward_results`](core/base.py#L525) | Request batched FK including validity and collision flags. | Validate complete sampled paths efficiently. |
| [`InteraKinematics.forward`](core/base.py#L572) | Extract validated FK poses from controller responses. | Give planners a simple pose-query interface. |
| [`read_signals`](core/base.py#L588) | Decode reported IO signal values and metadata. | Interpret gripper state from the controller's JSON-valued IO protocol. |
| [`signal_value`](core/base.py#L608) | Extract and validate one expected signal value. | Make absent or malformed device state explicit. |
| [`command_args`](core/base.py#L635) | Encode the arguments of a device command. | Keep command JSON construction consistent. |
| [`time_key`](core/base.py#L651) | Reduce a message timestamp to a comparison key. | Match acknowledgements to the command that produced them. |
| [`SawyerGripper.__init__`](core/base.py#L656) | Initialize SawyerGripper state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`SawyerGripper.on_state`](core/base.py#L671) | Merge incoming gripper signals and their timestamps. | Retain change-only feedback for completion checks. |
| [`SawyerGripper.wait_for`](core/base.py#L681) | Poll until a predicate succeeds or the operation expires. | Bound gripper acknowledgement and arrival waits. |
| [`SawyerGripper.wait_for_state`](core/base.py#L692) | Wait for the required gripper state feedback. | Do not operate from missing device state. |
| [`SawyerGripper.require_ready`](core/base.py#L699) | Check connection, calibration, errors and busy state for the requested operation. | Reject unsupported or unsafe device transitions. |
| [`SawyerGripper.make_message`](core/base.py#L733) | Build an identified, timestamped gripper IO command. | Allow acknowledgement matching. |
| [`SawyerGripper.send`](core/base.py#L742) | Publish a command and await the matching acknowledgement. | Distinguish delivery from merely constructing a request. |
| [`SawyerGripper.completed`](core/base.py#L774) | Check operation-specific measured completion criteria. | An accepted command is not proof the gripper reached its target. |
| [`SawyerGripper.wait_completion`](core/base.py#L823) | Require stable measured completion within the timeout. | Avoid returning on a transient target sample. |
| [`SawyerGripper.wait_completion.settled`](core/base.py#L826) | Track continuous satisfaction of the completion predicate. | Late movement or feedback must restart the settling interval. |
| [`SawyerGripper.request_stop`](core/base.py#L844) | Request an explicit gripper stop. | Clean up interrupted device operations. |
| [`SawyerGripper.perform`](core/base.py#L855) | Preview or execute one validated gripper operation with cleanup. | Centralize acknowledgement, cancellation and completion handling. |
| [`_ArmSession.__init__`](core/base.py#L926) | Initialize _ArmSession state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`_ArmSession.__getattr__`](core/base.py#L978) | Delegate transport operations to the owned DirectTransport. | Share one connection across control and feedback helpers. |
| [`_ArmSession.spin_once`](core/base.py#L981) | Dispatch transport callbacks and maintain session feedback. | Let synchronous control service incoming state. |
| [`_ArmSession.on_joints`](core/base.py#L984) | Merge incoming joints and update the session snapshot. | Feed control from canonical measured joints. |
| [`_ArmSession.on_state`](core/base.py#L1002) | Store assembly feedback with receipt metadata. | Gate commands on current controller state. |
| [`_ArmSession.on_collision`](core/base.py#L1005) | Store the latest collision-detection state. | Interrupt control when reported collision conditions prohibit motion. |
| [`_ArmSession.on_limits`](core/base.py#L1011) | Validate and retain reported joint limits. | Keep command bounds aligned with the controller. |
| [`_ArmSession.input_issues`](core/base.py#L1031) | Collect assembly, collision, limit and joint feedback problems. | Provide one readiness explanation to all control modes. |
| [`_ArmSession.fresh`](core/base.py#L1065) | Check an observation against the configured receipt-age bound. | Reject stale controller state. |
| [`_ArmSession.check_ready`](core/base.py#L1068) | Raise when session inputs or controller readiness prohibit arm control. | Keep all motion entry points under the same guard. |
| [`_ArmSession.wait_inputs`](core/base.py#L1100) | Wait boundedly for initial feedback and limits. | Connect without silently proceeding with incomplete state. |
| [`_ArmSession.wait_robot_state`](core/base.py#L1114) | Wait for usable assembly and joint observations. | Permit stopped-state diagnostics without enabling the robot. |
| [`_ArmSession.status_snapshot`](core/base.py#L1127) | Return already-dispatched state without networking or mutation. | Support telemetry without disrupting the control loop. |
| [`_ArmSession.routes_connected`](core/base.py#L1152) | Check required command subscriptions are connected. | Do not treat an undeliverable command as execution. |
| [`_ArmSession.publish_velocity`](core/base.py#L1158) | Publish canonical joint velocity commands. | Provide the tracker with one validated command route. |
| [`_ArmSession.spin_until`](core/base.py#L1168) | Dispatch feedback for an actual monotonic interval. | Busy callbacks must not shorten a required wait. |
| [`_ArmSession.stop`](core/base.py#L1183) | Exit the active arm mode using zero velocity or measured-position hold as appropriate. | Bound normal cleanup without sending motion commands after a controller stop. |
| [`_ArmSession.check_trajectory_limits`](core/base.py#L1231) | Validate planned joint position, velocity and acceleration extrema. | Catch violations between reference samples. |
| [`_ArmSession.track`](core/base.py#L1271) | Track a timed reference with readiness, limits, cancellation and deadline checks. | Execute plans through one bounded control loop. |
| [`_ArmSession.wait_pose`](core/base.py#L1410) | Await fresh endpoint feedback. | Avoid Cartesian operations from an absent pose. |
| [`_ArmSession.planning_health`](core/base.py#L1426) | Check session health during iterative planning. | Cancellation or transport changes must prevent later execution. |
| [`SawyerBase.__init__`](core/base.py#L1476) | Initialize SawyerBase state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`SawyerBase._operation`](core/base.py#L1486) | Acquire the single operation owner; allow an explicit bounded wait for stop. | Prevent concurrent command sources and preserve stop access. |
| [`SawyerBase._open_runtime`](core/base.py#L1502) | Create the direct transport and arm session. | Keep network setup separate from object construction. |
| [`SawyerBase.devices`](core/base.py#L1515) | Lazily expose SawyerDevices on the same driver. | Avoid a second peripheral connection or command owner. |
| [`SawyerBase.capabilities`](core/base.py#L1521) | Load the packaged capability ledger without commanding hardware. | Expose implementation and validation limits to applications. |
| [`SawyerBase.check_version`](core/base.py#L1539) | Compare controller identity with the selected SDK baseline. | Report compatibility without changing the robot. |
| [`SawyerBase.connect`](core/base.py#L1555) | Open feedback and command routes without automatic enable or movement. | Support both ready operation and stopped-controller diagnosis. |
| [`SawyerBase._require_connected`](core/base.py#L1594) | Reject operations without an open session. | Give disconnected calls a clear failure. |
| [`SawyerBase._refresh`](core/base.py#L1599) | Dispatch pending state before a read. | Use the newest available observations. |
| [`SawyerBase.is_connected`](core/base.py#L1606) | Report whether this object owns an open session. | Separate local connection lifecycle from readiness and freshness. |
| [`SawyerBase.joint_positions`](core/base.py#L1613) | Read fresh canonical joint positions in radians. | Supply measured starts and application state. |
| [`SawyerBase.joint_velocities`](core/base.py#L1619) | Read fresh canonical joint velocities in rad/s. | Support tracking and settling checks. |
| [`SawyerBase.joint_efforts`](core/base.py#L1625) | Read independently fresh measured joint efforts in Nm. | Avoid presenting old or unknown torque as current feedback. |
| [`SawyerBase.joint_limits`](core/base.py#L1644) | Return validated published joint bounds. | Expose the same limits used by execution. |
| [`SawyerBase.get_parameter`](core/base.py#L1649) | Read a controller parameter through XML-RPC. | Support configuration discovery without a ROS installation. |
| [`SawyerBase.robot_description`](core/base.py#L1655) | Read the controller's URDF parameter. | Use the installed robot model. |
| [`SawyerBase.joint_effort_limits`](core/base.py#L1659) | Combine available published and URDF effort limits. | Bound experimental torque commands from robot data. |
| [`SawyerBase.endpoint_state`](core/base.py#L1674) | Return the selected complete endpoint state. | Expose more than the endpoint pose alone. |
| [`SawyerBase.get_tip_state`](core/base.py#L1677) | Read a named tip's complete fresh state in its declared frame. | Let applications select an actual advertised endpoint. |
| [`SawyerBase.tip_names`](core/base.py#L1687) | List observed named endpoint identifiers. | Avoid guessing tool names from URDF links. |
| [`SawyerBase.poll`](core/base.py#L1700) | Dispatch feedback and registered callbacks while idle. | Keep direct applications responsive between commands. |
| [`SawyerBase.endpoint_velocity`](core/base.py#L1714) | Return fresh endpoint linear/angular velocity. | Support motion observation in the reported frame. |
| [`SawyerBase.endpoint_effort`](core/base.py#L1731) | Return fresh endpoint force and torque. | Expose measured wrench without claiming force-control qualification. |
| [`SawyerBase.endpoint_pose`](core/base.py#L1748) | Return the selected Cartesian pose in metres and xyzw quaternion. | Give applications a ROS-independent pose representation. |
| [`SawyerBase._kinematics`](core/base.py#L1755) | Lazily create kinematics clients on the current session. | Avoid unnecessary clients and duplicate connections. |
| [`SawyerBase.forward_kinematics`](core/base.py#L1771) | Query validated poses for seven-joint configurations. | Preview geometry without commanding motion. |
| [`SawyerBase.configuration_collisions`](core/base.py#L1782) | Read controller FK collision flags for proposed configurations. | Reject model collisions; this is not external obstacle avoidance. |
| [`SawyerBase.inverse_kinematics`](core/base.py#L1798) | Query a seeded seven-joint solution. | Support Cartesian planning without implicit motion. |
| [`SawyerBase.gripper_state`](core/base.py#L1817) | Return the current standard electric-gripper signals. | Make tool state readable independently of actuation. |
| [`SawyerBase.get_status`](core/base.py#L1839) | Refresh and return state, readiness and issues. | Give applications a single diagnostic snapshot. |
| [`SawyerBase._motion_options`](core/base.py#L1852) | Merge per-call motion options with configured defaults. | Keep preview and execution settings consistent. |
| [`SawyerBase._close_runtime`](core/base.py#L1871) | Close kinematics, devices and transport resources. | Clean up partial failures and normal disconnects. |
| [`SawyerBase.disconnect`](core/base.py#L1886) | Release this driver's connection resources. | Make ownership and teardown explicit. |
| [`SawyerBase.__enter__`](core/base.py#L1891) | Connect when entering a context manager. | Provide scoped connection ownership. |
| [`SawyerBase.__exit__`](core/base.py#L1895) | Disconnect when leaving a context manager, including exceptions. | Prevent leaked sessions after application failure. |


### `core/control.py`

[core/control.py](core/control.py)

**What:** Runs explicitly bounded direct joint command sessions. **Why:** Provide lower-level control with limits, fresh feedback and deterministic mode exit.

| Class / data type | Responsibility |
| --- | --- |
| [`JointStream`](core/control.py#L24) | One finite direct joint-command session with mode-specific limits and cleanup. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`JointStream.__init__`](core/control.py#L32) | Initialize JointStream state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`JointStream.torque_limits`](core/control.py#L50) | Require seven usable effort limits and apply the configured fraction. | Keep experimental torque requests inside model/published envelopes. |
| [`JointStream.command`](core/control.py#L61) | Validate mode-specific vectors, dimensions and numeric limits. | Reject malformed commands before publication. |
| [`JointStream.check_displacement`](core/control.py#L102) | Compare measured joints with the session's displacement envelope. | Bound how far an experimental session may move. |
| [`JointStream.check_health`](core/control.py#L110) | Check readiness, route generation, cancellation, deadline and measured speed. | Abort when the session's assumptions change. |
| [`JointStream.prepare`](core/control.py#L128) | Validate the starting state and configure the bounded command route. | Avoid entering a mode from an invalid start. |
| [`JointStream.run`](core/control.py#L154) | Sample the current source, validate and publish until the bounded session ends. | Do not replay buffered commands or publish after a blocking callback expires. |
| [`stream_joint_commands`](core/control.py#L242) | Preview or execute a stream through the driver's single operation owner. | Keep low-level streaming compatible with the public driver lifecycle. |


### `core/controller.py`

[core/controller.py](core/controller.py)

**What:** Implements planners, tracking primitives and the public task-level arm/gripper API. **Why:** An application states the task while this reusable layer validates and executes it.

| Class / data type | Responsibility |
| --- | --- |
| [`TrajectorySample`](core/controller.py#L40) | Position, velocity and acceleration at a reference time. |
| [`QuinticTrajectory`](core/controller.py#L47) | Seven-joint rest-to-rest quintic segment with analytic derivative bounds. |
| [`JointSequence`](core/controller.py#L146) | Timed rest-to-rest waypoint segments. |
| [`JointPIDController`](core/controller.py#L187) | Velocity feedforward and PID correction with saturation/anti-windup. |
| [`CartesianLine`](core/controller.py#L335) | Straight translation and shortest rotation with quintic time scaling. |
| [`JointPath`](core/controller.py#L430) | C2 piecewise quintic Hermite fit through sampled IK solutions. |
| [`TimedJointPath`](core/controller.py#L574) | Uniform time scaling of a normalized joint path against derivative bounds. |
| [`CartesianPlan`](core/controller.py#L645) | Cartesian reference, timed joint path and validation-error results. |
| [`CartesianPlanner`](core/controller.py#L653) | Seeded IK fitting, interpolated FK validation and path timing. |
| [`CartesianMoveResult`](core/controller.py#L921) | Cartesian preview plan and optional measured execution outcome. |
| [`JointMoveResult`](core/controller.py#L927) | Joint preview plan and optional measured execution outcome. |
| [`ParameterTransport`](core/controller.py#L932) | Timeout-limited XML-RPC connection for named-pose lookup. |
| [`SawyerController`](core/controller.py#L943) | Application-facing movement, gripper and bounded advanced-control API. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`QuinticTrajectory.__init__`](core/controller.py#L60) | Initialize QuinticTrajectory state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`QuinticTrajectory.sample`](core/controller.py#L121) | Evaluate position, velocity and acceleration on a quintic segment. | Start and finish joint moves at rest. |
| [`JointSequence.__init__`](core/controller.py#L149) | Initialize JointSequence state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`JointSequence.sample`](core/controller.py#L179) | Evaluate the appropriate rest-to-rest waypoint segment. | Track a complete timed multi-waypoint request. |
| [`JointPIDController.__init__`](core/controller.py#L201) | Initialize JointPIDController state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`JointPIDController.reset`](core/controller.py#L252) | Reset integrator and previous command state. | Do not carry tracking history into a new operation. |
| [`JointPIDController.update`](core/controller.py#L270) | Combine feedforward and PID errors with velocity/acceleration and position guards. | Track references without integral windup or outward motion at a joint limit. |
| [`CartesianLine.__init__`](core/controller.py#L344) | Initialize CartesianLine state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`CartesianLine.between`](core/controller.py#L360) | Build a straight Cartesian reference between two poses. | Use one interpolation contract for relative and absolute goals. |
| [`CartesianLine.sample`](core/controller.py#L373) | Evaluate quintic translation and shortest quaternion interpolation. | Produce smooth Cartesian reference samples. |
| [`slerp`](core/controller.py#L385) | Interpolate the shortest rotation between unit quaternions. | Handle antipodal quaternion representations consistently. |
| [`extrema`](core/controller.py#L411) | Find polynomial extrema over the complete normalized interval. | Validate limits between knots, not only at endpoints. |
| [`JointPath.__init__`](core/controller.py#L441) | Initialize JointPath state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`JointPath.sample`](core/controller.py#L529) | Evaluate a quintic Hermite joint path and derivatives. | Preserve derivative continuity between IK samples. |
| [`JointPath.validation_fractions`](core/controller.py#L559) | Return knots and additional extrema/check fractions. | Validate interpolated paths between the fitted samples. |
| [`TimedJointPath.__init__`](core/controller.py#L583) | Initialize TimedJointPath state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`TimedJointPath.sample`](core/controller.py#L628) | Convert normalized path derivatives to timed references. | Apply velocity and acceleration bounds through time scaling. |
| [`CartesianPlanner.__init__`](core/controller.py#L667) | Initialize CartesianPlanner state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`CartesianPlanner.checked_joints`](core/controller.py#L717) | Validate a candidate seven-joint IK solution against bounds. | Reject malformed and out-of-range kinematics results. |
| [`CartesianPlanner.plan`](core/controller.py#L730) | Plan a Cartesian displacement from a measured start. | Share pose planning for relative motion. |
| [`CartesianPlanner.plan_pose`](core/controller.py#L744) | Plan a straight XYZ path with shortest rotation to a requested pose. | Make the intended tool path explicit. |
| [`CartesianPlanner.plan_line`](core/controller.py#L772) | Sample seeded IK, fit a path, refine FK checks and time-scale it. | Reject discontinuities, collisions and inaccurate interpolation before execution. |
| [`ParameterTransport.__init__`](core/controller.py#L933) | Initialize ParameterTransport state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`ParameterTransport.make_connection`](core/controller.py#L937) | Apply the configured timeout to parameter XML-RPC connections. | Prevent named-pose lookup from hanging indefinitely. |
| [`SawyerController.run_interaction`](core/controller.py#L946) | Preview or execute an explicitly bounded interaction session. | Expose force/impedance through the same operation owner and qualification gates. |
| [`SawyerController.stream_joint_commands`](core/controller.py#L971) | Preview or execute a finite position/velocity/torque/trajectory stream. | Provide direct control without a retained command queue. |
| [`SawyerController.move_native_joint_path`](core/controller.py#L996) | Build and run a path through the Intera native action server. | Offer controller-native planning with explicit execution and cancellation. |
| [`SawyerController.execute_native_trajectory`](core/controller.py#L1023) | Validate and run a typed native joint or Cartesian trajectory. | Expose the native contract without arbitrary private command forwarding. |
| [`SawyerController.calibrate_arm`](core/controller.py#L1049) | Preview or request native arm calibration with attached-tool checks. | Keep homing explicit and reject incompatible gripper configuration. |
| [`SawyerController._neutral_pose`](core/controller.py#L1099) | Resolve configured joints or the controller's named neutral pose. | Avoid hard-coding a universal neutral target. |
| [`SawyerController._move_joints`](core/controller.py#L1126) | Validate starts/targets and create a joint reference. | Centralize joint planning for public movement methods. |
| [`SawyerController._run_joint_plan`](core/controller.py#L1156) | Preview or track an entire validated joint plan. | Keep validation ahead of command publication. |
| [`SawyerController._run_joint_plan.monitor`](core/controller.py#L1207) | Dispatch progress callbacks and recheck operation health. | User callbacks must not bypass cancellation or deadlines. |
| [`SawyerController.move_to_joints`](core/controller.py#L1229) | Preview or execute seven canonical joint angles in radians. | Provide a simple reusable joint-target API. |
| [`SawyerController.move_to_neutral`](core/controller.py#L1252) | Resolve neutral joints and use the normal bounded movement path. | Give applications one task call with the same guards as other moves. |
| [`SawyerController.move_trajectory`](core/controller.py#L1273) | Preview or execute timed rest-to-rest waypoints. | Support sequences while rejecting unsupported derivative semantics. |
| [`SawyerController.move_relative`](core/controller.py#L1320) | Move by base-frame XYZ offsets expressed in centimetres. | Preserve the existing irobot relative-movement interface. |
| [`SawyerController.move_to_pose`](core/controller.py#L1342) | Move to a CartesianPose using validated straight-line planning. | Expose absolute tool targets with explicit frame and units. |
| [`SawyerController._gripper_action`](core/controller.py#L1362) | Route one gripper request through operation ownership and preview handling. | Keep all public gripper methods consistent. |
| [`SawyerController.open_gripper`](core/controller.py#L1375) | Preview or execute opening to the configured separation. | Provide the common open task without duplicating device protocol. |
| [`SawyerController.move_gripper`](core/controller.py#L1378) | Set an explicit separation in metres with acknowledgement and arrival checks. | Support intermediate positions with measured completion. |
| [`SawyerController.close_gripper`](core/controller.py#L1398) | Preview or execute closing the electric gripper. | Provide the common close task with the same cancellation rules. |
| [`SawyerController.calibrate_gripper`](core/controller.py#L1401) | Explicitly request electric-gripper calibration. | Calibration must not happen implicitly on connection. |
| [`SawyerController.stop_gripper`](core/controller.py#L1404) | Explicitly stop the gripper. | Allow interruption independently of arm movement. |
| [`SawyerController.start_gripper`](core/controller.py#L1407) | Resume the gripper's reported position request explicitly. | Keep resumption under application control. |
| [`SawyerController.reboot_gripper`](core/controller.py#L1414) | Power-cycle an idle gripper without automatic recalibration. | Expose recovery while keeping its side effects explicit. |
| [`SawyerController.stop`](core/controller.py#L1418) | Set cancellation for an active arm loop. | Let the owning operation perform mode-specific cleanup. |
| [`SawyerController._move_cartesian`](core/controller.py#L1425) | Plan, preview or track a Cartesian request and check measured arrival. | Share execution and cleanup across Cartesian APIs. |
| [`SawyerController._move_cartesian.monitor`](core/controller.py#L1529) | Record feedback, invoke progress callbacks and recheck health. | Keep Cartesian control responsive to cancellation and faults. |
| [`SawyerController._move_cartesian.at_goal`](core/controller.py#L1548) | Compare measured endpoint error with arrival tolerances. | Do not declare success solely from elapsed trajectory time. |


### `core/interaction.py`

[core/interaction.py](core/interaction.py)

**What:** Defines interaction command validation and timed activation/deactivation. **Why:** Expose limited force/impedance control without hiding state transitions.

| Class / data type | Responsibility |
| --- | --- |
| [`InteractionLimits`](core/interaction.py#L18) | Application wrench, stiffness, timing and displacement envelopes. |
| [`InteractionSession`](core/interaction.py#L108) | Finite interaction activation, monitoring and confirmed normal deactivation. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`InteractionLimits.__post_init__`](core/interaction.py#L28) | Validate wrench, stiffness, timing and displacement envelopes. | Require explicit finite application bounds. |
| [`interaction_command`](core/interaction.py#L44) | Build a maximum-impedance, zero-force initial command. | Offer a conservative explicit starting configuration. |
| [`validate_command`](core/interaction.py#L63) | Check supported frames, axes, numeric fields and experimental opt-in. | Reject unsupported force/compliance requests before activation. |
| [`InteractionSession.__init__`](core/interaction.py#L116) | Initialize InteractionSession state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`InteractionSession.on_state`](core/interaction.py#L130) | Cache interaction state and receipt time. | Confirm controller activation and deactivation from feedback. |
| [`InteractionSession.state_ready`](core/interaction.py#L133) | Check freshness and the expected active state. | Avoid treating stale interaction feedback as confirmation. |
| [`InteractionSession.wait_ready`](core/interaction.py#L139) | Wait boundedly for interaction state. | Fail clearly when state feedback is unavailable. |
| [`InteractionSession.deactivate`](core/interaction.py#L154) | Publish inactive mode and await fresh inactive confirmation. | Make exit explicit instead of assuming an expired Python loop stopped force mode. |
| [`InteractionSession.run`](core/interaction.py#L172) | Execute a timed session under joint/pose/wrench bounds, then deactivate. | Keep interaction under one owner with bounded normal cleanup; crash safety remains unqualified. |
| [`run_interaction`](core/interaction.py#L235) | Preview or execute an interaction request using the driver's operation guard. | Provide the controller's shared entry point. |


### `core/io.py`

[core/io.py](core/io.py)

**What:** Handles typed device IO metadata, merged state, writes, acknowledgements and events. **Why:** Change-only device messages need persistent state and permission-aware writes.

| Class / data type | Responsibility |
| --- | --- |
| [`IODevice`](core/io.py#L60) | One device configuration/state cache, write path and bounded change queue. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`validate_value`](core/io.py#L17) | Validate JSON scalar/vector type, dimensions and published ranges. | Prevent invalid settings from reaching devices. |
| [`IODevice.__init__`](core/io.py#L68) | Initialize IODevice state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`IODevice.on_config`](core/io.py#L88) | Cache the latest device configuration. | Use reported metadata to interpret permissions and values. |
| [`IODevice.on_state`](core/io.py#L91) | Merge changed signals/ports, track health and enqueue bounded change events. | Do not erase unchanged values or grow event memory without limit. |
| [`IODevice.wait`](core/io.py#L134) | Dispatch feedback until a device predicate or deadline. | Bound IO readiness and acknowledgement waits. |
| [`IODevice.wait_ready`](core/io.py#L144) | Require device configuration and connected state. | Avoid writing against missing metadata. |
| [`IODevice.snapshot`](core/io.py#L150) | Return independent state/configuration copies. | Callers must not mutate the driver's live cache. |
| [`IODevice.register_callback`](core/io.py#L164) | Register a callback for selected IO changes. | Let applications react without polling each signal manually. |
| [`IODevice.deregister_callback`](core/io.py#L173) | Remove a registered IO callback. | Support deterministic application cleanup. |
| [`IODevice.make_command`](core/io.py#L176) | Check permissions and values, then build a timestamped IO command. | Keep writes aligned with the published device contract. |
| [`IODevice.write`](core/io.py#L212) | Preview or send a signal/port write and match its acknowledgement. | Expose measured acceptance instead of fire-and-forget settings. |


### `core/lease.py`

[core/lease.py](core/lease.py)

**What:** Tracks one active ROS 2 streaming session and its latest command. **Why:** A stale, reordered or foreign command must not restart physical streaming.

| Class / data type | Responsibility |
| --- | --- |
| [`CommandLease`](core/lease.py#L14) | Latest accepted command for a single identified ROS 2 streaming session. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`CommandLease.__init__`](core/lease.py#L17) | Initialize CommandLease state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`CommandLease.accept`](core/lease.py#L27) | Accept only matching session, increasing sequence and valid source-age commands. | Reject late packets and commands from other sessions. |
| [`CommandLease._validate`](core/lease.py#L52) | Check names, mode, fields, timestamp and lease bounds. | Prevent malformed or delayed source data from reaching the stream. |
| [`CommandLease.get`](core/lease.py#L77) | Return the current command only while its receive/source lease remains valid. | Abort source loss instead of indefinitely repeating an old command. |


### `core/model.py`

[core/model.py](core/model.py)

**What:** Extracts arm bounds from the controller URDF. **Why:** Supplement published limits using the installed robot installed model.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`urdf_joint_limits`](core/model.py#L13) | Parse canonical joint lower/upper limits from URDF XML. | Keep model bounds tied to named joints. |
| [`effort_limits`](core/model.py#L28) | Parse and validate seven URDF joint effort limits. | Supply model-based bounds for torque validation. |


### `core/motion.py`

[core/motion.py](core/motion.py)

**What:** Validates and builds native Intera trajectories and lazily owns action clients. **Why:** Keep native message semantics separate from the public task API.

| Class / data type | Responsibility |
| --- | --- |
| [`MotionActions`](core/motion.py#L232) | Session-scoped lazy native action clients and shared health checks. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`check_finite`](core/motion.py#L16) | Check finite numeric vectors and expected dimensions. | Reject invalid native trajectory fields early. |
| [`validate_tracking`](core/motion.py#L28) | Validate native tracking options and limit fields. | Prevent unsupported controller tracking requests. |
| [`validate_waypoint_options`](core/motion.py#L49) | Validate native waypoint modes and numeric options. | Keep each waypoint within the supported public contract. |
| [`validate_cartesian_waypoint`](core/motion.py#L73) | Validate Cartesian waypoint pose, frame and related fields. | Avoid ambiguous frame or orientation interpretation. |
| [`validate_trajectory`](core/motion.py#L106) | Validate supported native joint/Cartesian paths and limits before a goal. | Preview must reject the same invalid paths as execution. |
| [`run_trajectory`](core/motion.py#L155) | Execute a validated native trajectory with health and cancellation handling. | Centralize typed native execution through one action owner. |
| [`joint_trajectory`](core/motion.py#L198) | Build a native path including measured starting joints. | Require continuity between current pose and the first target. |
| [`MotionActions.__init__`](core/motion.py#L235) | Initialize MotionActions state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`MotionActions.client`](core/motion.py#L239) | Lazily obtain an action client tied to this session. | Avoid duplicate action observers and stale clients after reconnect. |
| [`MotionActions.check_health`](core/motion.py#L259) | Recheck readiness and transport generation during native action waits. | A controller or transport change must abort the old operation. |
| [`MotionActions.run`](core/motion.py#L265) | Run a named native action with common health/cancel/deadline handling. | Reuse action semantics for motion and calibration. |
| [`run_joint_path`](core/motion.py#L286) | Preview, generate or explicitly execute a built native joint path. | Separate controller planning from intentional physical execution. |


### `core/peripherals.py`

[core/peripherals.py](core/peripherals.py)

**What:** Provides explicit robot-state, camera, display, head, digital IO and settings access. **Why:** Share the arm connection and ownership while keeping peripheral operations modular.

| Class / data type | Responsibility |
| --- | --- |
| [`SawyerDevices`](core/peripherals.py#L25) | Lazy peripheral operations sharing SawyerBase connection and ownership. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`SawyerDevices.__init__`](core/peripherals.py#L33) | Initialize SawyerDevices state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`SawyerDevices.transforms`](core/peripherals.py#L42) | Return known static and fresh dynamic TF edges. | Avoid presenting expired transforms as current. |
| [`SawyerDevices._io`](core/peripherals.py#L57) | Lazily create a shared typed IODevice for a known component. | Reuse state/metadata observers across reads and writes. |
| [`SawyerDevices.read_io`](core/peripherals.py#L71) | Read a typed device snapshot. | Expose merged configuration and values without actuation. |
| [`SawyerDevices.write_io`](core/peripherals.py#L79) | Preview or execute a validated signal/port write. | Keep generic settings under permissions and operation ownership. |
| [`SawyerDevices.register_io_callback`](core/peripherals.py#L103) | Register a typed device change observer. | Support event-driven applications on shared feedback. |
| [`SawyerDevices.deregister_io_callback`](core/peripherals.py#L107) | Remove an IO observer. | Release application subscriptions cleanly. |
| [`SawyerDevices.io_events`](core/peripherals.py#L111) | Drain bounded change events and expose drop counts. | Make missed changes observable. |
| [`SawyerDevices.diagnostics`](core/peripherals.py#L120) | Read aggregated controller diagnostics. | Support fault investigation without enabling the robot. |
| [`SawyerDevices.configure_gripper`](core/peripherals.py#L126) | Validate and write supported non-motion gripper settings. | Separate configuration from position/calibration operations. |
| [`SawyerDevices.configure_camera`](core/peripherals.py#L149) | Validate and write camera settings with device acknowledgement. | Keep camera changes explicit and measurable. |
| [`SawyerDevices.set_light`](core/peripherals.py#L172) | Write one of the six documented SDK light signals. | Restrict the input-direction exception to known SDK outputs. |
| [`SawyerDevices.node`](core/peripherals.py#L183) | Require and return the driver's connected session. | Prevent device access through an independent hidden connection. |
| [`SawyerDevices._wait`](core/peripherals.py#L187) | Poll a peripheral predicate under a deadline. | Bound device operations. |
| [`SawyerDevices._subscribe`](core/peripherals.py#L194) | Share or create an observation and its cache callback. | Avoid duplicate wire subscriptions. |
| [`SawyerDevices._read`](core/peripherals.py#L203) | Wait for a usable cached observation and return it. | Provide common freshness handling to peripheral reads. |
| [`SawyerDevices._read.available`](core/peripherals.py#L207) | Check whether the requested cached value is available and fresh. | Do not satisfy a read from old or absent feedback. |
| [`SawyerDevices._publisher`](core/peripherals.py#L218) | Reuse or create one direct publisher for a peripheral command. | Keep publication routes owned by the existing session. |
| [`SawyerDevices.inventory`](core/peripherals.py#L228) | Read graph, parameter and device discovery information. | Make supported and unavailable hardware visible to applications. |
| [`SawyerDevices._robot_state`](core/peripherals.py#L241) | Read and validate fresh assembly feedback. | Gate ordinary state-changing commands on actual controller state. |
| [`SawyerDevices.robot_state`](core/peripherals.py#L253) | Expose the current assembly state. | Allow stopped/disabled diagnostics independently of motion. |
| [`SawyerDevices._robot_command`](core/peripherals.py#L257) | Publish an explicit robot-state request and await its predicate. | Centralize enable/disable/reset acknowledgement handling. |
| [`SawyerDevices._robot_command.step`](core/peripherals.py#L269) | Refresh the command and evaluate fresh response state. | Wait for the requested transition rather than assuming delivery is success. |
| [`SawyerDevices.set_enabled`](core/peripherals.py#L279) | Preview or explicitly enable/disable with readiness and movement guards. | Connecting must never enable the arm implicitly. |
| [`SawyerDevices._enable_robot`](core/peripherals.py#L297) | Capture starting joints, request enable and stop on failure. | Bound unexpected movement while readiness changes. |
| [`SawyerDevices._wait_enabled`](core/peripherals.py#L319) | Require fresh ready feedback and enforce enable displacement limits. | Enable acknowledgement alone is insufficient. |
| [`SawyerDevices.reset_faults`](core/peripherals.py#L361) | Preview or explicitly request controller fault reset. | Keep recovery separate from enable and connection. |
| [`SawyerDevices.software_stop`](core/peripherals.py#L380) | Request cancellation, obtain bounded ownership and latch controller stop. | A busy operation must not silently discard a stop request. |
| [`SawyerDevices._stop_robot`](core/peripherals.py#L400) | Send stop even with stale assembly and require fresh stopped confirmation. | Stale feedback must neither block sending nor falsely confirm stopping. |
| [`SawyerDevices.head_state`](core/peripherals.py#L425) | Read current head feedback. | Retain diagnostic access while head-pan work is deferred. |
| [`SawyerDevices._head_limits`](core/peripherals.py#L431) | Read model head-pan bounds. | Avoid unbounded head targets. |
| [`SawyerDevices.move_head`](core/peripherals.py#L443) | Preview or execute bounded head pan with measured arrival and cleanup. | Retain the existing API; physical qualification is deferred. |
| [`SawyerDevices.move_head.arrived`](core/peripherals.py#L471) | Refresh the head target and check fresh measured pan arrival. | Wait for actual head position rather than command acceptance. |
| [`SawyerDevices._camera`](core/peripherals.py#L518) | Resolve supported camera identifiers and metadata. | Avoid arbitrary device names in camera operations. |
| [`SawyerDevices.camera_state`](core/peripherals.py#L525) | Read a camera's typed state and configuration. | Observe stream ownership without starting it. |
| [`SawyerDevices.register_camera_callback`](core/peripherals.py#L530) | Observe camera frames using the shared transport. | Allow applications to receive images; callbacks must remain fast. |
| [`SawyerDevices.deregister_camera_callback`](core/peripherals.py#L546) | Remove a camera frame observer. | Release consumers without changing stream ownership. |
| [`SawyerDevices.digital_state`](core/peripherals.py#L551) | Read a named digital IO state. | Expose value and input/output permission. |
| [`SawyerDevices._digital_state`](core/peripherals.py#L555) | Resolve and wait for digital feedback. | Share bounded read logic with output writes. |
| [`SawyerDevices.set_digital_output`](core/peripherals.py#L563) | Preview or write a permitted boolean digital output. | Reject attempts to drive input-only signals. |
| [`SawyerDevices.io_state`](core/peripherals.py#L594) | Read raw device status, including change-only and vector fields. | Preserve device detail beyond convenience APIs. |
| [`SawyerDevices.set_camera_streaming`](core/peripherals.py#L605) | Explicitly change streaming and confirm reported state. | Image reads must not silently start hardware streams. |
| [`SawyerDevices.set_camera_streaming.acknowledged`](core/peripherals.py#L624) | Check matching camera command acknowledgement and reported stream state. | Do not declare ownership changed from delivery alone. |
| [`SawyerDevices.camera_frame`](core/peripherals.py#L643) | Read CameraInfo and a currently advancing image without changing streaming. | Exclude retained or repeated pre-request frames. |
| [`SawyerDevices.camera_frame.current`](core/peripherals.py#L663) | Require advancing frame identities and usable CameraInfo after the request. | Exclude retained or duplicate frames from a current-frame read. |
| [`SawyerDevices.display_image`](core/peripherals.py#L690) | Validate and publish an explicitly supplied rgb8/bgr8 image. | Drive the display without cv_bridge or implicit image conversion. |


### `core/transforms.py`

[core/transforms.py](core/transforms.py)

**What:** Caches static and dynamic transform edges. **Why:** Retain static TF while expiring stale moving transforms.

| Class / data type | Responsibility |
| --- | --- |
| [`TransformCache`](core/transforms.py#L12) | Static transform retention and dynamic transform receipt-age filtering. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`TransformCache.__init__`](core/transforms.py#L13) | Initialize TransformCache state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`TransformCache.update`](core/transforms.py#L27) | Merge transform edges and receipt timestamps. | Handle partial TF messages without erasing other edges. |
| [`TransformCache.snapshot`](core/transforms.py#L43) | Copy static and unexpired dynamic edges. | Give consumers a bounded-age view of geometry. |


### `core/transport.py`

[core/transport.py](core/transport.py)

**What:** Implements ROS 1 XML-RPC/TCPROS wire communication using Python standard-library networking. **Why:** Talk directly to Intera without installing ROS 1 or running a bridge.

| Class / data type | Responsibility |
| --- | --- |
| [`TimeoutTransport`](core/transport.py#L94) | Timeout-limited XML-RPC HTTP connections. |
| [`RpcServer`](core/transport.py#L116) | Threaded XML-RPC callback server with nonblocking daemon-worker teardown. |
| [`Publisher`](core/transport.py#L121) | Typed unlatched TCPROS publisher and its current subscriber sockets. |
| [`Subscription`](core/transport.py#L172) | One shared TCPROS observation route with retained feedback and peer workers. |
| [`ServiceClient`](core/transport.py#L356) | Typed bounded TCPROS service request/response client. |
| [`DirectTransport`](core/transport.py#L412) | XML-RPC graph identity, TCPROS routes, resources and callback dispatch. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`receive_exact`](core/transport.py#L31) | Read exactly the requested socket bytes before a deadline. | Handle TCP fragmentation and disconnects explicitly. |
| [`receive_frame`](core/transport.py#L47) | Read a length-prefixed frame with a size limit before payload allocation. | Bound untrusted network messages. |
| [`frame`](core/transport.py#L55) | Prefix a payload with its little-endian TCPROS byte length. | Match the controller's framing protocol. |
| [`send_header`](core/transport.py#L59) | Encode and send bounded TCPROS key/value negotiation fields. | Establish typed topic/service connections. |
| [`receive_header`](core/transport.py#L69) | Decode a bounded header and reject malformed/duplicate/error fields. | Do not accept invalid protocol negotiation. |
| [`TimeoutTransport.__init__`](core/transport.py#L95) | Initialize TimeoutTransport state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`TimeoutTransport.make_connection`](core/transport.py#L99) | Apply an XML-RPC socket timeout. | Bound master and peer RPC waits. |
| [`rpc`](core/transport.py#L105) | Call an XML-RPC method and require its success result code. | Turn remote protocol failures into actionable connection errors. |
| [`Publisher.__init__`](core/transport.py#L122) | Initialize Publisher state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`Publisher.get_subscription_count`](core/transport.py#L127) | Return the connected subscriber count under a lock. | Check command delivery routes before control. |
| [`Publisher.publish`](core/transport.py#L131) | Validate type, serialize and send to current clients without retaining commands. | Never replay old motion to late subscribers. |
| [`Publisher.remove`](core/transport.py#L153) | Remove and close one failed subscriber socket. | Prevent a dead route from remaining active. |
| [`Publisher.close`](core/transport.py#L158) | Close all subscriber connections. | Release publisher resources during teardown. |
| [`close_socket`](core/transport.py#L165) | Shutdown and close a socket, tolerating an already-closed peer. | Make connection cleanup repeatable. |
| [`Subscription.__init__`](core/transport.py#L173) | Initialize Subscription state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`Subscription.callback`](core/transport.py#L180) | Deliver an observation to current callbacks. | Share one wire subscription among consumers. |
| [`Subscription.dispatch`](core/transport.py#L184) | Dispatch queued callbacks only if still registered. | Removed observers must not receive delayed events. |
| [`Subscription.add_observer`](core/transport.py#L189) | Register a consumer and enqueue retained feedback with original receipt time. | Initialize late observers without pretending old data is fresh. |
| [`Subscription.retain`](core/transport.py#L201) | Retain merged IO/static feedback and per-field observations. | Change-only messages must support late consumers without command replay. |
| [`Subscription.connected`](core/transport.py#L238) | Report whether a subscription has live peer workers. | Expose route health to control checks. |
| [`Subscription.update`](core/transport.py#L245) | Reconcile publisher URIs and start/stop peer workers. | Follow graph updates without duplicating connections. |
| [`Subscription.receive`](core/transport.py#L262) | Manage one publisher connection and bounded reconnect attempts. | Recover feedback routes independently of command execution. |
| [`Subscription.receive_stream`](core/transport.py#L284) | Negotiate and decode incoming frames with bounded queueing. | Keep network receipt separate from application callback execution. |
| [`Subscription.close`](core/transport.py#L352) | Stop workers and close subscription sockets. | Terminate background observation cleanly. |
| [`ServiceClient.__init__`](core/transport.py#L357) | Initialize ServiceClient state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`ServiceClient.wait_for_service`](core/transport.py#L360) | Poll service discovery within a timeout. | Avoid unbounded service startup waits. |
| [`ServiceClient.call_async`](core/transport.py#L368) | Run a service call on the transport executor. | Support asynchronous waits without importing ROS clients. |
| [`ServiceClient.call`](core/transport.py#L371) | Negotiate, send and decode one bounded TCPROS service exchange. | Provide FK/IK and other supported service access directly. |
| [`DirectTransport.__init__`](core/transport.py#L415) | Initialize DirectTransport state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`DirectTransport.master`](core/transport.py#L483) | Invoke the configured master with this node's identity. | Keep graph calls consistent. |
| [`DirectTransport.start_thread`](core/transport.py#L488) | Start and retain an owned background worker. | Make worker cleanup trackable. |
| [`DirectTransport.track_socket`](core/transport.py#L497) | Register a socket for bounded global cleanup. | Close even partially initialized connections. |
| [`DirectTransport.untrack_socket`](core/transport.py#L504) | Remove a no-longer-owned socket. | Keep resource accounting current. |
| [`DirectTransport.create_subscription`](core/transport.py#L508) | Create/register a typed topic subscription. | Receive controller feedback through direct TCPROS. |
| [`DirectTransport.create_publisher`](core/transport.py#L521) | Create/register an explicitly unlatched publisher. | Expose command routes without command retention. |
| [`DirectTransport.observe`](core/transport.py#L531) | Add an in-process observer to a shared wire subscription. | Avoid duplicate subscriptions for arm, IO and ROS 2 consumers. |
| [`DirectTransport.enqueue`](core/transport.py#L543) | Coalesce observations into a bounded pending queue and track drops. | Prevent slow consumers from growing memory indefinitely. |
| [`DirectTransport.publisher_for`](core/transport.py#L551) | Reuse a compatible publisher for a topic. | Keep one local publication route per command type/topic. |
| [`DirectTransport.remove_observer`](core/transport.py#L561) | Detach an observation callback. | Allow independent peripheral/telemetry cleanup. |
| [`DirectTransport.refresh_registration`](core/transport.py#L570) | Restore graph registrations after master identity/availability changes. | Recover discovery without publishing or replaying commands. |
| [`DirectTransport.check_command_owner`](core/transport.py#L605) | Check the graph for competing command publishers. | Reject detected ownership conflicts; ROS 1 cannot guarantee exclusive writers. |
| [`DirectTransport.maintain_registration`](core/transport.py#L619) | Periodically check and refresh graph registration. | Detect master changes during polling. |
| [`DirectTransport.get_publications`](core/transport.py#L629) | Return this node's published topic/type pairs for XML-RPC peers. | Implement the required ROS node graph contract. |
| [`DirectTransport.get_subscriptions`](core/transport.py#L637) | Return subscribed topic/type pairs for XML-RPC peers. | Support peer and graph introspection. |
| [`DirectTransport.get_bus_info`](core/transport.py#L648) | Describe active publisher/subscriber connections. | Support protocol-compatible diagnostics. |
| [`DirectTransport.create_client`](core/transport.py#L684) | Create a typed direct service client. | Share supported service calls through this transport. |
| [`DirectTransport.destroy_client`](core/transport.py#L687) | Release service-client compatibility resources. | Keep lifecycle API callers consistent. |
| [`DirectTransport.publisher_update`](core/transport.py#L691) | Handle master publisher-list callbacks. | Connect subscriptions when publishers change. |
| [`DirectTransport.request_topic`](core/transport.py#L697) | Negotiate TCPROS host/port with subscribers. | Let the controller connect to direct publishers. |
| [`DirectTransport.remote_shutdown`](core/transport.py#L704) | Handle a remote node shutdown request. | Release local resources on protocol shutdown. |
| [`DirectTransport.accept`](core/transport.py#L708) | Accept subscriber sockets and dispatch owned connection workers. | Serve published topics concurrently. |
| [`DirectTransport.serve_subscriber`](core/transport.py#L723) | Check topic/type/hash headers and attach a subscriber without replay. | Prevent mismatched peers from receiving commands. |
| [`DirectTransport.spin_once`](core/transport.py#L767) | Maintain registration and dispatch bounded queued callbacks. | Give synchronous applications explicit progress control. |
| [`DirectTransport.get_logger`](core/transport.py#L779) | Return the module logger. | Reuse standard logging without ROS. |
| [`DirectTransport.get_clock`](core/transport.py#L783) | Expose a small timestamp-producing compatibility adapter. | Build protocol messages without importing ROS clock classes. |
| [`DirectTransport.get_clock.stamp`](core/transport.py#L784) | Produce a current wire timestamp. | Populate command identities while deadlines remain monotonic. |
| [`DirectTransport.destroy_node`](core/transport.py#L790) | Unregister graph resources and close servers, workers and sockets. | Clean up both normal and partial-startup sessions. |


### `core/wire.py`

[core/wire.py](core/wire.py)

**What:** Builds portable message types and ROS 1 binary codecs from packaged schemas. **Why:** Native Intera wire compatibility must not require generated ROS Python packages.

| Class / data type | Responsibility |
| --- | --- |
| [`Reader`](core/wire.py#L145) | Size-limited binary decoding cursor over one wire payload. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`field_name`](core/wire.py#L36) | Map wire field names to Python aliases while retaining schema order. | Handle naming differences without changing wire layout. |
| [`split_type`](core/wire.py#L43) | Split scalar, fixed-array and variable-array type declarations. | Drive common schema-based encoding and decoding. |
| [`default_value`](core/wire.py#L50) | Build scalar, nested-message or array defaults. | Construct independent correctly shaped message fields. |
| [`message_type`](core/wire.py#L66) | Cache a dataclass with schema fields, constants, MD5 and definition. | Expose typed messages with no ROS runtime dependency. |
| [`service_type`](core/wire.py#L100) | Build a service descriptor with typed request/response classes. | Support direct service negotiation and serialization. |
| [`encode_value`](core/wire.py#L110) | Recursively encode validated scalar, array and message values. | Match ROS 1 little-endian field layout. |
| [`serialize`](core/wire.py#L137) | Encode a message and enforce maximum payload size. | Bound outbound wire data. |
| [`Reader.__init__`](core/wire.py#L146) | Initialize Reader state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`Reader.take`](core/wire.py#L153) | Read a bounded payload slice and advance the offset. | Reject truncated input before unpacking. |
| [`Reader.integer`](core/wire.py#L162) | Decode an unsigned 32-bit length field. | Share array/string framing logic. |
| [`Reader.read`](core/wire.py#L165) | Decode typed scalar, array and nested-message fields with limits. | Reject invalid booleans, times and excessive arrays. |
| [`deserialize`](core/wire.py#L194) | Decode one typed message and reject trailing bytes. | Do not silently accept malformed payloads. |
| [`from_dict`](core/wire.py#L203) | Rebuild a typed message from a mapping and validate its serialization. | Accept JSON-compatible requests without ignoring unknown fields. |
| [`__getattr__`](core/wire.py#L230) | Resolve packaged Intera messages/services by exported name. | Provide familiar type access without ROS-generated imports. |


### `examples/__init__.py`

[examples/__init__.py](examples/__init__.py)

**What:** Marks the runnable examples package. **Why:** Support python -m example entry points.

No explicitly defined Python functions.


### `examples/diagnose_direct.py`

[examples/diagnose_direct.py](examples/diagnose_direct.py)

**What:** Records diagnostics, joints and input changes using subscriptions only. **Why:** Investigate faults without creating command publishers or changing robot state.

| Class / data type | Responsibility |
| --- | --- |
| [`DiagnosticRecording`](examples/diagnose_direct.py#L24) | Read-only aggregate diagnostics, joint ranges and bounded input transitions. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`DiagnosticRecording.__init__`](examples/diagnose_direct.py#L25) | Initialize DiagnosticRecording state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`DiagnosticRecording.receive`](examples/diagnose_direct.py#L37) | Aggregate sparse diagnostic/status observations by identity. | Retain names absent from later partial messages. |
| [`DiagnosticRecording.receive_joints`](examples/diagnose_direct.py#L50) | Merge joint fields and observed ranges. | Measure state variation without moving hardware. |
| [`DiagnosticRecording.receive_input`](examples/diagnose_direct.py#L66) | Merge IO/legacy inputs and retain bounded transition events. | Distinguish actual button/wheel edges from unchanged samples. |
| [`record`](examples/diagnose_direct.py#L93) | Subscribe for a bounded duration and return state/counts/drops with no publishers. | Provide repeatable read-only fault and input evidence. |
| [`main`](examples/diagnose_direct.py#L135) | Parse connection/duration/report arguments and write diagnostic JSON. | Make the recorder usable independently of a ROS runtime. |


### `examples/move.py`

[examples/move.py](examples/move.py)

**What:** Demonstrates public neutral and relative Cartesian movement. **Why:** Provide a small application without exposing transport internals.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`main`](examples/move.py#L13) | Parse movement settings and call the preview/execute relative-move API. | Show reuse of validated planning instead of direct topic commands. |


### `examples/move_neutral.py`

[examples/move_neutral.py](examples/move_neutral.py)

**What:** Runs the public neutral API with preview as the default. **Why:** Demonstrate the exact application-to-controller task path.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`main`](examples/move_neutral.py#L15) | Validate CLI settings, preview neutral and execute only when requested. | Keep the example's physical side effects explicit. |


### `examples/status.py`

[examples/status.py](examples/status.py)

**What:** Prints direct connection and robot status. **Why:** Provide a minimal non-motion application.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`main`](examples/status.py#L13) | Parse configuration, connect and report measured status. | Check connectivity without enabling or commanding movement. |


### `examples/validate.py`

[examples/validate.py](examples/validate.py)

**What:** Previews or executes a bounded arm round trip and optional empty-gripper checks. **Why:** Keep a focused planning example beside the reusable driver.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`main`](examples/validate.py#L17) | Parse example settings and run the public Cartesian validation workflow. | Exercise the documented planner/controller API with explicit execution options. |


### `examples/validate_direct.py`

[examples/validate_direct.py](examples/validate_direct.py)

**What:** Collects direct-driver validation evidence with optional explicit actuation. **Why:** Separate stopped-state inspection from ready-state movement qualification.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`validate`](examples/validate_direct.py#L17) | Read status/model/planning evidence; optionally run explicitly enabled movement/gripper checks. | Never attempt planning or movement when stopped-state validation is requested. |
| [`main`](examples/validate_direct.py#L63) | Parse connection, execute and report options. | Produce reviewable validation artifacts. |


### `installation/build_capability_ledger.py`

[installation/build_capability_ledger.py](installation/build_capability_ledger.py)

**What:** Maps all indexed public SDK operations to implementation and evidence. **Why:** Separate API coverage from hardware qualification and helper utilities.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`feature`](installation/build_capability_ledger.py#L28) | Build a consistent feature record with interfaces, limits and evidence. | Keep ledger entries structured and comparable. |
| [`build`](installation/build_capability_ledger.py#L772) | Combine the pinned SDK index with feature mappings into the operation ledger. | Account for all 226 indexed methods without claiming all are qualified. |
| [`main`](installation/build_capability_ledger.py#L832) | Write the generated CAPABILITY_LEDGER.json. | Regenerate coverage without ROS or robot access. |


### `installation/generate_ros2_interfaces.py`

[installation/generate_ros2_interfaces.py](installation/generate_ros2_interfaces.py)

**What:** Generates the optional ROS 2 public message/service/action source closure. **Why:** Build typed Humble access from pinned definitions during development, not startup.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`field_name`](installation/generate_ros2_interfaces.py#L33) | Translate source field names to ROS 2-compatible names. | Retain clear correspondence with the wire schema. |
| [`ros2_type`](installation/generate_ros2_interfaces.py#L37) | Translate pinned field types to ROS 2 declarations. | Resolve nested dependencies and builtin time/duration types. |
| [`declaration`](installation/generate_ros2_interfaces.py#L46) | Render one field/constant declaration. | Keep generated interface syntax consistent. |
| [`generate`](installation/generate_ros2_interfaces.py#L61) | Generate the selected definitions and CMake/package metadata. | Produce a buildable optional interface package with provenance. |
| [`main`](installation/generate_ros2_interfaces.py#L228) | Parse generator paths and write interface sources. | Make regeneration explicit and repeatable. |


### `installation/generate_wire_schema.py`

[installation/generate_wire_schema.py](installation/generate_wire_schema.py)

**What:** Captures public ROS 1 definitions and hashes into a portable JSON schema. **Why:** Ship protocol data instead of requiring ROS generators at runtime.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`message_record`](installation/generate_wire_schema.py#L24) | Capture original fields, constants, expanded definition and MD5. | Preserve exact upstream protocol identity. |
| [`action_messages`](installation/generate_wire_schema.py#L34) | Expand a public action into goal/result/feedback wire definitions. | Support native action topics without actionlib runtime imports. |
| [`generate`](installation/generate_wire_schema.py#L68) | Collect the required message/service dependency closure and provenance. | Package only the pinned supported protocol data. |
| [`main`](installation/generate_wire_schema.py#L157) | Read source paths and write wire_schema.json. | Keep SDK/schema generation a development operation. |


### `installation/inventory_sdk.py`

[installation/inventory_sdk.py](installation/inventory_sdk.py)

**What:** Indexes public SDK source methods and provenance. **Why:** Make the full-access discussion traceable to actual SDK operations.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`inventory`](installation/inventory_sdk.py#L16) | Parse SDK Python files and record public classes/methods and source hashes. | Avoid relying on remembered SDK coverage. |
| [`main`](installation/inventory_sdk.py#L58) | Read the source tree and write SDK_INVENTORY.json. | Regenerate the versioned public-method index. |


### `ros2.py`

[ros2.py](ros2.py)

**What:** Exposes the shared Python driver as an optional Humble node. **Why:** ROS 2 applications reuse the same backend rather than adding a workstation bridge.

| Class / data type | Responsibility |
| --- | --- |
| [`SawyerNode`](ros2.py#L83) | Optional standard Humble node owning job admission, feedback and cancellation. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`trajectory_points`](ros2.py#L36) | Validate the supported FollowJointTrajectory subset and reorder joints. | Reject unsupported scheduling, derivatives and malformed waypoints before execution. |
| [`SawyerNode.__init__`](ros2.py#L91) | Initialize SawyerNode state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`SawyerNode._enable`](ros2.py#L201) | Schedule an explicit enable/disable job. | Reuse job ownership while keeping enable separate from connection. |
| [`SawyerNode._software_stop`](ros2.py#L209) | Cancel a busy job, wait boundedly for driver ownership and confirm latched stop. | Stop must not be rejected merely because normal work is running. |
| [`SawyerNode._reserve`](ros2.py#L238) | Atomically reserve one job while excluding stop admission races. | Reject overlap and prevent a new goal from clearing an active stop cancellation. |
| [`SawyerNode._finish`](ros2.py#L257) | Publish terminal job metadata and release job ownership. | Permit the next request only after execution cleanup. |
| [`SawyerNode._neutral`](ros2.py#L263) | Execute neutral through the driver with shared cancellation and feedback. | Preserve one motion implementation for Python and ROS 2. |
| [`SawyerNode._job_service`](ros2.py#L270) | Build an asynchronous Trigger handler for a driver operation. | Return acceptance promptly while reporting completion separately. |
| [`SawyerNode._job_service.callback`](ros2.py#L271) | Reserve and start a worker or reject a busy request. | Never queue or replay overlapping jobs. |
| [`SawyerNode._run_job`](ros2.py#L287) | Run one owned operation and record success/failure/cancellation. | Contain worker exceptions and release job ownership. |
| [`SawyerNode._stop`](ros2.py#L300) | Request cancellation of the current operation. | Keep ordinary motion cancellation distinct from latched software stop. |
| [`SawyerNode._goal`](ros2.py#L309) | Validate and reserve a standard trajectory action goal. | Reject unsupported or overlapping requests before execution. |
| [`SawyerNode._cancel`](ros2.py#L333) | Signal action cancellation and driver stop. | Let active control exit using its mode-specific cleanup. |
| [`SawyerNode._feedback`](ros2.py#L338) | Cache reference and measured samples without DDS publication. | Keep conversion/publication outside control callbacks. |
| [`SawyerNode._execute`](ros2.py#L351) | Run the accepted trajectory and return an action terminal result. | Connect measured driver completion to standard ROS 2 action semantics. |
| [`SawyerNode._poll_state`](ros2.py#L386) | Refresh idle driver state and measured endpoint pose. | Do not contend with an active control operation. |
| [`SawyerNode._publish`](ros2.py#L413) | Drain telemetry, publish fresh joints/action feedback and status. | Separate executor-side ROS work from direct control timing. |
| [`SawyerNode.destroy_node`](ros2.py#L476) | Cancel jobs, close actions/telemetry and release node resources. | Prevent background execution after server shutdown. |
| [`main`](ros2.py#L491) | Parse connection/options, own the ROS context and run/close the server. | Provide an explicit command-line entry point. |


### `ros2_images.py`

[ros2_images.py](ros2_images.py)

**What:** Encodes/decodes standard ROS 2 Image CDR directly. **Why:** Avoid per-pixel Python conversions delaying control leases for large images.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`serialize_image`](ros2_images.py#L12) | Encode standard XCDR1 Image data including alignment and strings. | Publish full-resolution frames without Python pixel-object expansion. |
| [`serialize_image.integer`](ros2_images.py#L21) | Align and append a CDR unsigned integer. | Preserve the standard serialized layout. |
| [`serialize_image.string`](ros2_images.py#L25) | Encode an aligned null-terminated UTF-8 CDR string. | Handle frame IDs and encodings correctly. |
| [`deserialize_image`](ros2_images.py#L45) | Parse a raw serialized Image into efficient field/data values. | Support fast rclpy raw subscribers. |
| [`deserialize_image.integer`](ros2_images.py#L53) | Read an aligned CDR unsigned integer. | Decode size and image metadata fields consistently. |
| [`deserialize_image.string`](ros2_images.py#L60) | Read a CDR UTF-8 string and terminator. | Decode frame IDs and encodings without pixel conversion. |


### `ros2_interfaces.py`

[ros2_interfaces.py](ros2_interfaces.py)

**What:** Adds optional typed actions, model queries and peripheral services. **Why:** Extend standard ROS 2 access only when the generated interface package is installed.

| Class / data type | Responsibility |
| --- | --- |
| [`ExtendedInterfaces`](ros2_interfaces.py#L77) | Optional generated-action/service and peripheral endpoint registration. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`cartesian_pose`](ros2_interfaces.py#L48) | Convert a ROS 2 pose request to validated CartesianPose. | Keep frame and quaternion checks in the driver contract. |
| [`pose_message`](ros2_interfaces.py#L62) | Convert a driver pose into a standard ROS 2 pose message. | Return model-query results to ROS 2 clients. |
| [`ExtendedInterfaces.__init__`](ros2_interfaces.py#L78) | Initialize ExtendedInterfaces state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`ExtendedInterfaces.goal_callback`](ros2_interfaces.py#L221) | Build a validated goal-admission callback. | Share one job reservation policy across custom actions. |
| [`ExtendedInterfaces.goal_callback.accept`](ros2_interfaces.py#L222) | Validate a typed goal and reserve operation ownership. | Reject malformed or overlapping goals before side effects. |
| [`ExtendedInterfaces.execute_callback`](ros2_interfaces.py#L236) | Build an action executor around a driver operation. | Reuse cancellation, result and cleanup handling. |
| [`ExtendedInterfaces.execute_callback.execute`](ros2_interfaces.py#L237) | Run the owned action and set its terminal result. | Report execution failure separately from goal acceptance. |
| [`ExtendedInterfaces.cache_feedback`](ros2_interfaces.py#L286) | Retain the latest custom action feedback. | Keep ROS publication outside direct control callbacks. |
| [`ExtendedInterfaces.publish`](ros2_interfaces.py#L290) | Publish cached custom action feedback on the executor. | Avoid ROS conversion in timing-sensitive driver callbacks. |
| [`ExtendedInterfaces.validate_stream`](ros2_interfaces.py#L298) | Validate stream mode, limits, timing and initial fields. | Reject unsupported low-level sessions before acceptance. |
| [`ExtendedInterfaces.stream_options`](ros2_interfaces.py#L311) | Translate action settings to StreamOptions. | Use identical bounds in Python and ROS 2 streaming. |
| [`ExtendedInterfaces.receive_command`](ros2_interfaces.py#L319) | Offer an incoming StreamCommand to the current lease. | Reject foreign, stale and reordered packets. |
| [`ExtendedInterfaces.stream`](ros2_interfaces.py#L324) | Run a leased bounded joint stream action. | Stop command sourcing when the accepted client disappears. |
| [`ExtendedInterfaces.stream.step`](ros2_interfaces.py#L345) | Cache stream progress/session feedback. | Give the client its session identity and progress without DDS work in control. |
| [`ExtendedInterfaces.validate_native`](ros2_interfaces.py#L371) | Validate a native motion goal. | Apply the supported native path contract before goal acceptance. |
| [`ExtendedInterfaces.native`](ros2_interfaces.py#L376) | Execute a typed native motion request. | Expose Intera motion semantics through the shared driver. |
| [`ExtendedInterfaces.native.feedback`](ros2_interfaces.py#L377) | Cache native action feedback. | Relay native progress without blocking controller callbacks. |
| [`ExtendedInterfaces.validate_calibration`](ros2_interfaces.py#L397) | Validate calibration settings and preconditions. | Keep attached-tool and command restrictions explicit. |
| [`ExtendedInterfaces.calibration`](ros2_interfaces.py#L404) | Run explicit native arm calibration. | Never home the robot as a hidden setup step. |
| [`ExtendedInterfaces.calibration.feedback`](ros2_interfaces.py#L405) | Cache calibration progress. | Publish typed action feedback outside control callbacks. |
| [`ExtendedInterfaces.validate_interaction`](ros2_interfaces.py#L425) | Validate interaction command, duration and experimental mode permission. | Reject unbounded or unsupported force requests early. |
| [`ExtendedInterfaces.interaction`](ros2_interfaces.py#L434) | Run a bounded interaction action. | Reuse the driver's activation/deactivation checks. |
| [`ExtendedInterfaces.interaction.step`](ros2_interfaces.py#L435) | Cache interaction progress and state. | Provide feedback without adding DDS work to control. |
| [`ExtendedInterfaces.validate_cartesian`](ros2_interfaces.py#L460) | Validate a Cartesian action target/options. | Reject invalid pose semantics before reserving hardware execution. |
| [`ExtendedInterfaces.cartesian`](ros2_interfaces.py#L468) | Execute the requested Cartesian move. | Reuse the same planning and arrival logic as Python. |
| [`ExtendedInterfaces.validate_gripper`](ros2_interfaces.py#L481) | Validate position/effort fields against electric-gripper capabilities. | Reject adjustable-force requests unsupported by this tool. |
| [`ExtendedInterfaces.gripper`](ros2_interfaces.py#L494) | Execute a standard GripperCommand using measured device state. | Expose position control while preserving unknown effort as unknown. |
| [`ExtendedInterfaces.query`](ros2_interfaces.py#L511) | Run a guarded service operation and translate expected exceptions to response failures. | Serialize model/peripheral access with command ownership. |
| [`ExtendedInterfaces.forward`](ros2_interfaces.py#L529) | Handle typed FK service requests. | Query installed kinematics without arm motion. |
| [`ExtendedInterfaces.forward.operation`](ros2_interfaces.py#L530) | Convert FK inputs and response poses under driver ownership. | Keep request conversion separate from common service failure handling. |
| [`ExtendedInterfaces.inverse`](ros2_interfaces.py#L549) | Handle typed seeded IK service requests. | Return solutions without commanding motion. |
| [`ExtendedInterfaces.inverse.operation`](ros2_interfaces.py#L550) | Convert IK pose/seed and populate solution fields. | Preserve explicit request units and frame semantics. |
| [`ExtendedInterfaces.collisions`](ros2_interfaces.py#L561) | Handle configuration-collision queries. | Expose model collision results, not workspace certification. |
| [`ExtendedInterfaces.collisions.operation`](ros2_interfaces.py#L562) | Validate joint sets and return controller collision flags. | Share the driver's canonical model query. |
| [`ExtendedInterfaces.read_parameter`](ros2_interfaces.py#L580) | Handle explicit read-only parameter queries. | Expose configuration without arbitrary parameter writes. |
| [`ExtendedInterfaces.read_parameter.operation`](ros2_interfaces.py#L581) | Read one parameter and encode its result. | Return structured values through the supported service contract. |
| [`ExtendedInterfaces.inventory`](ros2_interfaces.py#L588) | Handle graph/device discovery queries. | Let ROS 2 clients inspect attached capabilities. |
| [`ExtendedInterfaces.inventory.operation`](ros2_interfaces.py#L589) | Serialize the driver's inventory response. | Keep discovery consistent across Python and ROS 2. |
| [`ExtendedInterfaces.capabilities`](ros2_interfaces.py#L594) | Return the versioned operation ledger. | Make qualification limits discoverable by clients. |
| [`ExtendedInterfaces.digital_outputs`](ros2_interfaces.py#L599) | Handle boolean digital-output parameter requests. | Preserve input/output permission validation. |
| [`ExtendedInterfaces._set_digital_parameter`](ros2_interfaces.py#L623) | Validate and execute one boolean output setting. | Report per-item failures instead of silently coercing values. |
| [`ExtendedInterfaces.camera_callback`](ros2_interfaces.py#L631) | Build an explicit stream-control service for one camera. | Keep camera ownership independent of image subscriptions. |
| [`ExtendedInterfaces.camera_callback.callback`](ros2_interfaces.py#L632) | Apply a SetBool streaming request and return confirmed state. | A camera service success must reflect device feedback. |
| [`ExtendedInterfaces.display`](ros2_interfaces.py#L642) | Validate and forward an incoming display Image. | Reuse supported RGB/BGR constraints without cv_bridge. |
| [`ExtendedInterfaces.io_callback`](ros2_interfaces.py#L656) | Build a signal/port SetParameters handler for a device. | Reuse metadata checks and acknowledgements across devices. |
| [`ExtendedInterfaces.io_callback.callback`](ros2_interfaces.py#L657) | Convert each parameter, write its selected category and report acknowledgement. | Keep port and signal namespaces and per-item results distinct. |
| [`ExtendedInterfaces.parameter_value`](ros2_interfaces.py#L709) | Decode supported typed parameter values including JSON containers. | Preserve IO value types instead of guessing from strings. |
| [`ExtendedInterfaces.close`](ros2_interfaces.py#L730) | Destroy optional action/service/subscription resources. | Release the ROS 2 extension cleanly. |


### `ros2_telemetry.py`

[ros2_telemetry.py](ros2_telemetry.py)

**What:** Caches direct feedback and publishes standard/optional typed telemetry. **Why:** Keep message conversion and DDS publication off direct control callbacks.

| Class / data type | Responsibility |
| --- | --- |
| [`SawyerTelemetry`](ros2_telemetry.py#L96) | Small direct-receipt caches drained by the ROS 2 executor. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`to_ros`](ros2_telemetry.py#L26) | Copy wire fields to ROS 2 while handling header/time/name differences. | Reuse standard schemas without a ROS 1 runtime. |
| [`to_wire`](ros2_telemetry.py#L70) | Convert typed ROS 2 requests into independent wire types. | Use one validated backend for both application styles. |
| [`SawyerTelemetry.__init__`](ros2_telemetry.py#L105) | Initialize SawyerTelemetry state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`SawyerTelemetry.observe`](ros2_telemetry.py#L226) | Register a shared direct feedback observer and its publication mapping. | Avoid duplicate subscriptions. |
| [`SawyerTelemetry.observe.receive`](ros2_telemetry.py#L240) | Forward received state into the telemetry cache. | Keep the network/control callback lightweight. |
| [`SawyerTelemetry.receive`](ros2_telemetry.py#L248) | Cache incoming telemetry, counts and bounded IO changes. | Move conversion work to the executor. |
| [`SawyerTelemetry.merge_io_state`](ros2_telemetry.py#L273) | Merge partial IO signals and ports. | Change-only feedback must not erase stable values. |
| [`SawyerTelemetry.receive_joints`](ros2_telemetry.py#L302) | Merge independently timestamped effort fields. | Do not refresh missing effort values from unrelated updates. |
| [`SawyerTelemetry.joint_efforts`](ros2_telemetry.py#L315) | Return only a complete fresh effort vector. | Avoid publishing stale or fabricated joint torque. |
| [`SawyerTelemetry.publish`](ros2_telemetry.py#L326) | Drain caches, convert/publish messages and record conversion errors. | Prevent malformed telemetry from crashing control callbacks. |
| [`SawyerTelemetry.publish_io_health`](ros2_telemetry.py#L381) | Publish IO connection, receipt-age and drop metadata. | Help clients distinguish unchanged signals from unhealthy feedback. |
| [`SawyerTelemetry.close`](ros2_telemetry.py#L400) | Remove registered direct observers. | Allow server cleanup without destroying the shared driver connection. |
| [`SawyerTelemetry.publish_endpoint`](ros2_telemetry.py#L405) | Publish endpoint twist/wrench in the declared frame. | Expose the measured endpoint fields beyond pose. |


### `ros_sugar_component.py`

[ros_sugar_component.py](ros_sugar_component.py)

**What:** Implements an optional ros_sugar lifecycle application example. **Why:** Demonstrate an irobot application owning the reusable SawyerController.

| Class / data type | Responsibility |
| --- | --- |
| [`SawyerComponent`](ros_sugar_component.py#L13) | Optional lifecycle application adapter with one cancellable worker task. |

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`SawyerComponent.__init__`](ros_sugar_component.py#L21) | Initialize SawyerComponent state, validated inputs and its owned resources. | Keep setup with the object that owns these responsibilities; public driver construction remains offline. |
| [`SawyerComponent.custom_on_configure`](ros_sugar_component.py#L36) | Construct/configure the driver for lifecycle ownership. | Keep framework setup separate from connection and motion. |
| [`SawyerComponent.custom_on_activate`](ros_sugar_component.py#L40) | Connect the driver and prepare task execution without starting movement. | Activation must not implicitly enable or command Sawyer. |
| [`SawyerComponent._submit`](ros_sugar_component.py#L44) | Under a lock, admit one task and reject overlap/inactive state. | Avoid queued physical tasks and completion/submission races. |
| [`SawyerComponent.move_to_neutral`](ros_sugar_component.py#L57) | Submit the driver's neutral request, previewing by default. | Show how an application calls a reusable robot task. |
| [`SawyerComponent.open_gripper`](ros_sugar_component.py#L62) | Submit a gripper-open request. | Reuse driver semantics instead of implementing a second gripper protocol. |
| [`SawyerComponent.close_gripper`](ros_sugar_component.py#L65) | Submit a gripper-close request. | Keep task scheduling in the application layer. |
| [`SawyerComponent.cancel_motion`](ros_sugar_component.py#L68) | Signal the active task and driver cancellation. | Support framework-controlled task interruption. |
| [`SawyerComponent._execution_step`](ros_sugar_component.py#L72) | Collect completed futures under the task lock. | Do not discard a concurrently submitted replacement task. |
| [`SawyerComponent.custom_on_deactivate`](ros_sugar_component.py#L83) | Cancel and join active work before ending activation. | Stop application execution before lifecycle resources are released. |
| [`SawyerComponent.custom_on_cleanup`](ros_sugar_component.py#L100) | Disconnect and clear configured driver resources. | Make repeated lifecycle setup possible. |
| [`SawyerComponent.custom_on_shutdown`](ros_sugar_component.py#L107) | Perform lifecycle cleanup on shutdown. | Avoid leaving application-owned sessions alive. |
| [`SawyerComponent.custom_on_error`](ros_sugar_component.py#L110) | Perform cleanup after a framework error. | Use the same bounded ownership exit on failure. |


### `setup_runtime.py`

[setup_runtime.py](setup_runtime.py)

**What:** Installs/checks plain Python or pinned optional Humble runtimes. **Why:** Make workstation dependencies explicit without bridge-only setup checks.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`platform_issues`](setup_runtime.py#L42) | Report platforms outside the pinned Linux x86_64 Humble baseline. | Avoid claiming unsupported binary compatibility. |
| [`runtime_environment`](setup_runtime.py#L49) | Build isolated ROS/loader environment variables for a selected prefix/overlay. | Prevent contamination by an unrelated ROS installation. |
| [`inspect_runtime`](setup_runtime.py#L76) | Probe interpreter, NumPy, ROS ABI and optional interface imports without contacting hardware. | Verify installation independently of robot availability. |
| [`require_current_runtime`](setup_runtime.py#L148) | Raise if the current runtime probe finds problems. | Give examples actionable dependency failures. |
| [`installation_commands`](setup_runtime.py#L154) | Build explicit venv/Conda, package and optional CMake command arguments. | Support reviewable dry runs without executing installers. |
| [`write_activation`](setup_runtime.py#L222) | Write a shell activation file for the chosen prefix and overlay. | Make subsequent application startup reproducible. |
| [`install`](setup_runtime.py#L238) | Execute only an explicit install request, respecting dry-run mode. | Keep check/import paths free of downloads and installation side effects. |
| [`main`](setup_runtime.py#L268) | Parse install/check arguments and report results. | Provide one workstation setup entry point. |


### `../../../__init__.py`

[../../../__init__.py](../../../__init__.py)

**What:** Exports Sawyer APIs and lazily loads Crazyflie exports. **Why:** Importing Sawyer must not import the unrelated Crazyflie SDK.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`__getattr__`](../../../__init__.py#L17) | Load Crazyflie classes only when those exports are requested. | Keep standalone Sawyer imports independent of Crazyflie module startup. |


### `../../../../main.py`

[../../../../main.py](../../../../main.py)

**What:** Delegates the development entry point to the neutral example. **Why:** Preserve the existing preview/execute command-line workflow.

No explicitly defined Python functions.


### `../../../../sawyer_saved_pose.py`

[../../../../sawyer_saved_pose.py](../../../../sawyer_saved_pose.py)

**What:** Saves measured arm posture or previews/restores it via neutral. **Why:** Keep pose persistence in an application above the reusable driver.

| Function / method | What it does | Why it exists |
| --- | --- | --- |
| [`load_pose`](../../../../sawyer_saved_pose.py#L19) | Validate exactly seven named finite radian angles before connection. | Invalid saved files must not cause preceding neutral movement. |
| [`save_pose`](../../../../sawyer_saved_pose.py#L36) | Write measured joints to a new JSON file without overwriting an earlier capture. | Save posture without commanding the robot or losing the original. |
| [`restore_pose`](../../../../sawyer_saved_pose.py#L50) | Preview both targets, then explicitly execute neutral followed by the saved target if requested. | Validate the complete intention before the first movement and replan from fresh feedback. |
| [`main`](../../../../sawyer_saved_pose.py#L69) | Parse save/restore, path and motion options and own the connection. | Keep file validation ahead of robot access. |


## Packaged data, documentation and project integration

These files contain no manually defined callable functions. Their data/build or
documentation role is the reason they are part of the implementation. The
`irobot/src/robots/` directory is a namespace package without an `__init__.py`.

| File | What it contains | Why it exists |
| --- | --- | --- |
| [`README.md`](README.md) | Setup, Python/ROS 2 APIs and ownership/qualification contracts. | Start here to install and use the driver. |
| [`IMPLEMENTATION_GUIDE.md`](IMPLEMENTATION_GUIDE.md) | This file and function map. | Explain responsibilities and design reasons without reading every module first. |
| [`UNFINISHED_WORK.md`](UNFINISHED_WORK.md) | Prioritized remaining implementation/qualification work. | Keep blockers, deferred work and conditional hardware scope visible. |
| [`DIRECT_DRIVER_PLAN.md`](DIRECT_DRIVER_PLAN.md) | The itemized original implementation and qualification checklist. | Retain explicit checked/unchecked completion criteria. |
| [`CAPABILITIES.md`](CAPABILITIES.md) | Human-readable capability and physical-status summary. | Distinguish an implemented API from qualified hardware behavior. |
| [`VALIDATION.md`](VALIDATION.md) | Environment, measured results, failures and practical limits. | Tie completion claims to evidence rather than API existence. |
| [`SDK_INVENTORY.json`](SDK_INVENTORY.json) | Pinned index of 226 public methods across 28 SDK source files. | Provide the source baseline for coverage accounting. |
| [`CAPABILITY_LEDGER.json`](CAPABILITY_LEDGER.json) | Per-operation Python/ROS 2/controller mapping, limits, evidence and status. | Account for each indexed SDK operation while retaining unavailable/unqualified cases. |
| [`installation/wire_schema.json`](installation/wire_schema.json) | Pinned message/service definitions, hashes, fields and provenance. | Construct and serialize protocol types without ROS code generation at runtime. |
| [`installation/INTERA_LICENSE.txt`](installation/INTERA_LICENSE.txt) | Upstream license for retained public Intera definitions. | Preserve redistribution terms. |
| [`installation/THIRD_PARTY_NOTICES.md`](installation/THIRD_PARTY_NOTICES.md) | Definition origins and upstream attribution. | Keep generated protocol assets traceable to their sources. |
| [`installation/standard/tf2_msgs/msg/TFMessage.msg`](installation/standard/tf2_msgs/msg/TFMessage.msg) | The standard TransformStamped-array TF message declaration. | Include the TF wire dependency during schema generation. |
| [`installation/humble-linux-64.explicit.txt`](installation/humble-linux-64.explicit.txt) | Exact 177-package Conda Humble baseline. | Reproduce the tested direct Humble environment without a ROS 1/bridge package. |
| [`installation/fastdds-cameras.xml`](installation/fastdds-cameras.xml) | 32 MiB Fast DDS shared-memory transport profile. | Carry full-resolution images larger than the tested default SHM segment; apply on server and clients. |
| [`installation/ROS_SUGAR.md`](installation/ROS_SUGAR.md) | Optional real-framework build/setup and lifecycle example. | Keep ros_sugar optional and separate from the core runtime. |
| [`installation/ros-sugar-release.json`](installation/ros-sugar-release.json) | ros_sugar 0.8.2 repository and exact source revision. | Pin the upstream framework implementation used for verification. |
| [`installation/ros-sugar-linux-64.explicit.txt`](installation/ros-sugar-linux-64.explicit.txt) | Exact 201-package optional framework Conda baseline. | Reproduce the framework build including its ROS dependencies. |
| [`installation/ros-sugar-requirements.txt`](installation/ros-sugar-requirements.txt) | Pinned additional Python framework dependencies. | Complete the tested optional runtime without unconstrained dependency drift. |
| [`validation/2026-11-13.json`](validation/2026-11-13.json) | Original neutral/joint/Cartesian/gripper/ROS 2 milestone evidence. | Retain the initial observations as historical evidence. |
| [`validation/2026-11-13-extended.json`](validation/2026-11-13-extended.json) | Later load, process-loss, peripheral and mode/fault observations. | Preserve failures and bounded successes rather than overwrite earlier results. |
| [`validation/2026-11-13-continuation.json`](validation/2026-11-13-continuation.json) | Restart/retest, repeated enable failure, final state and software verification evidence. | Record the latest qualification boundary and final stopped state. |
| [`../../../../README.md`](../../../../README.md) | Repository-level driver/template overview and Sawyer development status. | Explain where this hardware driver fits in irobot. |
| [`../../../../pyproject.toml`](../../../../pyproject.toml) | Package exports/assets, NumPy 2 Sawyer extra, build and lint/test settings. | Ship schemas/licenses and keep optional driver dependencies explicit; cflib remains a declared package dependency. |
| [`../../../../irobot/src/__init__.py`](../../__init__.py) | Parent Python package marker. | Allow the established irobot.src import layout. |

## Optional ROS 2 interface source files

These declarative files define data and endpoint contracts, not Python functions.
CMake generates language bindings/type support; those build products are not
handwritten driver functions. Standard interfaces such as FollowJointTrajectory,
GripperCommand, Image and SetParameters come from the Humble dependencies.

| File, relative to `ros2/irobot_sawyer_interfaces/` | What it defines | Why it exists |
| --- | --- | --- |
| [`CMakeLists.txt`](../../../../ros2/irobot_sawyer_interfaces/CMakeLists.txt) | Lists interface files and invokes rosidl generation with dependencies. | Build native Humble type support explicitly instead of at application startup. |
| [`LICENSE`](../../../../ros2/irobot_sawyer_interfaces/LICENSE) | License covering the retained/generated public definitions. | Preserve upstream redistribution requirements. |
| [`NOTICE`](../../../../ros2/irobot_sawyer_interfaces/NOTICE) | Generated-definition origin and attribution notice. | Keep ROS 2 definitions traceable to the pinned source schema. |
| [`README.md`](../../../../ros2/irobot_sawyer_interfaces/README.md) | Optional interface setup, generation and build instructions. | Keep custom type support separate from the standard node and plain Python. |
| [`action/CalibrationCommand.action`](../../../../ros2/irobot_sawyer_interfaces/action/CalibrationCommand.action) | Native calibration goal, terminal result and progress. | Expose explicit calibration with action cancellation semantics. |
| [`action/InteractionSession.action`](../../../../ros2/irobot_sawyer_interfaces/action/InteractionSession.action) | Bounded interaction command, duration, limits, result and feedback. | Represent force/impedance as a finite action rather than an unbounded setting. |
| [`action/JointStream.action`](../../../../ros2/irobot_sawyer_interfaces/action/JointStream.action) | Finite stream settings, session identity, outcome and progress. | Associate leased command packets with one accepted control session. |
| [`action/MotionCommand.action`](../../../../ros2/irobot_sawyer_interfaces/action/MotionCommand.action) | Typed native motion goal, result and feedback. | Carry public Intera trajectory semantics over ROS 2. |
| [`action/MoveCartesian.action`](../../../../ros2/irobot_sawyer_interfaces/action/MoveCartesian.action) | Frame-qualified Cartesian target/options and execution feedback. | Expose the shared Cartesian planner as a cancellable action. |
| [`msg/CalibrationCommandFeedback.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/CalibrationCommandFeedback.msg) | Native calibration progress payload. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/CalibrationCommandGoal.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/CalibrationCommandGoal.msg) | Native calibration request payload. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/CalibrationCommandResult.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/CalibrationCommandResult.msg) | Native calibration terminal result payload. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/DigitalIOState.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/DigitalIOState.msg) | Digital value and input/output permission flags. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/EndpointState.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/EndpointState.msg) | One endpoint pose, twist, wrench, validity and header. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/EndpointStates.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/EndpointStates.msg) | Named endpoint states, preserving actual tool identity. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/IOComponentConfiguration.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/IOComponentConfiguration.msg) | Component-level IO configuration metadata. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/IOComponentStatus.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/IOComponentStatus.msg) | Component-level IO status metadata. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/IODataStatus.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/IODataStatus.msg) | One typed/JSON-valued signal or port state. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/IODeviceConfiguration.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/IODeviceConfiguration.msg) | Device signal/port declarations and type/range/permission metadata. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/IODeviceStatus.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/IODeviceStatus.msg) | Device connection/status and changed signal/port values. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/IOStatus.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/IOStatus.msg) | Aggregate controller IO status. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/InteractionControlCommand.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/InteractionControlCommand.msg) | Cartesian interaction mode, wrench, stiffness, frame and axis settings. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/InteractionControlState.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/InteractionControlState.msg) | Measured/reported interaction activation and controller state. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/JointLimits.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/JointLimits.msg) | Named joint position, velocity, acceleration and effort limits as defined by the schema. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/MotionCommandFeedback.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/MotionCommandFeedback.msg) | Native motion progress payload. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/MotionCommandGoal.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/MotionCommandGoal.msg) | Native motion operation and trajectory request. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/MotionCommandResult.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/MotionCommandResult.msg) | Native motion outcome and analysis payload. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/MotionStatus.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/MotionStatus.msg) | Native motion status/error information. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/RobotAssemblyState.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/RobotAssemblyState.msg) | Controller enabled, ready, homed, stopped, error and E-stop state. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/StreamCommand.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/StreamCommand.msg) | Session ID, sequence, source stamp, canonical joints, mode and command vectors. | Support source leases and reject stale/reordered/foreign stream commands. |
| [`msg/TrackingOptions.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/TrackingOptions.msg) | Native trajectory tracking options. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/Trajectory.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/Trajectory.msg) | Native waypoint sequence and trajectory-level settings. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/TrajectoryAnalysis.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/TrajectoryAnalysis.msg) | Native analysis fields; they may describe prior execution rather than generated waypoints. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/TrajectoryOptions.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/TrajectoryOptions.msg) | Native interpolation/planning trajectory options. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/Waypoint.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/Waypoint.msg) | Native joint/Cartesian waypoint payload. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`msg/WaypointOptions.msg`](../../../../ros2/irobot_sawyer_interfaces/msg/WaypointOptions.msg) | Per-waypoint mode and option fields. | Preserve typed public Intera data for optional ROS 2 actions or telemetry. |
| [`package.xml`](../../../../ros2/irobot_sawyer_interfaces/package.xml) | ROS package identity, build/runtime dependencies and interface-group metadata. | Let ament/rosidl resolve the generated package correctly. |
| [`srv/CheckJointCollision.srv`](../../../../ros2/irobot_sawyer_interfaces/srv/CheckJointCollision.srv) | Joint configurations and model collision flags. | Expose the controller collision query without claiming obstacle avoidance. |
| [`srv/ForwardKinematics.srv`](../../../../ros2/irobot_sawyer_interfaces/srv/ForwardKinematics.srv) | Joint configuration request and pose/validity response. | Query the installed model without moving. |
| [`srv/InverseKinematics.srv`](../../../../ros2/irobot_sawyer_interfaces/srv/InverseKinematics.srv) | Pose/seed request and joint-solution response. | Make frame-aware seeded IK available to ROS 2 clients. |
| [`srv/ReadParameter.srv`](../../../../ros2/irobot_sawyer_interfaces/srv/ReadParameter.srv) | Named parameter request and serialized read result. | Provide read-only controller configuration access. |

## Tests and their supporting functions

Test names describe the behavior asserted; they do not mean a physical robot was
used. Each file's purpose explains why its checks exist. Class names organize
related tests; nested helpers deliberately supply local peers, feedback or faults.
The last recorded full suite passed 164 tests and 27 subtests in the clean optional
framework runtime. That is previous verification, not a rerun for this documentation
change; see VALIDATION.md for scope. Framework/DDS checks require their optional
runtime, and framework tests use a fake hardware driver.


### [`tests/test_irobot/__init__.py`](../../../../tests/test_irobot/__init__.py)

Package marker for driver test imports; no functions.


### [`tests/test_irobot/smoke_sawyer_runtime.py`](../../../../tests/test_irobot/smoke_sawyer_runtime.py)

**Historical utility, outside normal test discovery.** Its `main` builds a localhost DDS feedback simulation using `intera_core_msgs` and attempts a driver connection. It assumes the earlier DDS-based transport path; the current driver connects through XML-RPC/TCPROS. It is retained for reference and is **not current direct-driver validation**. Updating or removing it is noted in UNFINISHED_WORK.md.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`main`](../../../../tests/test_irobot/smoke_sawyer_runtime.py#L17) | Run the historical synthetic DDS scenario; not a current direct TCPROS check. |


### [`tests/test_irobot/test_sawyer_api.py`](../../../../tests/test_irobot/test_sawyer_api.py)

**What/why:** Offline imports, connection cleanup, preview defaults, operation ownership and stopped-state validation. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`LifecycleTests.attach`](../../../../tests/test_irobot/test_sawyer_api.py#L21) | Test fixture/support: attach. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`LifecycleTests.test_import_and_construction_do_not_load_ros`](../../../../tests/test_irobot/test_sawyer_api.py#L29) | Assert: import and construction do not load ros. Protect this behavior against regression. |
| [`LifecycleTests.test_connect_and_reconnect_do_not_command_or_duplicate_runtime`](../../../../tests/test_irobot/test_sawyer_api.py#L47) | Assert: connect and reconnect do not command or duplicate runtime. Protect this behavior against regression. |
| [`LifecycleTests.test_failed_connect_cleans_partial_resources`](../../../../tests/test_irobot/test_sawyer_api.py#L61) | Assert: failed connect cleans partial resources. Protect this behavior against regression. |
| [`LifecycleTests.test_failed_connect_cleans_partial_resources.fail`](../../../../tests/test_irobot/test_sawyer_api.py#L64) | Local `fail` helper for `LifecycleTests.test_failed_connect_cleans_partial_resources`; supplies that scenario's callback, peer behavior or injected condition. |
| [`LifecycleTests.test_context_manager_disconnects_after_failure`](../../../../tests/test_irobot/test_sawyer_api.py#L78) | Assert: context manager disconnects after failure. Protect this behavior against regression. |
| [`LifecycleTests.test_disconnected_commands_fail`](../../../../tests/test_irobot/test_sawyer_api.py#L91) | Assert: disconnected commands fail. Protect this behavior against regression. |
| [`LifecycleTests.test_invalid_options_and_watchdog_widening_rejected`](../../../../tests/test_irobot/test_sawyer_api.py#L102) | Assert: invalid options and watchdog widening rejected. Protect this behavior against regression. |
| [`LifecycleTests.test_preview_default_overrides_options_execution_flag`](../../../../tests/test_irobot/test_sawyer_api.py#L124) | Assert: preview default overrides options execution flag. Protect this behavior against regression. |
| [`LifecycleTests.test_simultaneous_operations_are_rejected`](../../../../tests/test_irobot/test_sawyer_api.py#L132) | Assert: simultaneous operations are rejected. Protect this behavior against regression. |
| [`LifecycleTests.test_gripper_methods_preview_by_default_and_disabled_is_explicit`](../../../../tests/test_irobot/test_sawyer_api.py#L140) | Assert: gripper methods preview by default and disabled is explicit. Protect this behavior against regression. |
| [`test_readonly_validation_skips_planning_when_controller_is_stopped`](../../../../tests/test_irobot/test_sawyer_api.py#L157) | Assert: readonly validation skips planning when controller is stopped. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_component.py`](../../../../tests/test_irobot/test_sawyer_component.py)

**What/why:** Lifecycle ownership, cancellation, overlap rejection and concurrent task handoff. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`component`](../../../../tests/test_irobot/test_sawyer_component.py#L18) | Test fixture/support: component. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`component.BaseComponent.__init__`](../../../../tests/test_irobot/test_sawyer_component.py#L22) | Initialize the local fake/fixture state so tests control inputs without real hardware. |
| [`component.BaseComponent.get_logger`](../../../../tests/test_irobot/test_sawyer_component.py#L25) | Test fixture/support: get logger. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_lifecycle_connects_without_enabling_or_starting_a_task`](../../../../tests/test_irobot/test_sawyer_component.py#L44) | Assert: lifecycle connects without enabling or starting a task. Protect this behavior against regression. |
| [`test_lifecycle_rejects_queued_tasks_and_cancels_on_deactivate`](../../../../tests/test_irobot/test_sawyer_component.py#L61) | Assert: lifecycle rejects queued tasks and cancels on deactivate. Protect this behavior against regression. |
| [`test_lifecycle_rejects_queued_tasks_and_cancels_on_deactivate.move`](../../../../tests/test_irobot/test_sawyer_component.py#L65) | Local `move` helper for `test_lifecycle_rejects_queued_tasks_and_cancels_on_deactivate`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_completed_task_collection_preserves_concurrent_new_task`](../../../../tests/test_irobot/test_sawyer_component.py#L85) | Assert: completed task collection preserves concurrent new task. Protect this behavior against regression. |
| [`test_completed_task_collection_preserves_concurrent_new_task.completed`](../../../../tests/test_irobot/test_sawyer_component.py#L100) | Local `completed` helper for `test_completed_task_collection_preserves_concurrent_new_task`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_completed_task_collection_preserves_concurrent_new_task.submit`](../../../../tests/test_irobot/test_sawyer_component.py#L110) | Local `submit` helper for `test_completed_task_collection_preserves_concurrent_new_task`; supplies that scenario's callback, peer behavior or injected condition. |


### [`tests/test_irobot/test_sawyer_control.py`](../../../../tests/test_irobot/test_sawyer_control.py)

**What/why:** Virtual arm/Cartesian/gripper execution, cancellation, faults and measured completion. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`gripper_message`](../../../../tests/test_irobot/test_sawyer_control.py#L42) | Test fixture/support: gripper message. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`Plant.__init__`](../../../../tests/test_irobot/test_sawyer_control.py#L72) | Initialize the local fake/fixture state so tests control inputs without real hardware. |
| [`Plant.publish`](../../../../tests/test_irobot/test_sawyer_control.py#L109) | Test fixture/support: publish. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`Plant.feed_pose`](../../../../tests/test_irobot/test_sawyer_control.py#L117) | Test fixture/support: feed pose. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`Plant.spin`](../../../../tests/test_irobot/test_sawyer_control.py#L128) | Test fixture/support: spin. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`Plant.running`](../../../../tests/test_irobot/test_sawyer_control.py#L161) | Test fixture/support: running. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`Plant.kinematics`](../../../../tests/test_irobot/test_sawyer_control.py#L168) | Test fixture/support: kinematics. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`ArmTests.test_public_kinematics_and_invalid_inputs_never_command_motion`](../../../../tests/test_irobot/test_sawyer_control.py#L181) | Assert: public kinematics and invalid inputs never command motion. Protect this behavior against regression. |
| [`ArmTests.test_external_cancel_before_execution_never_publishes`](../../../../tests/test_irobot/test_sawyer_control.py#L199) | Assert: external cancel before execution never publishes. Protect this behavior against regression. |
| [`ArmTests.test_multi_waypoint_execution_and_preview_validate_whole_path`](../../../../tests/test_irobot/test_sawyer_control.py#L210) | Assert: multi waypoint execution and preview validate whole path. Protect this behavior against regression. |
| [`ArmTests.test_neutral_preview_and_infeasible_execution_never_publish`](../../../../tests/test_irobot/test_sawyer_control.py#L225) | Assert: neutral preview and infeasible execution never publish. Protect this behavior against regression. |
| [`ArmTests.test_neutral_execution_tracks_then_holds_position`](../../../../tests/test_irobot/test_sawyer_control.py#L235) | Assert: neutral execution tracks then holds position. Protect this behavior against regression. |
| [`ArmTests.test_faults_interruptions_and_cancellation_stop_streaming`](../../../../tests/test_irobot/test_sawyer_control.py#L244) | Assert: faults interruptions and cancellation stop streaming. Protect this behavior against regression. |
| [`ArmTests.test_cancellation_during_planning_never_starts_commands`](../../../../tests/test_irobot/test_sawyer_control.py#L260) | Assert: cancellation during planning never starts commands. Protect this behavior against regression. |
| [`ArmTests.test_cancellation_during_planning_never_starts_commands.plan_and_cancel`](../../../../tests/test_irobot/test_sawyer_control.py#L265) | Local `plan_and_cancel` helper for `ArmTests.test_cancellation_during_planning_never_starts_commands`; supplies that scenario's callback, peer behavior or injected condition. |
| [`ArmTests.test_disabled_or_missing_command_routes_never_start`](../../../../tests/test_irobot/test_sawyer_control.py#L282) | Assert: disabled or missing command routes never start. Protect this behavior against regression. |
| [`ArmTests.test_neutral_parameter_is_reordered_and_missing_master_is_actionable`](../../../../tests/test_irobot/test_sawyer_control.py#L299) | Assert: neutral parameter is reordered and missing master is actionable. Protect this behavior against regression. |
| [`CartesianTests.test_public_cartesian_preview_then_execution`](../../../../tests/test_irobot/test_sawyer_control.py#L325) | Assert: public cartesian preview then execution. Protect this behavior against regression. |
| [`CartesianTests.test_stale_pose_stops_execution`](../../../../tests/test_irobot/test_sawyer_control.py#L344) | Assert: stale pose stops execution. Protect this behavior against regression. |
| [`CartesianTests.test_stale_pose_stops_execution.lose_pose`](../../../../tests/test_irobot/test_sawyer_control.py#L348) | Local `lose_pose` helper for `CartesianTests.test_stale_pose_stops_execution`; supplies that scenario's callback, peer behavior or injected condition. |
| [`CartesianTests.test_kinematics_uses_same_node_and_configured_services`](../../../../tests/test_irobot/test_sawyer_control.py#L365) | Assert: kinematics uses same node and configured services. Protect this behavior against regression. |
| [`GripperTests.gripper`](../../../../tests/test_irobot/test_sawyer_control.py#L377) | Test fixture/support: gripper. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`GripperTests.test_preview_never_publishes_and_uncalibrated_execution_rejected`](../../../../tests/test_irobot/test_sawyer_control.py#L387) | Assert: preview never publishes and uncalibrated execution rejected. Protect this behavior against regression. |
| [`GripperTests.test_execution_waits_for_ack_and_measured_completion`](../../../../tests/test_irobot/test_sawyer_control.py#L395) | Assert: execution waits for ack and measured completion. Protect this behavior against regression. |
| [`GripperTests.test_execution_waits_for_ack_and_measured_completion.acknowledge`](../../../../tests/test_irobot/test_sawyer_control.py#L398) | Local `acknowledge` helper for `GripperTests.test_execution_waits_for_ack_and_measured_completion`; supplies that scenario's callback, peer behavior or injected condition. |
| [`GripperTests.test_failure_requests_stop`](../../../../tests/test_irobot/test_sawyer_control.py#L411) | Assert: failure requests stop. Protect this behavior against regression. |
| [`CalibrationReadinessTests.test_busy_device_is_not_completed_even_when_calibrated`](../../../../tests/test_irobot/test_sawyer_control.py#L427) | Assert: busy device is not completed even when calibrated. Protect this behavior against regression. |
| [`GripperSettlingTests.test_completion_must_remain_stable_after_late_feedback`](../../../../tests/test_irobot/test_sawyer_control.py#L442) | Assert: completion must remain stable after late feedback. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_diagnostics.py`](../../../../tests/test_irobot/test_sawyer_diagnostics.py)

**What/why:** Sparse diagnostics/joint preservation and bounded change-only input events. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`test_sparse_diagnostics_and_joint_fields_are_preserved`](../../../../tests/test_irobot/test_sawyer_diagnostics.py#L12) | Assert: sparse diagnostics and joint fields are preserved. Protect this behavior against regression. |
| [`test_change_only_input_cache_and_events_are_bounded`](../../../../tests/test_irobot/test_sawyer_diagnostics.py#L37) | Assert: change only input cache and events are bounded. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_extended_ros2.py`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py)

**What/why:** Typed conversion, native Image CDR, streaming client loss, merged telemetry and port acknowledgements. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`test_typed_standard_and_intera_conversions`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py#L40) | Assert: typed standard and intera conversions. Protect this behavior against regression. |
| [`test_fast_image_serialization_matches_native_ros2_cdr`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py#L65) | Assert: fast image serialization matches native ros2 cdr. Protect this behavior against regression. |
| [`test_stream_client_loss_requires_new_goal`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py#L93) | Assert: stream client loss requires new goal. Protect this behavior against regression. |
| [`test_stream_client_loss_requires_new_goal.stream`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py#L100) | Local `stream` helper for `test_stream_client_loss_requires_new_goal`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_stream_client_loss_requires_new_goal.publish`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py#L145) | Local `publish` helper for `test_stream_client_loss_requires_new_goal`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_telemetry_keeps_partial_io_and_bounded_change_events`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py#L227) | Assert: telemetry keeps partial io and bounded change events. Protect this behavior against regression. |
| [`test_telemetry_keeps_partial_io_and_bounded_change_events.update`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py#L237) | Local `update` helper for `test_telemetry_keeps_partial_io_and_bounded_change_events`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_telemetry_rejects_initial_and_repeated_camera_frames`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py#L258) | Assert: telemetry rejects initial and repeated camera frames. Protect this behavior against regression. |
| [`test_ros2_port_settings_forward_category_and_report_acknowledgement`](../../../../tests/test_irobot/test_sawyer_extended_ros2.py#L280) | Assert: ros2 port settings forward category and report acknowledgement. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_extensions.py`](../../../../tests/test_irobot/test_sawyer_extensions.py)

**What/why:** Stream/mode bounds, native goal IDs, IO metadata, interaction exit, leases and enable guards. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`test_stream_modes_preview_bounds_and_exit`](../../../../tests/test_irobot/test_sawyer_extensions.py#L29) | Assert: stream modes preview bounds and exit. Protect this behavior against regression. |
| [`test_stream_rejects_slow_callback_and_transport_restart_before_publish`](../../../../tests/test_irobot/test_sawyer_extensions.py#L82) | Assert: stream rejects slow callback and transport restart before publish. Protect this behavior against regression. |
| [`test_stream_rejects_slow_callback_and_transport_restart_before_publish.slow_callback`](../../../../tests/test_irobot/test_sawyer_extensions.py#L86) | Local `slow_callback` helper for `test_stream_rejects_slow_callback_and_transport_restart_before_publish`; supplies that scenario's callback, peer behavior or injected condition. |
| [`action_peer`](../../../../tests/test_irobot/test_sawyer_extensions.py#L101) | Test fixture/support: action peer. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`action_peer.publish`](../../../../tests/test_irobot/test_sawyer_extensions.py#L115) | Test fixture/support: publish. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`action_peer.finish`](../../../../tests/test_irobot/test_sawyer_extensions.py#L120) | Test fixture/support: finish. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`action_peer.spin`](../../../../tests/test_irobot/test_sawyer_extensions.py#L136) | Test fixture/support: spin. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_native_action_matches_ids_and_reports_rejection`](../../../../tests/test_irobot/test_sawyer_extensions.py#L154) | Assert: native action matches ids and reports rejection. Protect this behavior against regression. |
| [`test_native_action_failure_cancels_only_its_own_goal`](../../../../tests/test_irobot/test_sawyer_extensions.py#L165) | Assert: native action failure cancels only its own goal. Protect this behavior against regression. |
| [`test_cancelled_action_never_publishes`](../../../../tests/test_irobot/test_sawyer_extensions.py#L175) | Assert: cancelled action never publishes. Protect this behavior against regression. |
| [`io_peer`](../../../../tests/test_irobot/test_sawyer_extensions.py#L184) | Test fixture/support: io peer. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_io_types_ranges_input_only_and_independent_snapshot`](../../../../tests/test_irobot/test_sawyer_extensions.py#L216) | Assert: io types ranges input only and independent snapshot. Protect this behavior against regression. |
| [`test_io_acknowledgement_and_edges`](../../../../tests/test_irobot/test_sawyer_extensions.py#L234) | Assert: io acknowledgement and edges. Protect this behavior against regression. |
| [`test_io_acknowledgement_and_edges.acknowledge`](../../../../tests/test_irobot/test_sawyer_extensions.py#L241) | Local `acknowledge` helper for `test_io_acknowledgement_and_edges`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_effort_limits_must_be_published_for_all_seven_joints`](../../../../tests/test_irobot/test_sawyer_extensions.py#L261) | Assert: effort limits must be published for all seven joints. Protect this behavior against regression. |
| [`test_interaction_confirms_exit_and_reuses_one_observer`](../../../../tests/test_irobot/test_sawyer_extensions.py#L266) | Assert: interaction confirms exit and reuses one observer. Protect this behavior against regression. |
| [`test_interaction_confirms_exit_and_reuses_one_observer.observe`](../../../../tests/test_irobot/test_sawyer_extensions.py#L276) | Local `observe` helper for `test_interaction_confirms_exit_and_reuses_one_observer`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_interaction_confirms_exit_and_reuses_one_observer.publish`](../../../../tests/test_irobot/test_sawyer_extensions.py#L280) | Local `publish` helper for `test_interaction_confirms_exit_and_reuses_one_observer`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_interaction_rejects_wrench_and_invalid_frame_parameters`](../../../../tests/test_irobot/test_sawyer_extensions.py#L300) | Assert: interaction rejects wrench and invalid frame parameters. Protect this behavior against regression. |
| [`test_transform_snapshots_omit_expired_dynamic_edges_and_keep_static`](../../../../tests/test_irobot/test_sawyer_extensions.py#L311) | Assert: transform snapshots omit expired dynamic edges and keep static. Protect this behavior against regression. |
| [`test_command_lease_rejects_old_ids_sequences_timestamps_and_loss`](../../../../tests/test_irobot/test_sawyer_extensions.py#L330) | Assert: command lease rejects old ids sequences timestamps and loss. Protect this behavior against regression. |
| [`test_enable_confirms_ready_and_latches_stop_on_drift`](../../../../tests/test_irobot/test_sawyer_extensions.py#L361) | Assert: enable confirms ready and latches stop on drift. Protect this behavior against regression. |
| [`test_enable_confirms_ready_and_latches_stop_on_drift.publish`](../../../../tests/test_irobot/test_sawyer_extensions.py#L369) | Local `publish` helper for `test_enable_confirms_ready_and_latches_stop_on_drift`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_mode_exit_does_not_send_commands_after_controller_stop`](../../../../tests/test_irobot/test_sawyer_extensions.py#L397) | Assert: mode exit does not send commands after controller stop. Protect this behavior against regression. |
| [`test_unqualified_modes_require_explicit_opt_in_before_publication`](../../../../tests/test_irobot/test_sawyer_extensions.py#L407) | Assert: unqualified modes require explicit opt in before publication. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_feedback.py`](../../../../tests/test_irobot/test_sawyer_feedback.py)

**What/why:** Canonical partial-field merging and independent per-joint receipt freshness. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`FeedbackTests.test_unrelated_message_does_not_clear_or_refresh_arm`](../../../../tests/test_irobot/test_sawyer_feedback.py#L15) | Assert: unrelated message does not clear or refresh arm. Protect this behavior against regression. |
| [`FeedbackTests.test_partial_arm_messages_merge_by_name`](../../../../tests/test_irobot/test_sawyer_feedback.py#L24) | Assert: partial arm messages merge by name. Protect this behavior against regression. |
| [`FeedbackTests.test_positions_and_velocities_have_independent_freshness`](../../../../tests/test_irobot/test_sawyer_feedback.py#L31) | Assert: positions and velocities have independent freshness. Protect this behavior against regression. |
| [`FeedbackTests.test_other_joint_updates_cannot_hide_a_stale_joint`](../../../../tests/test_irobot/test_sawyer_feedback.py#L42) | Assert: other joint updates cannot hide a stale joint. Protect this behavior against regression. |
| [`FeedbackTests.test_invalid_relevant_message_does_not_commit_or_refresh`](../../../../tests/test_irobot/test_sawyer_feedback.py#L51) | Assert: invalid relevant message does not commit or refresh. Protect this behavior against regression. |
| [`FeedbackTests.test_empty_or_nonfinite_unrelated_fields_do_not_invalidate_arm`](../../../../tests/test_irobot/test_sawyer_feedback.py#L67) | Assert: empty or nonfinite unrelated fields do not invalidate arm. Protect this behavior against regression. |
| [`FeedbackTests.test_position_hold_requires_only_fresh_positions`](../../../../tests/test_irobot/test_sawyer_feedback.py#L76) | Assert: position hold requires only fresh positions. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_neutral_example.py`](../../../../tests/test_irobot/test_sawyer_neutral_example.py)

**What/why:** Neutral example routing, preview behavior, measured reporting and preconnection validation. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`result`](../../../../tests/test_irobot/test_sawyer_neutral_example.py#L17) | Test fixture/support: result. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`controller`](../../../../tests/test_irobot/test_sawyer_neutral_example.py#L29) | Test fixture/support: controller. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_main_routes_to_the_sawyer_neutral_example`](../../../../tests/test_irobot/test_sawyer_neutral_example.py#L37) | Assert: main routes to the sawyer neutral example. Protect this behavior against regression. |
| [`test_default_preview_uses_only_public_arm_api`](../../../../tests/test_irobot/test_sawyer_neutral_example.py#L41) | Assert: default preview uses only public arm api. Protect this behavior against regression. |
| [`test_execution_replans_after_preview_and_reports_measurement`](../../../../tests/test_irobot/test_sawyer_neutral_example.py#L56) | Assert: execution replans after preview and reports measurement. Protect this behavior against regression. |
| [`test_invalid_settings_fail_before_connecting`](../../../../tests/test_irobot/test_sawyer_neutral_example.py#L78) | Assert: invalid settings fail before connecting. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_peripherals.py`](../../../../tests/test_irobot/test_sawyer_peripherals.py)

**What/why:** Peripheral previews, camera ACKs, head cleanup, display validation and stale-feedback stop behavior. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`camera_message`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L18) | Test fixture/support: camera message. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`peer`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L34) | Test fixture/support: peer. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`peer.subscribe`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L47) | Test fixture/support: subscribe. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_previews_send_no_commands_and_enable_never_resets_faults`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L67) | Assert: previews send no commands and enable never resets faults. Protect this behavior against regression. |
| [`test_camera_needs_matching_timestamp_and_measured_signal`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L78) | Assert: camera needs matching timestamp and measured signal. Protect this behavior against regression. |
| [`test_camera_needs_matching_timestamp_and_measured_signal.acknowledge`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L83) | Local `acknowledge` helper for `test_camera_needs_matching_timestamp_and_measured_signal`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_head_commands_refresh_until_measured_arrival`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L96) | Assert: head commands refresh until measured arrival. Protect this behavior against regression. |
| [`test_head_commands_refresh_until_measured_arrival.respond`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L101) | Local `respond` helper for `test_head_commands_refresh_until_measured_arrival`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_head_failure_requests_hold_and_restores_original_mode`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L113) | Assert: head failure requests hold and restores original mode. Protect this behavior against regression. |
| [`test_display_validation_and_encoding_are_offline`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L125) | Assert: display validation and encoding are offline. Protect this behavior against regression. |
| [`test_head_interrupt_during_first_send_requests_hold`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L137) | Assert: head interrupt during first send requests hold. Protect this behavior against regression. |
| [`test_software_stop_sends_despite_stale_assembly_feedback`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L149) | Assert: software stop sends despite stale assembly feedback. Protect this behavior against regression. |
| [`test_software_stop_sends_despite_stale_assembly_feedback.stop`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L154) | Local `stop` helper for `test_software_stop_sends_despite_stale_assembly_feedback`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_software_stop_never_confirms_from_stale_stopped_state`](../../../../tests/test_irobot/test_sawyer_peripherals.py#L167) | Assert: software stop never confirms from stale stopped state. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_pid.py`](../../../../tests/test_irobot/test_sawyer_pid.py)

**What/why:** PID terms, saturation, anti-windup, joint bounds and reference tracking. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`PIDTests.controller`](../../../../tests/test_irobot/test_sawyer_pid.py#L18) | Test fixture/support: controller. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`PIDTests.test_pid_terms_and_feedforward`](../../../../tests/test_irobot/test_sawyer_pid.py#L30) | Assert: pid terms and feedforward. Protect this behavior against regression. |
| [`PIDTests.test_velocity_acceleration_and_reset`](../../../../tests/test_irobot/test_sawyer_pid.py#L36) | Assert: velocity acceleration and reset. Protect this behavior against regression. |
| [`PIDTests.test_no_integral_windup_and_unwinding`](../../../../tests/test_irobot/test_sawyer_pid.py#L49) | Assert: no integral windup and unwinding. Protect this behavior against regression. |
| [`PIDTests.test_integral_cap`](../../../../tests/test_irobot/test_sawyer_pid.py#L58) | Assert: integral cap. Protect this behavior against regression. |
| [`PIDTests.test_joint_position_bound_blocks_outward_velocity`](../../../../tests/test_irobot/test_sawyer_pid.py#L64) | Assert: joint position bound blocks outward velocity. Protect this behavior against regression. |
| [`PIDTests.test_per_joint_gains`](../../../../tests/test_irobot/test_sawyer_pid.py#L72) | Assert: per joint gains. Protect this behavior against regression. |
| [`PIDTests.test_invalid_inputs`](../../../../tests/test_irobot/test_sawyer_pid.py#L76) | Assert: invalid inputs. Protect this behavior against regression. |
| [`PIDTests.test_tracks_quintic_with_first_order_velocity_plant`](../../../../tests/test_irobot/test_sawyer_pid.py#L98) | Assert: tracks quintic with first order velocity plant. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_planner.py`](../../../../tests/test_irobot/test_sawyer_planner.py)

**What/why:** Cartesian geometry, IK continuity, FK checks, polynomial extrema and timing. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`affine_inverse`](../../../../tests/test_irobot/test_sawyer_planner.py#L26) | Test fixture/support: affine inverse. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`affine_forward`](../../../../tests/test_irobot/test_sawyer_planner.py#L30) | Test fixture/support: affine forward. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`GeometryTests.test_straight_line_fixed_orientation_and_rest_endpoints`](../../../../tests/test_irobot/test_sawyer_planner.py#L35) | Assert: straight line fixed orientation and rest endpoints. Protect this behavior against regression. |
| [`GeometryTests.test_pose_validation_and_quaternion_sign_equivalence`](../../../../tests/test_irobot/test_sawyer_planner.py#L50) | Assert: pose validation and quaternion sign equivalence. Protect this behavior against regression. |
| [`JointPathTests.path`](../../../../tests/test_irobot/test_sawyer_planner.py#L65) | Test fixture/support: path. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`JointPathTests.test_recovers_a_quintic_and_continuous_derivatives`](../../../../tests/test_irobot/test_sawyer_planner.py#L72) | Assert: recovers a quintic and continuous derivatives. Protect this behavior against regression. |
| [`JointPathTests.test_extrema_cover_between_knot_values_and_timing`](../../../../tests/test_irobot/test_sawyer_planner.py#L100) | Assert: extrema cover between knot values and timing. Protect this behavior against regression. |
| [`JointPathTests.test_longer_duration_scales_all_derivatives`](../../../../tests/test_irobot/test_sawyer_planner.py#L132) | Assert: longer duration scales all derivatives. Protect this behavior against regression. |
| [`JointPathTests.test_endpoint_distance_alone_does_not_determine_path_duration`](../../../../tests/test_irobot/test_sawyer_planner.py#L146) | Assert: endpoint distance alone does not determine path duration. Protect this behavior against regression. |
| [`JointPathTests.test_stationary_path_and_invalid_inputs`](../../../../tests/test_irobot/test_sawyer_planner.py#L153) | Assert: stationary path and invalid inputs. Protect this behavior against regression. |
| [`PlannerTests.planner`](../../../../tests/test_irobot/test_sawyer_planner.py#L170) | Test fixture/support: planner. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`PlannerTests.test_affine_kinematics_matches_straight_quintic_and_minimum`](../../../../tests/test_irobot/test_sawyer_planner.py#L175) | Assert: affine kinematics matches straight quintic and minimum. Protect this behavior against regression. |
| [`PlannerTests.test_nonlinear_ik_and_previous_solution_seeding`](../../../../tests/test_irobot/test_sawyer_planner.py#L192) | Assert: nonlinear ik and previous solution seeding. Protect this behavior against regression. |
| [`PlannerTests.test_nonlinear_ik_and_previous_solution_seeding.inverse`](../../../../tests/test_irobot/test_sawyer_planner.py#L195) | Local `inverse` helper for `PlannerTests.test_nonlinear_ik_and_previous_solution_seeding`; supplies that scenario's callback, peer behavior or injected condition. |
| [`PlannerTests.test_nonlinear_ik_and_previous_solution_seeding.forward`](../../../../tests/test_irobot/test_sawyer_planner.py#L209) | Local `forward` helper for `PlannerTests.test_nonlinear_ik_and_previous_solution_seeding`; supplies that scenario's callback, peer behavior or injected condition. |
| [`PlannerTests.test_unreachable_and_discontinuous_ik_rejected`](../../../../tests/test_irobot/test_sawyer_planner.py#L227) | Assert: unreachable and discontinuous ik rejected. Protect this behavior against regression. |
| [`PlannerTests.test_unreachable_and_discontinuous_ik_rejected.discontinuous`](../../../../tests/test_irobot/test_sawyer_planner.py#L239) | Local `discontinuous` helper for `PlannerTests.test_unreachable_and_discontinuous_ik_rejected`; supplies that scenario's callback, peer behavior or injected condition. |
| [`PlannerTests.test_fk_mismatch_collision_and_unvalidated_interpolation_rejected`](../../../../tests/test_irobot/test_sawyer_planner.py#L251) | Assert: fk mismatch collision and unvalidated interpolation rejected. Protect this behavior against regression. |
| [`PlannerTests.test_fk_mismatch_collision_and_unvalidated_interpolation_rejected.inaccurate`](../../../../tests/test_irobot/test_sawyer_planner.py#L269) | Local `inaccurate` helper for `PlannerTests.test_fk_mismatch_collision_and_unvalidated_interpolation_rejected`; supplies that scenario's callback, peer behavior or injected condition. |
| [`PlannerTests.test_joint_bounds_and_too_short_time`](../../../../tests/test_irobot/test_sawyer_planner.py#L285) | Assert: joint bounds and too short time. Protect this behavior against regression. |
| [`PlannerTests.test_joint_bounds_and_too_short_time.inaccurate`](../../../../tests/test_irobot/test_sawyer_planner.py#L288) | Local `inaccurate` helper for `PlannerTests.test_joint_bounds_and_too_short_time`; supplies that scenario's callback, peer behavior or injected condition. |
| [`PlannerTests.test_nonlinear_fk_refines_sampling_until_valid`](../../../../tests/test_irobot/test_sawyer_planner.py#L312) | Assert: nonlinear fk refines sampling until valid. Protect this behavior against regression. |
| [`PlannerTests.test_nonlinear_fk_refines_sampling_until_valid.inverse`](../../../../tests/test_irobot/test_sawyer_planner.py#L313) | Local `inverse` helper for `PlannerTests.test_nonlinear_fk_refines_sampling_until_valid`; supplies that scenario's callback, peer behavior or injected condition. |
| [`PlannerTests.test_nonlinear_fk_refines_sampling_until_valid.forward`](../../../../tests/test_irobot/test_sawyer_planner.py#L324) | Local `forward` helper for `PlannerTests.test_nonlinear_fk_refines_sampling_until_valid`; supplies that scenario's callback, peer behavior or injected condition. |
| [`PlannerTests.test_joint_interpolation_overshoot_rejected_between_valid_ik_points`](../../../../tests/test_irobot/test_sawyer_planner.py#L342) | Assert: joint interpolation overshoot rejected between valid ik points. Protect this behavior against regression. |
| [`PlannerTests.test_joint_interpolation_overshoot_rejected_between_valid_ik_points.inverse`](../../../../tests/test_irobot/test_sawyer_planner.py#L345) | Local `inverse` helper for `PlannerTests.test_joint_interpolation_overshoot_rejected_between_valid_ik_points`; supplies that scenario's callback, peer behavior or injected condition. |
| [`PlannerTests.test_zero_offset_does_not_call_inverse_or_move_joints`](../../../../tests/test_irobot/test_sawyer_planner.py#L372) | Assert: zero offset does not call inverse or move joints. Protect this behavior against regression. |
| [`PlannerTests.test_starting_limit_error_names_joint_and_does_not_call_kinematics`](../../../../tests/test_irobot/test_sawyer_planner.py#L385) | Assert: starting limit error names joint and does not call kinematics. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_ros2.py`](../../../../tests/test_irobot/test_sawyer_ros2.py)

**What/why:** Standard trajectory validation, real DDS ownership/cancellation and stop while a job holds the driver. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`trajectory`](../../../../tests/test_irobot/test_sawyer_ros2.py#L28) | Test fixture/support: trajectory. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_trajectory_reorders_joints_and_rejects_unsupported_fields`](../../../../tests/test_irobot/test_sawyer_ros2.py#L35) | Assert: trajectory reorders joints and rejects unsupported fields. Protect this behavior against regression. |
| [`test_trajectory_rejects_stale_schedule_and_nonfinite_values`](../../../../tests/test_irobot/test_sawyer_ros2.py#L50) | Assert: trajectory rejects stale schedule and nonfinite values. Protect this behavior against regression. |
| [`LocalDriver.__init__`](../../../../tests/test_irobot/test_sawyer_ros2.py#L66) | Initialize the local fake/fixture state so tests control inputs without real hardware. |
| [`LocalDriver.get_status`](../../../../tests/test_irobot/test_sawyer_ros2.py#L78) | Test fixture/support: get status. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`LocalDriver.move_trajectory`](../../../../tests/test_irobot/test_sawyer_ros2.py#L90) | Test fixture/support: move trajectory. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`LocalDriver._track`](../../../../tests/test_irobot/test_sawyer_ros2.py#L99) | Test fixture/support:  track. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`LocalDriver.stop`](../../../../tests/test_irobot/test_sawyer_ros2.py#L111) | Test fixture/support: stop. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`completed`](../../../../tests/test_irobot/test_sawyer_ros2.py#L115) | Test fixture/support: completed. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_real_dds_action_rejects_overlap_cancels_and_accepts_next_goal`](../../../../tests/test_irobot/test_sawyer_ros2.py#L126) | Assert: real dds action rejects overlap cancels and accepts next goal. Protect this behavior against regression. |
| [`test_software_stop_cancels_busy_job_and_waits_for_its_driver_lock`](../../../../tests/test_irobot/test_sawyer_ros2.py#L178) | Assert: software stop cancels busy job and waits for its driver lock. Protect this behavior against regression. |
| [`test_software_stop_cancels_busy_job_and_waits_for_its_driver_lock.active_job`](../../../../tests/test_irobot/test_sawyer_ros2.py#L198) | Local `active_job` helper for `test_software_stop_cancels_busy_job_and_waits_for_its_driver_lock`; supplies that scenario's callback, peer behavior or injected condition. |


### [`tests/test_irobot/test_sawyer_saved_pose.py`](../../../../tests/test_irobot/test_sawyer_saved_pose.py)

**What/why:** Nonmoving capture/preview and validation before the first restoration motion. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`pose_file`](../../../../tests/test_irobot/test_sawyer_saved_pose.py#L16) | Test fixture/support: pose file. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`controller`](../../../../tests/test_irobot/test_sawyer_saved_pose.py#L29) | Test fixture/support: controller. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_capture_sends_no_motion_and_preserves_original`](../../../../tests/test_irobot/test_sawyer_saved_pose.py#L39) | Assert: capture sends no motion and preserves original. Protect this behavior against regression. |
| [`test_preview_sends_no_motion`](../../../../tests/test_irobot/test_sawyer_saved_pose.py#L58) | Assert: preview sends no motion. Protect this behavior against regression. |
| [`test_execute_checks_target_then_moves_neutral_then_saved`](../../../../tests/test_irobot/test_sawyer_saved_pose.py#L69) | Assert: execute checks target then moves neutral then saved. Protect this behavior against regression. |
| [`test_invalid_target_is_rejected_before_neutral_motion`](../../../../tests/test_irobot/test_sawyer_saved_pose.py#L94) | Assert: invalid target is rejected before neutral motion. Protect this behavior against regression. |
| [`test_invalid_angles_fail_before_connect`](../../../../tests/test_irobot/test_sawyer_saved_pose.py#L110) | Assert: invalid angles fail before connect. Protect this behavior against regression. |
| [`test_named_joints_are_loaded_in_driver_order`](../../../../tests/test_irobot/test_sawyer_saved_pose.py#L122) | Assert: named joints are loaded in driver order. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_setup.py`](../../../../tests/test_irobot/test_sawyer_setup.py)

**What/why:** Offline/dry-run installation, pinned Humble dependency selection and ABI isolation. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`test_missing_environment_runs_no_subprocess`](../../../../tests/test_irobot/test_sawyer_setup.py#L18) | Assert: missing environment runs no subprocess. Protect this behavior against regression. |
| [`test_plain_dry_run_never_installs_or_contacts_robot`](../../../../tests/test_irobot/test_sawyer_setup.py#L24) | Assert: plain dry run never installs or contacts robot. Protect this behavior against regression. |
| [`test_humble_dry_run_uses_only_the_direct_dependency_lock`](../../../../tests/test_irobot/test_sawyer_setup.py#L40) | Assert: humble dry run uses only the direct dependency lock. Protect this behavior against regression. |
| [`test_humble_rejects_mixed_abi_and_legacy_runtime_before_import`](../../../../tests/test_irobot/test_sawyer_setup.py#L62) | Assert: humble rejects mixed abi and legacy runtime before import. Protect this behavior against regression. |
| [`test_interface_install_requires_humble_before_any_subprocess`](../../../../tests/test_irobot/test_sawyer_setup.py#L81) | Assert: interface install requires humble before any subprocess. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_sugar_runtime.py`](../../../../tests/test_irobot/test_sawyer_sugar_runtime.py)

**What/why:** Actual optional ros_sugar lifecycle/timer/cancellation/cleanup with a fake hardware driver. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`test_real_framework_timer_cancellation_and_cleanup`](../../../../tests/test_irobot/test_sawyer_sugar_runtime.py#L30) | Assert: real framework timer cancellation and cleanup. Protect this behavior against regression. |
| [`test_real_framework_timer_cancellation_and_cleanup.wait_cancel`](../../../../tests/test_irobot/test_sawyer_sugar_runtime.py#L55) | Local `wait_cancel` helper for `test_real_framework_timer_cancellation_and_cleanup`; supplies that scenario's callback, peer behavior or injected condition. |


### [`tests/test_irobot/test_sawyer_trajectory.py`](../../../../tests/test_irobot/test_sawyer_trajectory.py)

**What/why:** Analytic quintic endpoints, derivative bounds, durations and invalid inputs. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`QuinticTests.test_endpoints_and_stationary_joint`](../../../../tests/test_irobot/test_sawyer_trajectory.py#L17) | Assert: endpoints and stationary joint. Protect this behavior against regression. |
| [`QuinticTests.test_exact_velocity_limited_minimum`](../../../../tests/test_irobot/test_sawyer_trajectory.py#L29) | Assert: exact velocity limited minimum. Protect this behavior against regression. |
| [`QuinticTests.test_acceleration_extremum_determines_duration`](../../../../tests/test_irobot/test_sawyer_trajectory.py#L37) | Assert: acceleration extremum determines duration. Protect this behavior against regression. |
| [`QuinticTests.test_longer_duration_reduces_derivatives`](../../../../tests/test_irobot/test_sawyer_trajectory.py#L50) | Assert: longer duration reduces derivatives. Protect this behavior against regression. |
| [`QuinticTests.test_derivatives_match_finite_differences`](../../../../tests/test_irobot/test_sawyer_trajectory.py#L61) | Assert: derivatives match finite differences. Protect this behavior against regression. |
| [`QuinticTests.test_random_seven_joint_moves_obey_limits_and_monotonicity`](../../../../tests/test_irobot/test_sawyer_trajectory.py#L85) | Assert: random seven joint moves obey limits and monotonicity. Protect this behavior against regression. |
| [`QuinticTests.test_stationary_auto_and_requested_duration`](../../../../tests/test_irobot/test_sawyer_trajectory.py#L108) | Assert: stationary auto and requested duration. Protect this behavior against regression. |
| [`QuinticTests.test_invalid_parameters`](../../../../tests/test_irobot/test_sawyer_trajectory.py#L116) | Assert: invalid parameters. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_transport.py`](../../../../tests/test_irobot/test_sawyer_transport.py)

**What/why:** Local TCPROS/XML-RPC negotiation, bounded reads, feedback retention and master registration without replay. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`rpc_server`](../../../../tests/test_irobot/test_sawyer_transport.py#L42) | Test fixture/support: rpc server. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`config`](../../../../tests/test_irobot/test_sawyer_transport.py#L54) | Test fixture/support: config. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`wait_until`](../../../../tests/test_irobot/test_sawyer_transport.py#L66) | Test fixture/support: wait until. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_publisher_negotiates_and_never_replays_commands`](../../../../tests/test_irobot/test_sawyer_transport.py#L74) | Assert: publisher negotiates and never replays commands. Protect this behavior against regression. |
| [`test_bad_subscriber_hash_is_rejected`](../../../../tests/test_irobot/test_sawyer_transport.py#L127) | Assert: bad subscriber hash is rejected. Protect this behavior against regression. |
| [`PeerHandler.handle`](../../../../tests/test_irobot/test_sawyer_transport.py#L149) | Test fixture/support: handle. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`robot_peer`](../../../../tests/test_irobot/test_sawyer_transport.py#L199) | Test fixture/support: robot peer. Supplies controlled objects, feedback or peer behavior for this file's stated assertions. |
| [`test_direct_controller_connects_and_previews_neutral_without_ros`](../../../../tests/test_irobot/test_sawyer_transport.py#L236) | Assert: direct controller connects and previews neutral without ros. Protect this behavior against regression. |
| [`test_direct_service_response_and_fragmented_reads`](../../../../tests/test_irobot/test_sawyer_transport.py#L270) | Assert: direct service response and fragmented reads. Protect this behavior against regression. |
| [`test_service_failure_is_not_reported_as_success`](../../../../tests/test_irobot/test_sawyer_transport.py#L285) | Assert: service failure is not reported as success. Protect this behavior against regression. |
| [`test_joint_subsets_and_fields_survive_delayed_polling`](../../../../tests/test_irobot/test_sawyer_transport.py#L297) | Assert: joint subsets and fields survive delayed polling. Protect this behavior against regression. |
| [`test_delayed_callback_preserves_receive_time`](../../../../tests/test_irobot/test_sawyer_transport.py#L323) | Assert: delayed callback preserves receive time. Protect this behavior against regression. |
| [`test_frame_limits_before_payload_allocation`](../../../../tests/test_irobot/test_sawyer_transport.py#L335) | Assert: frame limits before payload allocation. Protect this behavior against regression. |
| [`test_mid_frame_disconnect_is_reported`](../../../../tests/test_irobot/test_sawyer_transport.py#L346) | Assert: mid frame disconnect is reported. Protect this behavior against regression. |
| [`test_partial_transport_startup_closes_bound_server`](../../../../tests/test_irobot/test_sawyer_transport.py#L357) | Assert: partial transport startup closes bound server. Protect this behavior against regression. |
| [`test_partial_transport_startup_closes_bound_server.fail_listener`](../../../../tests/test_irobot/test_sawyer_transport.py#L361) | Local `fail_listener` helper for `test_partial_transport_startup_closes_bound_server`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_master_restart_restores_registration_without_command_replay`](../../../../tests/test_irobot/test_sawyer_transport.py#L375) | Assert: master restart restores registration without command replay. Protect this behavior against regression. |
| [`test_master_restart_restores_registration_without_command_replay.get_pid`](../../../../tests/test_irobot/test_sawyer_transport.py#L379) | Local `get_pid` helper for `test_master_restart_restores_registration_without_command_replay`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_master_restart_restores_registration_without_command_replay.register`](../../../../tests/test_irobot/test_sawyer_transport.py#L385) | Local `register` helper for `test_master_restart_restores_registration_without_command_replay`; supplies that scenario's callback, peer behavior or injected condition. |
| [`test_shared_observers_and_late_publisher_connection`](../../../../tests/test_irobot/test_sawyer_transport.py#L441) | Assert: shared observers and late publisher connection. Protect this behavior against regression. |
| [`test_late_io_observer_receives_merged_retained_state_without_new_packets`](../../../../tests/test_irobot/test_sawyer_transport.py#L464) | Assert: late io observer receives merged retained state without new packets. Protect this behavior against regression. |


### [`tests/test_irobot/test_sawyer_wire.py`](../../../../tests/test_irobot/test_sawyer_wire.py)

**What/why:** Known wire layout, schema round trips and malformed payload rejection. Regress these contracts without relying on live robot movement.

| Defined function / method | What / why in this test file |
| --- | --- |
| [`test_joint_state_matches_known_ros1_layout`](../../../../tests/test_irobot/test_sawyer_wire.py#L12) | Assert: joint state matches known ros1 layout. Protect this behavior against regression. |
| [`test_all_packaged_schemas_round_trip`](../../../../tests/test_irobot/test_sawyer_wire.py#L30) | Assert: all packaged schemas round trip. Protect this behavior against regression. |
| [`test_wire_names_preserved_with_python_aliases`](../../../../tests/test_irobot/test_sawyer_wire.py#L36) | Assert: wire names preserved with python aliases. Protect this behavior against regression. |
| [`test_truncated_or_trailing_payloads_fail`](../../../../tests/test_irobot/test_sawyer_wire.py#L49) | Assert: truncated or trailing payloads fail. Protect this behavior against regression. |
| [`test_untrusted_array_length_is_bounded`](../../../../tests/test_irobot/test_sawyer_wire.py#L54) | Assert: untrusted array length is bounded. Protect this behavior against regression. |
| [`test_invalid_boolean_and_timestamp_fail`](../../../../tests/test_irobot/test_sawyer_wire.py#L59) | Assert: invalid boolean and timestamp fail. Protect this behavior against regression. |
| [`test_fixed_arrays_require_exact_length`](../../../../tests/test_irobot/test_sawyer_wire.py#L66) | Assert: fixed arrays require exact length. Protect this behavior against regression. |
| [`test_unknown_fields_fail_instead_of_being_ignored`](../../../../tests/test_irobot/test_sawyer_wire.py#L71) | Assert: unknown fields fail instead of being ignored. Protect this behavior against regression. |


## Removed bridge assets and local artifacts

- `examples/bridge.launch.py` was removed: the current server uses the direct
  Python backend and no longer launches a separate bridge.
- `installation/bridge-release.json` was removed: the explicit installer uses the
  direct Humble dependency lock instead of a bridge release.
- `.sawyer-runtime/` holds ignored local environments, generated builds, downloaded
  framework sources, reports and logs. These are reproduction artifacts rather
  than maintained driver source files. Versioned evidence is under `validation/`.
- `/tmp/` experiment scripts and wheel builds are temporary local artifacts, not
  an additional supported public API. Downloaded upstream SDK/reference genpy
  trees are development inputs, not runtime dependencies.

## Keep this map and status accurate

When changing a module, update its file/function explanation here. When changing
capability scope, update CAPABILITIES.md, the original checklist and the generated
operation ledger mapping; do not mark a hardware item complete from a unit test.
Record new physical evidence separately, preserving earlier failures. Resolve the
enable/homing blocker before any further automatic recovery or motion requalification.
See [UNFINISHED_WORK.md](UNFINISHED_WORK.md) for the remaining work and acceptance
criteria, including the legacy smoke-script mismatch found while writing this map.
