# Sawyer native topics and driver handling

Definition-line links refer to the current driver source.

This document distinguishes **Sawyer/Intera publishes → our driver subscribes**
from **our driver publishes → Sawyer/Intera subscribes**. These are native ROS 1
protocol interfaces reached directly by Python using XML-RPC/TCPROS. The
optional ROS 2 adapter exposes separate ROS 2 endpoints backed by this driver.

Topic defaults come from [config.py](config.py); changing a configuration field
changes the corresponding endpoint. Registering a publisher does not send a
command. Registering a subscription does not imply a current sample is available.
Most peripheral/action endpoints are opened lazily when that feature is used.

Quiet change-only/latched subscriptions remain connected between messages.
`Subscription.receive_stream` waits for socket readability before the bounded
frame read; silence does not refresh cached receipt timestamps. Partial frames
still have a deadline. See [transport implementation](core/communication/transport.py#L284).

## 1. Native state: Sawyer publishes, our driver subscribes

The six arm feedback subscriptions are installed by
[`ArmSession.__init__` — `core/hardware/session.py:14`](core/hardware/session.py#L14) when opening the driver session.
The remaining rows identify optional/lazy observations.

| Native topic (default) | Message type | Configuration / naming | When observed | Handler functions and source lines |
| --- | --- | --- | --- | --- |
| `/robot/joint_states` | `sensor_msgs/JointState` | [`joint_state_topic` — `config.py:177`](config.py#L177) | Every arm session | [`ArmSession.on_joints` — `core/hardware/session.py:72`](core/hardware/session.py#L72); [`JointFeedback.update` — `core/hardware/feedback.py:37`](core/hardware/feedback.py#L37); [`JointFeedback.issues` — `core/hardware/feedback.py:100`](core/hardware/feedback.py#L100) — Merge named position/velocity/effort fields with per-joint receipt times; reject invalid data and detect stale fields. Public reads use SawyerBase.joint_positions/joint_velocities/joint_efforts. |
| `/robot/state` | `intera_core_msgs/RobotAssemblyState` | [`robot_state_topic` — `config.py:178`](config.py#L178) | Every arm session | [`ArmSession.on_state` — `core/hardware/session.py:90`](core/hardware/session.py#L90); [`ArmSession.input_issues` — `core/hardware/session.py:119`](core/hardware/session.py#L119); [`ArmSession.check_ready` — `core/hardware/session.py:156`](core/hardware/session.py#L156) — Store enabled/ready/stopped/error/homed/E-stop feedback and its age. Enable/reset/stop completion checks read this state. |
| `/robot/limb/right/collision_detection_state` | `intera_core_msgs/CollisionDetectionState` | [`collision_topic` — `config.py:195`](config.py#L195) | Every arm session | [`ArmSession.on_collision` — `core/hardware/session.py:93`](core/hardware/session.py#L93); [`ArmSession.input_issues` — `core/hardware/session.py:119`](core/hardware/session.py#L119) — Cache collision_state and receipt time; missing/stale collision feedback blocks arm execution. |
| `/robot/joint_limits` | `intera_core_msgs/JointLimits` | [`joint_limits_topic` — `config.py:196`](config.py#L196) | Every arm session | [`ArmSession.on_limits` — `core/hardware/session.py:99`](core/hardware/session.py#L99); [`JointBounds.from_message` — `core/types.py:106`](core/types.py#L106); [`check_trajectory_limits` — `core/execution/trajectory.py:18`](core/execution/trajectory.py#L18) — Validate/reorder published bounds and effort limits; apply them to plans and commands. |
| `/robot/limb/right/endpoint_state` | `intera_core_msgs/EndpointState` | [`endpoint_topic` — `config.py:212`](config.py#L212) | Every arm session | [`EndpointFeedback.on_endpoint` — `core/hardware/feedback.py:149`](core/hardware/feedback.py#L149); [`EndpointFeedback.select` — `core/hardware/feedback.py:220`](core/hardware/feedback.py#L220) — Cache anonymous endpoint data; use it only under the named-tip matching rules rather than assuming its tool identity. |
| `/robot/limb/right/tip_states` | `intera_core_msgs/EndpointStates` | [`tip_states_topic` — `config.py:213`](config.py#L213) | Every arm session | [`EndpointFeedback.on_tips` — `core/hardware/feedback.py:162`](core/hardware/feedback.py#L162); [`EndpointFeedback.read` — `core/hardware/feedback.py:204`](core/hardware/feedback.py#L204); [`SawyerBase.get_tip_state` — `core/base.py:240`](core/base.py#L240) — Resolve advertised names, merge parent-header metadata and require fresh frame-qualified state. The tested named tip is right_hand. |
| `/io/end_effector/right_gripper/state` | `intera_core_msgs/IODeviceStatus` | [`gripper_namespace` — `config.py:216`](config.py#L216) | At connection if enable_gripper=True; also through optional typed IO | [`SawyerGripper.__init__` — `core/hardware/gripper.py:12`](core/hardware/gripper.py#L12); [`SawyerGripper.on_state` — `core/hardware/gripper.py:27`](core/hardware/gripper.py#L27); [`SawyerGripper.completed` — `core/hardware/gripper.py:132`](core/hardware/gripper.py#L132); [`SawyerGripper.wait_completion` — `core/hardware/gripper.py:181`](core/hardware/gripper.py#L181) — Merge change-only gripper signals, match command ACKs and check measured/stable completion. IODevice additionally exposes generic typed device state. |
| `/robot/head/head_state` | `intera_core_msgs/HeadState` | [`head_state_topic` — `config.py:187`](config.py#L187) | On head_state()/move_head() | [`SawyerDevices.head_state` — `core/hardware/peripherals.py:425`](core/hardware/peripherals.py#L425); [`SawyerDevices.move_head` — `core/hardware/peripherals.py:443`](core/hardware/peripherals.py#L443) — Read fresh pan feedback and check arrival for the existing head API. Head-pan physical qualification remains deferred. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | [`diagnostics_topic` — `config.py:206`](config.py#L206) | On diagnostics(); also telemetry/diagnostic recorder | [`SawyerDevices.diagnostics` — `core/hardware/peripherals.py:120`](core/hardware/peripherals.py#L120); [`SawyerDevices._read` — `core/hardware/peripherals.py:203`](core/hardware/peripherals.py#L203) — Expose the received diagnostic message. The dedicated recorder aggregates sparse statuses by name; this convenience getter itself caches the latest message. |
| `/tf` | `tf2_msgs/TFMessage` | [`tf_topic` — `config.py:207`](config.py#L207) | On devices.transforms(); also ROS 2 telemetry | [`TransformCache.__init__` — `core/hardware/transforms.py:13`](core/hardware/transforms.py#L13); [`TransformCache.update` — `core/hardware/transforms.py:27`](core/hardware/transforms.py#L27); [`TransformCache.snapshot` — `core/hardware/transforms.py:43`](core/hardware/transforms.py#L43) — Validate and cache transform edges; omit expired dynamic transforms from snapshots. |
| `/tf_static` | `tf2_msgs/TFMessage` | [`tf_static_topic` — `config.py:208`](config.py#L208) | On devices.transforms(); also ROS 2 telemetry | [`TransformCache.__init__` — `core/hardware/transforms.py:13`](core/hardware/transforms.py#L13); [`TransformCache.update` — `core/hardware/transforms.py:27`](core/hardware/transforms.py#L27); [`TransformCache.snapshot` — `core/hardware/transforms.py:43`](core/hardware/transforms.py#L43) — Validate and cache transform edges; retain static transforms without dynamic age expiry. |
| `/io/internal_camera/head_camera/image_rect_color` | `sensor_msgs/Image` | [`camera_namespace` — `config.py:190`](config.py#L190) | On camera_frame()/camera callback; optional --cameras telemetry | [`SawyerDevices._camera` — `core/hardware/peripherals.py:518`](core/hardware/peripherals.py#L518); [`SawyerDevices.register_camera_callback` — `core/hardware/peripherals.py:530`](core/hardware/peripherals.py#L530); [`SawyerDevices.camera_frame` — `core/hardware/peripherals.py:645`](core/hardware/peripherals.py#L645) — Read frames for head camera; camera_frame requires explicit streaming ownership, post-request advancing identities and recent receipt. Reading does not start streaming. |
| `/io/internal_camera/head_camera/camera_info` | `sensor_msgs/CameraInfo` | [`camera_namespace` — `config.py:190`](config.py#L190) | On camera_frame(); optional --cameras telemetry | [`SawyerDevices.camera_frame` — `core/hardware/peripherals.py:645`](core/hardware/peripherals.py#L645) — Return the camera calibration/intrinsic message alongside the image. Advancing-image/freshness checks do not establish synchronized calibration acquisition. |
| `/io/internal_camera/right_hand_camera/image_rect` | `sensor_msgs/Image` | [`camera_namespace` — `config.py:190`](config.py#L190) | On camera_frame()/camera callback; optional --cameras telemetry | [`SawyerDevices._camera` — `core/hardware/peripherals.py:518`](core/hardware/peripherals.py#L518); [`SawyerDevices.register_camera_callback` — `core/hardware/peripherals.py:530`](core/hardware/peripherals.py#L530); [`SawyerDevices.camera_frame` — `core/hardware/peripherals.py:645`](core/hardware/peripherals.py#L645) — Read frames for hand camera; camera_frame requires explicit streaming ownership, post-request advancing identities and recent receipt. Reading does not start streaming. |
| `/io/internal_camera/right_hand_camera/camera_info` | `sensor_msgs/CameraInfo` | [`camera_namespace` — `config.py:190`](config.py#L190) | On camera_frame(); optional --cameras telemetry | [`SawyerDevices.camera_frame` — `core/hardware/peripherals.py:645`](core/hardware/peripherals.py#L645) — Return the camera calibration/intrinsic message alongside the image. Advancing-image/freshness checks do not establish synchronized calibration acquisition. |
| `/robot/digital_io/<name>/state` | `intera_core_msgs/DigitalIOState` | Name supplied to digital_state()/set_digital_output() | On explicit digital IO read/write | [`SawyerDevices._digital_state` — `core/hardware/peripherals.py:555`](core/hardware/peripherals.py#L555); [`SawyerDevices.set_digital_output` — `core/hardware/peripherals.py:563`](core/hardware/peripherals.py#L563) — Read value and is_input_only; output writes require a permitted component and a matching post-send measured state. Optional telemetry observes right_hand_camera_power specifically. |
| `/robot/digital_io/right_upper_button/state` | `intera_core_msgs/DigitalIOState` | Literal recorder endpoint | Only in the subscription-only diagnostic CLI, or an explicit digital_state() read | [`record` — `examples/diagnose_direct.py:122`](examples/diagnose_direct.py#L122); [`DiagnosticRecording.receive_input` — `examples/diagnose_direct.py:95`](examples/diagnose_direct.py#L95) — Record legacy upper-cuff-button changes; no publisher is created. |
| `/robot/analog_io/right_wheel/state` | `intera_core_msgs/AnalogIOState` | Literal recorder endpoint | Only in the subscription-only diagnostic CLI | [`record` — `examples/diagnose_direct.py:122`](examples/diagnose_direct.py#L122); [`DiagnosticRecording.receive_input` — `examples/diagnose_direct.py:95`](examples/diagnose_direct.py#L95) — Record legacy navigator-wheel values/changes; this is not a general analog-output API. |
| `/robot/limb/right/interaction_controller/state` | `intera_core_msgs/InteractionControlState` | [`interaction_state_topic` — `config.py:203`](config.py#L203) | On a requested interaction session | [`InteractionSession.__init__` — `core/execution/interaction.py:20`](core/execution/interaction.py#L20); [`InteractionSession.on_state` — `core/execution/interaction.py:34`](core/execution/interaction.py#L34); [`InteractionSession.state_ready` — `core/execution/interaction.py:37`](core/execution/interaction.py#L37); [`InteractionSession.deactivate` — `core/execution/interaction.py:58`](core/execution/interaction.py#L58) — Confirm fresh active/inactive interaction states. Normal confirmed exit does not establish process-loss deactivation. |
| `/robot/homing_states` | `intera_core_msgs/HomingState` | [`homing_state_topic` — `config.py:179`](config.py#L179) | Subscription-only diagnostic CLI | [`record` — `examples/diagnose_direct.py:122`](examples/diagnose_direct.py#L122); [`DiagnosticRecording.receive` — `examples/diagnose_direct.py:58`](examples/diagnose_direct.py#L58) — Merge paired per-joint homing enums without sending homing commands. Homed=0, homing=1, not-homed=2. Malformed field lengths are rejected before updating the cache. |


## 2. Typed IO state/configuration: shared namespace pattern

These topics use the same handler implementation rather than a separate callback
for every device. [`SawyerDevices._io` — `core/hardware/peripherals.py:57`](core/hardware/peripherals.py#L57) creates/reuses
[`IODevice.__init__` — `core/hardware/io.py:142`](core/hardware/io.py#L142), which observes both `<namespace>/state`
and `<namespace>/config` on the shared transport.

- `state`: `intera_core_msgs/IODeviceStatus` → [`IODevice.on_state` — `core/hardware/io.py:165`](core/hardware/io.py#L165).
  Merges changed signals/ports, decodes JSON values, tracks connection metadata,
  invokes registered change callbacks and retains at most 256 queued events
  with drop accounting. Unchanged values are retained, not independently refreshed.
- `config`: `intera_core_msgs/IODeviceConfiguration` →
  [`IODevice.on_config` — `core/hardware/io.py:162`](core/hardware/io.py#L162). Configuration supports subsequent
  type/range/role validation; it is distinct from measured state.
- Public state/events: [`SawyerDevices.read_io` — `core/hardware/peripherals.py:71`](core/hardware/peripherals.py#L71),
  [`SawyerDevices.register_io_callback` — `core/hardware/peripherals.py:103`](core/hardware/peripherals.py#L103),
  [`SawyerDevices.io_events` — `core/hardware/peripherals.py:111`](core/hardware/peripherals.py#L111) and
  [`IODevice.snapshot` — `core/hardware/io.py:224`](core/hardware/io.py#L224).

The supported generic IO methods accept compatible `/io/...` device namespaces;
that is not permission to write every advertised signal. These are the explicit
namespaces used by the convenience APIs, recorder or optional typed telemetry:

| Device state topic | Device configuration topic | Use |
| --- | --- | --- |
| `/io/end_effector/right_gripper/state` | `/io/end_effector/right_gripper/config` | Gripper state/settings; dedicated motion protocol also consumes /state. |
| `/io/internal_camera/head_camera/state` | `/io/internal_camera/head_camera/config` | Head camera streaming and settings. |
| `/io/internal_camera/right_hand_camera/state` | `/io/internal_camera/right_hand_camera/config` | Hand camera streaming and settings. |
| `/io/robot/robot/state` | `/io/robot/robot/config` | Six SDK lights and compatible robot IO metadata/settings. |
| `/io/robot/cuff/state` | `/io/robot/cuff/config` | Cuff inputs; recorder and optional typed telemetry. |
| `/io/robot/navigator/state` | `/io/robot/navigator/config` | Navigator inputs; recorder and optional typed telemetry. |
| `/io/robot/right_end_of_arm/state` | `/io/robot/right_end_of_arm/config` | Convenience raw IO state read; generic typed IO can read its config. |

The raw [`SawyerDevices.io_state` — `core/hardware/peripherals.py:596`](core/hardware/peripherals.py#L596) convenience read
uses [`SawyerDevices._read` — `core/hardware/peripherals.py:203`](core/hardware/peripherals.py#L203) and returns the cached raw
status message. Use `read_io()`/IODevice for a merged typed device snapshot.
Likewise, camera_state() reads status; merely reading it does not change streaming.
The aggregate `/io/robot/state` has type `IONodeStatus`; it must not be confused
with the device `/io/robot/robot/state` of type `IODeviceStatus`.


## 3. Native commands: our driver publishes, Sawyer subscribes

These rows describe implementation, not a command to run. Publishers opened at
connection remain idle until an explicit operation sends data. Motion and
state-changing APIs preview by default unless execution is explicitly requested.

| Native command topic (default) | Message type | Configuration / naming | Sending functions and source lines |
| --- | --- | --- | --- |
| `/robot/limb/right/joint_command` | `intera_core_msgs/JointCommand` | [`joint_command_topic` — `config.py:197`](config.py#L197) | [`ArmSession.__init__` — `core/hardware/session.py:14`](core/hardware/session.py#L14); [`ArmSession.publish_velocity` — `core/hardware/session.py:246`](core/hardware/session.py#L246); [`track` — `core/execution/trajectory.py:59`](core/execution/trajectory.py#L59); [`ArmSession.stop` — `core/hardware/session.py:271`](core/hardware/session.py#L271) — The ordinary planned-move tracker sends VELOCITY_MODE commands. Cleanup requests zero velocity and measured POSITION_MODE hold when permitted; no arm cleanup commands are sent after a reported stop/disable/error. |
| `/robot/limb/right/joint_command` | `intera_core_msgs/JointCommand` | [`joint_command_topic` — `config.py:197`](config.py#L197) | [`JointStream.command` — `core/execution/streaming.py:60`](core/execution/streaming.py#L60); [`JointStream.run` — `core/execution/streaming.py:153`](core/execution/streaming.py#L153) — Explicit bounded streams use the same topic with mode-specific position/velocity/effort/trajectory fields. Torque/trajectory execution remains experimentally gated. |
| `/robot/limb/right/joint_command_timeout` | `std_msgs/Float64` | [`command_timeout_topic` — `config.py:198`](config.py#L198) | [`track` — `core/execution/trajectory.py:59`](core/execution/trajectory.py#L59) — Set/refresh the controller command watchdog. JointStream.prepare/run also uses the same session watchdog publisher. |
| `/robot/limb/right/set_speed_ratio` | `std_msgs/Float64` | [`speed_ratio_topic` — `config.py:199`](config.py#L199) | [`JointStream.prepare` — `core/execution/streaming.py:127`](core/execution/streaming.py#L127) — Set a bounded speed ratio when entering explicit position streaming; this is not a separate Cartesian target command. |
| `/robot/set_super_enable` | `std_msgs/Bool` | [`enable_topic` — `config.py:180`](config.py#L180) | [`SawyerDevices.set_enabled` — `core/hardware/peripherals.py:279`](core/hardware/peripherals.py#L279); [`SawyerDevices._enable_robot` — `core/hardware/peripherals.py:297`](core/hardware/peripherals.py#L297); [`SawyerDevices._wait_enabled` — `core/hardware/peripherals.py:319`](core/hardware/peripherals.py#L319) — Send explicit enable/disable requests. Enable requires fresh joints, bounds displacement, and confirms enabled plus ready from /robot/state. Recovery currently blocks physical release. |
| `/robot/set_super_reset` | `std_msgs/Empty` | [`reset_topic` — `config.py:181`](config.py#L181) | [`SawyerDevices.reset_faults` — `core/hardware/peripherals.py:361`](core/hardware/peripherals.py#L361); [`SawyerDevices._robot_command` — `core/hardware/peripherals.py:257`](core/hardware/peripherals.py#L257) — Request fault reset explicitly and check resulting /robot/state. Never called automatically by connect(). |
| `/robot/set_super_stop` | `std_msgs/Empty` | [`software_stop_topic` — `config.py:182`](config.py#L182) | [`SawyerDevices.software_stop` — `core/hardware/peripherals.py:380`](core/hardware/peripherals.py#L380); [`SawyerDevices._stop_robot` — `core/hardware/peripherals.py:400`](core/hardware/peripherals.py#L400) — Request latched controller stop despite stale assembly feedback; require fresh stopped confirmation. This differs from SawyerController.stop(), which requests current-operation cancellation. |
| `/io/end_effector/right_gripper/command` | `intera_core_msgs/IOComponentCommand` | [`gripper_namespace` — `config.py:216`](config.py#L216) | [`SawyerGripper.make_message` — `core/hardware/gripper.py:89`](core/hardware/gripper.py#L89); [`SawyerGripper.send` — `core/hardware/gripper.py:98`](core/hardware/gripper.py#L98); [`SawyerGripper.perform` — `core/hardware/gripper.py:213`](core/hardware/gripper.py#L213) — Send electric-gripper calibration/open/close/position/stop/start/reboot operations; correlate timestamps against /state command acknowledgements, then check operation-specific completion. |
| `/io/end_effector/right_gripper/command` | `intera_core_msgs/IOComponentCommand` | Compatible IODevice namespace | [`IODevice.make_command` — `core/hardware/io.py:250`](core/hardware/io.py#L250); [`IODevice.write` — `core/hardware/io.py:286`](core/hardware/io.py#L286) — Metadata-checked signal/port settings with timestamp-matched ACKs. Gripper settings do not bypass dedicated motion methods; camera/light convenience calls share this path. |
| `/io/internal_camera/head_camera/command` | `intera_core_msgs/IOComponentCommand` | Compatible IODevice namespace | [`IODevice.make_command` — `core/hardware/io.py:250`](core/hardware/io.py#L250); [`IODevice.write` — `core/hardware/io.py:286`](core/hardware/io.py#L286) — Metadata-checked signal/port settings with timestamp-matched ACKs. Gripper settings do not bypass dedicated motion methods; camera/light convenience calls share this path. |
| `/io/internal_camera/right_hand_camera/command` | `intera_core_msgs/IOComponentCommand` | Compatible IODevice namespace | [`IODevice.make_command` — `core/hardware/io.py:250`](core/hardware/io.py#L250); [`IODevice.write` — `core/hardware/io.py:286`](core/hardware/io.py#L286) — Metadata-checked signal/port settings with timestamp-matched ACKs. Gripper settings do not bypass dedicated motion methods; camera/light convenience calls share this path. |
| `/io/robot/robot/command` | `intera_core_msgs/IOComponentCommand` | Compatible IODevice namespace | [`IODevice.make_command` — `core/hardware/io.py:250`](core/hardware/io.py#L250); [`IODevice.write` — `core/hardware/io.py:286`](core/hardware/io.py#L286) — Metadata-checked signal/port settings with timestamp-matched ACKs. Gripper settings do not bypass dedicated motion methods; camera/light convenience calls share this path. |
| `/io/internal_camera/<head_camera\|right_hand_camera>/command` | `intera_core_msgs/IOComponentCommand` | [`camera_namespace` — `config.py:190`](config.py#L190) | [`SawyerDevices.set_camera_streaming` — `core/hardware/peripherals.py:607`](core/hardware/peripherals.py#L607) — Explicitly set camera_streaming and confirm matching command ACK plus reported state. configure_camera() uses IODevice for other supported settings. |
| `/robot/digital_io/command` | `intera_core_msgs/DigitalOutputCommand` | Literal native topic | [`SawyerDevices.set_digital_output` — `core/hardware/peripherals.py:563`](core/hardware/peripherals.py#L563) — Send name plus boolean value only when not input-only; confirm the named /state changed after sending. |
| `/robot/head/command_head_pan` | `intera_core_msgs/HeadPanCommand` | [`head_command_topic` — `config.py:188`](config.py#L188) | [`SawyerDevices.move_head` — `core/hardware/peripherals.py:443`](core/hardware/peripherals.py#L443) — Existing bounded target/speed/mode command with measured arrival and hold cleanup. Head-pan testing remains deferred. |
| `/robot/head_display` | `sensor_msgs/Image` | [`head_display_topic` — `config.py:189`](config.py#L189) | [`SawyerDevices.display_image` — `core/hardware/peripherals.py:692`](core/hardware/peripherals.py#L692) — Send a validated rgb8/bgr8 image. Transport delivery is not display-visibility confirmation; the response explicitly leaves delivery_confirmed false. |

Other compatible IO namespaces may be addressed through
[`SawyerDevices.write_io` — `core/hardware/peripherals.py:79`](core/hardware/peripherals.py#L79) and
[`IODevice.write` — `core/hardware/io.py:286`](core/hardware/io.py#L286) when metadata permits. The six SDK light names
are explicitly enumerated in `core/hardware/peripherals.py` (`SDK_LIGHTS`); only those
signals have the documented input-direction exception. Cuff/navigator convenience
access is observational; no blanket write permission is implied by a `/command`
registration in the graph.


## 4. Native actions are five-topic conversations

[`MotionActions.client` — `core/execution/native.py:71`](core/execution/native.py#L71) lazily chooses one of the following
configured action servers. These are native Intera actions over TCPROS, distinct
from ROS 2 actions offered by our optional node.

| Action topic | Direction | Message type | Namespace configuration | Driver handler / sender |
| --- | --- | --- | --- | --- |
| `/motion/motion_command/goal` | Driver publishes → Sawyer subscribes | `intera_motion_msgs/MotionCommandActionGoal` | [`native_motion_action` — `config.py:209`](config.py#L209) | [`NativeActionClient._start` — `core/communication/actions.py:118`](core/communication/actions.py#L118) — Publish a uniquely identified typed goal; scheduling stamps stay zero because clocks differ. |
| `/motion/motion_command/cancel` | Driver publishes → Sawyer subscribes | `actionlib_msgs/GoalID` | [`native_motion_action` — `config.py:209`](config.py#L209) | [`NativeActionClient.cancel` — `core/communication/actions.py:104`](core/communication/actions.py#L104) — Cancel only this nonempty goal ID; never issue a global cancel. |
| `/motion/motion_command/status` | Sawyer publishes → driver subscribes | `actionlib_msgs/GoalStatusArray` | [`native_motion_action` — `config.py:209`](config.py#L209) | [`NativeActionClient.on_status` — `core/communication/actions.py:64`](core/communication/actions.py#L64) — Track matching goal status and action-server heartbeat. |
| `/motion/motion_command/feedback` | Sawyer publishes → driver subscribes | `intera_motion_msgs/MotionCommandActionFeedback` | [`native_motion_action` — `config.py:209`](config.py#L209) | [`NativeActionClient.on_feedback` — `core/communication/actions.py:85`](core/communication/actions.py#L85) — Accept progress belonging to the active goal only. |
| `/motion/motion_command/result` | Sawyer publishes → driver subscribes | `intera_motion_msgs/MotionCommandActionResult` | [`native_motion_action` — `config.py:209`](config.py#L209) | [`NativeActionClient.on_result` — `core/communication/actions.py:72`](core/communication/actions.py#L72) — Accept a matching terminal result; a received result can still indicate failure. |
| `/calibration_command/goal` | Driver publishes → Sawyer subscribes | `intera_core_msgs/CalibrationCommandActionGoal` | [`calibration_action` — `config.py:210`](config.py#L210) | [`NativeActionClient._start` — `core/communication/actions.py:118`](core/communication/actions.py#L118) — Publish a uniquely identified typed goal; scheduling stamps stay zero because clocks differ. |
| `/calibration_command/cancel` | Driver publishes → Sawyer subscribes | `actionlib_msgs/GoalID` | [`calibration_action` — `config.py:210`](config.py#L210) | [`NativeActionClient.cancel` — `core/communication/actions.py:104`](core/communication/actions.py#L104) — Cancel only this nonempty goal ID; never issue a global cancel. |
| `/calibration_command/status` | Sawyer publishes → driver subscribes | `actionlib_msgs/GoalStatusArray` | [`calibration_action` — `config.py:210`](config.py#L210) | [`NativeActionClient.on_status` — `core/communication/actions.py:64`](core/communication/actions.py#L64) — Track matching goal status and action-server heartbeat. |
| `/calibration_command/feedback` | Sawyer publishes → driver subscribes | `intera_core_msgs/CalibrationCommandActionFeedback` | [`calibration_action` — `config.py:210`](config.py#L210) | [`NativeActionClient.on_feedback` — `core/communication/actions.py:85`](core/communication/actions.py#L85) — Accept progress belonging to the active goal only. |
| `/calibration_command/result` | Sawyer publishes → driver subscribes | `intera_core_msgs/CalibrationCommandActionResult` | [`calibration_action` — `config.py:210`](config.py#L210) | [`NativeActionClient.on_result` — `core/communication/actions.py:72`](core/communication/actions.py#L72) — Accept a matching terminal result; a received result can still indicate failure. |

Topic registration is in [`NativeActionClient.__init__` — `core/communication/actions.py:37`](core/communication/actions.py#L37);
[`NativeActionClient._await_result` — `core/communication/actions.py:132`](core/communication/actions.py#L132) enforces deadline,
heartbeat, generation and cancellation checks. Public entry points are
[`SawyerController.move_native_joint_path` — `core/controller.py:142`](core/controller.py#L142),
[`SawyerController.execute_native_trajectory` — `core/controller.py:172`](core/controller.py#L172) and
[`SawyerController.calibrate_arm` — `core/controller.py:201`](core/controller.py#L201).

The native action selector exposes only its declared motion and calibration
interfaces. Additional vendor actions need documented schemas and support.


## 5. Interaction command/state pair

This pair is separate from native motion actions and from normal joint tracking.

| Topic / direction | Type | Functions and handling |
| --- | --- | --- |
| `/robot/limb/right/interaction_control_command` — driver publishes → Sawyer subscribes | `intera_core_msgs/InteractionControlCommand` | [`InteractionSession.__init__` — `core/execution/interaction.py:20`](core/execution/interaction.py#L20); [`InteractionSession.run` — `core/execution/interaction.py:76`](core/execution/interaction.py#L76); [`InteractionSession.deactivate` — `core/execution/interaction.py:58`](core/execution/interaction.py#L58) — publish bounded active commands, then inactive and confirm exit. |
| `/robot/limb/right/interaction_controller/state` — Sawyer publishes → driver subscribes | `intera_core_msgs/InteractionControlState` | [`InteractionSession.on_state` — `core/execution/interaction.py:34`](core/execution/interaction.py#L34); [`InteractionSession.state_ready` — `core/execution/interaction.py:37`](core/execution/interaction.py#L37) — match active/inactive feedback with receipt freshness. |

The configured topics are [`interaction_command_topic` — `config.py:200`](config.py#L200) and
[`interaction_state_topic` — `config.py:203`](config.py#L203).
Compliant/force operation requires explicit experimental opt-in.


## 6. Services and parameters are not state/command topics

The master endpoint in `ros_master_uri` discovers peers; it is not the socket
through which every joint command is relayed. Topic payloads travel directly
between negotiated publishers/subscribers.

| Interface | Kind | Driver functions and role |
| --- | --- | --- |
| `/ExternalTools/right/PositionKinematicsNode/IKService` | `intera_core_msgs/SolvePositionIK` service | [`InteraKinematics.__init__` — `core/hardware/kinematics.py:17`](core/hardware/kinematics.py#L17); [`InteraKinematics.inverse` — `core/hardware/kinematics.py:96`](core/hardware/kinematics.py#L96) — request a seeded joint solution without moving. |
| `/ExternalTools/right/PositionKinematicsNode/FKService` | `intera_core_msgs/SolvePositionFK` service | [`InteraKinematics.forward_results` — `core/hardware/kinematics.py:151`](core/hardware/kinematics.py#L151); [`InteraKinematics.forward` — `core/hardware/kinematics.py:198`](core/hardware/kinematics.py#L198) — request poses and validity/collision information without moving. |
| Supported direct service exchange | TCPROS request/response | [`ServiceClient.wait_for_service` — `core/communication/transport.py:366`](core/communication/transport.py#L366); [`ServiceClient.call` — `core/communication/transport.py:377`](core/communication/transport.py#L377) — XML-RPC lookupService discovers the server, then TCPROS carries request/result. |
| `/named_poses/right` | XML-RPC parameter | [`SawyerController._neutral_pose` — `core/controller.py:251`](core/controller.py#L251) — read/reorder poses.neutral; explicit neutral_joints overrides this lookup. |
| `/robot_description` and version/config parameters | XML-RPC parameters | [`SawyerBase.get_parameter` — `core/base.py:212`](core/base.py#L212); [`SawyerBase.robot_description` — `core/base.py:218`](core/base.py#L218); [`SawyerBase.check_version` — `core/base.py:102`](core/base.py#L102) — read model and identity; no topic publisher is created for parameter reads. |


## 7. What happens in transport.py and wire.py

An application does not need to register topics manually. The driver wires the
handlers above through these functions:

| Stage | Source function / line | Behavior |
| --- | --- | --- |
| Subscribe/register | [`DirectTransport.create_subscription` — `core/communication/transport.py:514`](core/communication/transport.py#L514); [`DirectTransport.observe` — `core/communication/transport.py:537`](core/communication/transport.py#L537) | Register with master using registerSubscriber; share existing subscriptions with additional in-process observers. |
| Connect to a publisher | [`Subscription.update` — `core/communication/transport.py:245`](core/communication/transport.py#L245); [`Subscription.receive_stream` — `core/communication/transport.py:284`](core/communication/transport.py#L284) | Negotiate requestTopic/TCPROS, connect, check type/MD5, read frames and assign local monotonic receipt times. |
| Decode | [`deserialize` — `core/communication/wire.py:196`](core/communication/wire.py#L196); [`Reader.read` — `core/communication/wire.py:167`](core/communication/wire.py#L167) | Use packaged wire_schema.json to decode fields and enforce payload/type limits. |
| Queue feedback | [`Subscription.retain` — `core/communication/transport.py:201`](core/communication/transport.py#L201); [`DirectTransport.enqueue` — `core/communication/transport.py:549`](core/communication/transport.py#L549) | Keep bounded pending observations; preserve partial joint fields and change-only IO, retain late-observer feedback without refreshing its original age. |
| Dispatch the handler | [`DirectTransport.spin_once` — `core/communication/transport.py:773`](core/communication/transport.py#L773); [`Subscription.dispatch` — `core/communication/transport.py:184`](core/communication/transport.py#L184) | Run registered callbacks during driver polling/control; socket reception itself does not complete the public operation. |
| Create a publisher | [`DirectTransport.create_publisher` — `core/communication/transport.py:527`](core/communication/transport.py#L527); [`DirectTransport.publisher_for` — `core/communication/transport.py:557`](core/communication/transport.py#L557) | Register with master using registerPublisher; reuse compatible routes; registration sends no command payload. |
| Accept Sawyer subscriber | [`DirectTransport.request_topic` — `core/communication/transport.py:703`](core/communication/transport.py#L703); [`DirectTransport.serve_subscriber` — `core/communication/transport.py:729`](core/communication/transport.py#L729) | Advertise the local TCPROS endpoint, validate schema and attach the controller subscriber; latching is disabled. |
| Encode and publish | [`Publisher.publish` — `core/communication/transport.py:131`](core/communication/transport.py#L131); [`serialize` — `core/communication/wire.py:139`](core/communication/wire.py#L139) | Serialize the typed command, add TCPROS length framing and send to current subscribers; no command history is replayed to late subscribers. |
| Feedback health / control guard | [`ArmSession.input_issues` — `core/hardware/session.py:119`](core/hardware/session.py#L119); [`ArmSession.check_ready` — `core/hardware/session.py:156`](core/hardware/session.py#L156) | Require appropriate fresh joints/state/collision/limits before and during arm execution. |
| Registration recovery | [`DirectTransport.refresh_registration` — `core/communication/transport.py:576`](core/communication/transport.py#L576) | Restore graph registration after master changes without replaying commands; active sessions detect generation changes. |

Schema data is [installation/wire_schema.json](installation/wire_schema.json),
loaded by [core/communication/wire.py](core/communication/wire.py). `wire.py` does not choose topic names;
it defines/encodes/decodes messages. `transport.py` moves those messages. The
session/peripheral/action modules decide which topic and callback to connect.


The optional diagnostic recorder also **subscribes** to enable/reset/stop,
homing, joint-command and gripper-command topics. It observes other publishers;
it creates no publishers and sends no commands. `DiagnosticRecording.write_trace`
can save raw dispatched messages with their receipt/dispatch timestamps to a new
JSONL file. The summary retains the latest observed command per label. See
[recorder](examples/diagnose_direct.py). A dispatched-message trace can
coalesce messages and is not a lossless packet capture.

## 8. Example trace: move_to_neutral(execute=True)

This is a source walkthrough, not a request to move the currently faulted robot.

1. [`SawyerController.move_to_neutral` — `core/controller.py:351`](core/controller.py#L351) acquires operation
   ownership and resolves the neutral target through `_neutral_pose()`.
2. [`SawyerController._move_joints` — `core/controller.py:278`](core/controller.py#L278) reads measured joints
   already received on `/robot/joint_states` and constructs a validated joint plan.
3. [`SawyerController._run_joint_plan` — `core/controller.py:308`](core/controller.py#L308) invokes the session
   tracker for explicit execution. A preview returns the plan without this loop.
4. [`track` — `core/execution/trajectory.py:59`](core/execution/trajectory.py#L59) checks state/limits/freshness, publishes
   `/robot/limb/right/joint_command_timeout`, and samples the reference repeatedly.
5. [`ArmSession.publish_velocity` — `core/hardware/session.py:246`](core/hardware/session.py#L246) builds `JointCommand` with
   `VELOCITY_MODE` and publishes `/robot/limb/right/joint_command`. The normal
   neutral implementation does not send one special "neutral" topic message and
   does not use the native motion action server.
6. Incoming `/robot/joint_states`, `/robot/state` and collision/limit feedback
   reach their callbacks through `Subscription.receive_stream()` → decoding →
   queue → `DirectTransport.spin_once()` → `ArmSession` handlers.
7. The tracker evaluates position error, velocity and settling criteria; faults,
   stale data, missing routes, cancellation or deadlines interrupt the operation.
8. [`ArmSession.stop` — `core/hardware/session.py:271`](core/hardware/session.py#L271) performs the permitted mode-exit commands;
   the application receives a result only after normal execution completes.

`move_to_neutral()` without `execute=True` previews. Neither form automatically
connects, enables or resets Sawyer. Connection is owned by `SawyerBase.connect()`
or its context manager. Enable/homing recovery remains an unresolved physical
release blocker; head pan is deferred. See [UNFINISHED_WORK.md](UNFINISHED_WORK.md).


## 9. Optional ROS 2 and recorder consumers

These consumers reuse native observations; their ROS 2 topic names must not be
confused with Sawyer's native names.

| Consumer | Functions / lines | Native handling and output |
| --- | --- | --- |
| Standard ROS 2 node | [`SawyerNode.__init__` — `ros2.py:91`](ros2.py#L91); [`SawyerNode._feedback` — `ros2.py:338`](ros2.py#L338); [`SawyerNode._publish` — `ros2.py:413`](ros2.py#L413) | Publish measured joint_states, endpoint_pose, status and standard trajectory feedback under the node namespace (default /sawyer). Control callbacks cache feedback; executor timers publish it. |
| ROS 2 telemetry | [`SawyerTelemetry.__init__` — `ros2_telemetry.py:105`](ros2_telemetry.py#L105); [`SawyerTelemetry.observe` — `ros2_telemetry.py:226`](ros2_telemetry.py#L226); [`SawyerTelemetry.receive` — `ros2_telemetry.py:248`](ros2_telemetry.py#L248); [`SawyerTelemetry.publish` — `ros2_telemetry.py:326`](ros2_telemetry.py#L326) | Observe native joints, named endpoints, diagnostics and TF; optional interfaces add assembly/limits/IO/digital state/config/events/health. --cameras adds image and CameraInfo observations without implicitly starting streams. |
| ROS 2 typed request handling | [`ExtendedInterfaces.__init__` — `ros2_interfaces.py:78`](ros2_interfaces.py#L78); [`ExtendedInterfaces.io_callback` — `ros2_interfaces.py:657`](ros2_interfaces.py#L657); [`ExtendedInterfaces.receive_command` — `ros2_interfaces.py:319`](ros2_interfaces.py#L319) | ROS 2 services/actions/stream commands call the same Python backend. No ROS 2 client writes a native TCPROS topic directly. |
| Subscription-only recorder | [`record` — `examples/diagnose_direct.py:122`](examples/diagnose_direct.py#L122); [`DiagnosticRecording.receive` — `examples/diagnose_direct.py:58`](examples/diagnose_direct.py#L58); [`DiagnosticRecording.receive_joints` — `examples/diagnose_direct.py:79`](examples/diagnose_direct.py#L79); [`DiagnosticRecording.receive_input` — `examples/diagnose_direct.py:95`](examples/diagnose_direct.py#L95) | Observe assembly, per-joint homing, diagnostics, joints, cuff/navigator IO and legacy upper-button/right-wheel state. Aggregate sparse diagnostics and bounded events; create no command publishers. |

ROS 2 messages are converted by
[`to_ros` — `ros2_telemetry.py:26`](ros2_telemetry.py#L26) / [`to_wire` — `ros2_telemetry.py:70`](ros2_telemetry.py#L70);
large standard Image CDR uses [`serialize_image` — `ros2_images.py:12`](ros2_images.py#L12).
Receipt-time headers do not establish synchronized sensor acquisition because
controller and workstation clocks may differ.
