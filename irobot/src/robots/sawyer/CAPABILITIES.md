# Sawyer capability coverage

Updated November 13, 2026, branch `sawyer_py`. **Full SDK qualification is open.**
Head pan is deferred. Sawyer currently remains disabled, software-stopped and
in error following unexpected movement during enable recovery.

See the [implementation guide](IMPLEMENTATION_GUIDE.md) for files/functions and
their design reasons, and [unfinished work](UNFINISHED_WORK.md) for open items.

[CAPABILITY_LEDGER.json](CAPABILITY_LEDGER.json) accounts for all **226 public
methods in 28 SDK files**, including Python/ROS 2/controller interfaces, units,
limits, side effects, cancellation, availability and evidence. This is a
capability mapping, not an SDK-compatible import replacement. Method builders,
file utilities and random target generators are classified as application/data
helpers, rather than additional robot capabilities. Runtime access is available
as `robot.capabilities` without commanding hardware.

| Capability | Implementation and ROS 2 access | Physical status |
| --- | --- | --- |
| Identity, parameters, inventory | Direct master queries; inventory/read_parameter/capabilities services | Read-only passed |
| Assembly, joints, efforts, limits | Python feedback; status, robot_state, joint_states, joint_limits | Passed; stopped-state diagnostics preserved |
| Enable/reset/disable/software stop | Explicit Python calls and services; ready/displacement guards | **Failed recovery again after restart**; 0.005 rad guard physically latched stop; stale/busy stop refinements regression-tested |
| Neutral, joints, rest-to-rest waypoints | Bounded planner; neutral Trigger, FollowJointTrajectory subset | Neutral/joint/cancel passed; broad paths need qualification |
| Endpoint, FK/IK, collision query | Named-tip API; pose/twist/wrench and typed services | FK/IK, relative Cartesian and stopped collision query passed |
| Position/velocity streaming | Bounded sessions; JointStream action and leased StreamCommand | Short sessions, 60-second velocity load and client/process loss passed |
| Torque and trajectory mode | Validated command vectors, published/URDF limits; experimental opt-in | Earlier 150 ms torque exit failed; new 50 ms/0.02 Nm pulse and 0.8 s/0.001 rad trajectory passed small envelopes; broader operation unqualified |
| Native planning/execution | Typed native actions; IDs, feedback, cancel, timeouts | START out/return and cancellation/recovery passed; GENERATE failed initially, succeeded after native START; state dependence remains |
| Interaction/force/impedance | Typed timed sessions; compliant/force modes require experimental opt-in | Zero-force hold plus 100 ms compliant and 0.05 N force sessions passed; broader use/process-loss deactivation unqualified |
| Arm calibration | Typed action and attached-tool preconditions | Blocked by configured gripper; not run |
| Gripper | Calibrate/open/close/position/stop/start/reboot/settings; Trigger and GripperCommand | Calibration/positions/ROS 2, cancellation/stop/start/reboot/recalibration and speed/dead-zone changes passed; payload setting pending |
| IO and inputs | Merged typed state/config, IO health, bounded edges/callbacks, validated signal/port writes | Reads passed; physical button/wheel edges not captured |
| Lights/digital outputs | Metadata-checked writes; settings/digital services | Six lights toggled/restored; camera-power state read, output not power-cycled |
| Display | RGB/BGR Image publication in Python or ROS 2 | Operator confirmed image visible |
| Cameras | Explicit streaming/settings, current frames, CameraInfo, fast standard ROS 2 Image CDR | Both full-resolution cameras under 60-second control load; flip/restoration and new current-frame hand read passed |
| Diagnostics/TF/model | Python reads; diagnostics/tf/tf_static/read_parameter | Direct and ROS 2 telemetry passed |
| Head pan | Existing bounded Python implementation retained | **Deferred**; historical passive-mode arrival failure retained |
| ClickSmart/tool reconfiguration | Discovery only; no arbitrary private command forwarding | Tool absent; activation/persistent configuration unavailable |
| ros_sugar application | Optional lifecycle adapter using SawyerController | Real ros_sugar 0.8.2 lifecycle/timer/cancellation/cleanup passed with fake driver and actual Humble DDS |

The operator restarted the controller, then bounded physical rechecks were
completed. Disable/enable again triggered the tightened displacement guard and
software stop. Latest stale-feedback/busy-job stop refinements have software
evidence only. See [continuation evidence](validation/2026-11-13-continuation.json). Every advertised ROS 2 operation uses the same Python backend;
ROS 2 support does not add a ROS 1–ROS 2 bridge.

## Interfaces outside the public contract

The captured graph includes vendor types under `motor_control_msgs`,
`motor_controller`, `rethink_errors`, Cognex, camera field-calibration and
additional planning/controller-management packages. Public pinned schemas do
not define all of them. They are recorded in the extended graph inventory but
are not exposed as undocumented writes. Supported definitions and operating
requirements are needed if an application requires them. An advertised topic
or service is not proof that it belongs to the supported public SDK contract.

Fixed electric-gripper force is a hardware limitation. Unknown measured effort
is represented as unknown, not zero. ClickSmart qualification requires the
actual tool; workstation changes do not provide firmware-internal access.

## Fault coverage and limits

Local TCPROS/DDS tests cover stale data, invalid numeric/mode fields, collision,
limits, timeout, cancellation, shared observers, master registration recovery,
no command replay, source age/sequence/session identity, client leases,
concurrency and cleanup after stop. Physical evidence covers a killed velocity
publisher, streamed ROS 2 client loss, normal cancellation and software stop.
Cable disconnection and a physical master/controller restart remain untested.

The robot and PC clocks differ by years. Control uses local monotonic deadlines;
ROS 2 sensor headers use host receipt time, not synchronized acquisition time.
Finite ROS 2 trajectories continue after their client exits unless cancelled;
only streaming sessions have the source lease. FK collision checks do not
provide external obstacle avoidance or certify a collision-free workspace.
The 60-second run is a measured observation, not a hard real-time guarantee.

Regenerate the ledger from the pinned source index without ROS:

```bash
python irobot/src/robots/sawyer/installation/build_capability_ledger.py
```
