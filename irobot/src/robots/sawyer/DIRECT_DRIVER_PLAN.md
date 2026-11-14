# Sawyer direct-driver checklist

Updated November 13, 2026, on `sawyer_py`. Target: ROS 2 Humble.
Head pan is deferred at the user's request. **Physical release is blocked:**
arm movement occurred during enable recovery with `ready=false`; the operator
confirmed the cuff was released. Sawyer was software-stopped and disabled.
Later read-only feedback reports `error=true` and the model reports the current
configuration in collision. Do not reset/enable as part of automated validation.

The [implementation guide](IMPLEMENTATION_GUIDE.md) explains the files and
functions. [Unfinished work](UNFINISHED_WORK.md) collects every unchecked
requirement below with the evidence still needed, plus known maintenance work.

The application and ROS 2 server both use
`SawyerController → SawyerBase → direct XML-RPC/TCPROS → Intera`.
No workstation ROS 1 runtime, separate bridge process or robot-side agent is
required. Sawyer's existing Intera controller remains in place.

Checked items mean completion of the stated scope, not full SDK qualification.
[Operation ledger](CAPABILITY_LEDGER.json), [coverage](CAPABILITIES.md),
[results](VALIDATION.md), [extended evidence](validation/2026-11-13-extended.json),
and [continuation evidence](validation/2026-11-13-continuation.json)
record limitations separately from implementation.

## Capability contract and architecture

- [x] Confirm Intera 5.3.1.192, public SDK 5.3.0, standard electric gripper,
  Ubuntu 22.04.5 x86_64 and Humble/Python 3.12.16 baseline.
- [x] Inventory 232 published topics, 94 services, four action namespaces,
  parameter names and attached device metadata without actuation.
- [x] Index all 226 public SDK methods across 28 pinned source files.
- [x] Map every indexed operation to Python/ROS 2/controller interfaces, units,
  limits, side effects, cancellation and evidence. Separate helper utilities,
  unavailable tools and unqualified operations from verified hardware access.
- [x] List controller-advertised interfaces absent from public schemas.
- [x] Preserve the irobot configuration/base/controller layout and existing
  preview-first movement APIs; keep imports/construction offline.
- [x] Keep core types independent of ROS; share one transport and one operation
  owner. Multiple command owners and queued/replayed goals are rejected.
- [x] Remove bridge-only installer checks, launch file and release manifest.
- [ ] Obtain documented schemas/operating requirements for any additional
  vendor interface required by the application. Topic names alone are not APIs.

## Direct transport and feedback

- [x] Implement XML-RPC registration/callbacks, TCPROS pub/sub/services,
  schema negotiation, bounded queues and deterministic cleanup.
- [x] Pin 90 nonbuiltin wire definitions, retain upstream licenses, and compare
  hashes/default serialization with reference ROS 1 genpy, plus nonzero data.
- [x] Track per-field joint freshness, efforts, limits, named endpoints,
  assembly/collision state, diagnostics, TF and read-only parameters.
- [x] Share observations and retain merged change-only IO/static TF for late
  observers while preserving receipt timestamps; never replay commands.
- [x] Detect master availability/registration changes; test registration recovery
  and absence of command replay with localhost peers.
- [ ] Physically validate cable loss and master/controller restart. Existing
  process-loss tests do not establish these different fault cases.

## Arm and gripper control

- [x] Implement bounded neutral/joint/waypoint planning, FK/seeded IK,
  configuration collision queries and Cartesian planning.
- [x] Physically validate neutral, 0.01 rad joint move/return, 1 cm Cartesian
  Z move/return and explicit action cancellation/recovery before the enable fault.
- [x] Implement bounded position/velocity/torque/trajectory stream sessions,
  source leases, deadlines, canonical joint ordering and mode-specific cleanup.
- [x] Physically test small velocity/position sessions and killed-process
  watchdog behavior; record displacement and stationary restart observations.
- [x] Implement typed native motion/calibration actions with unique goal IDs,
  feedback/results, cancellation, timeout and no replay.
- [x] Implement bounded interaction sessions with state-confirmed activation
  and deactivation; physically validate a maximum-impedance, zero-force hold.
- [x] Require experimental opt-in for torque, trajectory mode and compliant/
  force interaction; preserve their failed/partial qualification results.
- [x] Implement gripper calibration/open/close/arbitrary position/stop/start/
  reboot, settings and typed ROS 2 access. Reject adjustable force requests
  unsupported by this tool; unknown effort is not reported as measured zero.
- [x] Physically validate empty calibration/open/close, 0.03 m positioning,
  ROS 2 gripper actions and dead-zone setting/restoration.
- [ ] Diagnose unexpected enable movement and establish ready-state recovery.
  After the operator restarted the controller, disable cleared homed and enable
  again exceeded a tightened 0.005 rad envelope. The revised guard physically
  latched software stop. The homing/recovery cause is still unresolved.
- [x] Recheck a 50 ms, 0.02 Nm damped torque pulse within a 0.005 rad envelope.
- [ ] Qualify longer/larger torque operation; the earlier 150 ms exit failed.
- [x] Validate native START out/return, goal-specific cancellation/recovery, and
  GENERATE after native START with no measured movement.
- [ ] Qualify cold-start GENERATE. It initially returned CONTROLLER_NOT_FOLLOWING;
  later success does not establish availability in every controller state.
- [x] Test a 0.8 s, 0.001 rad moving trajectory and 100 ms compliant/0.05 N
  force sessions with 0.005 rad / 3 mm bounds and active/inactive confirmation.
- [ ] Qualify broader trajectory/force operation and process-loss deactivation.
  Experimental opt-in remains required; brief tests do not establish safety
  after an application crash.
- [x] Physically validate empty-gripper cancellation/stop/start, reboot, explicit
  recalibration, opening and speed/dead-zone change/restoration.
- [ ] Qualify payload settings. Arm calibration requires removing/configuring
  out the attached gripper and has not been run.
- [ ] Requalify the final packaged build physically after fault resolution.

## Peripherals

- [x] Implement typed IO reads/writes, metadata validation, ACK matching,
  callbacks/edge queues with overflow counters, lights and digital outputs.
- [x] Physically toggle/observe/restore all six SDK-defined lights.
- [x] Implement display publication and obtain operator visibility confirmation.
- [x] Implement both cameras, CameraInfo, stream ownership, settings and
  advancing-frame checks. Reading a camera never starts streaming implicitly.
- [x] Physically validate individual camera reads, concurrent full-resolution
  streams, hand-camera flip/restoration and original stream restoration.
- [ ] Capture physical cuff-button and navigator-wheel transitions. The user
  replied yes, but the recording interval contained no edges; not passed.
- [ ] Qualify remaining writable ports/digital outputs and camera settings.
  Camera-power permission was read; power cycling was not tested.
- [ ] Qualify ClickSmart only if that tool is attached. It is currently absent;
  activation/persistent reconfiguration is not implemented or claimed.
- [ ] Head pan: **deferred**, no new work or physical test requested.

## ROS 2 and installation

- [x] Use the same driver from a standard Humble node with action feedback,
  cancellation, asynchronous jobs, stop and readiness services.
- [x] Add optional generated typed actions/services for streams, native motion,
  calibration, interaction, Cartesian targets, gripper, FK/IK/collision and
  parameters; expose IO state/config/events/settings, digital IO and display.
- [x] Publish efforts, endpoint twist/wrench, diagnostics, TF, images and camera
  calibration data without conversion work in control callbacks.
- [x] Validate separate real Humble client/server gripper/FK/IK/parameter paths,
  physical streamed-client loss and synthetic concurrency/cancellation faults.
- [x] Measure one 60-second dual-camera/control run: 5,740 commands, maximum
  gap 18.4 ms; record image counts, feedback and process RSS samples.
- [x] Provide a pinned 177-package Humble Conda lock excluding ROS 1/bridge,
  a direct Python installer, dry-run/check modes and optional CMake interfaces.
- [x] Build interfaces in the clean pinned Humble environment; install/import
  a wheel in plain Python without ROS and in clean Humble without a bridge.
- [x] Provide an optional ros_sugar lifecycle application adapter; verify its
  configure/activate/cancel/cleanup contract using a framework fixture.
- [x] Build and validate real ros_sugar 0.8.2 at a pinned source revision with
  actual Humble lifecycle/DDS, timer preview collection, cancellation and cleanup.
- [x] Add a regression for completed-task collection racing new submission.
- [x] Add a subscription-only diagnostic CLI and regressions for sparse feedback,
  merged inputs, bounded events, and stop requests despite stale feedback/busy
  ROS 2 jobs. These latest stop refinements have software evidence.
- [ ] Qualify ROS 2 interfaces added after the physical fault on hardware.
- [ ] Qualify other CPU/OS/Python combinations and longer memory/timing runs.

## Completion rule

Software regression checks, Ruff, reference codecs, clean imports and current
hardware observations are recorded in VALIDATION.md and its JSON reports.
Full access/control is **not signed off** until every required attached-hardware
operation has physical evidence and the enable fault is resolved. Deferred and
failed items remain visible rather than being marked complete.
