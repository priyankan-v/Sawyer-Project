# Direct-driver validation — November 13, 2026

Branch `sawyer_py`, development working tree. **Full physical qualification is
blocked by disable/enable recovery. Head pan remains deferred.** After the
operator restarted the controller and reconfirmed a clear path, empty gripper
and E-stop access, further bounded tests passed. A guarded disable/enable test
then exceeded its tightened 0.005 rad envelope and latched software stop.
The final five-second subscription-only check confirmed `enabled=false`,
`stopped=true`, `error=true`, `homed=false`, E-stop source 4 (engine request),
and zero observed joint range. No further reset/enable/actuation followed.

Disabling cleared `homed` before the failed enable attempt. Motion while not
ready may involve homing, but the cause and a safe recovery procedure are not
established. A controller restart restored readiness previously; it did not
resolve the reproducible disable/enable behavior. Do not automatically retry.

Evidence is preserved in three stages, including failures:

- [Original milestone](validation/2026-11-13.json): neutral, joint/Cartesian
  movement, gripper, standard ROS 2 actions and cameras.
- [Extended observations](validation/2026-11-13-extended.json): mode failures,
  60-second dual-camera/control load, process/client loss and first enable fault.
- [Latest continuation](validation/2026-11-13-continuation.json): operator
  restart, bounded rechecks, native execution/cancel, gripper recovery, real
  ros_sugar, repeated enable failure, final stopped state and software checks.

Earlier reports' final states and test counts are historical. Hardware tests
used development sources; the final packaged source is not a new full-motion
qualification. Post-run source hashes are not process-start module captures.

## Environment and isolation

Ubuntu 22.04.5 x86_64, Python 3.12.16, NumPy 2.5.4, RoboStack Humble;
Intera `5.3.1.192-.0-3d373b7`, public SDK 5.3.0. Master
`http://169.254.121.3:11311`, workstation `169.254.121.2`, tip `right_hand`.
The controller and PC clocks differ by years; deadlines use local monotonic
time and ROS 2 sensor headers use host receipt time, not acquisition time.

The original `ros2_env` contains an unused legacy bridge package. Fresh plain
Python and pinned Humble environments exclude ROS 1 and the bridge. The direct
Humble lock has 177 Conda packages; initial clean startup exposed a missing
rosidl_runtime_py dependency, now pinned with PyYAML. Its installed-wheel
read-only server/client check delivered state/joints/gripper, seven efforts,
13 hand images, parameters and collision results with zero attempted commands.
The optional 201-package ros_sugar Conda lock plus pinned Python requirements
was installed into another clean environment; source build and the combined
regression suite passed there.
Reference genpy is used only during development, not application runtime.

## Physical results and limits

| Operation | Evidence and practical limit |
| --- | --- |
| Neutral/joint/Cartesian | Original neutral, +0.01 rad joint/return and +1 cm base-Z/return passed. After restart, a 0.003 rad move/return passed with errors ~0.000286/0.000205 rad. |
| Position hold | Post-restart 0.3-second current-position hold passed; no neutral reposition was required. |
| Standard ROS 2 trajectory | Separate-client execution, feedback, cancellation, stationary recovery and next goal passed in original milestone. |
| Feedback/model | Seven efforts; published/URDF effort bounds (80,80,40,40,9,9,9 Nm); endpoint, FK/IK, collision queries, diagnostics, parameters and TF read. |
| Native START | 0.003 rad out/return succeeded, measured arrival errors ~0.000721/0.001139 rad. |
| Native GENERATE | Initially ABORTED/CONTROLLER_NOT_FOLLOWING. After explicit native START, succeeded with zero measured displacement. Controller-state dependence remains; returned analysis can describe prior execution and is not a generated waypoint list. |
| Native cancellation | Goal-specific PREEMPTED result, monitored recovery within a 0.02 rad envelope, then successful next goal. One-second observed joint range ~0.00497 rad; residual feedback velocities were nonzero, so strict stationarity is not claimed. |
| Torque | Earlier 150 ms exit exceeded 0.01 rad. Post-restart 50 ms, 0.02 Nm damped pulse passed a 0.005 rad envelope; maximum monitored post-exit displacement ~0.003466 rad. Longer/larger torque operation remains unqualified and opt-in. |
| SDK trajectory mode | Earlier speed-envelope failure retained. A post-restart 0.8 s, 0.001 rad moving cycle sent 79 commands and passed a 0.005 rad envelope. Reduced controller collision handling and broader paths remain experimental. |
| Interaction | Maximum-impedance zero-force hold passed. New 100 ms compliant and 0.05 N force sessions confirmed active/inactive within 0.005 rad / 3 mm envelopes. Larger forces and process-loss deactivation remain unqualified. |
| Gripper | Calibration/open/close/positions and ROS 2 actions passed. New cancellation/stop/start, reboot, lost calibration, explicit recalibration/opening and speed/dead-zone restoration passed. |
| Gripper tolerance | At 0.03 m/s and 0.002 m dead zone, opening stopped ~0.002232 m short of target and correctly timed out against 0.002 m arrival tolerance. Retry with 0.001 m dead zone passed. Original 1.5 m/s and 0.002 m dead zone restored. |
| IO/lights/display | Six SDK lights toggled/measured/restored; operator confirmed display image visible. Camera-power state/permission read, no power cycle. |
| Cuff/navigator | Operator confirmed first actions but recordings contained unchanged values. A second comparison recorded IO and legacy digital/analog interfaces; operator could not repeat then. Physical edges remain unverified. |
| Disable/enable | Reproduced excessive movement despite prior healthy/homed state. Tightened 0.005 rad guard triggered software stop; final stopped/disabled/error state confirmed. Release blocker. |
| Head pan | No new test or head-pan command; deferred by user. Historical passive-mode arrival failure retained. |

No arm calibration with the attached gripper, payload-setting qualification,
physical cable pull, physical master/controller failure test, or interaction
process-kill test was performed. The operator-initiated restart confirms a
later healthy session, not an injected-restart/reconnection qualification.

## Load and process/client-loss evidence

The earlier 60-second separate-process ROS 2 test delivered both full-resolution
cameras while streaming **5,740 commands**, maximum gap **0.018392 s**, with
1,201 action feedback samples. The client received 441 head frames (1280×800
BGRA, 4,096,000 bytes) and 301 hand frames (752×480 mono8, 360,960 bytes).
Stopping the command client caused lease expiry/abort with no automatic resume.

Recorded server /proc RSS samples ranged from 158,000 to 194,712 KiB; client
samples were approximately 126,440–145,444 KiB. The raw ru_maxrss can include
inherited peak history. One minute does not establish indefinite memory
stability or hard real-time scheduling.

Initial full head-image delivery needed a 32 MiB Fast DDS shared-memory profile.
Python per-pixel conversion delayed leases; standard Image CDR publication/raw
client parsing removed that bottleneck in the successful run. Native Humble
serialization/deserialization checks cover encodings, alignment and Unicode;
CDR padding bytes are not assumed deterministic.

Killing a velocity publisher after 20 commands gave maximum observed post-kill
displacement 0.0012285 rad and settled variation 0.0002061 rad; read-only restart
observed no further displacement. This does not establish interaction-mode
watchdog behavior or safety when the controller/network itself fails.

## Software and real-framework verification

The full driver regression covers local TCPROS/RPC peers, virtual arm behavior,
real Humble DDS/type support, wire codecs, source leases, cancellation/deadlines,
limits, freshness, master registration recovery/no replay, merged late-observer
IO, bounded edges, image CDR, mode guards and stopped diagnostics. Latest
results are in the continuation JSON and local final-test log.

Reference genpy verified 90 nonbuiltin hashes/default serializers and nonzero
JointState data. Interface and 226-operation ledger generation are reproducible.
The combined clean-runtime suite passed **164 tests and 27 subtests**, including
the real-framework test without skips. Ruff lint/format and 79-character
Python checks pass.

Real ros_sugar 0.8.2 at commit
`5d64e015488fe9f0ca7b844028e7907f43785db0` was built and tested with actual Humble
lifecycle/DDS: configure, activation without motion, timer-collected preview,
worker cancellation on deactivate, cleanup and disconnection. These checks use
a fake driver; they do not qualify physical motion through the framework.
A concurrency regression prevents completed-task collection losing a newly
submitted task. [Pinned optional setup](installation/ROS_SUGAR.md) is provided.

The enable guard has physical failure-detection evidence. New stop refinements
also send requests despite stale assembly feedback, require fresh confirmation,
and cancel/wait for an active ROS 2 job while blocking new goal admission.
Those refinements have software evidence only; they were not physically
requalified after the final fault. No automatic fault reset was added.

The new `examples.diagnose_direct` CLI aggregates sparse diagnostics, joints
and IO/legacy inputs without creating command publishers. Live controller
error/log definitions were MD5-checked with development-only genpy; no private
command interface was guessed or added to the runtime. The post-fault log
reported an engine-source stop and torso NO_EXTERNAL_ENABLE status, which do
not by themselves establish the original movement's cause.

Use the [checklist](DIRECT_DRIVER_PLAN.md) for remaining physical requirements.
Full access/control is not signed off while enable/homing recovery is unresolved.
