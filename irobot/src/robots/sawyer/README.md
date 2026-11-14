# Sawyer direct Python driver

Branch `sawyer_py`: plain Python and ROS 2 Humble applications share the irobot
Sawyer driver. The workstation requires no ROS 1 installation, `rospy`, separate
bridge process, embedded message bridge, or robot-side application agent.
Sawyer's existing Intera controller continues to run its native software.

See the [file and function guide](IMPLEMENTATION_GUIDE.md) for what each part
does and why, and [unfinished work](UNFINISHED_WORK.md) for remaining requirements.

```text
Python application ──┐
                     ├─ SawyerController → SawyerBase → direct TCPROS → Sawyer
ROS 2 Humble node ───┘
```

**Physical qualification is blocked.** Unexpected arm movement occurred during
an enable/reset recovery test with `ready=false`; the operator confirmed the
cuff was released. The robot was software-stopped and disabled and now reports
an error. After an operator restart, a guarded disable/enable retest again
exceeded a tightened 0.005 rad envelope and latched software stop. Disable
cleared homed; the safe homing/recovery procedure is unresolved. Do not
automatically reset or retry enable. Head pan is deferred at the user's request. See the
[checklist](DIRECT_DRIVER_PLAN.md), [capabilities](CAPABILITIES.md) and
[validation evidence](VALIDATION.md) for passed, failed and pending work.

## Install plain Python

From the repository root, use Python >=3.11 and NumPy 2:

```bash
python -m pip install -e '.[sawyer]'
```

Or create an isolated runtime with the direct installer:

```bash
python irobot/src/robots/sawyer/setup_runtime.py install \
  --project "$PWD" --python /path/to/python3.12
source .sawyer-runtime/activate.sh
python -m irobot.src.robots.sawyer.setup_runtime check \
  --prefix "$PWD/.sawyer-runtime/envs/python-direct"
```

`install --dry-run` prints commands without running them. `check` only probes
imports; it does not contact or change the robot. Imports/construction/connect
never automatically enable, reset, calibrate, start cameras or move hardware.
Connect supports stopped/disabled diagnostics; actual arm commands require fresh
feedback, valid limits and a ready enabled controller. Wire definitions and
licenses ship in the wheel; `sawyer_rs` and ROS generators are development
references, not runtime dependencies.

## Python application

```python
from irobot import SawyerConfig, SawyerController

config = SawyerConfig(
    ros_master_uri='http://169.254.121.3:11311',
    ros_ip='169.254.121.2',
    tip='right_hand',
)
with SawyerController(config) as robot:
    print(robot.get_status())
    print(robot.devices.robot_state())
    # Once the controller fault is resolved and the arm is ready:
    # preview = robot.move_to_neutral()  # no movement
    # result = robot.move_to_neutral(execute=True)
```

Keep one hardware owner. Synchronous driver methods serialize operations and
reject competing ownership; callbacks run during polling/control and must be
fast. `poll()` services callbacks while idle. `stop()` requests cancellation;
`devices.software_stop(execute=True)` latches the controller stop. Neither call
automatically resets or enables. Physical stops remain owned by the controller.
Readiness is distinct from receipt of an enable acknowledgement.

| Python API | Contract |
| --- | --- |
| `joint_positions`, `joint_velocities`, `joint_efforts`, `joint_limits`, `joint_effort_limits` | Seven canonical joints; rad, rad/s, Nm; per-field freshness |
| `endpoint_pose`, `endpoint_state`, `tip_names`, `endpoint_velocity`, `endpoint_effort` | Named tip, declared frame; m, xyzw quaternion, m/s, rad/s, N, Nm |
| `get_parameter`, `robot_description`, `check_version`, `capabilities` | Read-only configuration, compatibility check and coverage ledger |
| `move_to_neutral`, `move_to_joints`, `move_trajectory` | Preview first; bounded rest-to-rest reference/tracking |
| `forward_kinematics`, `inverse_kinematics`, `configuration_collisions` | Model queries; no external obstacle-avoidance guarantee |
| `move_relative`, `move_to_pose` | Cartesian planning; relative offsets in cm, pose in m |
| `stream_joint_commands` | Finite position/velocity/torque/trajectory session with StreamOptions |
| `move_native_joint_path`, `execute_native_trajectory`, `calibrate_arm` | Typed native actions and tool preconditions; native planning currently fails qualification |
| `run_interaction` | Timed typed interaction command; confirmed activation/deactivation |
| `calibrate_gripper`, `open_gripper`, `close_gripper`, `move_gripper`, `stop_gripper`, `start_gripper`, `reboot_gripper` | Explicit electric-gripper commands, ACK and measured completion where available |
| `devices.read_io`, `write_io`, `register_io_callback`, `io_events` | Metadata-checked signal/port values, bounded events and drop counters |
| `devices.set_light`, `digital_state`, `set_digital_output` | Published permissions; only six SDK light signals have a documented direction exception |
| `devices.camera_frame`, `set_camera_streaming`, `camera_state`, `configure_camera`, `display_image` | Explicit stream ownership; read never enables a stream |
| `devices.diagnostics`, `transforms`, `inventory` | Diagnostics, fresh dynamic/retained static TF, graph/tool discovery |

See the [operation ledger](CAPABILITY_LEDGER.json) for exact APIs and limits.
The actual named endpoint is `right_hand`; `right_gripper_tip` is not exposed
by the tested named-tip feedback/FK interface. Neutral defaults to
`/named_poses/right`; use `neutral_joints` to override. Set
`enable_gripper=False` if the matching electric tool is absent.

Torque, SDK trajectory mode and force/compliant interaction require
`SawyerConfig(allow_unvalidated_modes=True)` for execution. This is an explicit
experimental opt-in, not a qualification claim. Preview validation is available
without it. A new 50 ms torque pulse, small moving trajectory and 100 ms compliant/0.05 N
sessions passed bounded tests. Earlier torque exit failed. Native START and
cancellation passed; GENERATE succeeded only after native START. Planning does
not silently initialize control by moving the arm.

`main.py` and the saved-pose utility keep their existing preview/execute
interfaces. Physical examples require fault resolution before use:

```bash
python main.py                      # neutral preview, requires usable feedback
python sawyer_saved_pose.py save    # read current joints
python sawyer_saved_pose.py restore # preview saved pose
```

Read-only validation also records a stopped controller without attempting
motion planning:

```bash
python -m irobot.src.robots.sawyer.examples.validate_direct \
  --master http://ROBOT_IP:11311 --ip PC_IP --tip right_hand \
  --report direct-readonly.json
```

For aggregate diagnostics/input recording with no command publishers:

```bash
python -m irobot.src.robots.sawyer.examples.diagnose_direct \
  --master http://ROBOT_IP:11311 --ip PC_IP --duration 30 \
  --report direct-diagnostics.json
```

`validate_direct --execute` adds neutral, Cartesian and empty-gripper tests; do not use it while
the current fault remains open. It never automatically clears the fault.

## Optional ROS 2 Humble server

The standard server needs rclpy/control_msgs and standard message packages.
The pinned Linux x86_64 runtime uses **Humble, Python 3.12.16, NumPy 2**; system
Humble Python 3.10 bindings cannot be mixed with this package's interpreter.
The 177-package Conda lock contains no workstation ROS 1 or bridge package.
The legacy `ros2_env` still contains an unused bridge package; use the fresh
runtime below for dependency isolation.

```bash
python irobot/src/robots/sawyer/setup_runtime.py install \
  --project "$PWD" --ros2 --interfaces --conda /path/to/conda
source .sawyer-runtime/activate.sh
python -m irobot.src.robots.sawyer.setup_runtime check \
  --prefix "$PWD/.sawyer-runtime/envs/humble-direct" \
  --runtime "$PWD/.sawyer-runtime" --ros2 --interfaces
python -m irobot.src.robots.sawyer.ros2 \
  --master http://ROBOT_IP:11311 --ip PC_IP --tip right_hand --interfaces
```

The optional `--interfaces` package is compiled with CMake during explicit
installation; it is not needed for plain Python or standard trajectory control.
See [interface sources](../../../../ros2/irobot_sawyer_interfaces/README.md).
No SDK or ROS 1 message generation runs at application startup.
`--speed` defaults to 0.05 and `--max-accel` to 0.1 rad/s².
`--allow-unvalidated-modes` enables the same experimental modes as Python.

| Endpoint under default `/sawyer` | Interface |
| --- | --- |
| `follow_joint_trajectory` | Standard FollowJointTrajectory action |
| `move_to_neutral`, `open_gripper`, `close_gripper`, `calibrate_gripper`, `stop` | Trigger jobs; accepted is not completed, observe status |
| `set_enabled`, `reset_faults`, `software_stop` | Enable/reset are Trigger/SetBool jobs; software_stop cancels an active job and synchronously confirms the latched controller stop |
| `joint_states`, `endpoint_pose`, `endpoint_twist`, `endpoint_wrench`, `diagnostics`, `tf`, `tf_static`, `status` | Standard telemetry and status JSON |
| `joint_stream`, `stream_command` | Optional JointStream action and leased command topic |
| `native_motion`, `calibrate_arm`, `interaction`, `move_cartesian`, `gripper_command` | Optional typed actions |
| `forward_kinematics`, `inverse_kinematics`, `check_joint_collision`, `read_parameter` | Optional typed services |
| `inventory`, `capabilities`, `start_gripper`, `stop_gripper`, `reboot_gripper` | Optional Trigger services |
| `robot_state`, `endpoint_states`, `joint_limits`, `{device}/state`, `{device}/config` | Optional typed controller feedback |
| `io_events`, `io_health` | Optional JSON edge events/drop counts and change-only connection/age metadata |
| `{device}/set_parameters`, `{device}/ports/set_parameters` | Optional typed rcl_interfaces SetParameters; per-item ACK/measured results |
| `digital_io/right_hand_camera_power`, `digital_outputs/set_parameters` | Optional known digital input/permission read and explicit boolean writes |
| `display_image`, `{head,hand}/streaming` | Optional Image input and SetBool camera ownership |
| `{head,hand}/image`, `{head,hand}/camera_info` | Standard streams with `--cameras`; no implicit camera startup |

Device labels for writes are `gripper`, `head_camera`, `hand_camera`, `robot_io`;
cuff/navigator state is read-only. Object/array IO uses a JSON string parameter.
Motion-related gripper signals must use the gripper operations. Settings can
have hardware side effects; metadata validation does not qualify every setting.

The FollowJointTrajectory subset requires all seven joints, immediate zero
header timestamp, strictly increasing positive waypoint times, and zero
waypoint velocity/acceleration. Unsupported derivatives, effort, tolerances and
multi-DOF requests are rejected. This does not accept arbitrary MoveIt paths.
Finite trajectories continue if the client disappears unless cancelled.
The software_stop service requests cancellation, waits boundedly for driver
ownership, then requires fresh stopped-state confirmation; busy/stale feedback
never count as confirmation. It does not reset the robot.
Streaming uses a separate lease: use the session ID in action feedback, strictly
increasing sequence, current ROS 2 source stamp, correct joint names/mode and
only that mode's fields. Lease timeout must not exceed half the command timeout.
Expired sessions abort and need a new goal; late packets never resume them.

Callbacks use bounded caches; conversion and DDS publication run outside robot
control callbacks. ROS 2 telemetry timestamps are host receipt times because
the robot clock is not synchronized. For high-rate image consumers, `raw=True`
with `ros2_images.deserialize_image` avoids Python per-pixel conversion.

For both full-resolution cameras on this Fast DDS 2.6.11 runtime, set the
included 32 MiB shared-memory transport profile in **server and clients**:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/irobot/src/robots/sawyer/installation/fastdds-cameras.xml"
```

The measured head frame is 4,096,000 bytes; Fast DDS's default segment is
512 KiB. See the [Fast DDS shared-memory documentation](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/transport/shared_memory/shared_memory.html).
One 60-second dual-camera/control run passed with this profile; indefinite
memory stability and hard real-time scheduling are not established.

## irobot / ros_sugar application template

`SawyerController` is the reusable hardware driver. An application chooses tasks
and when to run them. `ros2.SawyerNode` exposes the driver to ROS 2 clients.
`ros_sugar_component.SawyerComponent` is an optional application example with
lifecycle connect/activate/cancel/cleanup; activation itself starts no motion.
Choose one hardware owner. The adapter now passes real ros_sugar 0.8.2 configure/activate/timer/cancel/cleanup
checks with a fake driver and real Humble DDS. It also has a task-handoff race
regression. See [pinned optional installation and example](installation/ROS_SUGAR.md).
These framework checks are separate from physical motion qualification.
