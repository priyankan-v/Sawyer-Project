# Using the installed Sawyer bridge in a new ROS 2 project

This guide describes the installed `sawyer_bridge` 0.1.0 package, build
`robostack_humble_py312_1`, and the accompanying `bridge_extended.yaml` configuration.
It is intended to travel with your application project. The bridge is installed
in the Conda environment; your application files can live in any directory.

## 1. Installation and network reference

| Item | Value on this workstation |
|---|---|
| Conda installation | `/home/pi/miniconda3` |
| Runtime environment | `ros2_env` |
| Runtime prefix | `/home/pi/miniconda3/envs/ros2_env` |
| ROS distribution | RoboStack ROS 2 Humble |
| Python | 3.12 |
| Required ABI family | NumPy 2, Boost 1.88, `console_bridge` 1.0, Humble mutex 0.9 |
| Platform | Linux x86_64, glibc >= 2.35 |
| Robot ROS 1 master | `http://169.254.121.3:11311` |
| PC address reachable by the robot | `169.254.121.2` |
| ROS 2 domain | `42` |
| ROS package and normal node name | `sawyer_bridge`, `/sawyer_bridge` |

The network addresses describe the existing setup. Confirm the robot-facing PC
interface with `ip -br addr`; update `ROS_IP` if its reachable address differs.
`ROS_DOMAIN_ID` must match between the bridge and your application terminals.
It selects the ROS 2 domain; it is unrelated to the ROS 1 master address.

The installation has passed its package check, and live robot telemetry was
observed. This does not establish that every command, service, or action has
been validated on the physical robot.

## 2. How the bridge works

```text
Your ROS 2 application
   topics / service clients / action clients
                   |
        installed sawyer_bridge process
                   |
        Sawyer ROS 1 master and Intera nodes
```

Your application normally uses ROS 2 interfaces: `rclpy`, `rclcpp`, or ROS 2 CLI
commands. The bridge translates supported interfaces between ROS 2 and the
robot's existing ROS 1 system. It includes private ROS 1 client libraries, so
your application does not need a complete ROS 1 installation on the PC.

The launcher first runs `configure_bridge`, which reads YAML and writes the
`/sawyer_bridge` namespace on the ROS 1 parameter server. It then runs the C++
`parameter_bridge`, which creates the configured topic, service, and action
connections. Use the launch file: running `parameter_bridge` alone does not
load a YAML file.

Starting the bridge creates transport. It does not itself generate movement
commands, enable the robot, or start camera acquisition. Commands from connected
applications can be forwarded once their routes are enabled.

Run one bridge for these robot endpoints. The older workspace's `/ros_bridge`
is a separate bridge, not a required companion. Running both can duplicate
telemetry and command forwarding. Both launchers also use the ROS 1
`/sawyer_bridge` configuration namespace.

## 3. Create a new application directory

Choose your project path. The example below uses `~/sawyer_project`:

```bash
mkdir -p ~/sawyer_project/config ~/sawyer_project/scripts
cp /home/pi/0.1.0/SAWYER_BRIDGE_GUIDE.md ~/sawyer_project/
cp /home/pi/0.1.0/bridge_extended.yaml ~/sawyer_project/config/bridge.yaml
cd ~/sawyer_project
```

After these copies, daily use does not depend on `/home/pi/0.1.0`. Keep that
release bundle separately if you need to reinstall the package later. If you
received this guide and YAML elsewhere, copy those files from that location.

Initial layout:

```text
sawyer_project/
  SAWYER_BRIDGE_GUIDE.md
  config/
    bridge.yaml
  scripts/
    env.sh                  # create below
    read_joint_states.py    # create below
```

Do not copy the Conda environment into the project. Do not add `/opt/ros/humble`
or the older `sawyer_rs` workspace to this project's environment.

## 4. Create the project environment script

From the project root, create `scripts/env.sh`:

```bash
cat > scripts/env.sh <<'BASH'
# Source this file in Bash: source scripts/env.sh
source /home/pi/miniconda3/etc/profile.d/conda.sh
conda activate ros2_env
source "$CONDA_PREFIX/local_setup.bash"

export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
export ROS_DOMAIN_ID=42
hash -r
BASH
```

In each terminal, change to the project root and source the script:

```bash
cd ~/sawyer_project
source scripts/env.sh
command -v python
command -v ros2
ros2 run sawyer_bridge check
```

Both executable paths should start with
`/home/pi/miniconda3/envs/ros2_env/bin/`. The package check should report `PASS`.
It checks loading and interface mappings; it does not connect to the robot.

If your shell inherited another ROS environment, start an isolated Bash shell,
then repeat the `cd` and `source` commands above:

```bash
env -i HOME="$HOME" USER="$USER" TERM="${TERM:-xterm}" \
  PATH=/usr/local/bin:/usr/bin:/bin \
  /bin/bash --noprofile --norc
```

This shell omits desktop environment variables such as `DISPLAY`; use a normal
uncontaminated terminal for graphical applications. Type `exit` to leave it.

## 5. First connection: the installed telemetry configuration

Connect the PC to Sawyer's network. The robot must be running its ROS 1 master
and Intera nodes. Stop any previous bridge launch in its original terminal.

In terminal 1, from your project root:

```bash
source scripts/env.sh
ros2 launch sawyer_bridge bridge.launch.py \
  all_interfaces:=false commands:=false head_commands:=false
```

This command uses the package's installed default YAML, not your extended
`config/bridge.yaml`. Its six enabled robot topics are:

| Topic | ROS 2 type |
|---|---|
| `/robot/joint_states` | `sensor_msgs/msg/JointState` |
| `/robot/state` | `intera_core_msgs/msg/RobotAssemblyState` |
| `/robot/limb/right/endpoint_state` | `intera_core_msgs/msg/EndpointState` |
| `/robot/limb/right/collision_detection_state` | `intera_core_msgs/msg/CollisionDetectionState` |
| `/robot/joint_limits` | `intera_core_msgs/msg/JointLimits` |
| `/robot/head/head_state` | `intera_core_msgs/msg/HeadState` |

In terminal 2:

```bash
cd ~/sawyer_project
source scripts/env.sh
ros2 node list
ros2 topic echo /robot/joint_states --qos-reliability best_effort --once
ros2 topic hz /robot/joint_states
```

An actual joint-state message confirms reception in a ROS 2 subscriber. A
continuing receive rate confirms streaming. Press Ctrl+C to stop the rate check.

## 6. Use your project's extended configuration

Stop the previous launch with Ctrl+C before starting another one. In terminal 1:

```bash
cd ~/sawyer_project
source scripts/env.sh
ros2 launch sawyer_bridge bridge.launch.py \
  config:="$PWD/config/bridge.yaml" \
  all_interfaces:=false \
  commands:=true \
  head_commands:=true
```

This selects the project's file and enables its arm/head command entries.
No package rebuild is required after a YAML edit, but the bridge must restart.

The supplied extended file has 23 topic entries, two IK/FK service entries and
two action entries. Its initial settings are:

| Feature | Initial setting in the supplied file |
|---|---|
| Six base telemetry topics | Enabled |
| Head/hand rectified images and CameraInfo | Enabled, ROS 1 to ROS 2 |
| Right accelerometer | Enabled, ROS 1 to ROS 2 |
| Gripper state | Enabled, reliable + transient local |
| Interaction-controller state | Enabled |
| Head-display image route | Enabled, ROS 2 to ROS 1 |
| Gripper command route | Enabled, ROS 2 to ROS 1 |
| Interaction-control command route | Enabled, ROS 2 to ROS 1 |
| Arm command, speed-ratio and timeout routes | Disabled in YAML; enabled by `commands:=true` |
| Head command route | Disabled in YAML; enabled by `head_commands:=true` |
| Robot enable, software reset and software stop | Disabled; edit their individual `enabled` fields if needed |
| Existing robot IK/FK services | Exposed to ROS 2 |
| `/motion/motion_command` and `/calibration_command` actions | Exposed to ROS 2 |

The extended file is not a telemetry-only profile: gripper/interaction commands
and actions remain exposed even with `commands:=false head_commands:=false`.
Those launch flags only control the specific arm/head entries described above.
Use section 5 for the initial telemetry-only connection.

## 7. Configuration reference

The installed defaults are at:

```text
$CONDA_PREFIX/share/sawyer_bridge/config/sawyer_bridge.yaml
```

Keep project changes in `config/bridge.yaml` and select it with `config:=...`.
Always use one top-level list for each key; append entries to the existing list
rather than adding duplicate `topics:`, `services_2_to_1:`, or `actions:` keys.

### Topics

Example entry, already present in the extended configuration:

```yaml
topics:
  - topic: /io/internal_camera/head_camera/image_rect_color
    type: sensor_msgs/msg/Image
    queue_size: 1
    direction: 1to2
    enabled: true
    qos:
      history: keep_last
      depth: 1
      reliability: best_effort
```

| Field | Meaning |
|---|---|
| `topic` | Endpoint name used on both graphs |
| `type` | ROS 2 message type; a compiled ROS 1 conversion must exist |
| `queue_size` | ROS 1 queue size used by the bridge |
| `direction: 1to2` | Robot ROS 1 data to a ROS 2 publisher |
| `direction: 2to1` | ROS 2 commands to a robot ROS 1 publisher |
| `direction: bidirectional` | Forward in both directions; use only when intended |
| `enabled` | Whether to create this topic route |
| `qos` | ROS 2 history, depth, reliability, durability and optional timing settings |

Telemetry such as joint states and images uses best-effort delivery in the
supplied configuration. Late subscribers to retained gripper state/joint limits
should request reliable + transient-local QoS. Command publishers should match
the configured command subscription QoS.

The 0.2-second DDS lifespan on the supplied arm/head command entries limits DDS
sample lifetime. It is not a robot stop guarantee, a command watchdog, or a limit
on the duration of a physical action.

Camera forwarding only carries frames already produced by Sawyer. Starting a
camera requires its separate supported IO command interface. The supplied file
does not include camera IO command/state routes; add them explicitly or use
Intera discovery. The same applies to other supported Intera peripherals not
listed in your YAML.

### Services

The list names are inherited from the bridge's convention. To call an existing
ROS 1 robot service from a ROS 2 client, use `services_2_to_1` and a ROS 1 type:

```yaml
services_2_to_1:
  - service: /ExternalTools/right/PositionKinematicsNode/IKService
    type: intera_core_msgs/SolvePositionIK
    package: ''
```

To expose a native ROS 2 service to ROS 1 clients, use `services_1_to_2` and a
ROS 2 service type such as `intera_core_msgs/srv/SolvePositionIK`. The native
service server must exist on the destination side. Do not treat these lists as
topic-direction names, and do not add a topic-style `enabled` flag to services;
remove a service entry to disable its forwarding.

### Actions

To expose Sawyer's existing motion action to a ROS 2 action client:

```yaml
actions:
  - action: /motion/motion_command
    type: intera_motion_msgs/action/MotionCommand
    direction: 2to1
    enabled: true
```

`direction: 1to2` exposes a native ROS 2 action server to ROS 1 clients. An
action name uses one direction; use distinct names if native servers exist in
both graphs. The optional `server_timeout` defaults to five seconds for server
loss/goal acknowledgement; it is not a trajectory execution deadline.

Action adapters forward goals, feedback, results and cancellation. They handle
the protocol differences between ROS 1 and ROS 2; forwarding individual ROS 1
action topics as ordinary messages is not a substitute.

### Discovery

The launch options, independent of your project's directory, are:

| Argument | Package default | Effect |
|---|---|---|
| `config` | Installed YAML path | Select the configuration file |
| `all_interfaces` | `true` | Discover supported advertised Intera interfaces |
| `commands` | `false` | Enable configured arm joint-command, timeout and speed-ratio entries |
| `head_commands` | `false` | Enable the configured head-pan command entry |

Use `all_interfaces:=false` for exact control through your YAML. Use
`all_interfaces:=true` to augment it from supported endpoints in the graphs.
Discovery can enable Intera command entries even if their YAML `enabled` field
is false. Therefore `commands:=false` is not a global write-disable switch.

Discovery selects `intera_core_msgs` and `intera_motion_msgs` topics, three
Intera service types, and the two supported action types. Standard ROS messages
such as Image, CameraInfo, Imu, Bool and Empty are not generally added by it;
retain their explicit YAML entries. Discovery is a startup snapshot, including
about five seconds for ROS 2 discovery. Start native ROS 2 servers before the
bridge when discovering reverse service/action routes; restart after adding
endpoints.

## 8. Inspect and use the ROS 2 interfaces

From another project terminal with `source scripts/env.sh`:

```bash
ros2 node info /sawyer_bridge
ros2 topic list -t
ros2 service list -t
ros2 action list -t

ros2 interface show intera_core_msgs/msg/JointCommand
ros2 interface show intera_core_msgs/msg/InteractionControlCommand
ros2 interface show intera_core_msgs/srv/SolvePositionIK
ros2 interface show intera_motion_msgs/action/MotionCommand

ros2 topic info /robot/limb/right/joint_command --verbose
ros2 service type /ExternalTools/right/PositionKinematicsNode/IKService
ros2 action info /motion/motion_command -t
```

These inspection commands do not send movement commands or action goals.

Read streaming endpoint feedback:

```bash
ros2 topic echo /robot/limb/right/endpoint_state \
  --qos-reliability best_effort --once
```

Read retained gripper state using the extended configuration:

```bash
ros2 topic echo /io/end_effector/right_gripper/state \
  --qos-reliability reliable --qos-durability transient_local --once
```

An advertised camera topic does not prove frames are arriving. Check reception:

```bash
ros2 topic hz /io/internal_camera/head_camera/image_rect_color
```

For commands, use a ROS 2 publisher of the configured type. For services, use a
ROS 2 service client. For trajectories/calibration, use a ROS 2 action client.
Always inspect the installed interface definition before constructing requests.
The Intera Python SDK is a ROS 1 client library; importing it is not a substitute
for using these ROS 2 interfaces in your application.

Important message conventions for application development:

| Interface | Fields / behavior to account for |
|---|---|
| `JointCommand` | `names` identifies joint order. Modes are POSITION=1, VELOCITY=2, TORQUE=3, TRAJECTORY=4. Position mode uses `position`; velocity uses `velocity`; torque uses `effort`; trajectory uses position, velocity and acceleration. Joint angles are radians and effort is Nm. |
| `HeadPanCommand` | `target` is radians, `speed_ratio` is a fraction in [0, 1], and `pan_mode` selects the requested head behavior. |
| `InteractionControlCommand` | Supplies per-axis modes, stiffness/damping, force/torque targets, endpoint/frame selection and controller options. Populate the full required vectors; read the installed definition for units and mode constants. |
| `IODeviceStatus` | Device/port/signal feedback includes status tags and JSON-encoded signal values. Parse signal names and formats from the connected device rather than assuming a fixed array order. |
| `IOComponentCommand` | Uses a timestamp, operation name and JSON arguments, not direct `open`/`close` fields. Implement the device's actual command schema and inspect responses/feedback. |
| `SolvePositionIK` | Takes stamped poses and optional seeds/nullspace goals; inspect each `result_type` before using returned joints. A successful IK result is not execution of a motion. |
| `SolvePositionFK` | Takes joint configurations and tip names; inspect `is_valid` and `in_collision` in the response. |
| `MotionCommand` | Goal commands include `generate`, `start` and `stop`, with an Intera `Trajectory`. Check goal acceptance, action terminal status, result/error fields and feedback. |
| `CalibrationCommand` | Uses `start`/`stop` requests and returns calibration progress and result/status information; it is not the separate camera field-calibration action. |

The SDK's gripper holding-force setter is deprecated and reports that holding
force is fixed. Transport support does not make an unsupported device setting
adjustable.

The action adapter reports a ROS 1 rejection after local ROS 2 acceptance as an
aborted ROS 2 goal. In the reverse direction a ROS 2 goal rejection has no result
payload, so the ROS 1 result includes `ROS2_GOAL_REJECTED`. Server loss aborts
outstanding goals; a disconnected or aborted goal must not be treated as success.

Your control application is responsible for robot readiness, joint/force limits,
command timing and stale-state handling. A software stop over this network is
not a hardware emergency stop. Stopping the bridge disconnects transport and
does not itself disable Sawyer.

## 9. Start application development with a Python subscriber

A normal Python project can use the installed bridge without colcon or another
ROS workspace. From the project root, create this read-only starter:

```bash
cat > scripts/read_joint_states.py <<'PY'
"""Print joint feedback from the installed Sawyer bridge; publishes nothing."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class SawyerJointReader(Node):
    def __init__(self):
        super().__init__('sawyer_project_joint_reader')
        self.latest_state = None
        self.subscription = self.create_subscription(
            JointState, '/robot/joint_states', self.on_state,
            qos_profile_sensor_data)
        self.timer = self.create_timer(1.0, self.report)

    def on_state(self, message):
        self.latest_state = message

    def report(self):
        if self.latest_state is None:
            self.get_logger().info('Waiting for joint states...')
            return
        message = self.latest_state
        positions = dict(zip(message.name, message.position))
        self.get_logger().info(str(positions))


def main(args=None):
    rclpy.init(args=args)
    node = SawyerJointReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
PY
```

Leave the bridge running in terminal 1. Run your application in terminal 2:

```bash
cd ~/sawyer_project
source scripts/env.sh
python scripts/read_joint_states.py
```

The example reports the latest received joint positions once a second. It is a
development starter, not a state-freshness monitor for motion control: add
receive-time tracking before using cached feedback to make control decisions.

Use the same pattern for other subscriptions. For example:

```python
from intera_core_msgs.msg import EndpointState, JointCommand
from intera_core_msgs.srv import SolvePositionIK
from intera_motion_msgs.action import MotionCommand
```

Use `create_subscription`, `create_publisher`, `create_client`, and
`rclpy.action.ActionClient` as appropriate. ROS 2 applications consume the
bridge's endpoints; they do not need to rebuild or launch a second bridge.

## 10. Optional: make the application an ament_python package

Use this when you want `ros2 run`, installed package resources or your own launch
files. The initial project in section 9 does not require this step.

This workstation currently has a system `/usr/bin/colcon`, but `colcon_core` was
not installed in `ros2_env` during inspection. Use development tools from the
Conda environment so the build uses its Python 3.12. If needed, install them in
an activated environment and review the proposed dependency changes:

```bash
source scripts/env.sh
conda install --override-channels -c conda-forge -c robostack-humble \
  colcon-common-extensions
ros2 run sawyer_bridge check
```

Keep the bridge's Python/NumPy/Boost/Humble ABI requirements from section 1.
If the package check fails after a dependency change, resolve that before
developing against the environment.

Create the package from the project root:

```bash
mkdir -p src
ros2 pkg create sawyer_app \
  --destination-directory src \
  --build-type ament_python \
  --node-name read_joint_states \
  --dependencies rclpy sensor_msgs intera_core_msgs intera_motion_msgs

cp scripts/read_joint_states.py src/sawyer_app/sawyer_app/read_joint_states.py

"$CONDA_PREFIX/bin/colcon" build --symlink-install --packages-select sawyer_app
source install/local_setup.bash
ros2 run sawyer_app read_joint_states
```

The generated `setup.py` should contain the console entry point
`read_joint_states = sawyer_app.read_joint_states:main`. Keep your source code
under `src/sawyer_app/`; `build/`, `install/`, and `log/` are generated output.
Your project package must not be named `sawyer_bridge`, `intera_core_msgs`, or
`intera_motion_msgs`, which would shadow the installed packages.

In every new terminal that runs the packaged application:

```bash
cd ~/sawyer_project
source scripts/env.sh
source install/local_setup.bash
ros2 run sawyer_app read_joint_states
```

## 11. Supported features and current boundaries

The installed build includes 47 regular Intera message mappings, these three
Intera service types, and these two Intera action types:

| Kind | Supported Intera types |
|---|---|
| Service | `IOComponentCommandSrv`, `SolvePositionIK`, `SolvePositionFK` |
| Action | `CalibrationCommand`, `MotionCommand` |

The topic mappings cover arm/head control and feedback, interaction control,
gripper/peripheral IO, buttons/navigators, homing, collision status, URDF
configuration, and motion tracking data. Standard sensor/geometry/std message
mappings provide interfaces such as images, CameraInfo, Imu and simple commands.
Some message types are fields of other interfaces, not separate robot topics.
Mapping availability does not imply a corresponding endpoint is enabled or a
particular peripheral is attached.

List the exact installed message/service conversion pairs without connecting
to the robot:

```bash
"$CONDA_PREFIX/lib/sawyer_bridge/dynamic_bridge" --print-pairs
```

Despite its name, this `--print-pairs` invocation only prints mappings; it does
not start a second running bridge. The listing does not enumerate the separate
action protocol adapters.

This build does not currently provide:

- `/tf` or `/tf_static` conversion, or the robot's LookupTransform action.
- `control_msgs/FollowJointTrajectory`; Intera MotionCommand and JointCommand
  trajectory mode are separate supported interfaces.
- The robot's CameraFieldCalibrationCommand action.
- Missing vendor-specific `motor_control_msgs`, controller services,
  `external_tools` planning services, legacy planning-scene interfaces, Cognex
  command/job services or the tool URDF-update service.
- Automatic ROS 1 parameter to ROS 2 parameter mirroring or general
  dynamic_reconfigure bridging.
- Every general ROS mapping in the older workspace build, including its
  navigation, visualization, shape, trajectory, diagnostic, clock and std_srvs
  coverage.

Adding a YAML entry cannot create a missing converter. Such additions require
the correct interface definitions and a bridge rebuild or a separate adapter.
The release directory contains binaries and a local Conda channel, not the
bridge's complete build sources. Do not invent schemas for unavailable types.

## 12. Troubleshooting and stopping

| Symptom | Check / action |
|---|---|
| Python 3.11/3.12 paths mixed, or `_rclpy_pybind11` import fails | Check `command -v python`, `command -v ros2`, and `python -c 'import sys; print(sys.executable)'`. Use the clean shell in section 4, then source only this project's environment. |
| ROS 1 master connection fails | Check robot availability, `ROS_MASTER_URI`, PC network interface and robot connectivity. |
| Topics exist but telemetry never arrives | Check that `ROS_IP` is reachable from the robot, ROS 2 domains match, the source is publishing, and subscriber QoS is compatible. |
| Joint telemetry subscriber receives nothing | Use best-effort QoS as in the starter. |
| Late gripper subscriber receives nothing | Request reliable + transient-local QoS; confirm the bridge received the retained ROS 1 state. |
| Cameras have topics but no frames | Check actual acquisition and camera IO state; forwarding does not start cameras. |
| Command route is absent | Check YAML `enabled`, direction and relevant launch flags. Restart after changes. |
| More routes appear than expected | Check whether `all_interfaces:=true` augmented the YAML. |
| Duplicate robot data | Inspect `ros2 topic info /robot/joint_states --verbose`; identify and stop an older bridge in its original terminal. |
| New YAML appears ignored | Confirm the absolute `config:=...` path and restart the launch. |
| Unsupported conversion | Check `--print-pairs`; a missing definition/converter needs implementation, not just configuration. |

Logs are written under `~/.ros/log/` by default; the launcher prints the exact
directory. `Passing message ... (showing msg only once per type)` is a normal
first-message log, not a sign that forwarding stopped after one message.

To finish a control session, stop/cancel the application's active motion and
actions as appropriate, stop the application, then press Ctrl+C in the bridge
terminal. Disconnecting transport does not guarantee a physical stop or disable
the robot. Use the robot's operating procedures for its physical state.

## 13. Daily startup reference

Terminal 1, using the explicit extended profile:

```bash
cd ~/sawyer_project
source scripts/env.sh
ros2 launch sawyer_bridge bridge.launch.py \
  config:="$PWD/config/bridge.yaml" \
  all_interfaces:=false commands:=true head_commands:=true
```

Terminal 2, running your starter application:

```bash
cd ~/sawyer_project
source scripts/env.sh
python scripts/read_joint_states.py
```

If you changed the project path, replace `~/sawyer_project` throughout. If you
changed network addressing, edit `scripts/env.sh` once and source it again in
each terminal before restarting the bridge/application.
