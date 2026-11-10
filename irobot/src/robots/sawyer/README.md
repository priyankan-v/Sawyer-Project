# Sawyer

A class-based driver for Sawyer's right arm and standard Intera electric gripper.
It follows the existing irobot pattern: `SawyerConfig` stores hardware settings,
`SawyerBase` owns the connection and state, and `SawyerController(SawyerBase)`
provides motion primitives.

```text
Application → SawyerController → ROS 2 → sawyer_bridge → ROS 1 / Intera → Sawyer
```

ROS 2 and the separately installed native bridge are required even from ordinary
Python. `ros_sugar` is optional. Construction and `connect()` send no commands.
The external bridge is started separately and remains running after disconnect.

## Layout

```text
sawyer/
├── __init__.py
├── config.py                 # Hardware settings, limits, timeouts and bridge routes
├── core/
│   ├── __init__.py
│   ├── base.py               # Bridge connection, feedback, IO, IK/FK and lifecycle
│   └── controller.py         # Joint/Cartesian trajectories, PID and public commands
├── examples/                 # Python usage and ROS launch example
├── installation/             # Pinned native dependency manifest
├── setup_runtime.py          # Dependency bootstrap; usable before Python SDK installation
└── README.md
```

The installation files are the addition to the standard robot template. There
are no task recordings, experiment workflows, plots, display assets, or native
bridge binaries in this driver.

## Prerequisites and installation

The supplied bridge release requires **Linux x86_64, glibc >= 2.35, Python 3.12,
NumPy 2, Boost 1.88, and its pinned RoboStack Humble builds**. An ARM Raspberry
Pi or system `/opt/ros/humble` installation cannot use this particular binary.
Other irobot drivers retain the repository's normal Python >= 3.11 requirement.

Obtain the Sawyer bridge `0.1.0 / robostack_humble_py312_1` release channel from
its maintainer. Keep these files together:

```text
release-directory/
├── linux-64/
│   ├── repodata.json
│   └── sawyer_bridge-0.1.0-robostack_humble_py312_1.conda
└── noarch/
    └── repodata.json
```

There is no public bridge download URL configured here. The installer requires
its extracted location and verifies the binary against the bundled SHA-256.
From the irobot checkout, with Bash and system Python 3.10+ available:

```bash
# Check an existing runtime without changing it.
python3 irobot/src/robots/sawyer/setup_runtime.py check \
  --env-prefix /path/to/conda/envs/sawyer

# Optional plan only: no downloads or package changes.
python3 irobot/src/robots/sawyer/setup_runtime.py install \
  --bridge-channel /path/to/release-directory \
  --env-prefix "$PWD/.sawyer-runtime/env" --dry-run

# Install the runtime and irobot with Sawyer's Python dependencies.
python3 irobot/src/robots/sawyer/setup_runtime.py install \
  --bridge-channel /path/to/release-directory \
  --env-prefix "$PWD/.sawyer-runtime/env"

source .sawyer-runtime/activate.sh
```

Use the file path above before installation: the normal `irobot` package imports
its existing Crazyflie SDK, which pip installs as a project dependency. The
bootstrap file itself uses only the standard library and does not import irobot.

Setup discovers Conda or accepts `--conda /path/to/bin/conda`. If absent, it
checks and installs the official [Miniforge](https://github.com/conda-forge/miniforge)
installer under `.sawyer-runtime/miniforge`. It resolves the bridge dependencies
from the supplied local channel, conda-forge, and robostack-humble, checks native
ROS type support, and runs `pip install -e '.[sawyer]'` through the selected
Conda environment. pip uses the repository's original Poetry build backend.
Internet access is needed for uncached dependencies; no sudo or ROS 1 build is
required. Setup never contacts or commands the robot.

A healthy runtime skips the Conda transaction. `--env-prefix` defaults to the
active Conda environment, otherwise `.sawyer-runtime/env`. Select a new prefix
to keep an existing environment unchanged. Conda base is rejected.
`--runtime-root` changes the bootstrap/activation-script directory.

Do not source system ROS setup files into this environment; see
[RoboStack's environment guidance](https://robostack.github.io/GettingStarted.html).
Importing or connecting the driver never installs dependencies implicitly.

## Network and bridge

Activate the same environment in each terminal and set the deployment's network
values. Sawyer must be able to reach the PC's `ROS_IP`:

```bash
source .sawyer-runtime/activate.sh
export ROS_MASTER_URI=http://ROBOT_IP:11311
export ROS_IP=PC_IP
unset ROS_HOSTNAME
export ROS_DOMAIN_ID=42

ros2 launch ./irobot/src/robots/sawyer/examples/bridge.launch.py commands:=true
```

Run one bridge. The example generates its routes from `SawyerConfig`, including
feedback, command/watchdog, gripper, and IK/FK interfaces; hardware endpoint
names are not duplicated in a separate YAML. Broad discovery is disabled.
An existing bridge may also be used if it provides those interfaces and the
same ROS domain. Disable competing arm/gripper command applications.

For custom deployments, `SawyerConfig.bridge_configuration()` returns the bridge
configuration dictionary. Save it as YAML and pass `config:=/path/to/bridge.yaml`
when launching. Configure the driver with the same endpoint settings.

Omitting `commands:=true` disables the arm command routes. Gripper transport is
still configured; this flag is not a global write-disable switch. Starting the
bridge creates transport, without enabling Sawyer, calibrating the gripper,
or generating motion commands.

## Python API

```python
from irobot import SawyerConfig, SawyerController
from irobot.src.robots.sawyer.config import CartesianMotionOptions

config = SawyerConfig(motion=CartesianMotionOptions(speed=0.1))

with SawyerController(config) as robot:
    print(robot.get_status())
    print(robot.joint_positions)   # right_j0..right_j6, radians
    print(robot.endpoint_pose)     # meters, quaternion xyzw, reference frame
    print(robot.gripper_state)

    preview = robot.move_to_neutral()  # No commands
    print(preview.plan.duration)

    # Execute explicitly after inspecting the preview and workspace:
    # robot.move_to_neutral(execute=True)
    # robot.move_relative(dz_cm=1, duration=3, execute=True)
    # robot.open_gripper(execute=True)
```

| API | Behavior |
| --- | --- |
| `connect()` / `disconnect()` | Open/close a private session; leave the external bridge running |
| `get_status()` | Refresh readiness, feedback and diagnostics |
| `joint_positions` / `joint_velocities` | Refresh and return seven ordered joint values |
| `endpoint_pose` / `gripper_state` | Refresh endpoint/gripper feedback |
| `move_to_joints(joints, ...)` | Quintic joint move, angles in radians |
| `move_to_neutral(...)` | Plan toward the configured/robot neutral joint pose |
| `move_relative(dx_cm=0, dy_cm=0, dz_cm=0, ...)` | Straight base-frame Cartesian offset, centimeters, fixed orientation |
| `open_gripper()` / `close_gripper()` | Preview by default; execution confirms IO acknowledgement and settled feedback |
| `calibrate_gripper()` / `stop_gripper()` | Explicit operations, also preview by default |
| `stop()` | Request arm cancellation; the control loop performs its normal stop |

Motion methods accept `duration=None`, `execute=False`, and `options=None`.
They return a plan plus an execution result (`None` for a preview/already met
joint target). Gripper methods return action/result dictionaries.
Read-only connection works with a disabled arm; motion requires fresh feedback,
readiness, no reported collision, and enabled controllers. Opening/closing the
gripper requires calibration. Pass `enable_gripper=False` without this tool.

Neutral lookup reads `/named_poses/right` through XML-RPC using `ROS_MASTER_URI`
or `SawyerConfig.ros_master_uri`. Provide `neutral_joints` to avoid that lookup.
IK/FK are read-only ROS 2 service calls through the bridge.

Each instance owns a private ROS context, node and persistent executor. Methods
are synchronous and service callbacks while working; poll state/status to refresh
feedback while idle. Concurrent operations are rejected. Another thread can
call `stop()` to request arm cancellation. The application/global ROS context
is not shut down by the driver.

The repository's `main.py` now runs the Sawyer neutral-position example.
Start the bridge first, then use either entry point:

```bash
python main.py                                             # Preview only
python main.py --execute                                   # Move to neutral
python -m irobot.src.robots.sawyer.examples.move_neutral    # Same preview
python -m irobot.src.robots.sawyer.examples.move_neutral --execute
```

The neutral example uses the public `SawyerController.move_to_neutral()` API,
prints the current/target joints and minimum feasible duration, and reacquires
feedback before execution. Defaults are 10% velocity limits and a 0.3 rad/s²
acceleration cap; `--speed`, `--max-accel`, and `--time` override them. A requested
duration below the feasible minimum is rejected. Gripper feedback and commands
are excluded from this arm-only example. The robot is not enabled automatically.

Runnable examples:

```bash
python -m irobot.src.robots.sawyer.examples.status
python -m irobot.src.robots.sawyer.examples.move neutral
python -m irobot.src.robots.sawyer.examples.move relative --dz-cm 1 --time 3
# Add --execute explicitly for motion; --no-gripper excludes gripper feedback.
```

## Validation

The repeatable hardware check previews by default. Explicit execution makes a
1 cm upward move and returns to the measured start. The optional gripper test
requires an empty, stationary gripper and finishes with it open:

```bash
python -m irobot.src.robots.sawyer.examples.validate
python -m irobot.src.robots.sawyer.examples.validate --execute --gripper
```


With the runtime and the repository's test dependencies installed:

```bash
python -m pytest tests/test_irobot -q
```

On a ROS environment with an incompatible third-party pytest plugin, run the
same offline suite with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
The generated-message tests skip if ROS interfaces are absent; numerical,
lifecycle and installer tests remain available. No unit test connects to the
robot or initializes ROS. A separate synthetic DDS test is explicit:

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=231 python -m tests.test_irobot.smoke_sawyer_runtime
```

Physical Sawyer validation on **November 9, 2026** covered joint/endpoint/gripper
feedback, read-only neutral planning, seeded IK/FK, a 1 cm upward Cartesian move
and return, and electric-gripper calibration/open/close/open. The arm used
5% velocity limits and 0.1 rad/s² acceleration with 3-second references; its
measured final return error was approximately **0.092 mm** in the final repeatable check.
The gripper test exposed an asynchronous completion race; completion now
requires calibrated, stationary, ready feedback maintained for the configured
confirmation interval. Recalibration followed immediately by open/close/open
passed after that fix. The test ended with the arm returned and gripper open,
calibrated and stationary. The temporary bridge was stopped.

The neutral example was also executed on the physical Sawyer using
`python main.py --execute`, with 10% velocity limits and a 0.3 rad/s²
acceleration cap. All 201 sampled joint-path configurations passed the robot's
FK validity/collision check before execution. The 32.874-second reference
completed tracking and settling in 33.086 seconds, with maximum measured joint
error **0.000371 rad**. The arm was left at neutral and the temporary bridge
was stopped; this example sent no gripper commands.

The driver retains freshness/watchdog bounds of at most 0.2 seconds, joint and
Cartesian tracking checks, published joint bounds, collision/readiness checks,
and zero-velocity/position-hold stopping. Sampled FK validation is not workspace
obstacle avoidance or a proof of continuous collision freedom. The limited
hardware tests do not establish every operating pose, load, control rate, or
failure scenario. Clean-machine installation orchestration is unit-tested;
a full fresh download/install has not been performed on another computer.
