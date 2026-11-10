# Sawyer bridge: from installation to a Python command

This document explains the current project in two ways: a simple overview first,
then the technical flow. It describes the installed `sawyer_bridge` version
`0.1.0`, build `robostack_humble_py312_1`.

The paths below describe this computer's installation. The username `pi` is
part of the filesystem path; the installed bridge package targets Linux x86-64.
The command examples explain startup and message flow; writing this document
does not start the bridge or send commands to Sawyer.

## Part 1: Simple explanation

1. **The bridge is built before we install it.**

   The bridge is a C++ program compiled with ROS 1 libraries, ROS 2 libraries,
   and message converters. ROS 2 is used to communicate with our applications;
   ROS 1 is used to communicate with Sawyer.

2. **The compiled result is packaged for distribution.**

   The executable, supporting libraries, message interfaces, configuration tools,
   and launch files are distributed as a Conda package:

   ```text
   sawyer_bridge-0.1.0-robostack_humble_py312_1.conda
   ```

   It includes the ROS 1 client libraries needed by the bridge. The destination
   computer does not need to compile the bridge or install a complete separate
   ROS 1 system.

3. **Conda installs it on our computer.**

   Our package came from the local channel at `/home/pi/0.1.0` and is installed
   inside:

   ```text
   /home/pi/miniconda3/envs/ros2_env/
   ```

   This Conda environment is a directory containing programs and libraries. It
   is not a separate computer, container, or operating system. Installation puts
   files on disk; it does not automatically run the bridge.

4. **We prepare a terminal.**

   ```bash
   source scripts/env.sh
   ```

   This selects the correct Python and ROS installation and sets the network
   information for Sawyer. It sources `local_setup.bash`, which helps the shell
   and ROS tools find installed packages and libraries. These setup scripts do
   not stay running or forward messages.

5. **We start the bridge on our computer.**

   ```bash
   ros2 launch ./launch/sawyer.launch.py
   ```

   The launcher reads our configuration and starts `parameter_bridge`, the actual
   compiled bridge program. The bridge stays running in this terminal. Our
   launcher also starts the cuff-button and display helpers.

6. **The bridge uses ROS 1 and ROS 2 at the same time.**

   A single bridge process loads both sets of library code. Its ROS 2 side
   connects to our applications. Its ROS 1 side connects over the network to
   Sawyer's existing ROS 1 master and controller programs.

7. **We run a Python application in another terminal.**

   For example, a read-only status request is:

   ```bash
   source scripts/env.sh
   python scripts/gripper_control.py status
   ```

   Python runs the script on our computer. The script creates a ROS 2 node and
   its communication endpoints. It connects to the bridge through ROS 2 topics;
   it does not start another bridge.

8. **Commands pass through the bridge to Sawyer.**

   When an application publishes a command, the route is:

   ```text
   OUR COMPUTER                                    SAWYER

   Python application
          |
          | ROS 2 message
          v
   Bridge program
          |
          | Convert into a ROS 1 message
          |
          +------------- robot network ----------> Robot controller
                                                          |
                                                          v
                                                    Arm or gripper
   ```

   Our application decides what to request. The bridge translates and forwards
   the message. Sawyer's controller operates the hardware.

9. **Feedback returns through the bridge.**

   ```text
   Sawyer state -> ROS 1 -> bridge -> ROS 2 -> Python callback
   ```

   The application uses the feedback to inspect state and check command
   acknowledgement and completion. Sending a message alone does not confirm
   that physical movement has finished.

10. **The application and bridge have separate lifetimes.**

    When a one-shot Python application finishes, its process exits. The bridge
    stays running for the next application. Stopping the launcher stops the
    bridge and its launched helpers; it does not shut down the controller
    software inside Sawyer.

## Part 2: Detailed technical flow

### 1. Build: source code becomes executable files

The installed package identifies its build type as `ament_cmake`, the ROS 2
CMake-based build system. Its build uses ROS 2 C++ interfaces, ROS 1 C++ client
support, and corresponding message definitions.

Conceptually, the build produces:

```text
C++ bridge source + ROS 1 and ROS 2 interfaces
                         |
               Compile and link
                         |
                         +-- parameter_bridge executable
                         +-- libsawyer_bridge.so
                         +-- Supporting message libraries
```

The message converters need both representations of a message. For example:

```text
ROS 2: intera_core_msgs/msg/IOComponentCommand
ROS 1: intera_core_msgs/IOComponentCommand
```

The installed `libsawyer_bridge.so` contains compiled conversion functions for
this pair. Adding a YAML entry selects supported conversion code; it cannot
create a converter for an unknown message type.

The release bundle contains the prebuilt package, not the complete bridge build
sources or recipe. This document therefore describes the verified build
components rather than an unverified sequence of original build commands.
The installed `package-build.json` records the bundled ROS 1 source versions.

### 2. Packaging and installation

The release metadata identifies the artifact as:

```text
/home/pi/0.1.0/linux-64/
  sawyer_bridge-0.1.0-robostack_humble_py312_1.conda
```

Conda installs the package and resolves its declared dependencies from the
configured channels. The installed Conda record confirms that our package came
from the local channel at `/home/pi/0.1.0`.

Important installed files are:

| Component | Location relative to `/home/pi/miniconda3/envs/ros2_env/` |
| --- | --- |
| Python interpreter | `bin/python` |
| ROS CLI entry point | `bin/ros2` |
| Bridge executable | `lib/sawyer_bridge/parameter_bridge` |
| Configuration utility | `lib/sawyer_bridge/configure_bridge` |
| Bridge shared library | `lib/libsawyer_bridge.so` |
| ROS 2 C++ client library | `lib/librclcpp.so` |
| Private ROS 1 libraries | `libexec/sawyer/ros1/lib/` |
| Installed bridge launcher | `share/sawyer_bridge/launch/bridge.launch.py` |
| Default bridge configuration | `share/sawyer_bridge/config/sawyer_bridge.yaml` |
| Package environment setup | `local_setup.bash` |

The application source remains in `/home/pi/pri/sawyer_project/`; installing the
bridge does not install this project's scripts as a separate ROS package.

### 3. Environment setup: what `source scripts/env.sh` does

The project environment script:

```bash
source /home/pi/miniconda3/etc/profile.d/conda.sh
conda activate ros2_env
source "$CONDA_PREFIX/local_setup.bash"

export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME
export ROS_DOMAIN_ID=42
hash -r
```

`source` executes the file in the current shell, allowing its environment changes
to affect later commands. New child processes inherit those exported settings.
Each terminal used for the project needs the setup.

`local_setup.bash` invokes the installed setup helper and runs package environment
hooks in dependency order. These hooks extend search paths such as:

| Variable | Purpose |
| --- | --- |
| `PATH` | Locate executables such as `python` and `ros2` |
| `PYTHONPATH` | Add Python module search locations |
| `AMENT_PREFIX_PATH` | Locate installed ROS packages and their resources |
| `LD_LIBRARY_PATH` | Help the native loader find shared libraries |

Python also uses its interpreter's normal standard-library and `site-packages`
locations. `hash -r` clears Bash's remembered executable locations after the
environment changes.

The communication settings have separate roles:

| Variable | Used for |
| --- | --- |
| `ROS_DOMAIN_ID=42` | ROS 2 discovery domain shared by the application and bridge |
| `ROS_MASTER_URI` | Address of Sawyer's existing ROS 1 master |
| `ROS_IP` | Address advertised by this computer to ROS 1 peers |

Setup prepares the runtime environment. It does not create a bridge process.

### 4. Launch: starting the persistent bridge process

From the project root:

```bash
ros2 launch ./launch/sawyer.launch.py
```

The startup sequence is:

```text
ros2 launch
   |
   +-- Load project launch/sawyer.launch.py
          |
          +-- Find installed sawyer_bridge package
          +-- Include its launch/bridge.launch.py
          |      |
          |      +-- Start configure_bridge
          |      |      Read selected bridge.yaml
          |      |      Apply options/discovery
          |      |      Upload ROS 1 parameters
          |      |      Exit
          |      |
          |      +-- On successful configuration:
          |             Start parameter_bridge
          |
          +-- Start gripper cuff-button helper, if enabled
          +-- Start display manager, if enabled
```

The project launcher uses `get_package_share_directory('sawyer_bridge')` to find
the installed launch file. It passes the project configuration path and launch
options to that file.

`configure_bridge` uses Python's XML-RPC client to contact the master at
`ROS_MASTER_URI`. It uploads configuration under `/sawyer_bridge`, including
topic, service, and action settings. With `all_interfaces` enabled, discovery
can extend the configuration; the effective configuration can therefore include
more than the entries written explicitly in the YAML.

The installed launcher uses `ExecuteProcess` to describe the executable and an
`OnProcessExit` handler to start it only after configuration succeeds. The
effective bridge command is:

```text
/home/pi/miniconda3/envs/ros2_env/lib/sawyer_bridge/parameter_bridge
    /sawyer_bridge/topics
    /sawyer_bridge/services_1_to_2
    /sawyer_bridge/services_2_to_1
    __name:=sawyer_bridge
```

The `/sawyer_bridge/...` arguments are ROS 1 parameter names, not filesystem
paths. The C++ process reads the configuration and maintains the communication
routes. The launcher manages its lifetime but does not forward message payloads.

### 5. How the executable contains ROS 1 support

Linux loads the bridge's shared-library dependencies when starting the
executable. Its embedded library search path, called `RUNPATH`, includes:

```text
$ORIGIN/../../libexec/sawyer/ros1/lib
$ORIGIN/..
```

`$ORIGIN` is the executable's directory. In this installation, those paths find
the private ROS 1 libraries and the environment's main library directory.

Verified linked components include:

| Library | Responsibility |
| --- | --- |
| `libroscpp.so` | ROS 1 C++ nodes, endpoints, and networking |
| `libxmlrpcpp.so` | XML-RPC communication used by ROS 1 |
| `libroscpp_serialization.so` | ROS 1 message serialization |
| `librostime.so` | ROS 1 time support |
| `librclcpp.so` | ROS 2 C++ client functionality |
| `librcl.so` | Common ROS 2 client functionality |
| `libsawyer_bridge.so` | Bridge functionality and compiled converters |

The binary references both `ros::init(...)` and `rclcpp::init(...)`, along with
ROS 1 `NodeHandle` and ROS 2 `Node` functionality. It participates in both ROS
graphs within one operating-system process.

Sawyer supplies the existing ROS 1 master and Intera controllers. The bridge
does not need to start another ROS 1 master on this computer.

### 6. Configuration becomes publishers and subscribers

For the gripper command, the project configuration includes:

```yaml
- topic: /io/end_effector/right_gripper/command
  type: intera_core_msgs/msg/IOComponentCommand
  queue_size: 1
  direction: 2to1
  enabled: true
  qos:
    history: keep_last
    depth: 1
    reliability: reliable
```

The corresponding route connects a ROS 2 subscriber to a ROS 1 publisher.
The gripper state route goes the opposite way:

| Route | Bridge's ROS 2 endpoint | Bridge's ROS 1 endpoint |
| --- | --- | --- |
| Gripper command, `2to1` | Subscriber | Publisher |
| Gripper state, `1to2` | Publisher | Subscriber |

In the installed `factory.hpp`, ROS 1 publisher creation uses:

```cpp
return node.advertise<ROS1_T>(topic_name, queue_size, latch);
```

ROS 2 subscription creation registers a conversion callback:

```cpp
return node->create_subscription<ROS2_T>(
    topic_name, qos, callback, options);
```

`ROS1_T` and `ROS2_T` are template parameters for the paired message types. The
endpoint factory binds the callback to the ROS 1 publisher it should forward to.

The same topic name exists in two separate ROS graphs. Its spelling alone does
not connect the graphs; the bridge creates that connection explicitly.

### 7. Starting the Python application

Consider the command syntax:

```text
python scripts/gripper_control.py <action> [options]
```

The read-only `status` action can be used to inspect the application flow. The
following command-publishing explanation describes `open` with `--execute`;
omitting `--execute` previews that action without publishing a gripper command.

At startup:

1. Bash finds the Conda environment's Python executable.
2. Linux starts a Python process with the shell's exported environment.
3. Python loads the script and its imports, including `rclpy`, message classes,
   and the local `gripper_io.py` helper.
4. The `if __name__ == '__main__'` block calls `main()`.
5. `argparse` reads the requested action and options.
6. `rclpy.init(...)` initializes a ROS 2 context.
7. `GripperControl(options)` constructs the ROS 2 node.
8. The constructor creates the state subscription and command publisher.
9. `node.run()` starts the application's state checks and requested action.

`rclpy` loads a native extension named `_rclpy_pybind11...so`. The message package
supplies generated Python classes and native type support. These libraries run
inside the Python process; they are not additional standalone programs.

The application does not require `ros2 run`: that command finds and starts an
installed package executable. Communication comes from the ROS libraries used
inside the program, regardless of how it was started.

### 8. Publishing: Python to the bridge's ROS 2 subscriber

The gripper script creates a publisher for:

```text
Topic: /io/end_effector/right_gripper/command
Type:  intera_core_msgs/msg/IOComponentCommand
```

ROS 2 middleware discovers compatible publishers and subscribers in the same
domain. Topic, message type, and QoS must be compatible. The Python application
does not need the bridge's IP address or a direct function call into its process.

For opening, `perform()` prepares:

```python
{'position_m': 0.041667, 'go': True}
```

The application validates signal encoding and, for execution, checks gripper
readiness and a connected command subscriber. It builds an `IOComponentCommand`
containing a timestamp, `op='set'`, and JSON-encoded signal arguments, then calls:

```python
self.publisher.publish(message)
```

The layers are:

```text
Python IOComponentCommand object
              |
              v
rclpy -> native extension -> rcl -> rmw -> DDS middleware
              |
       Interprocess transport
              |
              v
Bridge DDS middleware -> ROS 2 subscription -> C++ callback
```

`rmw` is ROS 2's interface to the selected middleware. The transport and
serialization details depend on the middleware configuration, including whether
local shared-memory transport is available. ROS 2 topics are named channels,
not a central message server.

### 9. Converting: inside the bridge callback

The installed `factory.hpp` contains the ROS 2 callback's forwarding operation:

```cpp
ROS1_T ros1_msg;
convert_2_to_1(*ros2_msg, ros1_msg);
ros1_pub.publish(ros1_msg);
```

The callback creates a ROS 1 object, converts the ROS 2 fields, and publishes the
result. For `IOComponentCommand`, the command timestamp, operation, and JSON
arguments retain their meaning across the conversion.

The bridge forwards the JSON argument string; Sawyer's controller interprets
the requested gripper signals. Conversion does not calculate a trajectory or
directly drive the gripper motor.

`factory.hpp` is source code used to build this implementation. At runtime, the
compiled executable and libraries execute; the program does not reread or
recompile the header for each message.

### 10. ROS 1 transport: bridge to Sawyer

The bridge's ROS 1 publisher registers with the master at `ROS_MASTER_URI`.
Sawyer's subscriber discovers that publisher through ROS 1 registration and
lookup. The subscriber negotiates a connection with the publisher.

With the usual TCPROS transport:

1. XML-RPC is used for registration and connection negotiation.
2. The subscriber connects to the publisher's negotiated TCP port.
3. Connection metadata checks message compatibility.
4. The bridge serializes ROS 1 messages and sends them through that connection.
5. Sawyer deserializes the messages and its controller processes the command.

Port `11311` is the master's lookup/control endpoint. The command payload travels
over the negotiated publisher-to-subscriber connection, not through the master.

### 11. Feedback and callback execution

Sawyer publishes `IODeviceStatus` on the gripper state topic. The reverse route
is:

```text
Sawyer ROS 1 publisher
          |
          v
Bridge ROS 1 subscriber
          |
     convert_1_to_2()
          |
          v
Bridge ROS 2 publisher
          |
          v
Python ROS 2 subscription
          |
          v
Executor invokes on_gripper(message)
```

The bridge references a ROS 1 `AsyncSpinner` and ROS 2 executor processing to
service callbacks while it remains running.

In the Python application, `wait_for()` repeatedly calls:

```python
rclpy.spin_once(self, timeout_sec=0.02)
```

The middleware receives data; the executor takes ready messages and invokes
registered Python callbacks. `on_gripper()` decodes the signals and updates the
application's stored state.

For an executed open/close request, the script checks two separate stages:

- **Acknowledgement:** the returned device status lists the command timestamp.
  The send loop can republish the same timestamped command while waiting.
- **Completion:** the gripper reports no movement and a measured position within
  `0.002 m` of the requested target. Closing may also complete when the device
  reports that it is gripping an object.

A matched ROS 2 subscription, successful `publish()` call, and device
acknowledgement each provide different evidence. None alone proves physical
motion has completed.

### 12. Process lifetimes and shutdown

The normal process arrangement is:

```text
Computer: terminal 1
  ros2 launch ./launch/sawyer.launch.py
    +-- parameter_bridge
    +-- gripper_control.py buttons ...
    +-- display_manager.py ...

Computer: terminal 2
  python scripts/gripper_control.py <action> ...

Sawyer
  Existing ROS 1 master and parameter server
  Existing Intera controller nodes
```

`configure_bridge` is a temporary startup process and has already exited once
the bridge is running. A ROS node is a logical ROS participant inside a process;
a shared library is code loaded into a process. Neither necessarily corresponds
to another operating-system process.

When a one-shot Python request finishes, its `finally` block destroys the node
and shuts down its ROS context. Python exits and the terminal gets its prompt
back. The bridge remains available. Ctrl+C in the launch terminal shuts down its
launched processes without shutting down Sawyer's internal controller software.

## Files to inspect

Project links are relative to this document. Installed-package links below are
specific to this computer.

| File | What it explains |
| --- | --- |
| [scripts/env.sh](scripts/env.sh) | Conda, ROS setup, and network variables |
| [launch/sawyer.launch.py](launch/sawyer.launch.py) | Project startup and helper processes |
| [bridge.yaml](bridge.yaml) | Selected topic, service, action, and QoS settings |
| [scripts/gripper_control.py](scripts/gripper_control.py) | Node creation, publishing, and feedback handling |
| [scripts/gripper_io.py](scripts/gripper_io.py) | Gripper signal validation and JSON encoding |
| [SAWYER_BRIDGE_GUIDE.md](SAWYER_BRIDGE_GUIDE.md) | Existing installation and interface guide |
| [local_setup.bash](/home/pi/miniconda3/envs/ros2_env/local_setup.bash) | Package environment-hook execution |
| [Installed bridge.launch.py](/home/pi/miniconda3/envs/ros2_env/share/sawyer_bridge/launch/bridge.launch.py) | Configuration process and bridge process startup |
| [configure.py](/home/pi/miniconda3/envs/ros2_env/lib/python3.12/site-packages/sawyer_bridge/configure.py) | YAML loading and ROS 1 parameter upload |
| [factory.hpp](/home/pi/miniconda3/envs/ros2_env/include/ros1_bridge/factory.hpp) | Publisher/subscriber creation and conversion callbacks |
| [package.xml](/home/pi/miniconda3/envs/ros2_env/share/sawyer_bridge/package.xml) | Build type and package dependencies |
| [package-build.json](/home/pi/miniconda3/envs/ros2_env/share/sawyer_bridge/package-build.json) | Recorded private ROS 1 build inputs |
| [Release README](/home/pi/0.1.0/README.md) | Prebuilt package distribution and installation instructions |
