# Optional ros_sugar lifecycle application

The direct Python driver and standard Humble server do not require ros_sugar.
Real lifecycle validation used Sugarcoat/ros_sugar 0.8.2, commit
`5d64e015488fe9f0ca7b844028e7907f43785db0`, with Humble/Python 3.12.16 on
Ubuntu 22.04 x86_64. The upstream source is
[automatika-robotics/sugarcoat](https://github.com/automatika-robotics/sugarcoat).
See `ros-sugar-release.json` for provenance. This is an optional application,
not a second hardware driver or a ROS 1–ROS 2 bridge.

To reproduce in an isolated environment from the repository root:

```bash
conda create --yes --prefix "$PWD/.sawyer-runtime/envs/ros-sugar" \
  --file irobot/src/robots/sawyer/installation/ros-sugar-linux-64.explicit.txt
export PATH="$PWD/.sawyer-runtime/envs/ros-sugar/bin:$PATH"
source .sawyer-runtime/envs/ros-sugar/setup.bash
python -m pip install -e '.[sawyer]' \
  -r irobot/src/robots/sawyer/installation/ros-sugar-requirements.txt

git clone --no-checkout https://github.com/automatika-robotics/sugarcoat.git \
  .sawyer-runtime/vendor/sugarcoat
git -C .sawyer-runtime/vendor/sugarcoat checkout \
  5d64e015488fe9f0ca7b844028e7907f43785db0

export ROS_DISTRO=humble
export ROS_LOG_DIR="$PWD/.sawyer-runtime/ros-sugar/log"
export AMENT_PREFIX_PATH="$PWD/.sawyer-runtime/envs/ros-sugar"
cmake -S .sawyer-runtime/vendor/sugarcoat \
  -B .sawyer-runtime/ros-sugar/build \
  -DCMAKE_PREFIX_PATH="$PWD/.sawyer-runtime/envs/ros-sugar" \
  -DPython3_EXECUTABLE="$PWD/.sawyer-runtime/envs/ros-sugar/bin/python" \
  -DPYTHON_EXECUTABLE="$PWD/.sawyer-runtime/envs/ros-sugar/bin/python" \
  -DCMAKE_INSTALL_PREFIX="$PWD/.sawyer-runtime/ros-sugar/install" \
  -DBUILD_TESTING=OFF
cmake --build .sawyer-runtime/ros-sugar/build --parallel 2
cmake --install .sawyer-runtime/ros-sugar/build
source .sawyer-runtime/ros-sugar/install/local_setup.bash
```

The PATH line selects the pinned Python/CMake; Conda activation is also an
option. System Humble Python 3.10 bindings cannot be mixed with this runtime.
The optional lock adds framework ROS dependencies to the direct-driver lock;
the source package's Python dependencies are pinned separately. No robot is
contacted during this installation. A C/C++ compiler and Git are needed.

The adapter can be used with the upstream Launcher or initialized explicitly:

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
from irobot import SawyerConfig
from irobot.src.robots.sawyer.ros_sugar_component import SawyerComponent

rclpy.init()
component = SawyerComponent(sawyer_config=SawyerConfig(
    ros_master_uri='http://ROBOT_IP:11311', ros_ip='PC_IP', tip='right_hand'
))
component.rclpy_init_node()  # The framework already has the node name.
component.trigger_configure()  # Connect; never enable or reset.
component.trigger_activate()  # Starts the application timer; no movement.
component.move_to_neutral()  # Preview only; result is in last_result.
executor = MultiThreadedExecutor()
executor.add_node(component)
try:
    executor.spin()
finally:
    component.trigger_deactivate()
    component.trigger_cleanup()
    executor.shutdown()
    component.destroy_node()
    rclpy.shutdown()
```

Select this application or SawyerNode as the hardware owner. Actual movement
still needs explicit `execute=True` and a ready controller. The current enable
recovery problem remains a hardware release blocker; framework activation does
not fix it.

The real-framework regression uses a fake driver and actual ROS lifecycle/DDS:

```bash
SAWYER_SUGAR_TEST=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python -m pytest -q tests/test_irobot/test_sawyer_component.py \
  tests/test_irobot/test_sawyer_sugar_runtime.py
```

It verifies configure/activate, preview collection by the framework timer,
worker cancellation on deactivate, and cleanup. A separate concurrency test
checks that collecting a completed task cannot lose a newly submitted task.
