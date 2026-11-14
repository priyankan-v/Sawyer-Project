# Optional Sawyer Humble interfaces

These typed ROS 2 actions/services/messages call the same direct Python driver.
They do not implement a ROS 1–ROS 2 bridge and do not require ROS 1 packages.
Generated Intera field definitions derive from the pinned public schema; see
LICENSE and NOTICE. No schema generation is required at runtime.

Use the repository's explicit installer from the repository root:

```bash
python irobot/src/robots/sawyer/setup_runtime.py install \
  --project "$PWD" --ros2 --interfaces --conda /path/to/conda
source .sawyer-runtime/activate.sh
```

This builds with CMake in the pinned Linux x86_64 Humble/Python 3.12 environment.
The plain Python driver and standard ROS 2 trajectory server do not need this
package. Custom type support must match the rclpy interpreter and middleware.

Development-only regeneration:

```bash
python irobot/src/robots/sawyer/installation/generate_ros2_interfaces.py \
  --output ros2/irobot_sawyer_interfaces
```

See the [driver guide](../../irobot/src/robots/sawyer/README.md) and
[capability checklist](../../irobot/src/robots/sawyer/DIRECT_DRIVER_PLAN.md).
Native planning, torque and compliant interaction are not hardware qualified.
