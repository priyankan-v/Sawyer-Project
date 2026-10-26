## In every teminal

```bash
source scripts/env.sh
```

### in terminal 1
```bash
ros2 launch sawyer_bridge bridge.launch.py \
  config:="$PWD/bridge.yaml" \
  all_interfaces:=true commands:=true head_commands:=true
```

### In another terminal
```bash
ros2 topic echo /robot/joint_states --once
```
### Move the arm to neutral

Keep the bridge running. In another terminal at the project root:

```bash
source scripts/env.sh
python scripts/move_neutral.py
```

This previews current and neutral joint angles without sending a motion. The
neutral pose comes from `/named_poses/right` on the robot's ROS 1 parameter
server, using `ROS_MASTER_URI` from `scripts/env.sh`.

Enable the robot using its normal controls and ensure the arm's path is clear.
Request the motion at 10% of maximum joint speed:

```bash
python scripts/move_neutral.py --execute
```

The ROS 2 node sends a joint trajectory through `/motion/motion_command`,
monitors robot telemetry, checks the action result, and confirms measured joint
angles. It requests cancellation on timeout, interruption, or lost readiness;
cancellation requires a working bridge/controller connection. Keep the physical
stop accessible. This joint-space move does not plan around workspace obstacles
or keep the end effector on a straight Cartesian path.

No ROS 2 package build is needed. Offline tests (no robot connection):

```bash
python -m unittest discover -s scripts -p 'test_move_neutral.py'
```
