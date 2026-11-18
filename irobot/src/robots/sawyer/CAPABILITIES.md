# Sawyer implementation capabilities

This is an implementation map. Recorded hardware results and fault reports
are not retained. Hardware qualification is `not_recorded`; no successful
physical validation is claimed by this document or the operation ledger.

The [operation ledger](CAPABILITY_LEDGER.json) maps the pinned public SDK
operations to Python, ROS 2 and controller interfaces, units and limits.
The core driver uses direct XML-RPC/TCPROS; ROS 2 uses the same Python backend.
See the [file/function guide](IMPLEMENTATION_GUIDE.md) and
[remaining checklist](UNFINISHED_WORK.md).

| Capability | Implementation and ROS 2 access |
| --- | --- |
| Identity, parameters, inventory | Direct master queries; inventory/read_parameter/capabilities services |
| Assembly, joints, efforts, limits | Python feedback; status, robot_state, joint_states, joint_limits |
| Enable/reset/disable/software stop | Explicit Python calls and services; ready/displacement guards |
| Neutral, joints, rest-to-rest waypoints | Bounded planner; neutral Trigger, FollowJointTrajectory subset |
| Endpoint, FK/IK, collision query | Named-tip API; pose/twist/wrench and typed services |
| Position/velocity streaming | Bounded sessions; JointStream action and leased StreamCommand |
| Torque and trajectory mode | Validated command vectors, published/URDF limits; experimental opt-in |
| Native planning/execution | Typed native actions; IDs, feedback, cancel, timeouts |
| Interaction/force/impedance | Typed timed sessions; compliant/force modes require experimental opt-in |
| Arm calibration | Typed action and attached-tool preconditions |
| Gripper | Calibrate/open/close/position/stop/start/reboot/settings; Trigger and GripperCommand |
| IO and inputs | Merged typed state/config, IO health, bounded edges/callbacks, validated signal/port writes |
| Lights/digital outputs | Metadata-checked writes; settings/digital services |
| Display | RGB/BGR Image publication in Python or ROS 2 |
| Cameras | Explicit streaming/settings, current frames, CameraInfo, fast standard ROS 2 Image CDR |
| Diagnostics/TF/model | Python reads; diagnostics/tf/tf_static/read_parameter |
| Head pan | Existing bounded Python implementation retained |
| ClickSmart/tool reconfiguration | Discovery only; no arbitrary private command forwarding |
| ros_sugar application | Optional lifecycle adapter using SawyerController |

Public wire schemas define the supported protocol contract. Additional vendor
interfaces require documented schemas; graph names alone are not an API.
Model collision queries do not establish external-obstacle clearance.
Experimental torque, trajectory and force modes require explicit opt-in.
Head-pan and disable/recovery tests remain outside the requested test scope.

Regenerate the implementation ledger without contacting hardware:

```bash
python irobot/src/robots/sawyer/installation/build_capability_ledger.py
```
