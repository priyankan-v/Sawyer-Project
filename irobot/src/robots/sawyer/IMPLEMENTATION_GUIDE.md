# Sawyer implementation guide

Python applications, the optional ROS 2 node, and ros_sugar use the same
Sawyer driver. Modules run in one process; folders separate responsibilities.

```text
Application / ROS adapter → SawyerController
                               ├── planner → paths and timing
                               ├── executor → measured tracking
                               └── SawyerBase → shared hardware session
                                                    ↓
                                            transport + wire
                                                    ↓
                                               Intera robot
```

## Files and responsibilities

| File | Responsibility |
| --- | --- |
| [config.py](config.py) | Configuration, motion/stream options, and InteractionLimits |
| [core/base.py](core/base.py) | SawyerBase connection lifecycle, measured state, and model queries |
| [core/controller.py](core/controller.py) | SawyerController public motion API and operation coordination |
| [core/types.py](core/types.py) | Poses, joint bounds, plans, samples, and result data |
| [core/planning/cartesian.py](core/planning/cartesian.py) | CartesianLine and CartesianPlanner; seeded IK and sampled FK checks |
| [core/planning/joint.py](core/planning/joint.py) | QuinticTrajectory, JointSequence, JointPath |
| [core/planning/timing.py](core/planning/timing.py) | TimedJointPath and uniform time scaling |
| [core/planning/native.py](core/planning/native.py) | Native trajectory builders and interaction command validation |
| [core/execution/trajectory.py](core/execution/trajectory.py) | Tracking loop and joint/Cartesian execution |
| [core/execution/pid.py](core/execution/pid.py) | JointPIDController |
| [core/execution/streaming.py](core/execution/streaming.py) | Finite joint command sessions |
| [core/execution/interaction.py](core/execution/interaction.py) | InteractionSession and confirmed exit |
| [core/execution/native.py](core/execution/native.py) | Native motion/calibration action coordination |
| [core/hardware/session.py](core/hardware/session.py) | ArmSession shared feedback, readiness, polling, and command access |
| [core/hardware/feedback.py](core/hardware/feedback.py) | JointFeedback and EndpointFeedback |
| [core/hardware/kinematics.py](core/hardware/kinematics.py) | InteraKinematics IK/FK service clients |
| [core/hardware/gripper.py](core/hardware/gripper.py) | SawyerGripper protocol, acknowledgement, completion |
| [core/hardware/peripherals.py](core/hardware/peripherals.py) | SawyerDevices robot/head/camera/display operations |
| [core/hardware/io.py](core/hardware/io.py) | IO signals, permissions, events, acknowledged writes |
| [core/hardware/model.py](core/hardware/model.py) | URDF joint/effort limits |
| [core/hardware/transforms.py](core/hardware/transforms.py) | TransformCache |
| [core/communication/transport.py](core/communication/transport.py) | DirectTransport XML-RPC discovery and TCPROS exchange |
| [core/communication/wire.py](core/communication/wire.py) | Bundled native message schema and binary codecs |
| [core/communication/actions.py](core/communication/actions.py) | NativeActionClient identified goals and cancellation |
| [core/lease.py](core/lease.py) | ROS 2 streaming session identity, sequence, source-age checks |
| [ros2.py](ros2.py) | Standard ROS 2 services, trajectory action, and status |
| [ros2_interfaces.py](ros2_interfaces.py) | Additional typed ROS 2 endpoints |
| [ros2_telemetry.py](ros2_telemetry.py) | Native/ROS conversion and telemetry outside control callbacks |
| [ros2_images.py](ros2_images.py) | Efficient image CDR conversion |
| [ros_sugar_component.py](ros_sugar_component.py) | Optional lifecycle application adapter |

## Public API and internal imports

Import SawyerConfig, SawyerBase, and SawyerController from `irobot`.
The controller inherits the base connection/state API and coordinates the
planner and executor. Construction is offline; connect explicitly or use a
context manager. Preview/execution defaults and endpoint names are unchanged.

Import internal helpers directly from their layered modules. For example,
use `core.communication.transport.DirectTransport` and
`core.planning.cartesian.CartesianPlanner`. The top-level `irobot` driver
exports and existing controller methods remain available.

The planner receives poses, joint configurations, limits, and IK/FK callables.
The executor uses the owned session for commands and measured feedback.
The session keeps readiness and low-level stop operations; the timed tracking
loop lives in execution. ROS imports remain in optional adapters.

## Further references

- [API reference](../../../../API_REFERENCE.md): signatures, defaults, results, ROS 2 endpoints.
- [Topic map](TOPIC_MAP.md): native communication directions and handlers.
- [Driver README](README.md): installation and examples.
- [Validation guidance](VALIDATION.md) and [unfinished work](UNFINISHED_WORK.md): runtime and hardware scope.

The bundled wire schema and licenses remain under installation/. Custom ROS 2
interface definitions remain in the repository's ros2/irobot_sawyer_interfaces/.
