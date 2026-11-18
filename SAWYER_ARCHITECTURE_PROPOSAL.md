# Sawyer layered architecture proposal

Status: Sawyer layer extraction and template updates implemented on `sawyer_api`.
This branch contains Sawyer only; the later planning API remains a proposal.

## Purpose

Make the code easy for a future developer to find, understand, and extend.
Keep the irobot template, existing features, and public APIs. Organize the
existing implementation by responsibility, using ordinary Python modules.

## Suggested structure

Keep `SawyerConfig`, `SawyerBase`, and `SawyerController` in their existing
locations. Move the larger implementations into the folders below. Each
folder represents a responsibility; it does not introduce another process.

```text
sawyer/
├── __init__.py                  Public exports
├── config.py                    Connection settings and operation options
│
├── core/
│   ├── __init__.py
│   ├── base.py                  Connection and public state/device access
│   ├── controller.py            Public commands; call planner and executor
│   ├── types.py                 Shared poses, plans, and results
│   │
│   ├── planning/
│   │   ├── __init__.py
│   │   ├── cartesian.py         Straight lines, orientation, IK/FK planning
│   │   ├── joint.py             Joint paths and interpolation
│   │   ├── timing.py            Trajectory speed and duration
│   │   └── native.py            Build native Intera trajectories
│   │
│   ├── execution/
│   │   ├── __init__.py
│   │   ├── trajectory.py        Follow a plan using measured feedback
│   │   ├── pid.py               Joint PID calculations
│   │   ├── streaming.py         Position, velocity, torque, trajectory streams
│   │   ├── interaction.py       Interaction sessions
│   │   └── native.py            Execute native Intera actions
│   │
│   ├── hardware/
│   │   ├── __init__.py
│   │   ├── session.py           Shared connection, polling, command access
│   │   ├── feedback.py          Joint/endpoint state and freshness
│   │   ├── kinematics.py        IK/FK service access
│   │   ├── gripper.py           Gripper commands and completion
│   │   ├── peripherals.py       Enable/reset/stop, head, cameras, display
│   │   ├── io.py                IO values, metadata, events, and writes
│   │   ├── model.py             Robot model and joint limits
│   │   └── transforms.py        Transform state
│   │
│   └── communication/
│       ├── __init__.py
│       ├── transport.py         Network discovery and connections
│       ├── wire.py              Native message encoding/decoding
│       └── actions.py           Native goal, feedback, result, cancellation
│
├── ros2.py                      Existing ROS 2 entry point
├── ros2_interfaces.py           Typed services/actions
├── ros2_telemetry.py            State conversion and publication
├── ros2_images.py               Image conversion
├── ros_sugar_component.py       Lifecycle adapter
├── examples/                    Small examples using public APIs
└── installation/                Existing setup assets and wire schema
```

The tree shows the main destinations. Keep other existing helpers, such as
stream lease handling, until moving them makes their purpose clearer. A file
can stay together when splitting it would make the code harder to follow.

## How the parts work together

| Part | Responsibility |
| --- | --- |
| Application / ROS adapters | Receive requests and call the public controller |
| `SawyerController` | Coordinate an operation: obtain inputs, plan, optionally execute, return results |
| `SawyerBase` | Own one session and expose connection, state, model, and device access |
| Planner | Calculate paths and timing using supplied poses, joints, limits, and IK/FK functions |
| Executor | Follow the plan, monitor feedback, handle cancellation, and stop cleanly |
| Hardware helpers | Provide robot-specific state, kinematics, and device operations |
| Communication | Exchange commands and measurements with Intera |

Keep `SawyerController(SawyerBase)`. The controller calls planner and executor
objects. Planning code should not reach into controller internals or publish
commands. The executor uses the shared session. ROS adapters call the same
controller methods as Python applications.

For a Cartesian movement:

```text
Application calls controller
    → controller obtains the start state, limits, and kinematics
    → planner produces a timed straight-line plan
    → controller returns a preview or asks the executor to follow it
    → executor uses hardware access to send commands and read feedback
```

## Keep the implementation simple

- Move existing classes and functions before rewriting their logic.
- Keep one connection/session shared by the driver and its device helpers.
- Use direct method calls. Additional queues, processes, or framework classes
  are unnecessary for this organization.
- Keep shared data in `types.py` and settings/options in `config.py`.
- Pass the inputs a component needs; avoid circular imports and hidden access
  to another component's private state.
- Keep all current motion, gripper, IO, camera, telemetry, and adapter features.
  Existing readiness checks, limits, cancellation, watchdogs, and stop behavior
  move with their code; this proposal adds no extra checking layers.
- Preserve top-level `irobot` driver imports, method behavior, and ROS entry
  points. Import internal helpers directly from their layer; remove the old
  forwarding modules to keep the structure clear.
- When moving `wire.py`, keep its bundled schema accessible from an installed
  package as well as a checkout.

Start by extracting planning and execution from `controller.py` and `base.py`.
Group the remaining helpers afterward. Use the existing relevant checks to
catch regressions; a separate validation framework is unnecessary.

## Improvements to the irobot template

The following five template improvements are agreed. Apply behavior changes
to existing robot drivers separately from the Sawyer file reorganization.

| Current area | Suggested improvement |
| --- | --- |
| Broad base/controller split | Define base as connection/state access and controller as public operation coordination. Allow supporting modules for larger drivers. |
| Optional planning/execution organization | Document `planning/` and `execution/` as available extensions. A small driver can keep a few files; every robot need not copy the full Sawyer tree. |
| Connection lifecycle | Use explicit `connect()` and `disconnect()`, with context-manager support. Constructors prepare the driver. Sawyer already follows this lifecycle. |
| Public imports and examples | Use actual robot-specific exports such as `SawyerController`. Use exported driver classes in the root README and examples. |
| Configuration and adapters | Keep connection settings and operation options clearly named in `config.py`; keep ROS dependencies optional and adapter-specific. |

Add a short responsibility table and one basic usage example to each robot's
README. Let developers find the implementation through clear names and a small
module map rather than a large architecture manual.

## Work plan

Implement the agreed changes in this order. Keep each step focused and retain
existing algorithms, features, and public names while moving code.

1. **Clarify the template responsibilities.** Update the root README's
   new-robot instructions: base owns connection/state access; controller
   coordinates operations. Describe planning/execution folders as optional
   extensions for larger drivers, with a small example structure.

2. **Organize Sawyer into the proposed layers.** Extract shared types, then
   planning and execution from the current large files. Group hardware and
   communication helpers afterward. Keep the public base/controller files
   small, reuse existing classes, and update internal imports to the new paths.

3. **Keep the explicit connection lifecycle.** Retain Sawyer's explicit
   `connect()` / `disconnect()` and context-manager cleanup. Constructors stay
   offline. Other robot drivers and dependencies are excluded from this branch.

4. **Correct public imports and examples.** Replace the root README's generic
   `RobotController`/`RobotConfig` example with actual robot-specific exports.
   Update each robot README with a basic connection/use/cleanup example and
   a short responsibility table. Examples should use public driver methods.

5. **Clarify configuration and optional adapters.** Group connection settings
   and operation options clearly in each robot's `config.py`, retaining
   existing names and defaults. Keep ROS imports in adapters and connect
   adapters to the same controller implementation. Refresh the API reference
   and module links after the moves.

Use the relevant existing checks to catch regressions and confirm examples
and imports still work. No new testing framework or per-layer validation
process is part of this plan. The result should be easier to navigate while
retaining the current Sawyer capabilities.

## Later API addition

After the code is organized, the controller could expose
`plan_cartesian(start_pose, end_pose, *, seed=None, options=None)` and
`execute_plan(plan)`. The controller supplies limits and kinematics and chooses
a starting joint configuration; execution requires the arm to match that start.
The existing `move_to_pose()` remains available. These APIs are a separate
feature proposal and are not required for the structural refactor.

Existing API details: [API reference](API_REFERENCE.md).
