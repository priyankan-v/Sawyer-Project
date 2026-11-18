# Sawyer remaining work

Recorded hardware data and fault reports are not retained. This file lists
pending qualification requirements without historical pass/fail measurements.
Removing records does not qualify hardware or change driver safeguards.

## Active scope

Python and ROS 2 Humble on the configured workstation, an electric gripper
and cameras. Head-pan and disable/recovery tests are excluded by request.

## Qualification requirements

- [ ] Establish a safe operating condition and fresh controller feedback
  before actuation.
- [ ] Qualify the required neutral/joint/Cartesian/trajectory operations,
  including application-selected speed and acceleration bounds.
- [ ] Qualify Python and Humble cancellation, stop and active-stream-loss
  behavior under the required operating conditions.
- [ ] Qualify post-startup gripper readiness, explicit calibration, positioning,
  cancellation and required settings with the intended tool/load.
- [ ] Qualify required camera settings/streams, input events, model services
  and attached IO through the selected application interfaces.
- [ ] Qualify the packaged build under representative application workloads.

## Conditional scope

- Experimental torque, broader trajectory and force/compliant operation.
- Nonzero payloads and arm calibration with documented tool prerequisites.
- Additional vendor interfaces with documented schemas.
- ClickSmart or other tools, additional writable ports and camera geometry.
- Additional CPU/OS/Python environments and longer control-load workloads.

The [implementation guide](IMPLEMENTATION_GUIDE.md) describes existing source
features. The [plan](DIRECT_DRIVER_PLAN.md) separates implementation from
qualification, and [validation guidance](VALIDATION.md) describes verification
without retaining prior hardware outcomes.
