# Sawyer validation guidance

Recorded hardware datasets, traces, measured results and fault reports are not
retained. This document describes verification procedures only. No hardware
qualification is claimed from the absence of records.

Run relevant synthetic tests for protocol, planning, control, operation
ownership, cancellation and lifecycle behavior after changing their code.
ROS 2 and ros_sugar checks require their matching optional runtime and generated
interfaces. Use Ruff for linting and formatting.

Before physical operations, establish appropriate on-site operating conditions,
use fresh controller feedback and retain existing limits, timeout and stop
checks. Connection and preview do not automatically enable, reset, calibrate
or move hardware. Hardware and application qualification are separate from
unit-test correctness.

Head-pan and disable/recovery tests remain excluded from the active request.
The remaining qualification requirements are in
[UNFINISHED_WORK.md](UNFINISHED_WORK.md).
