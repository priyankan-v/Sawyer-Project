# Sawyer direct-driver plan

Target: Python applications and ROS 2 Humble using the irobot driver layout.
The selected application scope uses an electric gripper and cameras.
Recorded hardware data and fault reports are not retained. The checklist below
separates implemented source features from work requiring qualification.
No hardware success is inferred from the removal of records.

## Architecture and implementation

- [x] Preserve SawyerConfig, SawyerBase and SawyerController responsibilities.
- [x] Connect directly through XML-RPC/TCPROS without a workstation ROS 1
  installation, separate bridge process or robot-side application agent.
- [x] Retain typed wire definitions, source provenance and upstream licenses.
- [x] Share feedback caches, freshness checks and one operation owner.
- [x] Implement bounded joint/neutral/Cartesian planning and model queries.
- [x] Implement explicit gripper, IO, camera and display interfaces.
- [x] Retain explicit enable/reset calls, stop handling and movement guards.
- [x] Provide experimental mode gates, source leases and cancellation.
- [x] Expose the same backend through optional Humble actions and services.
- [x] Provide optional ros_sugar lifecycle integration and installation assets.
- [x] Provide synthetic transport, control, protocol and concurrency tests.
- [x] Document file/function responsibilities and native topic directions.

## Hardware qualification checklist

Hardware results are not retained; these entries are qualification requirements,
not a claim that an operation has never been attempted.

- [ ] Establish the appropriate safe operating condition and fresh feedback.
- [ ] Qualify neutral, joint, Cartesian and trajectory operations through
  Python and Humble at the required velocity/acceleration settings.
- [ ] Qualify cancellation, active-stream loss and required stop behavior.
- [ ] Qualify the electric gripper, including explicit calibration, settings,
  measured arrival and post-startup readiness.
- [ ] Qualify cameras, input events, model queries and required IO outputs.
- [ ] Qualify the selected packaged artifact and application workload.

Head-pan and disable/recovery testing are excluded from the active request.
Experimental torque/force/compliant control, alternate tools, extra vendor
interfaces, nonzero payloads, arm calibration, alternate camera geometry and
additional platforms require separately defined scope and prerequisites.

See [UNFINISHED_WORK.md](UNFINISHED_WORK.md) for pending work and
[VALIDATION.md](VALIDATION.md) for validation guidance. Existing source safety
checks and explicit execution requirements remain in place.
