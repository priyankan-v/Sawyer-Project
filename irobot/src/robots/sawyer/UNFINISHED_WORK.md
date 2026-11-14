# Sawyer unfinished work

Snapshot: November 13, 2026, branch `sawyer_py`.

This records every unchecked requirement in [DIRECT_DRIVER_PLAN.md](DIRECT_DRIVER_PLAN.md)
and one maintenance issue found while documenting the implementation. It is a
remaining-work index, not a new completion claim. The original checklist remains
the itemized plan; [CAPABILITIES.md](CAPABILITIES.md) and
[VALIDATION.md](VALIDATION.md) record coverage and evidence.

The last recorded physical state was **disabled, software-stopped and in error**.
Disable cleared `homed`; the subsequent enable exceeded a 0.005 rad guard and
latched software stop, even after an operator restart had restored readiness.
The cause and a safe recovery procedure are unresolved. This is a historical
observation, not a live status check. No physical commands were run for this
documentation update. **Head pan remains deferred at the user's request.**

## Release blocker

- [ ] Diagnose unexpected disable/enable movement and establish a documented
  ready-state recovery procedure. Preserve the failed runs and obtain the
  controller/vendor operating requirements needed to explain homing behavior.
  Completion requires measured, repeatable recovery with understood transitions,
  fresh state confirmation and an agreed movement envelope. A restart restoring
  readiness once is insufficient. Do not automatically reset or retry enable.

## Remaining planned requirements

Except for explicitly conditional/deferred scope, these items remain open.
Hardware tests involving actuation depend on resolving the recovery blocker and
establishing the appropriate controlled test conditions.

| Remaining requirement | What is already available | What must finish / evidence needed |
| --- | --- | --- |
| Physical cable loss and master/controller restart | Local registration recovery/no replay; physical velocity-process and stream-client loss tests | Controlled fault injection and reconnection observations for each distinct fault. Record command cessation, state freshness, terminal results and absence of automatic command replay/resume. The operator's earlier restart was not this qualification. |
| Longer/larger torque control | Experimental opt-in; bounded 50 ms, 0.02 Nm pulse passed; earlier 150 ms exit failed | Explain the earlier exit failure and qualify the required duration/load envelope, including entry, cancellation and exit. Keep the experimental gate until justified. |
| Native cold-start GENERATE | Native START/cancellation passed; GENERATE succeeded after explicit START | Establish documented controller prerequisites and repeat cold-state planning without implicit preparatory movement. Initial CONTROLLER_NOT_FOLLOWING remains unresolved. |
| Broader trajectory and force/compliant interaction | A 0.8 s, 0.001 rad trajectory and 100 ms compliant/0.05 N sessions passed bounded tests | Qualify required paths, duration, forces and normal exit. Establish controller-supported process-loss deactivation before a force/compliant process-kill test; brief normal exits do not establish crash safety. |
| Payload settings and arm calibration | Settings/typed calibration APIs exist; arm calibration rejects incompatible attached-tool configuration | Qualify payload settings. For arm calibration, first satisfy documented gripper removal/configuration prerequisites, then validate explicit calibration and resulting state. No arm calibration was performed with the attached gripper. |
| Final packaged-build physical validation | Wheel installation, clean imports and software regressions passed | After fault resolution, install the chosen final artifact and record its identity plus physical results. Later source hashes do not prove the tested process loaded those exact final sources. |
| Physical cuff-button and navigator-wheel edges | IO/legacy input subscriptions, merged caches, bounded event recording and software regressions | Record actual press/hold/release and wheel transitions with an available operator and confirmed active recorder. Earlier confirmations produced no recorded edges; the operator could not repeat the later attempt. |
| Remaining writable ports/digital outputs and camera settings | Metadata validation, acknowledgements, six lights and selected camera changes passed | Select required settings/outputs, record originals, verify intended effects and restoration. Camera-power permission was read; power cycling was not tested. Published writability is not physical qualification. |
| Additional vendor interfaces, if required | Advertised graph captured; supported public-schema closure implemented | Obtain documented schemas and operating requirements for application-required interfaces outside the public contract. Implement and validate only the required supported scope; names alone do not define an API. |
| ClickSmart, if attached and required | Discovery only; the tool is absent | Obtain the tool and supported activation/persistent-configuration requirements, then implement and qualify that scope. These commands are not currently implemented. This is conditional scope, not a failed electric-gripper test. |
| Head pan | Existing bounded implementation and historical arrival failure retained | **Deferred.** Do not resume implementation or physical testing without a change to the user's scope. It is not marked complete. |
| ROS 2 hardware qualification after the fault | Real DDS/type support and synthetic ownership/cancellation tests passed | Qualify the later endpoint changes and final stop refinements on hardware after recovery is understood. Explicitly check stop with a busy job/stale feedback and fresh stopped-state confirmation; current evidence for these refinements is software-only. |
| Other platforms and longer load runs | Ubuntu 22.04.5 x86_64/Python 3.12.16 baseline; one 60-second dual-camera/control observation | Test any additional required OS/CPU/Python combinations in isolated environments. Run representative longer-duration load tests and record timing, drops and memory trends. No hard real-time or indefinite memory-stability claim is established. |

## Maintenance issue found during the code inventory

- [ ] Update, replace or remove
  [tests/test_irobot/smoke_sawyer_runtime.py](../../../../tests/test_irobot/smoke_sawyer_runtime.py).
  Its `main()` publishes synthetic ROS 2 `intera_core_msgs` feedback and assumes
  the earlier DDS-based driver connection. The current backend requires direct
  XML-RPC/TCPROS peers. It is outside normal `test_*.py` discovery and is not
  evidence that the direct runtime works. Existing direct transport and real
  ROS 2 tests cover their stated scopes; this old utility needs an explicit
  maintenance decision and a working documented invocation if retained.

## Closing an item

Record the tested artifact/environment, measured result and limitations in a
new validation report. Update the original checklist, capability summary and
ledger mapping together when scope or evidence changes. Full access/control is
not signed off merely because all public methods have a coverage entry. Preserve
failed, deferred, unavailable and conditional cases explicitly.
