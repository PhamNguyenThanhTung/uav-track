# Fix false-pass runners and isolated demo

## Goal

Make both SITL runners fail when required landing or tracking evidence is missing, keep each demo's process cleanup scoped to processes it started, and prevent concurrent camera audits from writing to the same explicit output directory.

## Scope and ownership

One implementation package owns only these paths and focused regression coverage:

- `simulation/run_tracking_3_people_test.sh`
- `simulation/run_live_demo.py`
- `evaluation/audit_camera_fps_accounting.py`
- `evaluation/test_camera_audit_export.py`
- Any narrowly scoped test file for runner behavior

Preserve all pre-existing worktree changes outside this package. Do not modify `../software`, historical reports, or unrelated launchers.

## Required behavior

1. **Tracking runner:** require the production summary to contain observed frames and a selected target before it can pass. Stop this run's altitude setpoint publisher before commanding PX4 Land, and retain the Land command's exit status. Report success only after telemetry read with volatile QoS has PX4 source timestamps newer than the completed Land command and within the freshness window, and confirms `landed: true`, disarmed state (`arming_state: 1`), and finite local NED `z` within 0.25 m of ground. Read the three PX4 messages concurrently so their freshness window is evaluated against one current time. The altitude-only cleanup fallback may attempt descent but cannot mark landing successful. Preserve any earlier nonzero observer/startup/signal result; turn an otherwise successful capture into nonzero if the landing command or confirmation fails. Always write cleanup status and stop only owned processes.
2. **Live demo:** remove process-name-wide `pkill`; stop only process groups created by this run, including children if a wrapper exits. Isolate its Gazebo partition, ROS domain, and XRCE UDP port. Use PX4 instance 0 because the altitude controller targets MAVLink system 1; permit commander commands only after fresh PX4 telemetry arrives on this run's private ROS domain. Use PX4's synchronized message timestamps for source freshness and landing proof. Use a private temporary copy of PX4's built `etc` excluding `4001_gz_x500.post`, strip inherited `PX4_PARAM_*`, and send a normal GCS heartbeat so stock failsafes remain active. Count nonempty detection arrays. Return nonzero on startup, detector, tracking-evidence, land-command, fresh telemetry, touchdown, or exact-disarm failure. Landing requires post-command, current `landed: true`, `ARMING_STATE_DISARMED`, and valid finite near-ground `z` samples.
3. **Camera audit output:** retain support for an existing empty `--output-dir`, reject historical result files, and use the same exclusive marker to reserve both explicit and default directories so overlapping audit processes cannot share them. Hold the marker during the run and release it on exit; result files block later reuse. Leave default run naming, CSV/JSON schemas, and existing history unchanged.

## Verification

- Focused tests cover explicit/default audit directory collisions, pre-existing CSV/JSON preservation, empty/populated detection arrays, queued pre-command and stale PX4 source timestamps, Land command failure, confirmation timeout, and owned process-group cleanup without touching an unrelated process. The tracking runner's summary gate is checked against a prior valid capture and the current zero-frame result.
- Run `bash -n` and Python syntax checks plus focused pytest.
- Run the tracking runner and GUI demo serially in headless SITL. Capture runner exit codes, PX4 Land result, telemetry-confirmed touchdown/disarm, and cleanup status. Confirm GUI demo uses the stock parameter values and an unrelated sentinel process survives.
- Inspect logs and final Git diff. Do not run `launch_simulation.sh` or edit external PX4 files as part of this package.

## Out of scope

No changes to flight-control behavior, model weights, perception algorithms, historical reports, PX4 source/build, or unrelated cleanup-plan tasks. The GUI check validates the headless execution path; it does not claim a visible desktop window was verified.
