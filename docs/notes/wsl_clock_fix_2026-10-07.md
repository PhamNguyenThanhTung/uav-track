# WSL clock synchronization repair — 2026-10-07

## Applied change

The kernel command line contains `hv_utils.timesync_implicit=1`, while `systemd-timesyncd` was enabled and actively synchronizing against ntp.ubuntu.com. This machine therefore had two time authorities. Ubuntu documents this conflict and recommends disabling the guest NTP service on WSL 24.04 when using host synchronization: [official guidance](https://ubuntu.com/wsl/docs/stable/explanation/time-sync/).

Stopped `systemd-timesyncd.service`, checked 35 seconds of realtime versus monotonic clock, then disabled its startup unit after the probe passed. WSL root execution was available through `wsl.exe --distribution Ubuntu-24.04 --user root --exec systemctl ...`; sudo without a password was unavailable. The unit is now disabled/inactive. Hyper-V implicit synchronization remains enabled, so Linux follows Windows time. No Windows time service, timezone, kernel clocksource, `.wslconfig`, Gazebo binary, flight safety setting, external source tree, or production launcher was changed. No LD_PRELOAD clock interposer was used for acceptance.

Before state: enabled/active, preserved in `timesyncd_before.txt`. Rollback, as root: `systemctl enable --now systemd-timesyncd.service`. This restores the earlier guest NTP behavior and may restore the conflict on this host. No package or vendor drop-in was removed.

## Verification

PASS: 35-second probe after stopping guest NTP: 3446 samples, zero jumps over 100 ms; maximum realtime-minus-monotonic interval difference 21.95 microseconds.

PASS: `systemctl is-enabled systemd-timesyncd` returns disabled; `systemctl is-active systemd-timesyncd` returns inactive. Those query commands intentionally return nonzero for disabled/inactive; the disable operation itself succeeded.

PASS: Normal `./launch_simulation.sh --offboard 2.0` GUI launch, ROS bridge, XRCE Agent, PX4 SITL, altitude control, and YOLO26 CUDA/DeepSORT. A 60-second full-stack probe measured scene-state 50.85 Hz, camera 29.33 wall FPS, state p95 gap 27.02 ms and maximum 210.82 ms. No realtime jumps over 100 ms. Before repair, the isolated baseline had state 3.49 Hz and repeated gaps of 5.23 seconds while camera remained near 30 FPS. The remaining occasional 0.21-second gap under full-stack load means this is not a claim of perfectly uniform rendering. Windows compositor FPS was not measured.

PASS: 60-second ROS position/Offboard-input probe: 2929 position samples; mean altitude 2.0001 m, min 1.9559 m, max 2.0390 m; 600 Offboard input messages; largest arrival gap 109.86 ms. Zero clock jumps; max interval discrepancy 36.40 microseconds.

PASS via native ULog: 192.892 seconds armed; 385 armed vehicle-status records, all nav_state 14 (OFFBOARD); zero failsafe records; 358 armed failsafe-flag records with zero offboard_control_signal_lost; no ULog dropouts. Boot EKF preflight warning occurred before takeoff. PX4 also logged occasional duplicate IMU sample timestamp errors; they did not trigger a recorded failsafe in this run and remain unaddressed.

FAIL / existing interface limitation: ROS VehicleStatus and FailsafeFlags subscriptions failed Fast CDR deserialization because their installed message layouts differ from PX4's actual ULog schemas. These topics produced no decoded samples, so their status/flags summary values are null, not zero. Offboard safety conclusions above use native ULog data, not these absent ROS samples. No external px4_msgs workspace was modified.

PASS for sampled perception tracking: `evaluation/benchmark_evaluator.py --verify-sec 25 --output-dir evaluation/results/runs/wsl_clock_fix_20261007` received 234 camera frames, 146 available raw person detections with no detector misses, 125 available tracker outputs with no losses or ID switches. Processing median 50.0 ms, p95 83.2 ms; mean radial error 32.4 px. 109 tracking matches were incomplete, so retention 100% applies only to available outputs. This does not establish complete telemetry coverage or overall Phase 4A acceptance.

PASS: Landed by setting `/altitude_control` target_altitude_m to 0.0; observed altitude ~0.03 m before stopping owned runtime processes. No restart of the WSL distribution was performed, because it would interrupt the user's running workspace. Persistence is established by the disabled systemd unit; startup/resume across an actual host reboot remains NOT RUN. Absolute UTC accuracy relative to an external time authority was not assessed; the repair establishes stable host-following time.

PASS: `git diff --check` for affected documentation.

## Artifacts and scope

Results are in `evaluation/results/runs/wsl_clock_fix_20261007/`: clock_after_stop.json, fullstack_timing.json, timing_summary.json, flight_stability.json, ulog_summary.json, flight.ulg, launch.log, px4.log, yolo.log, evaluator results/snapshot, and diagnostic probe sources. Only current system-contract documentation and this new verification note were changed in the project for this repair; earlier user edits and historical runs were preserved.

The measured clock conflict, multi-second scene freezes, and Offboard-loss failures are resolved for this captured run. ROS status/flags schema repair, complete telemetry matching, occasional IMU duplicate timestamps, and full Phase 4A acceptance remain separate outstanding work.
