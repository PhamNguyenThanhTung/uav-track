# Gazebo GUI stutter diagnosis — 2026-10-07

## Finding

The measured low apparent scene-update FPS is primarily a Gazebo scene-state publication stall caused by WSL CLOCK_REALTIME jumps. This reproduced with Gazebo alone, without PX4, ROS bridge, or YOLO. The native GUI continued swapping buffers around 51 FPS and camera sensors around 30 FPS while the state stream used by GuiRunner stalled for over five seconds.

Gazebo Sim 8.15.0 SceneBroadcaster stores lastStatePubTime and checks its publication period using std::chrono::system_clock. After the host clock jumps forward and then backward, elapsed wall time becomes negative and routine state publication waits for time to catch up. GUI rendering continues with old scene state. Dynamic pose publication is separate and mostly remained around 50 Hz in the final baseline; an earlier discovery-inclusive run also had a 1.76-second pose gap, which alone did not establish causation.

Source evidence: [SceneBroadcaster 8.15.0](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/scene_broadcaster/SceneBroadcaster.cc#L403), [GuiRunner state subscription](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/gui/GuiRunner.cc). The loaded runtime plugin and gz sim --versions both confirm 8.15.0. Exact-release SceneBroadcaster source is preserved with the measurements.

## Controlled comparison

Two 30-second probes used the same world and RTX 4060/D3D12 renderer. Server and GUI ran separately, with no perception or flight stack. A temporary LD_PRELOAD probe recorded GUI glXSwapBuffers entry/exit using CLOCK_MONOTONIC. This measures GUI buffer presentation calls, not distinct changing 3D scene frames or Windows compositor FPS. The second server run temporarily mapped CLOCK_REALTIME reads to a monotonic progression from an initial realtime epoch, only within that server process. The GUI and monitoring process still experienced the original host clock jumps. No OS clock, timesync service, graphics configuration, external software, or production launch source was changed.

| Measured stream | Original clock | Stabilized clock in server only |
|---|---:|---:|
| GUI buffer swaps | 51.35 FPS | 55.59 FPS |
| Camera arrival rate | 29.88 FPS | 29.83 FPS |
| GUI scene-state arrival rate | 3.49 Hz | 50.17 Hz |
| Largest scene-state gap | 5227.79 ms | 33.34 ms |
| Dynamic pose arrival rate | 50.38 Hz | 50.17 Hz |
| Median GPU utilization | 32% | 31.5% |
| Median GUI CPU, one core = 100% | 107% | 118% |
| Median server CPU, one core = 100% | 125% | 130% |

Arrival rates use (message count - 1) divided by first-to-last arrival duration; warmup/discovery can shorten this interval. Raw timestamps and full observation boundaries are preserved. This is a controlled diagnostic comparison, not validation of a production clock workaround or a complete flight-system performance test.

In the final baseline, four scene-state gaps were 5.21–5.23 seconds. Each began just after a measured backward clock adjustment near -7 seconds and ended around the next forward adjustment. The host continued jumping by about +6.5/-7 seconds in baseline and +6.8/-7 seconds during the stable-server test, yet the stable server's state stream no longer stalled. This isolates the server wall-clock dependency as the cause of the reproduced freeze.

systemd-journald independently reports “Time jumped backwards, rotating.” timedatectl timesync-status reported an NTP offset of -7.005638 seconds. The exact process or host synchronization mechanism generating the forward adjustments has not been identified; competing WSL/Windows and NTP corrections remain a hypothesis.

## Other observations and limitations

The renderer is D3D12 on NVIDIA GeForce RTX 4060, not llvmpipe. Memory before launch had ~8.9 GB available and no swap use. GPU samples and isolated reproduction do not support GPU exhaustion or YOLO contention as the cause of these periodic multi-second freezes. They do not rule out other performance limits under a heavier full-stack load.

Prior Ogre logs contain repeated highlight_material fixed-function shader errors. New runs also contain sampler validation warnings. The stable-clock run recovered normal state updates despite rendering warnings, so those warnings are not established as the primary stall cause.

Windows Computer Use failed before window access with sandboxCwd not a local file URI. A Linux X11 screenshot attempt returned black content under WSLg and was not used as visual evidence. GUI swap instrumentation, transport timings, source inspection, and the controlled server-clock comparison provide the diagnosis.

## Reproduction and evidence

PASS: Reproduce stalls with ./simulation/run_sim_phase3.sh -s and a separate gz sim -g using the existing GPU environment helper.

PASS: Subscribe to camera, observer, dynamic_pose/info, world stats, and /world/gimbal_test/state through /tmp diagnostic probe; measure clock adjustments against CLOCK_MONOTONIC.

PASS: A/B server-only stable-realtime diagnostic removes state stalls while camera rate and GPU utilization remain comparable.

NOT RUN: Production clock repair, full-stack FPS acceptance, Windows compositor FPS measurement. Temporary diagnostic processes were stopped after measurement; no clock interposer was added to normal launch scripts.

Evidence: evaluation/results/runs/gazebo_fps_diagnosis_20261007/comparison.json, gui_state_clock.json, stable_server_clock.json, GUI swap CSV files, captured logs, and diagnostic source files. The binaries remain temporary under /tmp and are not a suggested flight configuration. The instrumented server returned exit code 139 during termination after the completed measurements; its shutdown failure was not investigated and is another reason this experiment is not a production workaround.

Next: identify and repair the host/WSL time synchronization issue, then verify ordinary Gazebo scene-state timing and PX4 offboard stability with the normal clock. The process-local interposer was used only to establish causation.
