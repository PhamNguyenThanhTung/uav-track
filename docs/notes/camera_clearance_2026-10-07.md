# Camera obstruction and Gazebo timing check — 2026-10-07

Camera clearance: **PASS at the five sampled target-facing configurations**. GUI stutter: **observed, not fixed**. Phase 4A: **not fully accepted** because of incomplete telemetry matching and intermittent PX4 failsafe/RTL transitions.

## Cause and change

The user's 14.26-second Gazebo recording and the earlier E2E snapshots show the X500 landing gear occupying a large part of the camera image. The centered sensor origin was approximately 0.165 m below `base_link`; the landing skid sits approximately 0.2195 m below it. At the default yaw of -pi/2, positive pitch looks toward +Y and brings the +Y skid into view.

`simulation/models/custom_x500_gimbal_down/model.sdf` moves the gimbal mount from `(0, 0, -0.08)` to `(0, 0.22, -0.08)` m and adds a support visual. The assembly remains above the landing skid's ground-contact plane. Its mass, camera optical rotation, intrinsics, clipping, sensor rate, joint limits, and controller interfaces remain as before. `gz sdf --inertial-stats` reports the same total mass of 2.23431 kg and a lateral center-of-mass shift of 0.016739 m; the subsequent flight check includes this changed mass distribution. This is a mounting correction; no visual masks or larger near-clipping distance hide the obstruction.

Rear-facing yaw and nonzero roll are not covered by the clearance check. The existing upside-down target orientation in these camera views is also unchanged.

## Runtime evidence

`gz sdf -k` passed with `SDF_PATH` containing the project and PX4 model directories. The retained `gz_frame_id` extensions produce warnings, but the SDF is valid. `git diff --check` passed.

One master-launch attempt exited during Gazebo startup. To expose console errors, Gazebo was then started using `./simulation/run_sim_phase3.sh`, which stayed running. The same official stack components were attached: `MicroXRCEAgent udp4 -p 8888`, the bridge using `config/sim/ros_gz_bridge.yaml`, PX4 standalone with autostart 4001 and model `custom_x500_gimbal_down`, and `ros2 run uav_control altitude_control --ros-args -p target_altitude_m:=2.0`. The initial launcher failure was not reproduced or repaired in this check.

Actual PX4 telemetry confirmed a 2 m hover. Five 7-second samples commanded these yaw/pitch pairs in radians:

| Sample | Yaw | Pitch | Result |
|---|---:|---:|---|
| Nadir | -1.5708 | 0 | No landing gear in view |
| Initial | -1.5708 | 0.35 | No landing gear in view |
| Video center | -1.50 | 0.73 | No landing gear in view |
| Video left | -2.14 | 1.04 | No landing gear in view |
| Video right | -0.98 | 0.97 | No landing gear in view |

Measured joint positions and original RGB images are saved in [camera clearance artifacts](../../evaluation/results/runs/camera_clearance_20261007/). During these samples the altitude ranged from 1.985 to 2.016 m. No new unit test was added for this geometry change; the runtime images directly check the failure.

The unmodified YOLO node was then started with the project's YOLO virtualenv, and the evaluator ran:

```bash
python3 -u evaluation/benchmark_evaluator.py --verify-sec 25 \
  --output-dir evaluation/results/runs/camera_clearance_20261007/with_perception
```

In 25.04 simulation seconds, the evaluator received 205 camera frames. Raw detector telemetry was available for 143 frames, including 135 with a person. Tracking output was available for 104 frames, including 97 with a target: 93.27% retention **among frames with tracking output**, not among all source frames. There were 101 incomplete matches, 0 observed ID switches, 4 sampled track losses and 4 reacquisitions. Median processing duration was 57.25 ms and p95 was 90.39 ms. Mean altitude was 1.968 m, with a brief dip to 1.488 m and maximum 2.028 m. PX4 continued to log intermittent failsafe/RTL transitions; this check does not close the flight-timing issue.

After collecting evidence, the altitude target was set to 0 m. A fresh PX4 sample measured 0.0346 m before all processes belonging to this check were stopped. PX4 and perception logs are preserved in the artifact directory.

## Why the image panel can look smoother than the 3D view

The image panel in the recording consumes `/custom_x500/camera/image_raw` through Gazebo transport. The 3D scene is drawn by the separate Gazebo GUI process. In a sampled scene region, the recording had approximately 0.78 seconds with little visual change while the camera region continued changing. This demonstrates different update behavior; it does not identify a driver defect.

| Configuration | Gazebo sensor receive FPS, wall time | ROS receive FPS, wall time |
|---|---:|---:|
| GUI + PX4 + bridge, five angle samples | 29.37–30.05 | 14.12–15.49 |
| Same stack + YOLO + evaluator, 20-second observation | 27.51 | 6.69 |

The observer used best-effort sensor QoS; these ROS counts are subscriber receive rates and cannot alone identify whether loss occurred in the bridge, DDS, or subscriber. Under full load, the Gazebo camera's p95 wall gap was 48.8 ms and maximum gap was 88.5 ms; ROS p95 was 373 ms and maximum was 603 ms. Source timestamps remained about 30.3 frames per simulation second. D3D12/NVIDIA rendering was confirmed in the OGRE log. No GUI rendering or OS configuration was changed.

## System clock anomaly

A separate 12-second probe, independent of ROS callbacks and publications, found genuine `CLOCK_REALTIME` steps: about +6.55 s followed by -7.04 s, repeated twice. `CLOCK_MONOTONIC` remained continuous. Direct kernel `clock_gettime` and Python's realtime clock agreed within 1.31 ms, ruling out a Python-only or vDSO-only measurement artifact. The sample capture uncertainty was at most 1.33 ms.

`timedatectl timesync-status` also reported an NTP offset of approximately -7.052 s. This supports investigating competing host/guest time corrections, but does not establish which component is changing the clock. The clock fault can invalidate realtime-based measurements and timestamp freshness; its causal contribution to GUI stutter and Offboard loss has not been isolated. [Clock trace](../../evaluation/results/runs/camera_clearance_20261007/clock_probe.json) and CSV are preserved. No Windows/WSL clock or synchronization service was changed.
