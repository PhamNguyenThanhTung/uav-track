# System Interface Contract

This document specifies the exact active ROS 2 and Gazebo interfaces, message types, Quality of Service (QoS) profiles, and control contracts between subsystems.

---

## 1. Camera Sensor Interface

| Topic Name | Message Type | Direction | QoS Profile | Frequency |
|---|---|---|---|---|
| `/custom_x500/camera/image_raw` | `sensor_msgs/msg/Image` | GZ -> `ros_gz_bridge` -> ROS 2 | Bridge publisher: RELIABLE, VOLATILE, KEEP_LAST depth 10; perception reader: BEST_EFFORT, VOLATILE, KEEP_LAST depth 1 | 30 Hz configured; measured native source about 29 FPS. The bridge parameter query reports depth 10; graph endpoint discovery itself reports depth UNKNOWN. |
| `/custom_x500/camera/camera_info`| `sensor_msgs/msg/CameraInfo` | GZ -> ROS 2 | Best Effort, Volatile, Depth 1 | Matches image stream |

- **Image Dimensions**: 1280 (width) × 720 (height)
- **Pixel Encoding**: `rgb8`
- **Image Orientation and Servo**: Sensor roll is pi, pitch is pi/2, yaw is zero. With roll joint zero, the image is upright when looking toward the person. Positive pixel errors command negative yaw/pitch increments. Initial yaw/pitch are -1.5708/1.15 rad.
- **Scenario Placement**: UAV spawns at `(0, -2, 0.24)` m; the offboard controller holds its starting XY while flying to 2 m, giving the walking actor more room in the image.
- **Default Optical Axis**: Points straight down (pitch = 0°) along -Z of `camera_link`
- **Mount Position**: `gimbal_mount_link` is at `(0, 0.22, -0.08)` m relative to `base_link`, outside the +Y landing skid. With all gimbal joints at zero, the sensor origin is `(0, 0.22, -0.165)` m relative to `base_link`.
- **Self-occlusion Verification**: The target-facing angles sampled on 2026-10-07 (yaw -2.14 to -0.98 rad, pitch 0 to 1.04 rad) showed no landing gear in the image at a 2 m hover. This verifies those sampled configurations; rear-facing yaw and nonzero roll have not been verified for clearance. Camera resolution and clipping are unchanged. The subsequent upright-image fix rotates sensor roll by pi and increases the simulated pitch upper limit to 70 degrees; see [upright framing verification](notes/upright_fullbody_2026-10-07.md). See [runtime evidence](notes/camera_clearance_2026-10-07.md).

---

## 2. Gimbal Actuation Interfaces

| Topic Name | Message Type | Direction | Bridged in config? | Actively Used in Control? | Range / Limits |
|---|---|---|---|---|---|
| `/custom_x500/gimbal/yaw_cmd` | `std_msgs/msg/Float64` | ROS 2 -> GZ | **YES** | **YES** (by `yolo_detector`) | $[-3.14159, +3.14159]\text{ rad}$ ($\pm 180^\circ$) |
| `/custom_x500/gimbal/pitch_cmd`| `std_msgs/msg/Float64` | ROS 2 -> GZ | **YES** | **YES** (by `yolo_detector`) | $[0.0, +1.2217]\text{ rad}$ ($0^\circ \text{ to } +70^\circ$) |
| `/custom_x500/gimbal/roll_cmd` | `std_msgs/msg/Float64` | ROS 2 -> GZ | **YES** | **NO** (idle / zero) | $[-0.7854, +0.7854]\text{ rad}$ ($\pm 45^\circ$) |

> **IMPORTANT**: Although `roll_cmd` is bridged in `ros_gz_bridge.yaml`, the current visual servoing algorithm in `yolo_detector.py` **only commands yaw and pitch**. Roll is left unactuated (held at 0.0 rad).

---

## 3. Perception Output Interfaces

| Topic Name | Message Type | Publisher Node | QoS Profile | Description |
|---|---|---|---|---|
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | `uav_perception/yolo_detector` | Reliable, Volatile, Depth 10 | 2D person boxes and confidence; active track ID when tracking is enabled |
| `/yolo/raw_detections` | `vision_msgs/msg/Detection2DArray` | `uav_perception/yolo_detector` | Reliable, Volatile, Depth 10 | Person boxes emitted directly after YOLO inference and before BoT-SORT or target selection; header stamp is copied from the source camera image; empty array means detector ran and found no person |
| `/yolo/telemetry` | `std_msgs/msg/Float64MultiArray` | `uav_perception/yolo_detector` | Reliable, Volatile, Depth 10 | Per-frame `[source_sec, source_nanosec, yolo_ms, tracker_ms, processing_ms]`; source stamp is encoded in the first two values because this message has no header |
| `/yolo/annotated_image` | `sensor_msgs/msg/Image` | `uav_perception/yolo_detector` | Reliable, Volatile, Depth 10 | Annotated camera image with HUD reticle, error vector, and tracking telemetry, retaining input encoding |

- `/yolo/telemetry` durations use a monotonic clock. `processing_ms` spans `process_frame` from image conversion through detection/tracking, target selection, publication, and optional annotation rendering, ending immediately before telemetry publication.
- The evaluator correlates camera images, raw detections, tracker output, and processing telemetry using exact source seconds/nanoseconds. An absent component remains unknown.

- `/yolo/reset_target` (`std_srvs/srv/Trigger`) explicitly resets target identity. The next fresh person observation can be acquired after reset.
- `/yolo/performance` (`std_msgs/msg/String`, reliable depth 10) is optional and disabled by default. `enable_performance_debug=true` adds source/receipt, YOLO, tracker/ReID/GMC, guard, publish and callback-completion timestamps and durations. It does not change the existing five-value `/yolo/telemetry` message.
- Ordinary `LOST` preserves bounded identity references and can recover the same logical target after three fresh stable identity confirmations, including a new tracker ID. Appearance ambiguity produces no bbox. Clock rewind/invalid source time invalidates the session and requires reset.
- `enable_lazy_diagnostics=true` skips annotation drawing/encoding/publication while the annotated-image publisher has no subscribers. Its default is true; false restores always-render behavior. Connected subscribers receive the original source header/encoding and HUD. This does not change target output, guard JPEG policy, source-time checks or the serialized tracker path. See the [concurrency/ReID experiments](performance/perception_concurrency_reid_experiments.md).
- With `enable_guard_debug=true`, `/yolo/guard_debug` (`std_msgs/msg/String`, reliable depth 10) provides optional JSON candidate inputs, gate/transition diagnostics, and JPEG data for local evaluation/replay. It is absent by default; evaluation labels are never supplied to the manager.
- With `enable_gimbal_tracking=false`, target detection and identity validation continue, but no gimbal commands are issued. Annotated images retain the input image encoding.
- The production tracker is configured in `src/uav_perception/config/botsort.yaml`; startup fails if ORB GMC, the pretrained ReID encoder, or CUDAExecutionProvider is unavailable. Image timestamps are checked against ROS time, and the camera watchdog uses monotonic receipt time plus source-clock age.
- Source-clock age is rechecked after detector/tracker processing and before target learning/output, using the existing `max_image_age_sec` budget (default 0.75 s). An expired result emits an empty target and no gimbal update; raw detector output and processing telemetry retain the original image stamp. Its GMC projects existing history without learning a target or replacing the last real manager image timestamp. Optional guard-debug records it as a watchdog event carrying that GMC.
- The node uses two executor threads. Image processing, explicit target reset and the statistics/watchdog timer share one mutually exclusive callback group; `/clock` stays in the default group so it can update during inference. Source time remains authoritative when simulation is paused. JPEG guard diagnostics are emitted after target/controller output.
- The package installs `config/fastdds_images.xml`, increasing Fast DDS shared-memory message capacity to 4 MiB for the camera sample (about 2.76 MB) and the shared segment to 64 MiB. The simulation launchers select it by default before bridge/node startup and respect explicit `FASTDDS_DEFAULT_PROFILES_FILE` or `FASTRTPS_DEFAULT_PROFILES_FILE` overrides. The measured image transport and perception timing are documented in the [pipeline audit](performance/perception_video_pipeline_audit.md).

---

## 4. PX4 Autopilot Offboard Interfaces

Published and subscribed by `uav_control/altitude_control` via `MicroXRCEAgent` (DDS port 8888):

| Topic Name | Message Type | Direction | QoS Profile | Description |
|---|---|---|---|---|
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Node -> PX4 | Best Effort, Volatile, Depth 10 | Streams position mode enabled (`position = true`) at 10 Hz |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | Node -> PX4 | Best Effort, Volatile, Depth 10 | Desired NED position: $(X_{hold}, Y_{hold}, -Z_{target})$ and $Yaw_{hold}$ |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | Node -> PX4 | Best Effort, Volatile, Depth 10 | Sends Arming (`ARM_DISARM`) and Offboard mode (`DO_SET_MODE: 1, 6`) commands |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` | PX4 -> Node | Best Effort, Transient Local, Depth 10 | Vehicle NED coordinates, velocity, and heading. Node falls back to `/fmu/out/vehicle_local_position` if unversioned. |

---

## 5. Current Control Contract

1. **Perception controls the gimbal only**:
   - `uav_perception/yolo_detector` uses BoT-SORT observations with `TargetIdentityManager` (`target_identity.py`) to maintain a logical `target_id` across tracker-ID changes and coasting/loss/recovery transitions. Only fresh detector measurements refresh the target; ambiguous nearby candidates provide no gimbal bbox. Recovery preserves the enrolled target; `/yolo/reset_target` explicitly allows new-target selection.
   - It delegates visual servoing calculations to `GimbalController` (`gimbal_controller.py`), which applies EMA smoothing, deadband filtering, rate limits, and joint limits.
   - It directly updates and publishes `/custom_x500/gimbal/yaw_cmd` and `/custom_x500/gimbal/pitch_cmd`.
   - Perception does **not** command UAV body position, velocity, or attitude.

2. **Flight control maintains static position hold + target altitude**:
   - `uav_control/altitude_control` locks the horizontal coordinates $(X_{hold}, Y_{hold}, Yaw_{hold})$ observed at node startup.
   - It regulates the vertical axis to $-target\_altitude\_m$ in NED frame.
   - It is completely unaware of targets, detections, and gimbal orientation.

3. **Absence of MotionArbiter**:
   - There is **no arbitration layer** linking target position to vehicle movement.
   - Any future work introducing target-following flight must create this layer explicitly without altering `altitude_control` without an approved plan.


## 6. WSL timing prerequisite

On WSL 24.04 with Hyper-V implicit time synchronization enabled, use the Windows host as the clock authority and disable the guest `systemd-timesyncd` service. Running both caused repeated +6.5/-7 second clock steps on the verified host. Gazebo 8.15.0 scene-state publication uses system time and stalled after backward steps; PX4 also exhibited Offboard failsafe events.

The host service was stopped and disabled on 2026-10-07, following [Ubuntu WSL guidance](https://ubuntu.com/wsl/docs/stable/explanation/time-sync/). Normal full-stack runtime then measured scene-state delivery ~50.85 Hz, camera ~29.33 FPS, no clock jumps over 100 ms in 60 seconds, and no Offboard loss or failsafe in the captured ~193-second armed ULog. This establishes the sampled runtime, not Windows compositor FPS or all Phase 4A acceptance criteria. The ROS vehicle-status/failsafe schemas remain mismatched; use native ULog evidence until that interface is repaired. See [clock fix verification](notes/wsl_clock_fix_2026-10-07.md).
