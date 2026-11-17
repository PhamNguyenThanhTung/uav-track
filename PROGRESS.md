# Project Progress

## Active Architecture

The system operates an end-to-end simulation, flight control, perception, and tracking pipeline:

- **Simulation & Autopilot**: Gazebo Harmonic with custom X500 model (`custom_x500_gimbal_down`), downward 3-DOF gimbal, 1280×720 RGB camera, and PX4 SITL v1.17 connected via Micro XRCE-DDS Agent.
- **Flight Control**: C++ ROS 2 package `uav_control` (`AltitudeControlNode`), decoupled from perception. Regulates target altitude in NED coordinates ($z = -\text{target\_altitude}$) and locks horizontal position ($X, Y$). Dynamic parameter updates validate finite float bounds and preserve state atomically on rejection.
- **Perception & Tracking**: Python ROS 2 package `uav_perception` (`yolo_detector`). Integrates YOLO26n person detection on CUDA, Ultralytics BoT-SORT multi-object tracking with ORB global motion compensation (GMC), pretrained ONNX Re-ID (`yolo26n-reid.onnx` on `CUDAExecutionProvider`), `TargetIdentityManager`, and proportional closed-loop gimbal visual servoing (`GimbalController`).
- **Performance & Freshness**: Source-age freshness gating rejects stale frames expiring during inference while maintaining GMC motion history. CUDA Graph forward replay optimizes YOLO inference latency. Lazy rendering skips HUD annotation when no viewer is subscribed to `/yolo/annotated_image`.
- **Evaluation**: Frame-accurate evaluation framework (`benchmark_evaluator.py`, `audit_camera_fps_accounting.py`) saving to exclusive, non-overwriting run directories under `evaluation/results/runs/`.

---

## Dated Verified Milestones

### 2026-11-16: SITL Demo, Camera Benchmark & Cleanup
- Verified the flight-parameter guard, camera-audit output protection, and perception cleanup; the full CUDA/ReID test suite reported **189 passed**.
- A live SITL GUI run received 1,346 raw camera frames and 63 nonempty detection arrays, then confirmed touchdown and disarm. The viewer process exited cleanly; its rendered window was not inspected automatically.
- Three short on/off pairs found no material change in simulator RTF or source-camera FPS from the secondary camera. Keep its configuration unchanged pending a user-visible comparison.
- After a cold power cycle the RTX 4060 was again available to CUDA, and the live demo completed. One intermittent `nvidia-smi` no-device result remains unexplained.
- See the [secondary-camera benchmark](docs/simulation/secondary_camera_benchmark_report.md) and [runner/runtime verification](plans/evidence/runner-fixes/verification.md).

### 2026-10-16: Perception Concurrency & Lazy Diagnostics Optimization
- Evaluated latest-frame perception workers, clock rates, and diagnostic concurrency. Selected default `enable_lazy_diagnostics=true`: skips HUD rendering and encoding when `/yolo/annotated_image` has no active subscribers.
- Headless perception throughput improved from **12.36 FPS** to **17.83 FPS**; viewer-connected rate maintains **10.94 FPS**. Full simulation runs reached **10.63 AI FPS** with clean landing.
- Full test suite achieved **187 passed, 32 subtests passed** with actual CUDA and ReID enabled (`RUN_BOTSORT_REID_RUNTIME=1`).
- See [perception concurrency and ReID experiments report](docs/performance/perception_concurrency_reid_experiments.md).

### 2026-10-16: External Environment & Dependency Integrity Audit
- Validated external package integrity: GPU ONNX Runtime 1.30.0 active, 28,712 installed package file hashes verified against distribution records.
- Added project-level auto-install guard (`YOLO_AUTOINSTALL=false`) to prevent Ultralytics from replacing GPU ONNX Runtime with CPU wheels.
- Full test suite verified at **183 tests PASS** with CUDA/ReID enabled.
- See [external environment audit](docs/performance/external_environment_audit.md).

### 2026-10-16: Perception Video Pipeline & Latency Optimization
- Tuned Fast DDS transport profile (`config/fastdds_images.xml`) for 1280×720 RGB payload; enabled CUDA Graph forward pass replay.
- Processing rate on 30 FPS camera increased from **8.49 FPS** to **11.34 FPS**; decision-age p95 reduced from **186 ms** to **112 ms**; CUDA Graph forward pass p50 reduced from **36.7 ms** to **15.0 ms**.
- Completed full 236-second simulation run (1,510 profiled frames) with clean landing and shutdown.
- See [perception and video pipeline audit](docs/performance/perception_video_pipeline_audit.md).

### 2026-10-15: Source-Age Freshness Checks & Clock Handling
- Implemented strict source-time freshness checks (0.75 s budget) at image ingress and after detector/tracker execution.
- Configured multithreaded ROS executor separating `/clock` from image callbacks, preventing simulated clock starvation during inference.
- Verified test suite at **175 tests PASS** with actual CUDA Re-ID and ROS clock rewind/pause tests.
- See [source-age verification report](docs/tracking/source_age_verification_2026-10-15.md).

### 2026-10-14: BoT-SORT Migration & Target Reacquisition
- Migrated tracking backend from legacy DeepSORT to Ultralytics BoT-SORT with ORB global motion compensation and ONNX Re-ID.
- Verified autonomous reacquisition of target person across multiple field-of-view exits and re-entries in `tracking_3_people_reentry_test` scenario.
- See [BoT-SORT migration report](docs/tracking/botsort_migration_report.md) and [target reacquisition report](docs/tracking/target_reacquisition_report.md). Historical DeepSORT reviews archived under `docs/archive/pre-botsort/`.

---

## Current Limitations

- **Tracking Continuity**: While zero wrong-target associations occur across test scenarios, tracking continuity under extreme yaw, prolonged occlusion, or crossing remains below full acceptance threshold.
- **Decoupled Flight Authority**: The UAV hovers at takeoff coordinates and commands gimbal pitch and yaw only. Autonomous horizontal chassis-following (`MotionArbiter`) is not yet enabled.

---

## Next Steps

1. **Camera Viewing Check**: In the simulator, compare `/custom_x500/camera/image_raw` with `/yolo/annotated_image` in `rqt_image_view` under the same scene. Record whether the raw stream or only the annotated stream appears delayed before changing the pipeline.
2. **Tracking Robustness**: Improve target continuity through yaw, crossing, occlusion, and re-entry; this still has not met acceptance.
3. **Chassis Following**: Consider `MotionArbiter` only after target continuity is reliable; horizontal body-following is not enabled.
