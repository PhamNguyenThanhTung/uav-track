# Project Progress

This document tracks the current verified state of the UAV tracking simulation project. It serves as the quick status reference for developers and agents.

---

## Current Baseline

- **Platform**: Ubuntu 24.04 under WSL2
- **Frameworks**: ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8.15.0), PX4 SITL v1.17.0
- **Simulation**: Custom X500 multicopter (`custom_x500_gimbal_down`) with 3-DOF gimbal and downward RGB camera in world `gimbal_test.sdf`
- **Perception**: Real-time YOLO26n person detection + DeepSORT appearance tracking node (`uav_perception/yolo_detector`)
- **Gimbal Control**: Closed-loop 2D pixel visual servoing directly driving gimbal yaw and pitch joints
- **Flight Control**: Independent ROS 2 C++ Offboard node (`uav_control/altitude_control`) commanding PX4 position hold and target altitude

---

## Implemented

- [x] Gazebo Harmonic simulation with custom models and ground targets (`walking_actor`, `target_car`, `target_person`).
- [x] 3-DOF gimbal joint position controllers (`gz-sim-joint-position-controller-system`) and downward RGB camera sensor (`1280x720@30Hz`).
- [x] ROS 2 <-> Gazebo bidirectional bridge (`config/sim/ros_gz_bridge.yaml`) for camera image, camera info, and gimbal joint commands.
- [x] PX4 SITL integration (v1.17.0 standalone on `custom_x500_gimbal_down`) via Micro XRCE-DDS Agent on UDP port 8888.
- [x] ROS 2 C++ offboard controller node (`altitude_control`) executing automated takeoff to configured target altitude and holding horizontal position $(X_{hold}, Y_{hold})$.
- [x] All-in-one launch automation script (`launch_simulation.sh`) supporting standard GUI, headless mode (`--headless`), and auto-offboard takeoff (`--offboard`).
- [x] ROS 2 Python perception node (`yolo_detector`) performing GPU-accelerated YOLO26n inference (class 0: person) and DeepSORT tracking.
- [x] Gimbal visual servoing (proportional control with EMA smoothing, deadband, rate limiting, and joint limits).
- [x] Benchmark and evaluation suite (`evaluation/metric_calculator.py`, `evaluation/benchmark_evaluator.py`, `evaluation/audit_camera_fps_accounting.py`) recording latency, tracking error, ID switches, and loss metrics.
- [x] Synthetic and sliding-buffer unit test suites (`evaluation/test_metric_calculator.py`, `evaluation/test_evaluator_matching.py`).

---

## Verified

- **SITL Flight Stack**: Verified takeoff to 2.0 m, 3.0 m, 5.0 m, stable hover, and safe landing/disarm in Gazebo Harmonic.
- **Offboard Node**: Verified C++ build (`colcon build --packages-select uav_control`) and dynamic parameter reconfiguration (`target_altitude_m`).
- **Gimbal Motion**: Verified joint limits (Yaw: $\pm 180^\circ$, Pitch: $0^\circ \text{ to } +60^\circ$, Roll: $\pm 45^\circ$) and independent axis response.
- **Perception Pipeline**: Verified YOLO26n CUDA inference on RTX 4060 GPU and DeepSORT tracking on animated walking actor.
- **Evaluation Tests**: Verified `test_metric_calculator.py` (6 unit tests pass 100%) and `test_evaluator_matching.py` (3 unit tests pass 100%).
- **Runtime Benchmark**: 25-second live simulation run captured 337 processed frames, verified tracking retention, and logged telemetry without simulation crash.

---

## Known Limitations

1. **Monolithic Perception Node**: Detection, DeepSORT tracking, target selection, and gimbal visual servoing currently live in a single node ([`yolo_detector.py`](file:///home/tungt/UAV_Project/project/src/uav_perception/uav_perception/yolo_detector.py)).
2. **Headless Software Rendering FPS**: In virtualized / CPU LLVMpipe headless environments without GPU passthrough for Gazebo OGRE2, camera output drops to ~2.5–3.0 Hz. Full update rate requires hardware OpenGL acceleration.
3. **Open-Loop Failsafe on Target Loss**: When a target is lost, the gimbal currently holds its last commanded angle; there is no active scanning or recovery behavior.
4. **No Unit Tests in ROS Packages**: Packages in `src/` (`uav_perception`, `uav_control`) do not yet have unit tests inside their respective package directories.

---

## Not Implemented

- [ ] **Persistent Target Identity Layer**: Target identity is currently tied directly to DeepSORT `track_id` without external association or re-entry identity memory.
- [ ] **MotionArbiter**: No arbitration component exists between target observation and flight control.
- [ ] **UAV Body Target Following**: The drone body does not fly horizontally to chase or center the target; horizontal $(X, Y)$ position is held fixed at takeoff coordinates (`uav_movement_used_for_target_tracking = "NO"`).
- [ ] **Perception <-> Flight Control Integration**: No topic or service links `uav_perception` with `uav_control`.
- [ ] **Target Loss Search / Recovery Behavior**: No search patterns (spiral, yaw sweep) are triggered when tracking is lost.
- [ ] **Complete TF2 Coordinate Tree**: No ROS 2 TF transform broadcaster links `camera_link`, `base_link`, and the local NED map frame.

---

## Current Next Architecture Step

**Target Identity Abstraction & Perception/Control Interface Contract**:
Design a clean target state interface that decouples the target's identity from raw tracker IDs, preparing the data foundation for a future MotionArbiter without breaking existing gimbal servoing.

---

## Last Updated

- Date: 2026-10-06
- Git Baseline: `baseline: current hover perception and gimbal tracking system`
