# uav_perception

ROS 2 package for YOLO26 person detection, BoT-SORT tracking, target identity protection, and gimbal visual servoing.

## Runtime path

`/custom_x500/camera/image_raw` → YOLO person detections → BoT-SORT → `TargetIdentityManager` → `GimbalController` → gimbal yaw/pitch commands. Only fresh, identity-validated detector measurements are published as `/yolo/detections` or used to update the gimbal. Ambiguous observations produce no target bbox. Ordinary LOST retains the enrolled identity and evaluates evidence for recovery; `/yolo/reset_target` explicitly permits selecting a new target.

The tracker uses the installed Ultralytics BoT-SORT implementation. Its upstream parameters are in [`config/botsort.yaml`](config/botsort.yaml): ORB global motion compensation, enabled ReID, `yolo26n-reid.onnx`, and upstream thresholds. The same current-frame affine GMC transform moves the target guard's short motion history into the camera's current image coordinates. ReID weights are downloaded to `~/.cache/uav_perception/`, outside the repository. Startup fails if the configured ONNX model, ReID encoder, or CUDAExecutionProvider is unavailable; it does not silently fall back to IoU-only tracking.

The adapter rejects malformed, nonfinite, singular or orientation-reversing GMC matrices through the native tracker's identity-transform fallback, and reports that fallback to the guard and telemetry. ORB fitting is unchanged. This validity correction is independent of the rejected identity-policy experiments; see the [redesign review](../../docs/tracking/target_guard_redesign_report.md).

Image source age is checked on ingress and again after detector/tracker work, before identity learning and gimbal output. Late results publish an empty target and hold the gimbal setpoint; their GMC still advances existing motion history without teaching identity. Two executor threads let `/clock` advance during inference, while images, target reset and the watchdog share one serialized callback group. A paused simulation does not expire an image by wall time alone. See [source-age verification](../../docs/tracking/source_age_verification_2026-10-15.md); tracking continuity remains below acceptance.

## Modules

- `yolo_detector.py`: ROS 2 image subscriber, YOLO inference, BoT-SORT integration, freshness checks, target manager, gimbal controller, HUD, and latency telemetry.
- `botsort_adapter.py`: validates runtime backends and maps only activated current-frame tracks to original detector bbox/confidence by detection index; records ReID/GMC timing and fallback counts.
- `frame_freshness.py`: rejects stale or non-monotonic camera frames and detects a stalled stream while respecting simulated-clock pauses.
- `target_identity.py`: preserves one logical person across tracker-ID changes; maintains a protected appearance anchor, bounded references, short motion history, and optional HSV torso cue; states are `SEARCHING`, `TRACKING`, `UNCERTAIN`, `PREDICTING`, and `LOST`. Reacquisition needs strong identity evidence, clean stable crops, and three fresh confirmations. A clock rewind invalidates the session until reset.
- `gimbal_controller.py`: EMA smoothing, deadband, rate limits, and joint limits for yaw/pitch.
- `image_utils.py`: ROS image decoding/encoding and generic torso HSV descriptor.

## Dependencies

ROS dependencies are declared in `package.xml`. Python runtime packages for the project's Python 3.12 perception virtual environment are listed in [`requirements-perception.txt`](requirements-perception.txt): NumPy, Ultralytics 8.4.164, ONNX, ONNX Runtime GPU, protobuf, and `ml-dtypes` constraints. PyTorch and OpenCV are supplied by the existing environment.

From the project directory, install/update the perception environment with:

```bash
../software/venvs/yolo26/bin/python -m pip install -r src/uav_perception/requirements-perception.txt
```

The runtime requires an NVIDIA driver and CUDA-capable ONNX Runtime provider for pretrained ReID. The detector uses `yolo26n.pt` on the configured device (default `cuda:0`).

## Run

```bash
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
source ../software/venvs/yolo26/bin/activate
source install/setup.bash
python -m uav_perception.yolo_detector --ros-args -p use_sim_time:=true
```

The ROS-Gazebo bridge must publish `/clock` from the active world and provide the configured camera topic before starting the node.

For candidate/gate diagnostics set `enable_guard_debug:=true`. The optional `/yolo/guard_debug` JSON topic includes source inputs, gate/state reasons, and an evaluation JPEG; it is disabled by default. The simulation observer records complete replay inputs and adds independent actor labels outside the guard:

```bash
TRACKING_GUARD_DEBUG=true TRACKING_LATERAL=true TRACKING_WORLD=target_reacquisition_test \
  ./simulation/run_tracking_3_people_test.sh
```

The lateral maneuver uses a simulation-only PX4 setpoint stream after hover, with a bounded 0.6 m NED-y excursion and telemetry recording. Production altitude control and perception flight authority are unchanged. The runner pauses simulation time during perception model startup after hover, then resumes the unchanged scenario; this prevents model loading from consuming the static/yaw intervals.

## Tests

```bash
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" \
  ../software/venvs/yolo26/bin/python -m pytest -q src/uav_perception/test evaluation
```

The pretrained GPU ReID smoke test is opt-in and can be run with `RUN_BOTSORT_REID_RUNTIME=1`. Set `YOLO_AUTOINSTALL=false` during verification to prevent Ultralytics from changing dependencies when CUDA is unavailable.
