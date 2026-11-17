# Phase 2 — Rename Perception Node Verification Evidence

Date: 2026-11-17
Starting commit: `4d26f27` on branch `codex/verified-uavtrack-2026-11-17`

## 1. Summary of Changes

Phase 2 bounded package:
- Moved ROS orchestration in `src/uav_perception/uav_perception/yolo_detector.py` to `src/uav_perception/uav_perception/perception_node.py` with class `PerceptionNode`. Moved `spin_perception` and `main(args=None)` with it.
- Kept `src/uav_perception/uav_perception/yolo_detector.py` as a minimal backward-compatibility shim:
  - Re-exports `PerceptionNode`, `main`, and `spin_perception`.
  - Defines `YOLODetectorNode = PerceptionNode`.
  - Module entry guard `if __name__ == '__main__': main()` preserves `python3 -m uav_perception.yolo_detector --ros-args ...` execution.
- Retargeted entry point in `src/uav_perception/setup.py`:
  - `yolo_detector = uav_perception.perception_node:main`
- Updated direct implementation tests to import canonical module and patch canonical lookup targets:
  - `src/uav_perception/test/test_source_clock_ros.py`
  - `src/uav_perception/test/test_lazy_diagnostics.py`
  - `src/uav_perception/test/test_target_guard_pipeline.py`
- Added compatibility test:
  - `src/uav_perception/test/test_perception_node_compat.py` (verifies `legacy.YOLODetectorNode is canonical.PerceptionNode`, `legacy.main is canonical.main`, and installed console script entry point resolution).
- Updated module inventory in `src/uav_perception/README.md`.
- Preserved existing dirty/untracked files: launchers, profiler, and plans.
- Intentionally untouched: `TrackingPipeline`, algorithms, ROS topics/services/types/QoS runtime contracts, flight control, gimbal control, historical documentation, and no Phase 3 work.

## 2. Command Execution and Exit Codes

### A. Colcon Build
Command:
```bash
source ../software/venvs/yolo26/bin/activate
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
colcon build --packages-select uav_perception uav_control
```
- **Exit code**: `0`
- **Output summary**:
  ```
  Starting >>> uav_control
  Starting >>> uav_perception
  Finished <<< uav_control [0.17s]
  Finished <<< uav_perception [0.73s]
  Summary: 2 packages finished [1.18s]
  ```

### B. Executables and Entry Point Resolution
Commands:
```bash
source install/setup.bash
ros2 pkg executables uav_perception
python3 -c "import importlib.metadata; eps = [ep for ep in importlib.metadata.entry_points(group='console_scripts') if ep.name == 'yolo_detector']; print(eps[0].name, '->', eps[0].value)"
```
- **Exit code**: `0`
- **Output**:
  ```
  uav_perception yolo_detector
  yolo_detector -> uav_perception.perception_node:main
  ```

### C. Pytest Suite
Command:
```bash
PYTHONPATH="$PYTHONPATH:src/uav_perception:src/uav_control:evaluation" python3 -m pytest -q src/uav_perception/test evaluation/test_*.py
```
- **Exit code**: `0`
- **Output**:
  ```
  208 passed, 4 skipped, 38 subtests passed in 5.41s
  ```

Focused compatibility test:
```bash
PYTHONPATH="$PYTHONPATH:src/uav_perception:src/uav_control:evaluation" python3 -m pytest -v src/uav_perception/test/test_perception_node_compat.py
```
- **Exit code**: `0`
- **Output**:
  ```
  src/uav_perception/test/test_perception_node_compat.py::test_yolo_detector_shim_identity PASSED
  src/uav_perception/test/test_perception_node_compat.py::test_installed_entry_point_target PASSED
  2 passed in 1.45s
  ```

### D. Colcon Test & Test-Result
Commands:
```bash
colcon test --packages-select uav_control
colcon test-result --verbose
```
- **Exit code**: `0`
- **Output**:
  ```
  Finished <<< uav_control [4.61s]
  Summary: 1 package finished [5.11s]
  Summary: 27 tests, 0 errors, 0 failures, 2 skipped
  ```

### E. Git Diff Check
Command:
```bash
git diff --check
```
- **Exit code**: `0` (clean, no trailing whitespace or merge conflict markers)

### F. 25-Second Annotated Live Demo
Command:
```bash
./launch_live_demo.sh --hover-time 25 --view-topic /yolo/annotated_image
```
- **Exit code**: `0`
- **Result status**: `RESULT: PASS`
- **Run directory**: `evaluation/results/runs/live_demo_20261117T104308Z_76173`
- **Run result file**: `evaluation/results/runs/live_demo_20261117T104308Z_76173/result.json`
- **Node log file**: `evaluation/results/runs/live_demo_20261117T104308Z_76173/yolo_detector.log`
- **Viewer log file**: `evaluation/results/runs/live_demo_20261117T104308Z_76173/viewer.log`

#### Live Demo Telemetry & Evidence Verification:
- **Legacy invocation**: Launched as `sys.executable -m uav_perception.yolo_detector ...`
- **Node startup**:
  - `[INFO] [yolo_detector]: Initializing PerceptionNode with BoT-SORT, ORB GMC, and ReID...`
  - `[INFO] [yolo_detector]: PerceptionNode ready for person tracking.`
- **Viewer topic**: `/yolo/annotated_image` (subscribed by `rqt_image_view` during tracking)
- **Tracking window**:
  - Duration: 25.0 s
  - Camera frames: 1,482
  - Detection messages: 639
  - Positive detection arrays during tracking: 80
  - Detections count: 80
- **Touchdown & Disarm**:
  - `Touchdown confirmed: landed=true, arming_state=1, NED z=0.024 m.`
  - `checks.landed_disarmed_near_ground: true`
- **Owned process exit codes**:
  - `yolo_detector`: `0`
  - `altitude_control`: `0`
  - `px4`: `0`
  - `bridge`: `0`
  - `gazebo`: `-15` (SIGTERM clean shutdown)
  - `agent`: `-2` (SIGINT clean shutdown)
  - `viewer`: `0`
- **Final Outcome**: `0` (PASS)

### G. Direct Annotated-Frame Delivery Check

The first run result counts raw camera frames, so a second annotated-topic sample was captured during the active tracking window:

- **Run directory**: `evaluation/results/runs/live_demo_20261117T105141Z_79737` (`RESULT: PASS`)
- **Topic observer**: `timeout 5s ros2 topic hz /yolo/annotated_image` on ROS domain `137`
- **Observed**: 70 annotated messages over the sample window; reported average rate reached `22.55 Hz`.
- **Observer exit**: `124`, expected because `timeout` stopped the still-running rate subscriber after 5 seconds; it had received messages before termination.
- **Viewer**: `rqt_image_view` was launched for `/yolo/annotated_image`; `ros2 topic info -v` observed its subscription. The query ran during teardown and showed publisher count 0, so the message-rate sample is the direct evidence of annotated-frame delivery.
- **Result**: annotated frames were published and received during the live demo; touchdown, disarm, and process cleanup passed in the run result.

### H. Visual Viewer Check and Final Smoke

- **Run directory**: `evaluation/results/runs/live_demo_20261117T105531Z_82117`
- **Screenshot**: [`annotated_view.png`](annotated_view.png), captured from the `rqt_image_view` X11 window during active tracking and copied into this evidence folder after the run.
- **Visible frame**: viewer selected `/yolo/annotated_image`; the image shows the person detection box, track ID `2`, and HUD state `TrackingStatus.TRACKING`.
- **Run result**: `RESULT: PASS`; 25.0 s tracking, 1,467 camera frames, 603 detection messages, 84 positive detection arrays, and 84 boxes.
- **Touchdown**: `landed=true`, `arming_state=1`, NED z `-0.068 m`.
- **Shutdown**: no Gazebo/PX4/perception/bridge/viewer processes remained. Owned process exits were zero except the XRCE agent's expected SIGINT (`-2`). The viewer log contains a class-loader warning during teardown; viewer and Gazebo both exited zero, with no process left running.
