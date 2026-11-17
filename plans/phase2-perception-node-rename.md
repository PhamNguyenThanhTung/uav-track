# Phase 2 — Rename the perception node implementation

## Scope and contracts

Move the ROS orchestration in `src/uav_perception/uav_perception/yolo_detector.py` to `perception_node.py` and rename its implementation class to `PerceptionNode`. Move `spin_perception` and `main(args=None)` with it. Keep a single implementation.

- Keep `yolo_detector.py` as a small compatibility shim. Export `YOLODetectorNode` as an alias of `PerceptionNode` and re-export the same `main` function. Its module guard calls `main()` so `python3 -m uav_perception.yolo_detector --ros-args …` continues to work.
- Change the existing `setup.py` console-script target to `yolo_detector = uav_perception.perception_node:main`. Keep the installed command `ros2 run uav_perception yolo_detector`.
- Preserve the ROS node name `yolo_detector`, all topics, services, message schemas, parameters, defaults, QoS, callback groups, executor, DDS setup and shutdown behavior. Keep annotation, telemetry, camera watchdog, freshness checks, reset, gimbal actions and all algorithms as they run today. Phase 2 ends with this rename. TrackingPipeline and its tests, domain logic, flight control, gimbal control and Phase 3 stay outside this work.

## Callers and ownership

One Anti package owns the two node files, `src/uav_perception/setup.py`, the three direct test callers and one compatibility test. Take the starting point from `4d26f27` on `codex/verified-uavtrack-2026-11-17` and preserve all pre-existing worktree changes. In particular keep the dirty launchers, profiler and its tests, plans and evidence.

| Caller | Required treatment |
|---|---|
| `test_source_clock_ros.py`, `test_lazy_diagnostics.py`, `test_target_guard_pipeline.py` | Update direct imports and class references to the canonical module. Update monkeypatch paths for `cv2.imencode` and `time.monotonic` to patch where the moved implementation looks them up. Preserve test behavior. |
| `launch_phase4a_review.sh`, `simulation/run_live_demo.py`, `simulation/run_tracking_3_people_test.sh` | Keep their legacy module commands and options. The 25-second demo exercises that path end to end. |
| `evaluation/profile_perception_pipeline.py` and its tests | Keep existing process discovery and tests. It matches the legacy module/filename and will continue to see the same command lines. |
| ROS CLI, README run commands and ROS users | Preserve command and node names. If touching current documentation, limit it to locating the moved implementation and identifying the shim in the module inventory. Keep historical references intact. |

Sequence: record caller/dirty baseline → move implementation and add shim → retarget console script and update tests → build and validate both launch paths → independent review. The shim and canonical module land together. Keep the patch mechanical and compare the moved implementation against the baseline, allowing only rename-related changes.

## Verification and acceptance

Add a compatibility import test asserting `legacy.YOLODetectorNode is canonical.PerceptionNode` and `legacy.main is canonical.main`. Direct tests cover the canonical implementation. The installed entry point must resolve to `uav_perception.perception_node:main`.

Run the requested commands from the repository root:

```bash
source ../software/venvs/yolo26/bin/activate
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
colcon build --packages-select uav_perception uav_control
source install/setup.bash
PYTHONPATH="$PYTHONPATH:src/uav_perception:src/uav_control:evaluation" python3 -m pytest -q src/uav_perception/test evaluation/test_*.py
colcon test --packages-select uav_control
colcon test-result --verbose
git diff --check
ros2 pkg executables uav_perception
./launch_live_demo.sh --hover-time 25 --view-topic /yolo/annotated_image
```

- Build, pytest, control package test and test-result checks pass. The executable listing includes `uav_perception yolo_detector`. Check the installed entry point target separately because the listing checks the command name only. Confirm the active overlay contains the rebuilt package if imports or metadata appear stale.
- The launcher completes a 25-second tracking window via the legacy module with its intended viewer on `/yolo/annotated_image`. Verify annotated frames in the viewer, the run result's viewer topic and tracking evidence, and touchdown, disarm and owned process shutdown. Save run and log paths with exit codes.
- Review the diff and changed-file list against the starting state. Check that only approved rename changes are present, the shim imports without a cycle, the tests patch the canonical namespace, and the ROS contract and local edits are intact. Report a concrete `FAIL` or `NOT RUN` if build/overlay, CUDA, display or SITL preconditions prevent acceptance.
