# BoT-SORT migration report

> Migration snapshot before the subsequent guard recovery fix. Current target-loss/reacquisition behavior and lateral-flight acceptance are documented in [target_reacquisition_report.md](target_reacquisition_report.md). Tracker/model/configuration remain unchanged.

**Result: implementation and runtime integration passed; person-following acceptance is incomplete.** The production node now runs BoT-SORT with ORB GMC and pretrained ONNX Re-ID. It did not switch to a distractor in the final simulation, but the target guard entered terminal `LOST` at the first person crossing and did not resume without an explicit reset. Do not advance to autonomous body-following until that continuity gap and update-rate target are addressed.

## Architecture

The production path changed from DeepSORT to **YOLO26n → Ultralytics BoT-SORT → `TargetIdentityManager` → `GimbalController`**. Ultralytics 8.4.164 already provides the selected GMC and Re-ID mechanisms, so the migration uses its implementation instead of adding another tracker or custom motion compensator. The manager remains authoritative: only accepted fresh target boxes reach the controller-facing `/yolo/detections` output. Rejected/ambiguous observations abstain, and terminal `LOST` requires `/yolo/reset_target`.

## Final configuration

| Setting | Value |
|---|---|
| Detector | YOLO26n, person class only, CUDA |
| Tracker | Ultralytics 8.4.164 BoT-SORT |
| GMC | ORB, upstream affine estimate; identity fallback on failure |
| Re-ID | `yolo26n-reid.onnx`, ONNX Runtime CUDAExecutionProvider |
| Thresholds | high 0.25, low 0.10, new track 0.25, match 0.8, buffer 30 frames, proximity 0.5, appearance 0.8, fuse score enabled |

The model is cached in `~/.cache/uav_perception/`, outside the project. The adapter fails at startup if the model or CUDA provider is unavailable. Runtime requirements are in `src/uav_perception/requirements-perception.txt`.

## Code changes

- Added the BoT-SORT adapter/config and preserved current detection bbox, confidence, source timestamp, and active-track semantics at the target-manager boundary.
- Added stale/duplicate/out-of-order frame rejection and simulation-clock-aware watchdog behavior.
- Applied GMC's affine warp to the guard's stored image-space motion history.
- Kept accepted target output as the only bbox source for gimbal control; updated overlay and telemetry with tracker, guard, GMC, and Re-ID status. Hardened the node and simulation launcher shutdown order so perception stops while ROS clock/bridge are still live.
- Updated simulation launch/observer wiring to exercise the production perception node and to record target actor identity independently from tracker IDs.
- Removed seven obsolete DeepSORT-specific files: `evaluation/tracking_3_people_test.py`, `tracking_identity_candidate.py`, `tracking_appearance.py`, `replay_tracking_identity.py`, `test_tracking_identity_candidate.py`, `replay_phase4a_target_guard.py`, and `profile_phase4a_target_guard.py`. Production tracker lifecycle and target-manager behavior remain covered by the BoT-SORT adapter and guard suites.
- Removed the project-level delegation skill at the user's request; only maintenance procedures remain under `skills/`.

## Tests

- **PASS — tests:** `135 passed in 9.32s` across perception and evaluation, with the real CUDA Re-ID smoke test enabled and no skips. Command:

  ```bash
  source /opt/ros/jazzy/setup.bash && source ../software/ros2_ws/install/setup.bash && \
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" RUN_BOTSORT_REID_RUNTIME=1 \
  ../software/venvs/yolo26/bin/python -m pytest -q -p no:cacheprovider src/uav_perception/test evaluation
  ```

- **PASS — build:** `colcon build --base-paths src --packages-select uav_perception`.
- **PASS — dependencies:** `../software/venvs/yolo26/bin/python -m pip check` reported no broken requirements.
- **PASS — syntax/whitespace:** `bash -n launch_simulation.sh launch_phase4a_review.sh simulation/run_tracking_3_people_test.sh` and `git diff --check`.
- Real ONNX inference produced embeddings; the BoT-SORT association check confirmed appearance distances affect matching. Tracker lifecycle, multiple detections, empty frames, reset, GMC failure, unavailable runtime, timestamp freshness, and target-manager integration are covered by the migrated tests.

## Simulation results

**PASS — execution/cleanup:** `./simulation/run_tracking_3_people_test.sh` completed through 236 simulation seconds in world `tracking_3_people_reentry_test`, recording 1,159 production frames at 6.18 frames/s wall-clock. Final evidence is in `evaluation/results/runs/tracking_3_people_20261014T090706Z`. Teardown reported `landed=true`, `observer_exit=0`, with no remaining simulator or perception processes and no shutdown exception. Ground-truth actor identity was inferred from independent shirt-color labels inside the accepted production bbox; this observer did not run another detector or tracker.

**FAIL — full continuity acceptance:** the guard entered terminal `LOST` during the first crossing. Static/yaw runs and all later safety behavior are reported below; crossing continuity and subsequent reacquisition did not pass.

| Stage | Accepted target frames | Wrong-actor frames | Result |
|---|---:|---:|---|
| Static camera | 49/63 | 0 | Mostly accepted, but includes 14 safe abstentions; selected actor was `person_a`. |
| Gimbal yaw | 17/22 | 0 | Target stayed correct, with 5 safe abstentions during the camera turn. |
| Person crossing | 2/102 | 0 | Guard abstained after the initial crossing and entered terminal `LOST`. |
| Three-person occlusion/crossing | 0/131 | 0 | No wrong target; continuity was already lost before this stage. |
| Repeated crossing plus altitude change | 0/373 | 0 | Altitude setpoints 3.0→3.5→3.0 m were accepted and observed, but tracking continuity could not be assessed because the guard was already `LOST`. |
| Short exit/re-entry | 0/154 | 0 | No automatic reacquisition after terminal `LOST`, consistent with the reset policy. |
| Long exit/re-entry | 0/314 | 0 | Same limitation; this was not an isolated re-entry after a fresh target reset. |

Across all stages there were zero wrong-actor classifications. This is evidence of fail-closed target selection, not continuous tracking acceptance. The tested vehicle movement was vertical altitude change only; horizontal translation was not tested.

## Performance

- Observed output rate: **6.18 frames/s** over the run (camera configured for 30 Hz).
- YOLO latency: mean **38.2 ms**, p95 **49.6 ms**, max **69.1 ms**.
- BoT-SORT step: mean **44.2 ms**, p95 **152.2 ms**, max **382.8 ms**. Periodic Re-ID timing-window means averaged **43.2 ms** (p95 **101.1 ms**, max **357.2 ms**); those windows include zero-cost periods after the target was lost.
- End-to-end processing latency: mean **98.1 ms**, p95 **207.6 ms**, max **433.4 ms**.
- GMC made **1,708** estimates, including **1,651** non-identity transforms. ORB warned once about too few matching points and returned its upstream identity transform without crashing; the adapter recorded **0** exception fallbacks.
- GPU sampling: mean utilization **33.2%**, max **47%**; VRAM was **547 MiB** of 8,188 MiB. One process snapshot reported about **3.61 CPU cores** and **2.0 GiB RSS**.

The observed rate and p95 latency do not establish a 30 Hz control loop. The current evidence is not enough to claim adequate flight-following responsiveness.

## Cleanup

- Removed the project-local agent delegation workflow under `skills/delegation` at the user's request; the remaining `skills/` entries are bug-fix, phase-review, and runtime-verification procedures. No other model was called for implementation.
- Deleted the seven obsolete DeepSORT-specific evaluation/prototype/test/profiler files and three superseded tracker-research notes.
- Deleted exploratory and superseded run directories from this migration after confirming no references; retained only the final run evidence at `evaluation/results/runs/tracking_3_people_20261014T090706Z`.
- Removed the accidental YOLO weight cache under the project-local literal `~` directory. The runtime cache remains outside the repository; no model weights or generated videos were added to the project.
- Historical review notes remain; they describe earlier runs and may mention DeepSORT.
- `docs/project_structure_audit.md` is labeled as a pre-migration snapshot because earlier reviews cite it; current status is maintained here and in `PROGRESS.md`.

## Known limitations and recommendation

The first crossing/occlusion drives the existing guard into terminal `LOST`; it requires `/yolo/reset_target` and does not automatically re-acquire. Later stages in this run therefore do not prove post-occlusion continuity or isolated leave/re-entry behavior. The 6.18 frames/s observed rate and 208 ms processing p95 also need a defined control-loop target and further optimization.

**Recommendation: keep BoT-SORT + ORB GMC + ONNX Re-ID as the production tracker, but mark the person-following milestone as changes required.** Fix and retest guard continuity without weakening the no-wrong-target behavior, then run isolated occlusion/re-entry and UAV-translation holdouts at the required update rate before proceeding to horizontal following.
