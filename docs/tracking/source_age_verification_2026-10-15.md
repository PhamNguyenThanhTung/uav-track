# Source-age verification and isolated motion projection — 2026-10-15

Timing correction: **PASS WITH NOTES**. Overall tracking: **CHANGES REQUIRED**. The corner-preserving geometric candidate is offline and rejected. The retained timing change does not alter identity thresholds, reference-learning rules, scenario paths or flight authority.

## Reproduced defects and retained change

The production node checked image age only on ingress. Detector/ReID delay could therefore enroll an expired observation, update appearance references and emit a gimbal command. Three new component checks failed on the original node. A real ROS reproduction also showed that even a two-thread executor could not advance `/clock` while inference occupied the default mutually exclusive callback group: both advancing-clock cases failed, while the two paused-clock controls passed.

The node now uses two executor threads and one separate mutually exclusive group shared by image processing, target reset and the statistics/watchdog timer. ROS time can advance in the default group while perception state writers remain serialized. The existing 0.75 s source-clock budget is checked again after detector/tracker work, before identity learning or controller output. A late result publishes an empty target, keeps raw detections/processing telemetry, and holds the existing gimbal command. JPEG diagnostics follow controller output so they cannot delay that decision.

A late completed tracker frame still changes the camera coordinates of historical motion samples. The first timing version omitted that GMC step; a focused regression reproduces the omission. The final version passes the frame's warp to the optional `mark_stream_stale(..., camera_warp=...)` argument. This projects existing history and ages identity without adding a measured target or changing the last real image timestamp. Timer-only watchdog calls keep the previous behavior. Replay applies saved watchdog GMC before aging, including when loading older guard snapshots. The identity decision rules themselves are unchanged.

SIGINT/SIGTERM first interrupt executor spin, then drain workers while ROS is alive, destroy the node and shut down its context. This addresses the teardown warnings observed in the first full capture with default rclpy signal handling.

## Isolated geometric experiment — rejected

Before timing changes, an independent trial retained projected quadrilateral corners between affine transforms. Production failed four rotation/inverse/composition checks; the isolated candidate passed all seven geometry/lifecycle checks and the existing non-GPU suite (163 passed, one skipped).

| Complete capture | Baseline accepted | Candidate accepted | Baseline yaw | Candidate yaw |
|---|---:|---:|---:|---:|
| Original, 1,510 updates | 406 | 413 | 28/35 | 28/35 |
| Final, 1,508 updates | 237 | 165 | 15/36 | 12/36 |
| Runtime, 1,467 updates | 186 | 179 | 14/35 | 14/35 |

All accepted labels remained correct, but per-scenario coverage and visible interruptions regressed. The candidate failed the pre-recorded acceptance gates and was never copied into production. No threshold search followed. Fixing the geometric identity does not by itself fix the crop/reference policy.

## Verification

- **PASS:** final source perception/evaluation/control suite: **175 tests**, including real pretrained CUDA ReID and real ROS delayed-detector/delayed-tracker delivery. The final suite ran after the full simulation and shutdown adjustment.
- **PASS:** advancing source clock suppresses expired results; paused source clock permits long wall processing; source resumption does not become a clock rewind; an actual clock rewind invalidates identity; rejected frames preserve enrollment and project GMC without learning a target.
- **PASS:** perception build and dependency check; shell syntax and whitespace checks.
- **PASS:** retained manager replays the three canonical captures byte-for-byte like baseline: 406/237/186 accepted and yaw 28/15/14. These remain continuity failures, not new acceptance passes.
- **FAIL, known independent defects:** biased-GMC clean-view learning, partial-crop GMC agreement and weak recovery chains still fail their three saved reduced tests. They are outside the retained timing correction and must remain visible in progress reporting.

## Runtime evidence

Final source-frozen capture: `tracking_3_people_20261015T065057Z`, world `target_reacquisition_test`. It ran through the 236 s scenario and landed (`observer_exit=0`, `landed=true`). Recorded guard source times span **46.156–235.028 s**. All frozen production source hashes match the retained sources and installed package. The node shut down without traceback, Destroyable errors or unretrieved exceptions.

- **339 recorded guard events:** 286 camera events and 53 late-result watchdog events. No timer-stall warning occurred during this run. Late-warning logging is throttled (40 messages); the last node statistics report 56 stale results, which can include frames after the observer stopped. These counts describe different boundaries and must not be equated.
- **11 accepted updates, zero wrong/unknown accepted labels.** Ten positives are in the static stage and one in warmup. Yaw is **0/7 recorded events** (0/5 usable-camera events); crossing, occlusion and both re-entry stages have no acceptance. This is a clear continuity failure, not successful following.
- Historical and retained identity policies give identical replay outputs on these same saved inputs, with **zero recorded-state/bbox mismatches**. The timing correction does not improve the underlying identity policy. Watchdog GMC is included during replay.
- **12 native GMC fallbacks** are reported. The invalid matrices were rejected rather than propagated into tracker/guard history.
- The joined observer matched **254 frames**, **1.23 observed joined frames/s**, processing mean **553.8 ms**, p95 **780.1 ms**. Joined FPS is not camera FPS or total detector throughput. Runtime latency and continuity remain blockers. Older runs are not a paired performance control, so this does not establish whether threading or environment load caused the difference.
- There are **23 source-time backsteps** when a synthetic current-clock watchdog event precedes the next real image's earlier capture stamp. Real-source ordering is preserved by the manager, and replay matches. Do not interpret naive mixed-event LOST-duration sums as a controller interruption budget; no such metric is used to claim acceptance here.

The first timing capture (`tracking_3_people_20261015T064104Z`) completed through 236 s and landed, but accepted only one target update and had the GMC omission described above. It is diagnostic evidence, not final verification. Its joined observer measured 1.46 frames/s and processing p95 684.4 ms; these are this capture's measurements, not a paired comparison with older runs.

The scenario disables autonomous gimbal tracking so the observer can drive prescribed camera yaw. The ROS integration tests exercise the actual command publication path with deterministic detector/tracker fixtures. Neither is a new measured-joint closed-loop gimbal acceptance or hardware flight test. UAV XY hold is retained; the scenario's existing altitude changes remain simulation-only test inputs.

## Reproduction and saved evidence

Task-local sources, exact before/after diffs, commands, logs, replay summaries and manifests are in `evaluation/results/runs/motion_projection_20261015T063300Z/`. The experiment plan was recorded before candidate edits. Existing earlier reports and captures were preserved.

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 YOLO_AUTOINSTALL=false RUN_BOTSORT_REID_RUNTIME=1 \
  PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" \
  ../software/venvs/yolo26/bin/python -m pytest -q -p no:cacheprovider \
  src/uav_perception/test evaluation src/uav_control/test
colcon build --base-paths src --packages-select uav_perception
../software/venvs/yolo26/bin/python -m pip check
bash -n simulation/run_tracking_3_people_test.sh
YOLO_AUTOINSTALL=false TRACKING_GUARD_DEBUG=true \
  TRACKING_WORLD=target_reacquisition_test TRACKING_LATERAL=false \
  ./simulation/run_tracking_3_people_test.sh
```

The ROS timing tests explicitly use domain 83, separate from the default-domain simulator. GPU/simulator checks run sequentially. No dependency installation, external software edit, body-following, worker, commit or push was introduced.
