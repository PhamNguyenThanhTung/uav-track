# Target reacquisition review — 2026-10-14

Subsequent yaw task: see [camera yaw regression report](camera_yaw_regression_report.md). Its candidate and one feedback trial failed broader scenario acceptance and were rolled back; the guard described here remains production. A full candidate simulation after the watchdog correction is documented there.

Subsequent geometry/appearance task: see [robustness report](geometry_appearance_robustness_report.md). Both reduced counterexamples pass in its candidate, but complete replay fails; guard and adapter are restored after one feedback round. No new full simulation was run for that task.

**Verdict: CHANGES REQUIRED; overall acceptance FAIL.** Ordinary loss no longer permanently locks the enrolled target out. Crossing, occlusion and both actual target-A re-entries recover the correct person, with zero observed wrong-target acceptances. However, camera-rotation acceptance regresses on identical inputs, and accepted coverage remains low. After one feedback correction and final review, policy iteration stops here as requested. Do not advance to body-following.

## Root cause and measured baseline

Complete pre-fix production capture: `evaluation/results/runs/tracking_3_people_20261014T093757Z` (1,510 processed frames, starting at SEARCHING). Before crossing, A is raw TID `1`, TRACKING, with eight appearance descriptors including its enrollment anchor. At 66.300 s, Re-ID/HSV similarity is 0.826/0.965 and motion passes; the last accepted observation is 66.364 s.

During crossing, BoT-SORT temporarily loses/reassigns A (`1 → 4 → 13`), and mixed/partial crops degrade Re-ID to approximately 0.48–0.56. Rejecting these observations is appropriate. `vertical_path` also rejects candidates as projection/pose/bbox geometry changes. At 68.908 s, the source-age timeout executes before matching, `_mark_lost()` clears all eight references and history, and the terminal LOST branch prevents all subsequent candidate evaluation. Thus there are both tracker/crop interruptions and a guard deadlock; a fixed raw-ID restriction or contaminated enrollment anchor is not the primary cause. Later A candidates exist with new IDs, but the old guard cannot score them at all. Replaying the complete sequence with preserved identity recovers A, including `1 → 17` at 70.720 s and `17 → 22` at 73.492 s.

The earlier auxiliary capture at 09:28 additionally showed a strong correct new-ID candidate rejected by `vertical_path`. It was superseded by the complete 09:37 trace; incomplete development captures are not acceptance evidence.

## Implemented policy

The implementation specification preceded policy changes. Changes stay inside the target guard, optional diagnostics, evaluation tools and simulation scenario; production BoT-SORT, ORB GMC, pretrained CUDA ONNX Re-ID and controller-facing ROS contracts remain in place.

- Ordinary LOST preserves the enrolled logical identity, immutable Re-ID/HSV anchors and bounded rolling references. It evaluates candidates explicitly; only an explicit reset allows new-target selection. New raw IDs can represent the same physical target.
- Recovery requires a fresh measured track, confidence ≥0.60, clean non-overlapping/non-border crop, Re-ID similarity ≥0.76, independent enrollment HSV similarity ≥0.90, and a ≥0.05 weighted appearance margin over eligible competitors. Without HSV, Re-ID must reach ≥0.90. Ranking includes close candidates just below the Re-ID threshold so that threshold filtering cannot hide ambiguity.
- Three clean strong observations must support one stable raw ID within the 2.5 s confirmation window. Adjacent observations remain fresh and geometrically stable with gaps ≤0.75 s. Uncounted weak/overlap frames can preserve pending positives only with the same fresh ID, strong independent appearance and bounded motion; they never count, teach references or emit a bbox. Missing tracks, conflicting evidence, ambiguity or expiry clear confirmation.
- Within short interruptions, recovery keeps a coarse spatial envelope. After the coasting window, obsolete vertical/heading extrapolation cannot veto identity indefinitely. Recovery resets motion history and warms up velocity estimation over at least three points spanning 0.2 s.
- Crowded, truncated, low-confidence and one-sided extent changes cannot train the rolling references or distort trusted geometry. Recovery appends one normalized descriptor averaged from its clean confirmations; the anchor remains fixed and memory stays bounded.
- Invalid source time/actual clock rewind invalidates the session until reset. The camera watchdog clears output without advancing the last actual image timestamp: a fresh resumed image can precede the watchdog tick while still being newer than the previous image.

`enable_guard_debug` defaults to false. Opt-in `/yolo/guard_debug` records source time, full measured guard inputs, candidate scores/gates, crop/extent status, transition reasons and GMC. The observer labels actor shirts independently; actor labels and stage labels never enter matching. There is no default production image/debug spam.

## Before/after replay

Replays use complete measured sequences, preserving source timestamps, detector boxes, active tracks, embeddings and GMC; no segment is selected out. Actor labels are excluded explicitly from guard inputs.

| Capture / policy | Frames | Accepted | LOST duration¹ | LOST → TRACKING | Wrong / unknown accepted |
|---|---:|---:|---:|---:|---:|
| Original baseline / old guard | 1,510 | 145 | 167.112 s | 0 | 0 / 0 |
| Original baseline / fixed guard | 1,510 | 406 | 33.592 s | 24 | 0 / 0 |
| Final scenario / old guard | 1,508 | 97 | 166.880 s | 0 | 0 / 0 |
| Final scenario / fixed guard, actual runtime | 1,508 | 237 | 76.636 s | 25 | 0 / 0 |

¹ Source intervals assigned to LOST samples; UNCERTAIN/PREDICTING abstention is additional time without accepted output. Reassociation within UNCERTAIN is not counted as a LOST → TRACKING recovery. Raw-ID changes are not physical-person switches.

Evidence: `target_reacquisition_replay/comparison.json`, `target_reacquisition_holdout_replay/comparison.json` and their `before/after/frames.csv`, under `evaluation/results/runs/`. The original old-guard replay matches its captured runtime exactly. Final-source replay matches all 1,508 final-runtime decisions, statuses, IDs and boxes: zero mismatches (`target_reacquisition_final_source_verification`).

## Production simulation

Final run: `evaluation/results/runs/tracking_3_people_20261014T100720Z`; actual production perception for 236 simulation seconds, clean landing, observer exit 0. This scenario first enrolls A alone, then introduces B/C and runs crossings. A itself leaves the FOV and returns twice; the older migration world mostly moved C out and therefore did not establish selected-target re-entry.

Primary metrics below use the complete guard trace, not the smaller joined ROS-topic sample. Wrong-target accepted frames are **zero in every stage**, with **zero unknown accepted labels**.

| Scenario | Frames | Accepted | LOST frames | LOST → TRACKING recoveries |
|---|---:|---:|---:|---:|
| Static target motion | 93 | 67 | 0 | 0 |
| Gimbal yaw | 36 | 15 | 0 | 0 |
| Person crossing | 124 | 18 | 48 | 2 |
| Three-person occlusion/crossing | 160 | 24 | 38 | 3 |
| Repeated crossing, including lateral flight | 476 | 93 | 76 | 16 |
| Short target-A exit/re-entry | 207 | 6 | 140 | 1 |
| Long target-A exit/re-entry | 412 | 14 | 363 | 3 |

First crossing recovery is 73.296 s (`1 → 22`, Re-ID-backed enrolled identity). Occlusion recoveries occur at 82.404, 86.496 and 89.764 s. Both exit stages contain absent A observations while distractors remain; the guard abstains instead of adopting another person.

| Re-entry | Raw ID before → after | First sustained shirt visibility² | First clean strong candidate | Recovered at | Delay / processed samples³ | Recovery Re-ID / HSV |
|---|---|---:|---:|---:|---|---|
| Short | 150 → 167 | 178.632 s | 180.280 s | 182.064 s | 3.432 s / 24 | 0.884 / 0.991 |
| Long | 167 → 193 | 219.652 s | 221.860 s | 222.588 s | 2.936 s / 25 | 0.778 / 0.986 |

² Sustained independent shirt-mask visibility after intermittent/partial sightings; mask threshold is 30 pixels, so isolated speckles are not a reliable return time. ³ Includes the first visibility and acceptance samples; delay after first clean strong candidate is respectively 1.784 s and 0.728 s. These are simulator source seconds, not wall time. Last accepted-to-recovered gaps are longer: 15.280 s and 40.524 s.

The final trace has 30 crowded/ambiguous-crop rejections and 29 unresolved-overlap rejections; 884 recovery decisions reject insufficient evidence. There are zero explicit competing-appearance ties in this differently clothed scene. Behavioral tests exercise similar-appearance competitors, including a competitor just below the threshold; do not interpret this run as validation against visually indistinguishable people.

**Actual horizontal motion:** simulation-only bounded NED-y excursion during 120–150 s, independent of target following. PX4 telemetry records 0.620 m horizontal displacement (719 samples, 120–155 s), altitude 2.970–3.026 m. All 280 processed frames in this interval have non-identity ORB transforms, 54 accepted A frames, eight correct LOST recoveries and zero wrong acceptances. Across the complete trace, 1,507/1,508 transforms are non-identity. Last runtime telemetry including landing reports GMC 1,550/1,570, adapter fallback 0; no perception traceback. GMC remained active, but this evidence does not establish continuous tracking during camera motion.

## Verification and final review

Commands run from the project root:

```bash
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" \
  RUN_BOTSORT_REID_RUNTIME=1 ../software/venvs/yolo26/bin/python \
  -m pytest -q -p no:cacheprovider src/uav_perception/test evaluation
colcon build --base-paths src --packages-select uav_perception
../software/venvs/yolo26/bin/python -m pip check
bash -n simulation/run_tracking_3_people_test.sh
TRACKING_GUARD_DEBUG=true TRACKING_LATERAL=true TRACKING_WORLD=target_reacquisition_test \
  ./simulation/run_tracking_3_people_test.sh
../software/venvs/yolo26/bin/python evaluation/replay_target_reacquisition.py \
  --capture evaluation/results/runs/tracking_3_people_20261014T100720Z \
  --before evaluation/results/runs/tracking_3_people_20261014T093757Z/target_identity_snapshot.py \
  --output-dir evaluation/results/runs/target_reacquisition_holdout_replay
../software/venvs/yolo26/bin/python evaluation/replay_target_reacquisition.py \
  --capture evaluation/results/runs/tracking_3_people_20261014T100720Z \
  --output-dir evaluation/results/runs/target_reacquisition_final_source_verification
```

- **PASS:** 157 tests, including A–I behavior, ambiguity/reference contamination/time regressions, node pipeline/watchdog integration, lateral-command limits and real CUDA ONNX Re-ID association. Build, dependency check and shell syntax pass. `pip check` emits a system pybind11 metadata warning but reports no broken requirements.
- **PASS:** measured before/after replay; final-source replay has zero runtime mismatches; correct selected-target recovery in crossing, occlusion, short/long re-entry and actual lateral flight; zero observed wrong/unknown acceptances; landing/shutdown clean. Snapshot and final source hashes are retained with final verification evidence.
- **FAIL before fix, PASS after fix:** the node integration test reproduces a fresh resumed image being incorrectly treated as clock rewind after a watchdog tick. The final full simulation preceded this narrow watchdog correction and contains zero watchdog events. **NOT RUN:** another full simulation after that correction; node integration plus unchanged complete-trace replay verify its affected path and normal-path regression respectively.
- **HIGH / CHANGES REQUIRED:** yaw acceptance on identical final-scene inputs falls from **22/36 to 15/36**, and on the original complete sequence from **33/35 to 28/35**. Static is essentially unchanged (68→67/93 final scene; 86→86/112 original). In yaw, the stricter protected-reference/geometry policy and confirmation pathway produce additional Re-ID rejects/pending frames: e.g. final-scene TID 1 scores 0.679/0.644 at 62.472/62.536 s under the protected memory; later recovery waits at 64.880/64.948 s. This is a real regression, not comparable-run noise, and violates the requested no-significant-regression criterion.
- **HIGH:** final accepted coverage is only 237/1,508 (15.7%); 25 recoveries indicate repeated interruption, not continuous person-following. Joined observer records 1,073 frames at 5.42 observed frames/s, processing p95 215.9 ms. The trace covers every processed guard update, not every 30 Hz camera image.

One feedback round replaced strict consecutive recovery positives with the bounded clean-positive history after the first actual short re-entry failed; the final run confirms both returns. Final review still finds the yaw regression, so no second policy/tuning round is performed. Required follow-up: preserve identity safety while restoring camera-rotation acceptance, then repeat independent motion/occlusion acceptance. No 3D estimator, motion arbiter, search or autonomous horizontal body-following was added. Shirt-based synthetic identity labels and a single final scenario cannot establish real-world identity safety.

All work was performed directly by Codex; no other model/worker was invoked. Project orchestration instructions/artifacts were removed earlier, and the leftover project-local orchestration dashboard was stopped. Existing unrelated working-tree changes and historical milestone evidence are preserved. Changes remain uncommitted.
