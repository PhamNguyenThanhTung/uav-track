# Camera yaw regression audit — 2026-10-14

Subsequent geometry/appearance review: [report](geometry_appearance_robustness_report.md). That task also failed after one feedback round and restored this same guard baseline; its replay results are separate from the historical yaw experiment below.

**Verdict: FAIL; stop after one feedback round.** The experimental repair reaches the two prescribed identical-input yaw targets, but regresses crossing/occlusion and total continuity. Feedback does not resolve those regressions. Both policy variants are rejected; production `target_identity.py` is restored byte-for-byte to the pre-task guard, including the watchdog correction. The yaw fix is **not shipped**. No Antigravity, other model or worker was used, and no body-following was added.

## Confirmed root cause

Complete replay reproduces old/pre-task yaw 33/35 → 28/35 on `tracking_3_people_20261014T093757Z`, and 22/36 → 15/36 on `tracking_3_people_20261014T100720Z`. Original old-guard and final pre-task-guard runtime comparisons have zero mismatches.

The reacquisition change applies one-sided crop-extent protection to clean crops, but compares current detector edges against repeatedly GMC-transformed history. Background scale error contracts the stored box; rejecting its refresh then accumulates that error. In the original capture, history freezes at 64.420 s; measured A height stays approximately 315–326 px while stored height contracts to 233 px by 65.540 s. This creates false `vertical_path` rejection. In the final capture, history freezes at 60.724 s: measured height remains about 342–349 px, while transformed height is 232 px by 61.316 s. Clean views cannot refresh the rolling reference, so later yaw Re-ID similarity also falls.

All twelve lost-acceptance frames below contain fresh measured A with raw ID 1, clean/non-border/non-ambiguous crops, confidence 0.864–0.905 and enrollment HSV 0.971–0.997. Source times and GMC are identical between policies. No detector absence, raw-ID lifecycle change, stale observation or clock rewind explains these frame differences. This confirms a guard-policy/GMC interaction and reference-memory effect, rather than a changed detector/model or an enrollment-anchor overwrite.

| Capture | Source time | Old / pre-task Re-ID | Rejection cause |
|---|---:|---|---|
| Original | 65.540 | .866 / .866 | vertical motion |
| Original | 65.608 | .819 / .826 | vertical motion |
| Original | 65.772 | .823 / .797 | vertical motion |
| Original | 65.836 | .778 / .795 | vertical motion |
| Original | 65.968 | .825 / .815 | vertical motion → pending recovery |
| Final | 62.472 | .820 / .679 | missing reference from 61.316 s |
| Final | 62.536 | .795 / .644 | missing reference from 61.316 s |
| Final | 63.364 | .810 / .706 | missing reference from 61.084 s |
| Final | 63.760 | .774 / .738 | missing reference from 61.480 s |
| Final | 64.780 | .830 / .661 | reference and vertical motion |
| Final | 64.880 | .876 / .798 | vertical motion → pending recovery |
| Final | 64.948 | .862 / .850 | vertical motion → pending recovery |

Detailed errors, crop flags and reference provenance: `evaluation/results/runs/yaw_regression_analysis_20261014/lost_yaw_frames.csv`. Diagnostic controls isolate effects: restoring only motion refresh yields original yaw 33/35; restoring only reference refresh yields final yaw 17/36. Simply undoing extent protection is rejected because it permits partial-crop learning. Saved specification and controlled variants remain in that evidence directory; none is installed in production.

## Reviewed implementation and feedback

The initial candidate separates the last trusted **measured** bbox from the GMC motion trajectory, accounts for two detections' existing 2% edge-noise allowance, and adds one bounded pose query. Query continuation requires same-ID TRACKING, a fresh clean trusted crop, confidence ≥.60, enrollment HSV ≥.90, confident motion, no close identity competitor and neural similarity ≥.76 to a recently accepted pose (≤.75 s). It cannot select/reacquire another ID, train the gallery, or seed another query-only step; abstention/watchdog/loss/reset clears it. Anchors, gallery-learning threshold .85, recovery thresholds and three-confirmation policy remain protected.

The input-based justification precedes implementation; measured static/yaw edge-noise residual p95 is 1.85% / 1.55% in the two captures. New behavioral tests cover bias under GMC, a partial crop that deceptively agrees with GMC, bounded query continuation, no gallery drift, stale/duplicate/watchdog/low-confidence/wrong-color/new-ID/competing-ID rejection and fresh controller output.

Final review rejects this candidate because repairing yaw changes reference/history evolution outside yaw and significantly reduces correct-target acceptance. One feedback trial replaces the latest matching rolling descriptor instead of evicting older views when memory is full. It improves some aggregate metrics but leaves original crossing at 16/131 versus baseline 31/131, and new-runtime yaw at 9/35 versus baseline 14/35. It is also rejected. No second feedback or further policy tuning follows.

## Complete identical-input comparisons

“Baseline” here means the **pre-task reacquisition guard**, not the terminal-LOST migration guard. All three policies have zero wrong/unknown accepted labels on all complete comparisons.

| Capture | Pre-regression yaw floor | Baseline yaw | Initial candidate | Feedback |
|---|---:|---:|---:|---:|
| Original, 1,510 frames | 33/35 | 28/35 | 33/35 | 35/35 |
| Final, 1,508 frames | 22/36 | 15/36 | 25/36 | 25/36 |

Accepted counts by scenario, baseline → candidate → feedback:

| Scenario | Original capture | Final capture | New runtime capture |
|---|---|---|---|
| Static | 86 → 88 → 112 /112 | 67 → 80 → 80 /93 | 23 → 27 → 27 /114 |
| Yaw | 28 → 33 → 35 /35 | 15 → 25 → 25 /36 | 14 → 10 → 9 /35 |
| Crossing | 31 → 21 → 16 /131 | 18 → 18 → 20 /124 | 27 → 8 → 8 /123 |
| Occlusion | 20 → 23 → 29 /171 | 24 → 28 → 25 /160 | 34 → 4 → 6 /152 |
| Repeated crossing/lateral | 76 → 82 → 65 /435 | 93 → 85 → 104 /476 | 65 → 52 → 53 /442 |
| Short exit/re-entry stage | 39 → 42 → 43 /217 | 6 → 5 → 6 /207 | 9 → 7 → 8 /199 |
| Long exit/re-entry stage | 104 → 82 → 109 /387 | 14 → 16 → 14 /412 | 14 → 4 → 10 /386 |

Original exit stage labels refer to the older world; selected A did not itself leave as in the final/new scenario. Use final/new captures to evaluate actual selected-person re-entry.

| Capture / metric | Baseline | Initial candidate | Feedback |
|---|---:|---:|---:|
| Original accepted /1,510 | 406 (26.9%) | 393 (26.0%) | 431 (28.5%) |
| Original LOST duration | 33.592 s | 53.328 s | 47.824 s |
| Original longest accepted-to-accepted gap | 7.328 s | 13.496 s | 11.120 s |
| Final accepted /1,508 | 237 (15.7%) | 257 (17.0%) | 274 (18.2%) |
| Final LOST duration | 76.636 s | 82.368 s | 76.924 s |
| New-runtime accepted /1,467 | 186 (12.7%) | 112 (7.6%) | 121 (8.2%) |
| New-runtime LOST duration | 83.028 s | 117.448 s | 107.484 s |
| New-runtime LOST recoveries | 21 | 15 | 18 |

LOST duration assigns source intervals to LOST samples; UNCERTAIN/PREDICTING adds further abstention. Long accepted gaps include actual target absence. These metrics show why more aggregate acceptance/recoveries is insufficient to declare improved continuity. No arbitrary coverage acceptance threshold is invented. A provisional regression floor for the next task is each measured baseline above: aggregate ≥26.9%, ≥15.7%, ≥12.7% respectively, while also meeting each scenario count and the yaw floors. These measured floors do not establish body-following readiness. Pre-register visible-target interruption budgets from the required control update rate; 2.5 s coasting is an existing upper guard boundary, not proof that such a control gap is acceptable.

Evidence under `evaluation/results/runs/`: `yaw_acceptance_original_20261014`, `yaw_acceptance_final_capture_20261014`, `yaw_runtime_comparison_20261014`, and corresponding `yaw_feedback_*` directories; all contain full before/after frame CSVs and summaries. Evaluation actor labels are removed from matching inputs.

## Actual production simulation after watchdog correction

Candidate run `tracking_3_people_20261014T144151Z` captures 48.808–235.952 s: **1,467 processed guard updates**, 112 accepted A frames, 15 correct LOST recoveries, zero wrong/unknown acceptance. Crossing recovers at 74.152 s; occlusion at 90.092 and 97.716 s. Actual short A return recovers at 181.404 s (`171 → 180`), long return at 222.456 s (`180 → 207`), and again at 233.776 s (`209 → 216`). Thus no permanent LOST state-machine deadlock reappears, but interruptions/coverage regress.

Actual lateral displacement is **0.624 m**, altitude 2.985–3.048 m, 723 PX4 samples. ORB GMC and real CUDA ONNX Re-ID remain active; final GMC telemetry 1,471/1,562, adapter fallback 0. The vehicle lands (NED z −0.050 m), observer exits 0, and all run-owned processes stop. Candidate source hash equals the run snapshot; replay matches every captured output with zero mismatches.

Joined observer records 1,040 frames at 5.09 observed frames/s; processing mean/p95 107.6/225.5 ms, compared with prior run 106.8/215.9 ms and 5.42 observed frames/s. Run-to-run inputs/initial poses and skipped camera samples differ, so latency/coverage differences alone do not establish policy causality; same-input comparisons above do. The guard trace covers processed updates, not every 30 Hz camera image.

An earlier full-duration run, `tracking_3_people_20261014T143555Z`, starts perception at 62.472 s and misses static/most yaw; it is explicitly excluded from scenario acceptance and retained as failed scheduling evidence. The runner now pauses **simulation time after hover during model startup**, then resumes through Gazebo's existing world-control service. Both service calls succeeded in the complete capture. Actor paths, scenario source-time intervals, tracker/model, ROS/PX4 contracts and flight maneuver remain unchanged. Cleanup also resumes a startup-paused world before requesting descent on failure.

The full run contains zero query-only continuation acceptances: the new measurements do not satisfy that short continuation pathway. It includes the watchdog correction, but has **zero watchdog events**. Resumed-stream timestamp behavior is verified by the node integration test; do not interpret this simulation as a deliberately stalled-stream runtime test.

## Final verification, retained changes and limits

**HIGH findings:** `target_identity.py::_accept/_motion_consistent` still confuses measured crop quality with transformed motion geometry; `_accept/_appearance_similarity` needs reference maintenance that survives view changes without learning partial crops. The tested replacements regress crossing/occlusion, so restoring the baseline is required pending a validated solution. See the per-frame CSV and two reduced failures before attempting another change.

- **PASS (candidate only):** 171 tests, real CUDA smoke enabled; node pipeline suite 6/6; build/dependency/syntax checks. A green suite did not establish acceptance.
- **FAIL:** complete replay and final review on scenario regressions; feedback still fails. Initial candidate ran in the full simulator; feedback was replayed only and was not promoted to runtime.
- **Restored production:** 157 existing tests PASS with real CUDA smoke, perception build PASS, dependency check PASS, runner syntax PASS. Guard bytes exactly match `before_yaw_guard.py`, retaining ordinary-LOST recovery, anchors, freshness and watchdog fix. Rollback replay matches the pre-task source exactly; it is a policy comparison, not agreement with the rejected candidate's recorded runtime output.
- **Known failures retained:** the two newly reduced GMC/crop regressions still fail on the restored baseline (see `unresolved_baseline_reproductions.txt`). These expose frozen motion/reference state and possible partial-crop reference contamination under misleading GMC; distinct-shirt simulation zero wrong-target observations do not prove general identity safety.
- **NOT RUN:** full simulator after rollback; already verified guard was restored exactly. Startup-failure cleanup branch was not forced; its resume service was exercised at ordinary startup. No hardware flight, body-following or real-world visually similar identity validation was performed.

Final changed files are the simulation runner, this report, current progress/documentation links. The rejected guard, implementation/feedback diffs, specification, candidate tests and verification records remain in `yaw_regression_analysis_20261014`; they are ignored experiment artifacts, not production code. Unrelated user changes/historical evidence are preserved; no commit was made.

Reproduction commands (from project root; replay output directories must be new):

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
# Actual run above used the retained implemented_guard.py candidate snapshot.
../software/venvs/yolo26/bin/python evaluation/replay_target_reacquisition.py \
  --capture evaluation/results/runs/tracking_3_people_20261014T144151Z \
  --before evaluation/results/runs/yaw_regression_analysis_20261014/before_yaw_guard.py \
  --after evaluation/results/runs/yaw_regression_analysis_20261014/implemented_guard.py \
  --output-dir evaluation/results/runs/yaw_review_reproduction
```

Next task requires reliable measured-crop/ego-motion separation and validated view-reference management across all three complete traces, including the new counterexamples. A yaw-only score improvement cannot justify another relaxation of identity safety. Current production yaw remains 28/35 and 15/36 on the two canonical captures; tracking has not reached acceptance, and body-following remains blocked.
