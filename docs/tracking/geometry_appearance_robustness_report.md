# Tracking geometry and appearance robustness — 2026-10-14

**Verdict: FAIL; stopped after exactly one feedback round.** Both reduced counterexamples pass in the candidate, but neither candidate satisfies complete-trace continuity/yaw acceptance. Guard, adapter and adapter tests are restored byte-for-byte to the verified baseline; the installed package matches. Full simulation for this task is **NOT RUN**, because replay failed. All work was performed by Codex alone; no other model, worker, orchestrator or body-following was used.

## Verified cause

Production is byte-identical to the rolled-back guard (`e7c5bd4…`), including the watchdog fix. Both reduced GMC/crop failures reproduce. Adapter observations use fresh original detector boxes/current embeddings, while BoT-SORT passes `results_high.xyxy` to ORB; no masking-coordinate bug is demonstrated. The guard mutates historical rectangles with GMC, then reuses them to judge crop extent and gate both motion refresh and appearance learning. Genuine detector measurements and transformed predictions therefore become conflated. Previous raw-box repair still coupled motion/reference updates, froze legitimate trajectories/view learning and regressed three complete traces. Latest-view replacement also regressed crossing; neither is an accepted solution.

Source order, stale-track flags, affine validity and raw-ID lifecycle are independently recorded in `evaluation/results/runs/geometry_appearance_20261014/root_verification.json`. Nonfinite/singular GMC is a robustness case, not the demonstrated cause of the recorded yaw delta. Crop completeness remains inferred from available boxes/overlap/borders; real-world indistinguishable identities are outside this task's demonstrated coverage.

## Recorded design and self-review before implementation

1. Retain measured motion samples unchanged, compose separate camera transforms and build projected geometry only for motion evaluation. Crop quality compares trusted measured edges; background scale cannot qualify or disqualify appearance learning by itself.
2. Separate output identity, motion updates, complete measured geometry and appearance learning. A clean identity-confirmed partial observation may constrain motion via its retained edge and previous body extent, explicitly as a motion proxy; it cannot overwrite trusted measured extent or train appearance. Ambiguous/crowded observations cannot supply this proxy. Confirmed controller output remains the current accepted detector box, never a proxy/prediction.
3. Evaluate compensated motion and, only for strong independently matched same-ID continuation with no close competitor, a short unwarped measured-motion hypothesis. Keep all existing appearance/confidence/margin/confirmation thresholds; this fallback does not choose or recover another ID.
4. Learn one normalized descriptor from three clean, complete, independently strong accepted samples, using existing confirmation gap/window bounds. Enrollment remains immutable; one bounded reference deque is retained. Weak or query-only observations never count as learning evidence. A temporary strong sample from that pending batch may support same-ID continuation for ≤0.75 s, with strong enrollment HSV and valid motion; it cannot seed a weak chain, change IDs or participate in recovery.
5. Carry provenance on recovery references: a descriptor confirmed below the strong-learning threshold cannot later turn its self-similarity into stronger independent evidence. Its matching score is capped by the independently established confidence. Only fresh independent strong samples can create a strong learned reference.
6. Clear transient learning/motion quality state on expiry, reset and invalid source time; preserve enrolled appearance through ordinary LOST. Timestamp/watchdog logic and public ROS/PX4 interfaces remain unchanged.

Risks: retained-edge proxies are imperfect under pose changes; stale measured crop models must expire. Averaging/reference confidence changes can reduce legitimate view coverage or cause ties. Therefore behavioral tests alone cannot authorize promotion. Test the two reductions, partial/overlap/drift/competition/time/GMC edge cases; run all three identical complete traces and compare every scenario count to baseline. Runtime is permitted only after all floors and safety checks pass. One final-review feedback round is allowed, then rollback and stop if acceptance still fails.

## First complete review and the single feedback round

Initial candidate: two reductions and 64 adjacent behavioral tests PASS, but complete replay FAIL. Original/final/new total acceptance is 396/188/115 versus baseline 406/237/186; yaw 16/12/11 versus required 33/22/14. It is not promoted to simulation. Frame review finds 100/73/83 baseline-accepted frames losing protected appearance evidence, and 34/32/11 losing motion. This confirms that uniformly averaging only high-threshold samples removes useful view exemplars and that the motion fallback remains coupled to complete-crop learning quality.

The one feedback correction will retain an observed medoid supported by three independently accepted clean complete samples, rather than manufacture an averaged view. Samples matched only by the temporary probe cannot teach. Confirmed samples below the strong threshold require strong independent HSV and retain their measured confidence cap; neither an all-weak batch nor weak recovery can bootstrap strong evidence. Unlearnable crops may preserve old positive samples within existing freshness/window bounds but never add positives. Motion fallback may use an independently matched retained-edge proxy during same-ID uncertainty; it still cannot recover another ID or learn a partial crop. Opposing edge movement is not coherent translation and cannot certify a complete crop.

All three captures also contain 7/7/9 singular GMC matrices (all on empty-observation frames; new-runtime includes yaw at 65.144 s). Adapter validation currently accepts them because they are finite and have shape 2×3. Reject non-positive affine determinant through the existing upstream identity fallback, without altering ORB or its fitting algorithm. Add real BoT-SORT integration coverage for this case. These changes form one feedback round; after tests and all complete replays, a remaining failure requires rollback and stop.


## Final review of the single feedback candidate

The feedback candidate additionally excludes weak references from independent learning scores. Weak references may match at their established confidence cap, but cannot teach another weak reference or recovery descriptor. A known partial crop does not become complete merely because its old full measurement ages while accepted observations continue. Recovery reference learning uses independent evidence; same-ID motion fallback uses the same retained-edge proxy for validation and residual. These complete the recorded one feedback correction, not a further tuning round.

**BLOCKER — complete-trace acceptance fails.** Initial candidate totals are 396/188/115; feedback totals are 281/90/69 against baseline 406/237/186. The feedback repairs original yaw from 28 to 30 but misses 33; final/new yaw and crossing/occlusion/re-entry regress. No further policy adjustment follows this review.

**HIGH — view coverage and motion remain coupled through evidence availability.** Among baseline-accepted frames rejected by feedback, protected neural evidence falls below .76 in 193/150/120 frames; motion is invalid in 133/107/87. These flags overlap and are not independent causal counts. Complete crops account for most rejected frames. Requiring multi-frame independent teaching and confidence provenance changes reference history sufficiently that legitimate later views no longer match; they cannot activate the strong motion fallback. Once LOST, another-ID recovery still requires independent appearance and confirmation, so geometry alone cannot restore continuity. A full eight-descriptor memory therefore does not imply useful strong view coverage. Medoids alone did not solve the reference loss, and loosening partial-crop/identity protection solely for scores is rejected.

Concrete feedback examples: original crossing at 66.168 s has protected/independent similarities .774/.755, inferred partial extent and `vertical_path` failure; final yaw at 62.108 s has .633/.573 despite a clean complete crop; new-runtime yaw at 63.460 s has .691/.606 despite valid motion. The original gains in static/yaw do not compensate for later losses. Detailed frame reasons, provenance, crop/motion flags and first examples per stage are in `continuity_metrics.json`. These are frozen-candidate diagnostics, not new production changes.

**HIGH — restored production defects remain.** Both reduced reproductions still fail on restored production. Distinct-costume captures with zero wrong/unknown labels do not eliminate the synthetic partial-crop contamination failure. The baseline adapter also still accepts singular finite affine transforms. Adapter rejection was tested successfully in the candidate, then restored with the rejected change set; it is not claimed shipped.

## Complete identical-input replay

Source captures and every source timestamp/observation/GMC input are preserved. Actor labels/fractions are explicitly removed before invoking the guard; labels and visibility enter evaluation only. Baseline means the pre-task guard saved as `baseline_guard.py`, not the historical terminal-LOST implementation. CSV row/source-time alignment and source SHA256 are recorded in `continuity_metrics.json`. Baseline replay exactly matches final-capture recorded output (zero mismatches); original/new runtime used different guards, so their baseline mismatches are expected. Candidate replay mismatch counts compare against recorded historical runtime, not an acceptance oracle.

Exact capture directories under `evaluation/results/runs/`: original `tracking_3_people_20261014T093757Z`; final `tracking_3_people_20261014T100720Z`; new-runtime `tracking_3_people_20261014T144151Z`.

| Capture | Frames | Baseline yaw → initial → feedback | Required yaw | Total baseline → initial → feedback | Required total |
|---|---:|---|---:|---|---:|
| Original | 1,510 | 28 → 16 → 30 /35 | ≥33/35 | 406 → 396 → 281 | ≥406 |
| Final | 1,508 | 15 → 12 → 12 /36 | ≥22/36 | 237 → 188 → 90 | ≥237 |
| Runtime | 1,467 | 14 → 11 → 8 /35 | ≥14/35 | 186 → 115 → 69 | ≥186 |

Every policy has **0 wrong-target and 0 unknown-target accepted frames on each complete trace**. Each scenario must also retain its baseline count; all three feedback comparisons FAIL independently.

Accepted counts, **baseline → initial → feedback / scenario frames**:

| Scenario | Original | Final | New-runtime |
|---|---|---|---|
| Warmup | 22 → 13 → 15 /22 | — | 0 → 0 → 0 /16 |
| Static | 86 → 66 → 99 /112 | 67 → 36 → 35 /93 | 23 → 27 → 27 /114 |
| Yaw | 28 → 16 → 30 /35 | 15 → 12 → 12 /36 | 14 → 11 → 8 /35 |
| Crossing | 31 → 6 → 8 /131 | 18 → 5 → 4 /124 | 27 → 5 → 6 /123 |
| Occlusion | 20 → 9 → 10 /171 | 24 → 14 → 5 /160 | 34 → 2 → 2 /152 |
| Repeated crossing / lateral stage | 76 → 79 → 25 /435 | 93 → 99 → 31 /476 | 65 → 45 → 17 /442 |
| Short exit/re-entry stage | 39 → 54 → 33 /217 | 6 → 8 → 1 /207 | 9 → 10 → 3 /199 |
| Long exit/re-entry stage | 104 → 153 → 61 /387 | 14 → 14 → 2 /412 | 14 → 15 → 6 /386 |

Original is the older world: selected A does not actually exit in its F/G stages, and its movement test changes altitude rather than the final/new lateral maneuver. Final/new captures are the evidence for actual selected-A re-entry and lateral movement. No scenario/ground-truth changes were made in this task.

Additional continuity metrics, **baseline → initial → feedback**:

| Metric | Original | Final | New-runtime |
|---|---|---|---|
| All-frame coverage | 26.9% → 26.2% → 18.6% | 15.7% → 12.5% → 6.0% | 12.7% → 7.8% → 4.7% |
| LOST duration (s) | 33.592 → 48.608 → 98.108 | 76.636 → 86.756 → 128.600 | 83.028 → 111.180 → 138.008 |
| LOST → TRACKING recoveries | 24 → 21 → 19 | 25 → 21 → 13 | 21 → 17 → 10 |
| Longest accepted gap (s) | 7.328 → 15.708 → 14.220 | 40.524 → 40.424 → 71.940 | 39.304 → 39.832 → 70.620 |
| Visible-frame coverage | 26.9% → 26.2% → 18.6% | 20.3% → 16.1% → 7.7% | 17.0% → 10.5% → 6.3% |
| Total visible interruption (s) | 142.792 → 144.112 → 157.292 | 120.564 → 126.236 → 137.604 | 123.944 → 132.192 → 137.704 |
| Longest visible interruption (s) | 7.160 → 15.576 → 13.992 | 9.536 → 14.556 → 15.744 | 9.968 → 15.444 → 19.204 |
| Accepted in 120–150 s window | 41 → 50 → 15 | 45 → 54 → 24 | 25 → 20 → 4 |

Visible sample denominators are 1,509/1,168/1,091; 120–150 s window denominators are 235/240/222. Visible duration is 189.124/148.792/146.428 s. Visibility uses the saved independent costume detector, which can miss fully obscured or tiny-shirt samples; it is an evaluation proxy, not exact actor-pose ground truth. Visible interruption integrates the prior sample's visibility/abstention over each successive source interval, resets at accepted output or observed absence, and excludes an unobserved tail. LOST duration retains the replay helper's convention assigning intervals to the current LOST sample. Accepted gaps include actual target absence; visible gaps separate that proxy. No new controller-ready interruption threshold is invented.

## Tests, code paths and restored state

| Check | Candidate | Restored production |
|---|---|---|
| Clean measured crop despite biased background scale | PASS | FAIL |
| Partial crop agrees with GMC but must not contaminate references | PASS | FAIL |
| Focused geometry / manager / real BoT-SORT tests | 88 PASS, 1 runtime smoke skipped before enabling flag | Existing tests included below |
| Full suite with real CUDA ONNX Re-ID | 174 PASS | 157 PASS |
| Watchdog/source resumption node integration; duplicate/stale stamps; crossing, new-ID recovery and distractors | PASS in full suite | PASS in full suite |
| Known partial crop persists without time-based promotion; composed rotation/inverse; invalid GMC; weak-reference teaching; query interruption | PASS in candidate additions | Candidate additions archived; not collected as production tests |
| Complete replay acceptance, all three captures | FAIL before and after feedback | Baseline reproduced; yaw floors still missed |
| Perception build, dependency check, runner syntax | Final candidate not separately built/promoted | PASS |
| Full production simulation for this task | NOT RUN: replay failed | NOT RUN: exact baseline restoration |

Candidate code paths: `src/uav_perception/uav_perception/target_identity.py` changes measured/projected geometry, crop classification, motion fallback/proxy, reference teaching/provenance and recovery learning; `botsort_adapter.py::_apply_gmc` rejects non-positive determinant through existing upstream fallback. Tests temporarily added `test_geometry_appearance.py` and two actual-tracker affine cases in `test_botsort_adapter.py`. All are restored/archived after failure. No detector, ONNX model, ORB algorithm, ROS/PX4 interface, controller, actor path, watchdog correction or simulation runner was changed.

Measured motion has six bounded samples with separate composed warps; complete measured extent is separate from motion proxies. Appearance retains an immutable anchor and bounded reference deque (eight total by default), plus one bounded confirmation batch. Ordinary LOST preserves enrollment; abstention/watchdog clears temporary query evidence, reset/invalid source-time clears the session. No view clustering, multi-bank memory or framework was introduced. A bbox-only completeness test still cannot distinguish every true camera-scale/pose change from a partial person crop; these local candidates do not establish a general safe solution.

Restored guard SHA256: `e7c5bd4affab7cde691cf7b7dc4e6561b876ffb42c58e10f27ca75f5aa955b49`. Guard, adapter and adapter tests match their saved pre-task bytes; the built installed guard/adapter also match. `rollback_verification.json` records hashes and final checks. Historical evidence and unrelated working-tree changes are preserved. No commit was made.

One rollback verification attempt ran while archived candidate adapter tests were still present: 157 existing tests passed and two candidate-only tests failed against restored adapter. The remaining test restoration was completed before the final 157-PASS verification. An unscoped colcon attempt discovered a duplicate package inside historical source snapshots; `--base-paths src` builds successfully without removing that evidence. Both retries are recorded, not counted as candidate or final baseline acceptance failures.

## Runtime, validity and unverified scope

No new full runtime latency/FPS measurement is available for either candidate. Historical joined-observer FPS is 5.23/5.42/5.09, processing p95 224.0/215.9/225.5 ms in original/final/new captures. These are historical pipeline measurements, not the new candidate's performance or guard replay throughput. The last two captures record lateral displacements 0.620/0.624 m and landing; those runs used historical guards and do not verify this task's candidate.

All three input traces have strictly increasing source time, zero stale observations and zero watchdog events. Independent A raw-ID counts are 55/44/58: same raw-ID permanence cannot be assumed. Seven/seven/nine finite singular GMC matrices occur, all with no observations; real BoT-SORT candidate tests demonstrate existing identity fallback for singular/reflection matrices. Identical-input replay consumes saved detections, embeddings and IDs: it does not rerun ORB, detector, CUDA model or tracker assignments. Therefore adapter changes affecting future IDs are covered only by component integration, not a new full pipeline run.

Matching ambiguity remains an abstention, controller output is an accepted current detector bbox, and recovery uses appearance/independent HSV plus fresh confirmation rather than motion alone. Watchdog timestamp correctness is verified by node integration; the three historical simulations did not deliberately exercise a stalled stream. Hardware flight, continuous body-following, general similar-person safety and reliable whole-person crop/pose/scale separation remain unverified. Full simulation and extra policy tuning are intentionally not attempted after the failed final review.

## Evidence and reproduction

Under `evaluation/results/runs/geometry_appearance_20261014`: pre-task source snapshots; `root_verification.json`; rejected `candidate_v1.py` / `candidate_v2.py`; source diffs; archived candidate geometry/adapter tests; first/final review JSON; `continuity_metrics.json`; `summarize_review.py`; `restored_reductions.log`; rollback hashes/checks. Complete replay directories `geometry_{original,final,runtime}_{v1,v2}` each retain before/after summaries and every output frame CSV. Experiment files are ignored artifacts and do not change normal production test collection.

From project root (choose a fresh replay output path; helper refuses overwrites):

```bash
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" \
  RUN_BOTSORT_REID_RUNTIME=1 ../software/venvs/yolo26/bin/python \
  -m pytest -q -p no:cacheprovider src/uav_perception/test evaluation
colcon build --base-paths src --packages-select uav_perception
../software/venvs/yolo26/bin/python -m pip check
bash -n simulation/run_tracking_3_people_test.sh
# Reproduce the two still-failing baseline reductions, without editing source:
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/uav_perception \
  ../software/venvs/yolo26/bin/python -m pytest -q -p no:cacheprovider \
  evaluation/results/runs/geometry_appearance_20261014/candidate_geometry_tests.py \
  -k 'full_measured_crop_survives or gmc_agreement_cannot_hide'
# Repeat for each exact capture listed in the table:
../software/venvs/yolo26/bin/python evaluation/replay_target_reacquisition.py \
  --capture evaluation/results/runs/tracking_3_people_20261014T144151Z \
  --before evaluation/results/runs/geometry_appearance_20261014/baseline_guard.py \
  --after evaluation/results/runs/geometry_appearance_20261014/candidate_v2.py \
  --output-dir evaluation/results/runs/geometry_review_reproduction
PYTHONDONTWRITEBYTECODE=1 ../software/venvs/yolo26/bin/python \
  evaluation/results/runs/geometry_appearance_20261014/summarize_review.py
```

**Final acceptance: FAIL.** The tested local separation fixes both counterexamples but does not jointly improve camera yaw, preserve all-scenario continuity and establish stronger identity safety. The required stop/rollback is complete. Further solution design is a separate task; these candidates are not production fixes or evidence for starting body-following.
