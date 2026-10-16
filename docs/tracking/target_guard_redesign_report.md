# Target guard redesign — 2026-10-15

**Final verdict: FAIL for identity-policy redesign; no promotion.** Production guard remains byte-identical to baseline. Independent singular-GMC adapter hardening is retained with seven native-tracker regression cases. Full simulation is NOT RUN. Non-GPU candidate/production suites pass; CUDA smoke fails because no GPU device is available in the current environment. No policy tuning follows final review.

## Diagnosis and experiment plan recorded before production edits

Production guard SHA256 is `e7c5bd4affab7cde691cf7b7dc4e6561b876ffb42c58e10f27ca75f5aa955b49`, matching the restored baseline. Previous candidates are rejected evidence, not the starting implementation. Codex performs this task alone.

Verified code flow: YOLO current person detections → native BoT-SORT (ORB masks use current detections; Kalman association/Re-ID) → adapter indexes original detector boxes with fresh activated track `idx`, uses `curr_feat`, retains Kalman velocity only as a direction cue → guard transforms `_history` with each GMC → scores neural/HSV, computes motion and overlap/border quality → gates identity/competition/confirmation → `_accept` emits the current detector box. `_accept` compares current top/bottom against transformed `_history[-1]`; one `geometry_is_trusted` controls both history append and reference learning. Enrollment is immutable, but bounded rolling descriptors can change every accepted clean confident frame. LOST recovery averages confirmed descriptors and appends without provenance. Watchdog ages the state while preserving the last actual image timestamp.

An accepted box is not automatically an appearance-learning sample. Conversely, the current code already permits some unclean but non-ambiguous boxes for motion; the additional extent gate couples that motion refresh to crop learning. `_history` mixes measured boxes at append time and repeatedly projected/reboxed predictions after GMC. This is directly verified in code; a raw detector/adapter coordinate bug is not demonstrated.

| Hypothesis | Expected benefit | Identity risk | Minimal state | Required tests / evidence |
|---|---|---|---|---|
| H1: immutable measured samples, separate composed projections; crop checks against measured extent | Avoid reboxing drift and GMC certifying crop completeness | Raw extent cannot identify every pose/zoom versus partial crop | Six measured samples with composed transforms; last complete measured box | Both reductions; rotation/inverse; reset; all complete traces |
| H2: motion update independent of reference learning | Keep continuity through unlearnable extent changes | Partial centroids can corrupt velocity or enable a crossing distractor | No extra identity memory; separate motion eligibility | Partial/crowded motion, opposite direction, competing identities; replay |
| H3: bounded measured exemplars with provenance | Preserve real views instead of averaging/medoid-only teaching | Weak recovery can bootstrap matching or evict stronger views | Provenance per existing bounded reference; replacement by redundant-pair eviction, anchor pinned | Weak recovery/drift, budget/anchor, view coverage; replay |
| H4: short-lived non-teaching query | Bridge immediate view change without permanent contamination | Query chains can drift across frames/IDs | One independently strong clean descriptor with source time/raw ID | Expiry, duplicate/stale/watchdog, competitor, partial crop, non-bootstrap; replay |

Offline protocol: audit baseline frame-level evidence and temporal loss onsets first; reproduce two counterexamples. Compare H1, H2, H3, H4 separately on identical saved inputs (including unchanged recorded GMC), then at most one evidence-selected composition. No scenario timestamps/labels enter policies and no threshold search is permitted. Source snapshots are under `evaluation/results/runs/target_guard_redesign_20261015`. Only a candidate passing reduced tests and every complete-trace floor can replace production. Final review ends policy iteration; failure leaves/restores baseline and skips simulation.

Singular GMC is a separate adapter bug: shape/finite checks currently accept zero or negative determinants. Verify with native BoT-SORT integration, reject through its existing identity fallback, and retain that independently tested correction separately from identity-policy promotion. Recorded policy replays do not silently replace GMC; a separately labelled validity-only control will show its guard-side effect without claiming to regenerate upstream IDs. No ORB fitting change.

## Results

### Evidence-led selection before the final candidate

Baseline frame audit reproduces every scenario count. The all-flags-off instrumentation control is identical to baseline. Original yaw first develops a new vertical-motion failure at 65.540 s while neural is .866; final yaw at 62.472 s loses neural (.679) and geometry eligibility; original crossing at 66.992 s first loses crop quality and neural (.458), before additional vertical-motion failure at 67.256 s. These are temporal onsets, not aggregate causal attribution. Full flags (including simultaneous failures), accepted/rejected status and raw-ID transitions are retained per frame.

Separate H1/H2/H3/H4 replay totals (original/final/runtime) are 386/224/129, 410/252/98, 307/148/33 and 406/237/186. H1 fixes both known reductions but misses original yaw and regresses runtime continuity. H2 gives original yaw 33 but fails existing partial-crop velocity tests. H3's capped recovery still permits a sequence of weak recoveries to confirm a new unsupported ID: confidence capping alone does not establish independent recovery evidence. H4 has no output change on the three captures. None is production-ready.

One diagnostic H3 FIFO control (271/114/33) removes redundant-pair replacement while keeping provenance unchanged; runtime remains 33. Thus provenance/recovery evidence, not replacement alone, constrains view coverage. This control is not a threshold search. An initially over-restrictive query test incorrectly prohibited a persistent independently matched continuation merely because a distant competitor existed; the test is corrected to check that temporary query support is blocked, while allowing valid persistent evidence. Original test logs are retained.

Final candidate selected before final review: H1 plus H3 measured exemplars/provenance, with the baseline FIFO rule. This tests whether trustworthy measured crop geometry restores enough independent view teaching to offset the provenance constraint. Recovery and new-ID reassociation must be supported by independent strong references/anchor; weak references cannot chain through their own similarities. Competition is assessed before this independent-evidence veto so a just-below-threshold competing person is not filtered out to force a choice. No numeric thresholds change. H2 is excluded because partial centroids create false motion; baseline already supports full-extent non-ambiguous limb-overlap motion without learning that crop. H4 is excluded because it contributes no trace benefit. No further composition/tuning will follow final review.


## Verified state ownership and failure taxonomy

Current detector boxes are already copied and remain current measurements through controller output. The mutation is in *historical copies*, not in the current YOLO box. The installed Ultralytics implementation applies GMC in `_pre_first_associate`, using `results_high.xyxy`; ORB downsamples masks and detections together. Adapter output uses current `idx`/`curr_feat`; Kalman `mean` supplies only a per-frame direction cue. All guard geometry is image pixels; motion timing uses image source seconds, not receipt wall time. No detector-coordinate or ground-truth leakage defect was found.

| State / decision | Retained baseline | Rejected offline H1-based candidate |
|---|---|---|
| Current bbox | Copied fresh detector rectangle | Same; output never uses projected/proxy boxes |
| Historical motion | Six time/box pairs mutated and reboxed by each GMC | Six immutable measured boxes plus composed transforms; projected cache rebuilt for motion |
| Crop completeness for teaching | Edge changes against projected history, plus overlap/border gates | Separate last complete measured box; raw signed edge agreement with existing 2% allowance |
| Motion eligibility | Non-ambiguous extent-trusted accepted sample; may have limb overlap | Same conservative eligibility evaluated against raw extent; rejected H2 would additionally append partial centroids |
| Appearance teaching | Clean, confident same-ID/motion/appearance sample; bounded FIFO | Actual measured exemplars; independent strong evidence, capped recovery provenance; bounded FIFO |
| Recovery | Three fresh clean matches, mean descriptor stored as an ordinary reference | Three confirmations still required; independent evidence for recovery/new ID, measured exemplar retained only with eligible crop/support |
| Session lifecycle | Ordinary LOST preserves enrollment; reset/clock rewind clears | Anchor/reference survival retained; measured history clears on recovery/reset, complete extent clears on LOST; temporary query disabled in final candidate |

BBox-only extent agreement is a heuristic. Opposing edges during genuine centered zoom can be judged unlearnable; a translated partial crop may resemble a full crop. Clearing an extent model after loss does not independently prove a returned crop complete. These remain design limitations, not solved general person-crop classification.

`baseline_*_evidence.csv` classifies **every accepted and rejected frame**, including static/yaw/crossing/occlusion/repeated crossing/short and long re-entry, plus the separate 120–150 s movement window. Flags cover geometry learning, motion reason, neural, HSV, overlap/border quality, confirmation, ambiguity and fresh-source availability. Raw A-ID transitions are recorded separately rather than treated as proof of identity failure. `new_flags` records temporal onsets relative to the preceding frame; simultaneous onsets remain simultaneous. Initial enrollment is explicitly unscored, not mislabelled as failed neural/motion evidence.

In accepted yaw frames the baseline geometry-learning gate is false **10/28, 13/15 and 5/14** times. Those frames emit a target but cannot refresh the affected history/reference path, explaining why later evidence can fail despite earlier acceptance. Rejected yaw frames contain neural failure **1/7, 13/21, 12/21** and vertical-motion failure **5/7, 8/21, 15/21**; counts overlap. HSV often remains strong when neural view coverage disappears. Complete temporal records, not these totals alone, support the diagnosis.

Representative archetypes (all timestamps are capture source seconds):

| Capture / time | Baseline decision | Final hypothesis decision | Evidence / interpretation |
|---|---|---|---|
| Original 65.540, yaw | Reject: motion | Pending two-frame confirmation | Baseline neural .866 survives; candidate motion now passes, but earlier uncertainty still requires confirmation |
| Final 62.472, yaw | Reject: neural/geometry | Accepted | Neural .679 → .776 with unchanged measured inputs; view/history evolution changes the decision |
| Original 66.992 → 67.256, crossing | Reject | Reject recovery | Dirty crop + neural loss precede added vertical-motion failure; HSV remains .94–.97; geometry repair cannot justify accepting it |
| Final 82.072, occlusion | Reject recovery | Reject recovery | Candidate crop dirty, neural .494 despite HSV .973; independent identity evidence is absent |
| Runtime 121.772, movement window | Reacquired | Pending recovery | Candidate neural .882/HSV .987 are sufficient, but confirmation history differs; one frame is not enough |
| Runtime 181.404, actual short return | Reacquired | Reject recovery | Clean measured crop, neural .620 despite HSV .994; source/raw-ID return exists but view coverage does not |
| Runtime 222.456, actual long return | Reacquired | Reject recovery | Clean measured crop, neural .611 despite HSV .976; old projected geometry is not the sole recovery veto |

See `archetypes.json`, `final_*_diagnostics.jsonl` and `final_*_reference_events.json` for exact flags and reference provenance. Among baseline acceptances rejected by final policy, independent neural score is below .76 in **133/116/125** frames. Other gates can fail simultaneously; these are not exclusive causal counts. H3 FIFO and redundancy replacement both fail, so changing the replacement rule alone does not establish sufficient coverage under this provenance scheme.

## Complete controlled replay

Exact inputs: original `tracking_3_people_20261014T093757Z` (1,510), final `tracking_3_people_20261014T100720Z` (1,508), runtime `tracking_3_people_20261014T144151Z` (1,467). All begin at SEARCHING. Source hashes are recorded. Actor label/fraction is stripped before guard evaluation; `person_a` is used only for evaluation. All policy trials retain the original GMC and observations, with no scenario-dependent policy branches. All-flags-off output CSVs match baseline byte-for-byte.

| Offline policy | Original total (yaw /35) | Final total (yaw /36) | Runtime total (yaw /35) |
|---|---:|---:|---:|
| Baseline/control | 406 (28) | 237 (15) | 186 (14) |
| H1 separation | 386 (28) | 224 (26) | 129 (10) |
| H2 motion eligibility | 410 (33) | 252 (21) | 98 (7) |
| H3 exemplars/provenance + redundant-pair eviction | 307 (28) | 148 (14) | 33 (6) |
| H4 temporary query | 406 (28) | 237 (15) | 186 (14) |
| H3 FIFO diagnostic | 271 (28) | 114 (10) | 33 (6) |
| Final: H1 + FIFO provenance + independent recovery | 318 (28) | 144 (16) | 66 (10) |

Every policy trial has **0 wrong-target and 0 unknown-target accepted frames** on all three captures. That measured safety does not prove that visually similar people are always distinguishable. The synthetic weak-recovery-chain test exposes a baseline failure outside those distinct-costume traces.

Final baseline → candidate scenario counts:

| Scenario | Original | Final | Runtime |
|---|---|---|---|
| Warmup | 22 → 22 /22 | — | 0 → 0 /16 |
| Static | 86 → 86 /112 | 67 → 48 /93 | 23 → 29 /114 |
| Yaw | 28 → 28 /35 | 15 → 16 /36 | 14 → 10 /35 |
| Crossing | 31 → 10 /131 | 18 → 8 /124 | 27 → 5 /123 |
| Occlusion | 20 → 17 /171 | 24 → 10 /160 | 34 → 2 /152 |
| Repeated crossing / movement stage | 76 → 37 /435 | 93 → 48 /476 | 65 → 15 /442 |
| Short exit/re-entry stage | 39 → 26 /217 | 6 → 3 /207 | 9 → 1 /199 |
| Long exit/re-entry stage | 104 → 92 /387 | 14 → 11 /412 | 14 → 4 /386 |

Final yaw **28/35, 16/36, 10/35** misses required **33/35, 22/36, 14/35**. Totals **318/144/66** miss **406/237/186**; multiple scenario counts regress independently. Original F/G are stage labels in the older world where A did not actually leave; actual selected-person re-entry and horizontal movement are assessed in final/runtime captures.

Continuity, baseline → final candidate:

| Metric | Original | Final | Runtime |
|---|---|---|---|
| LOST duration (s) | 33.592 → 80.844 | 76.636 → 121.108 | 83.028 → 143.848 |
| LOST recoveries | 24 → 24 | 25 → 14 | 21 → 9 |
| Longest accepted gap (s) | 7.328 → 13.992 | 40.524 → 70.620 | 39.304 → 72.700 |
| Total visible interruption (s) | 142.792 → 151.956 | 120.564 → 131.052 | 123.944 → 137.904 |
| Longest visible interruption (s) | 7.160 → 13.760 | 9.536 → 15.544 | 9.968 → 26.532 |
| Accepted in 120–150 s window | 41 → 21 | 45 → 26 | 25 → 4 |
| Visible-frame accepted coverage | 26.9% → 21.1% | 20.3% → 12.3% | 17.0% → 6.0% |

Visible denominators: 1,509/1,168/1,091. Visibility is the saved independent costume detector, not exact person-presence ground truth; full occlusion/tiny shirts can be missed. Visible interruption integrates the preceding frame's visibility/abstention over each source interval, resetting at accepted output or observed absence, with no unobserved tail. LOST uses the replay helper's current-sample interval convention. Accepted gaps include actual absence. No arbitrary controller-ready continuity threshold is introduced.

## Singular GMC correction, independently reviewed

Reproduced **4 FAIL / 3 PASS** before the fix, **7/7 PASS** after: zero/singular/rank-deficient/reflection, NaN/malformed matrices, empty-frame loss/recovery, and valid camera translation. These tests use native BoT-SORT with deterministic test embeddings; they are not CUDA Re-ID inference tests. Existing native adapter tests also pass. The only retained production source change is the float64 determinant check in `botsort_adapter.py::_apply_gmc`; invalid matrices raise into the existing `_pre_first_associate` identity fallback. `last_gmc_warp` and fallback counters agree with the native fallback. No ORB fitting or association thresholds change.

A labelled **guard-only validity control** substitutes identity for 7/7/9 invalid saved matrices. It yields totals **404/232/158** and yaw **28/15/17**, with 0 wrong/unknown. It is not included in policy acceptance. It does not regenerate Kalman means, covariance or raw IDs, which would differ when the real adapter rejects the original bad transform. Thus this control establishes sensitivity to those saved matrices, not a full-system improvement claim. The independent validity fix is retained as requested; its complete runtime effect remains NOT RUN.

## Tests, environment and final review

- **PASS, candidate offline:** both known reduced failures; weak recovery cannot confirm an unsupported new-ID chain; immutable history under rotation/inverse; overlap motion without appearance learning; known partial crop does not become teachable merely with time; source/query interruption, anchor/budget/reset; existing crossing/reacquisition/watchdog integration. Full non-GPU suite: **175 PASS / 1 skipped**.
- **PASS, H4 only:** temporary bridge does not teach or renew itself; source-time expiry. H4 nevertheless provides no complete-trace improvement and is excluded from final candidate.
- **FAIL, retained baseline:** both known reductions and the new synthetic weak-recovery-chain test. Final policy was never copied into production; exact baseline source hash is still `e7c5bd4affab7cde691cf7b7dc4e6561b876ffb42c58e10f27ca75f5aa955b49`.
- **PASS, retained production non-GPU suite:** **163 PASS / 1 skipped**, including seven new GMC cases. Existing adapter tests are unchanged. Perception build, dependency check and runner syntax PASS. Installed guard/adapter bytes match their retained sources.
- **FAIL / environment unavailable, CUDA smoke:** explicitly attempted before and after environment restoration; NVML fails, PyTorch reports no CUDA device. ORT GPU providers are installed, but model initialization cannot activate CUDA. A green non-GPU suite is not reported as full CUDA verification.
- **NOT RUN:** full simulation, new pipeline latency/FPS, hardware flight, body-following. Replay failure independently prevents simulator promotion. Old runtime FPS/latency are not reused as this candidate's measurements.

The first GPU test left Ultralytics auto-install enabled and it unexpectedly installed `onnxruntime` CPU 1.31.0 over 23 shared GPU-package files. This side effect was removed; the original `onnxruntime-gpu==1.30.0` was restored without dependency updates, CPU distribution removed, and **327 GPU package RECORD hashes** verified. The temporary restoration wheel was removed after verification. PyTorch 2.14.0+cu130 / Ultralytics 8.4.164 remain installed; device availability is still false. All subsequent tests set `YOLO_AUTOINSTALL=false`. Evidence is in `environment_restore.log`, `environment_restored.json` and CUDA logs. No remaining external dependency change is claimed or intended.

Two test-audit corrections are recorded: query tests must allow independently supported continuation with a distant competitor or after a short PREDICTING/watchdog interval; they now assert that the temporary query cannot provide the evidence. The initial full candidate invocation selected focused tests twice by combining their parent directory and explicit path; the final invocation uses `evaluation/test_*.py` and records 176 unique cases. Neither correction changes policy or source-time behavior. Baseline taxonomy also distinguishes initial unscored enrollment from failed scores; initial diagnostic files are preserved.

Independent final-review findings:

| Severity | Finding / disposition |
|---|---|
| BLOCKER | Every complete capture fails yaw/total and multiple scenario floors; final composition rejected, no post-review tuning |
| HIGH | Provenance prevents the demonstrated weak-reference chain but leaves insufficient useful independent view coverage; conservative matching cannot be justified as continuity success |
| HIGH | Bbox-only completeness still cannot certify all partial/pose/zoom cases; no general identity-safety claim |
| HIGH | CUDA device unavailable; required GPU runtime check is not PASS |
| PASS (component) | Singular-GMC rejection is separate, native fallback tested, no model/interface/ORB fitting changes |
| PASS (scope/state) | Actor metadata excluded; all output boxes are fresh measurements; anchor/budget/reset/LOST and watchdog checks exercised; guard production bytes unchanged |

Retained files: `src/uav_perception/uav_perception/botsort_adapter.py`, new `src/uav_perception/test/test_gmc_validity.py`, module README, `PROGRESS.md`, this report and ignored experiment evidence. Existing user changes and all earlier reports/captures remain preserved. `production_state.json` records source/installed-byte equality and environment restoration. No commit, delegation, worker or orchestrator.

## Reproduction and evidence

All task-local scripts, fixed candidate sources, diffs, temporal CSVs, reference-event diagnostics, before/after tests and complete replay outputs reside in `evaluation/results/runs/target_guard_redesign_20261015`. `final_review.json` records every failing floor. Run replay scripts only into a fresh experiment directory; they refuse existing replay output directories.

```bash
source /opt/ros/jazzy/setup.bash
source ../software/ros2_ws/install/setup.bash
# Retained production; enable GPU smoke only when the device is usable.
PYTHONDONTWRITEBYTECODE=1 YOLO_AUTOINSTALL=false RUN_BOTSORT_REID_RUNTIME=0 \
  PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" \
  ../software/venvs/yolo26/bin/python -m pytest -q -p no:cacheprovider \
  src/uav_perception/test evaluation
# Test frozen offline candidate without copying it into production.
PYTHONDONTWRITEBYTECODE=1 YOLO_AUTOINSTALL=false RUN_BOTSORT_REID_RUNTIME=0 \
  ../software/venvs/yolo26/bin/python \
  evaluation/results/runs/target_guard_redesign_20261015/run_tests.py final_candidate \
  src/uav_perception/test evaluation/test_*.py \
  evaluation/results/runs/target_guard_redesign_20261015/candidate_tests.py
colcon build --base-paths src --packages-select uav_perception
../software/venvs/yolo26/bin/python -m pip check
bash -n simulation/run_tracking_3_people_test.sh
```

**Final acceptance: FAIL.** The single final composition fixes the two known examples and a weak-recovery chain, but does not jointly meet yaw, continuity and complete verification requirements. Production identity policy stays at the verified baseline. The independently tested GMC validity correction is retained; full-system tracking acceptance remains unresolved.
