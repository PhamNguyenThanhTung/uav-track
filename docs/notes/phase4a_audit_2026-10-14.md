# Phase 4A audit — 2026-10-14

**Verdict: CHANGES REQUIRED.** Current source is newer than the final6 report. The D3 motion-history fix already exists and has passing development replay evidence; independent acceptance and live freshness remain incomplete. This audit implements no features and dispatches no workers.

## 1. Current State

- Git root is `project`, HEAD `dfad9dd`, branch `master`; substantial tracked and untracked changes predate this audit. The attached `perception-fresh-observation` worktree is an older, different candidate at the same HEAD, not an accepted version of the present guard. Preserve both.
- Production source really calls **YOLO → DeepSORT → TargetIdentityManager → GimbalController → ROS joint-position commands**: [node](../../src/uav_perception/uav_perception/yolo_detector.py#L326), [manager](../../src/uav_perception/uav_perception/target_identity.py#L162), [controller](../../src/uav_perception/uav_perception/gimbal_controller.py#L65). Confirmed tracks must have `time_since_update == 0`; feature and bbox use the original detection. Guard rejection emits `bbox=None`, empty target output, and no new tracking command. `LOST` requires explicit reset. Initial selection is the first fresh, clean candidate with an embedding; no manual person picker exists or is required by the current scope.
- Logical target identity remains separate from tracker ID. Every candidate, including the current TID, passes appearance/motion validation; reassignment and recovery from uncertainty require two fresh observations. Source stamps govern motion and the node's 2.5-second timeout. Duplicate/backward stamps are covered by existing tests. The initial neural anchor and optional HSV anchor are retained; later neural references have stricter update rules.
- HSV reuses DeepSORT's existing crops and MobileNet embeddings; no second neural model is introduced. `evaluation/tracking_identity_candidate.py` and `tracking_appearance.py` remain offline experiments, not the production manager.
- **The asymmetric bbox-extent guard already exists** at [target_identity.py:511](../../src/uav_perception/uav_perception/target_identity.py#L511), with top/bottom truncation and coherent-translation tests at [test_target_identity.py:737](../../src/uav_perception/test/test_target_identity.py#L737). Current manager SHA256 is `7ab6931f70d1e140a7255a53f45e3334dea027b156aea45c153766116bc7dc2f`; it matches the motion-extent neural replay manifest, and differs from final6.

I recomputed both existing acceptance reports with the current evaluator; JSON results exactly match the saved reports:

| Evidence read and recomputed | final6 | Current motion-extent candidate |
|---|---:|---:|
| Actual neural replay frames previously recorded | 3,166 | 1,330 |
| Correct / clear labeled frames | 127/176 (72.16%) | 137/176 (77.84%) |
| Wrong target bbox confirmations in 215 labeled frames | 0 | 0 |
| Confirmations on 39 ambiguous frames | 0 | 0 |
| D3 correct before / after crossing | 11 / 0 | 11 / 11 |
| Validated raw target-TID changes | 7 | 8 |
| Saved evaluator gate | INCONCLUSIVE | PASS, **development regression only** |

Sources: [final6 acceptance](../../evaluation/results/runs/phase4a_target_guard_20261013_final6/acceptance_v2.json), [current regression](../../evaluation/results/runs/phase4a_dev_motion_extent_neural_20261013_v1/regression_v2.json), [matching current source manifest](../../evaluation/results/runs/phase4a_dev_motion_extent_neural_20261013_v1/after_manifest.json).

D3's role in the CSV remains `held_out`, although the fix was designed using D3. Its PASS cannot establish independent acceptance. B/C/D1/D2/D3 are now development evidence. The [D4/D5/D6 protocol](../../evaluation/annotations/raw_holdout_review_20261013_v3/protocol.txt) exists, but no completed corresponding CSV/results were found. `PROGRESS.md`, perception README and final review still describe the older final6 failure; preserve historical reports and update current-status documents in the implementation handoff.

## 2. Gap Analysis

| Classification | Finding and evidence |
|---|---|
| Implemented with component/replay evidence | Appearance, HSV, same-TID identity validation, two-observation confirmation, abstention, bounded manager references, source-time motion. Current D3 development recovery is supported by matching source hashes. |
| HIGH — reproduced current bug | Crop quality is inferred from detection overlap; extent protection runs only when `not clean_crop`. A truncated target with no neighboring detection is treated as clean, enters motion history **and** can grow the reference gallery. Full-body return is then rejected despite unchanged identity features. See reproducer below and [quality calculation](../../src/uav_perception/uav_perception/target_identity.py#L338), [history/reference update](../../src/uav_perception/uav_perception/target_identity.py#L511). |
| HIGH — reproduced freshness gap | Increasing source stamps are accepted without comparing capture time to a compatible live clock or checking age at command publication. QoS depth 1 bounds queue length, not observation age. [process_frame](../../src/uav_perception/uav_perception/yolo_detector.py#L275) and [gimbal publishing](../../src/uav_perception/uav_perception/yolo_detector.py#L381) have no age gate. The [bridge configuration](../../config/sim/ros_gz_bridge.yaml) contains no `/clock` bridge. Never subtract wall epoch from Gazebo source seconds. |
| Implemented minimum hold; status unverified on stream loss | With no callbacks, no new tracking commands are generated, and Gazebo position controllers retain the last angle setpoint. Manager status nevertheless remains TRACKING because only image updates expire it; the node's timer only logs statistics. Physical hold behavior under an injected dropout has not been tested for this candidate. A lightweight monotonic freshness/watchdog contract belongs in live verification; no search behavior is needed. |
| Unverified risk, not proven physical ID hijack | Manager references are guarded, but DeepSORT's own gallery is updated before manager validation. Installed `deep_sort/track.py:248` appends detection features; `deep_sort/tracker.py:118–126` trains the metric. Guard veto cannot undo that update. Instrument it before deciding whether a tracker hook is necessary; do not patch external dependencies speculatively. |
| Operational mismatch | Installed `target_identity.py` and `yolo_detector.py` differ from source. Replay prepends source to `sys.path`; manual README launch loads `install/`. `launch_phase4a_review.sh` builds first, so that launcher can resolve the mismatch. An old installation alone does not prove a live session ran old code; log actual import paths/hashes before verification. |
| MEDIUM — evaluation semantics | [metric_calculator.py:416](../../evaluation/metric_calculator.py#L416) still exports `detector_misses = tracking_misses`; `id_switches` counts raw published TID changes. Raw no-person frames are not target-specific detector misses. [Acceptance gate](../../evaluation/evaluate_phase4a_target_guard.py#L173) requires at least one correct confirmation before/after, but has no coverage/recovery deadline, and permits omitting competing-detection checks. Existing reports did supply those detections. |
| Outside Phase 4A | Camera-motion compensation, per-second gimbal control, long-term recovery, target 3D state, UAV following and flight arbitration. Gimbal limits currently constrain radians per processed update, not radians/second. |

### New checks actually run in this audit

- **PASS:** `130` repository tests, `12.07 s`; three protobuf deprecation warnings. Command from the Git root:

  ```bash
  source /opt/ros/jazzy/setup.bash
  source ../software/ros2_ws/install/setup.bash
  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PYTHONPATH:src/uav_perception:evaluation" ../software/venvs/yolo26/bin/python -m pytest -q -p no:cacheprovider src/uav_perception/test evaluation src/uav_control/test
  git diff --check
  ```

- **PASS:** whitespace check; saved annotation/profile/evaluator hashes; all production/dependency files recorded in the current replay manifest match present files. Capture timestamps: 3,166 finite source times, 36.964–234.204 s, no non-increasing steps. These checks do not establish absolute live observation age.
- **PASS:** reevaluated saved final6/current CSVs using `evaluate_phase4a_target_guard.py`, reviewed v2 annotations, original anonymous detections, and their respective `after_profile_frames.csv`; no output file was overwritten.
- **BUG REPRODUCED, component level:** reused existing `MockTrack`, manager timeout 2.5 s. Six clean target observations at 10.0–10.5 s use bbox `[100+5i,100,200+5i,400]`. Single-person partial boxes at 10.6/10.7 s are `[130,100,230,370]` and `[135,100,235,340]`, with identical neural/HSV features. Both are accepted; references increase to 7 then 8 and partial geometry enters history. Full boxes with new TID at 11.4/11.5 s (`[170,100,270,400]`, `[175,100,275,400]`, horizontal Kalman direction) produce UNCERTAIN/no bbox; at 13.3 s terminal LOST. This demonstrates residual truncation failure, not a proven wrong-person switch in Gazebo.
- **FRESHNESS GAP REPRODUCED, node harness:** existing production `process_frame` fixture receives source stamps 10.0/10.1 s while an injected node clock reports 1,000 s. Target remains TRACKING and controller is called twice. The clock is never consulted. This establishes absence of age validation; the live clock-domain mapping still needs runtime verification.
- **NOT RUN:** new neural/GPU replay, package build, Gazebo/PX4 live run, gimbal feedback or new capture. Neural replay and timing in the table are historical artifacts, checked against source and reevaluated here; unit tests are not end-to-end acceptance.

Benchmark interpretation: manual target ownership comes from raw video plus anonymous detector boxes, not tracker/PID labels. It is still detector-conditioned geometry, not independent full-frame target visibility/box GT. It cannot estimate target detector recall where YOLO emitted no target box. Wrong-person events, localization mismatch, abstention and TID changes must remain distinct; a constant logical ID `1` proves none of them. Gimbal-call counts establish software gating, not actual joint response or centering. Live benchmark currently subscribes to command topics, not measured gimbal joints.

Current 1,330-frame replay measured mean processing 59.75 ms, p95 72.45 ms, throughput 16.74 FPS. Its plain control has only **10 measured frames after warmup**; it is unsuitable for a robust performance comparison. Historical final6's full plain control measured 16.59 FPS. Neither figure is full-stack capture-to-command FPS/age, and neither justifies replacing DeepSORT.

## 3. Critical Blockers

1. Close the reproduced single-person truncation/history/reference hole and prevent stale live images from producing tracking commands.
2. Obtain genuinely unused acceptance data, fix development/holdout provenance, and lock coverage/recovery criteria so minimal occasional confirmation cannot pass.
3. Verify the rebuilt production package with actual ROS/Gazebo delivery, command inhibition and measured joint hold. Current replay alone does not close this integration gate.

The historical D3 fix is already present; do not assign it as an unimplemented feature. Tracker gallery contamination remains an instrument-and-decide risk, not grounds for a wholesale tracker replacement.

## 4. Implementation Plan — proposed, not dispatched

| Packet / owner | Smallest deliverable | Files and dependency |
|---|---|---|
| P0 — Codex baseline/spec | Freeze current tracked **and untracked** source hashes, input/label manifests and thresholds. Mark D3 development. Record actual package imports. | Current contracts and evidence; first. Do not use clean HEAD as the handoff snapshot. |
| P1 — Antigravity, one production writer | Extend existing crop/extent trust to the reproduced isolated truncation case; prevent untrusted history/reference writes. Add regressions for top/bottom clipping without a neighboring detection and retain coherent translation/clear-view behavior. Specify same-clock live age gate at ingress and before command publication; monotonic stream freshness, explicit replay behavior and no new tracking commands on invalid input. | `target_identity.py`, `yolo_detector.py`, relevant existing perception tests; minimal bridge/launcher changes only for an explicit simulation clock contract. Depends P0. Do not globally relax motion gates or timeout. |
| P2 — bounded evaluation/label packet | Audit target-specific failure taxonomy; make roles/ownership/coverage/recovery eligibility explicit, preserving legacy metric compatibility. Raw-only label unused crossing/dropout/partial-body scenarios, including visible target with no YOLO target detection. User adjudicates uncertain ownership. | Existing `evaluation/` modules/tests and new annotation revision; can run alongside P1 after P0. Labeler does not inspect guard outputs. One owner per shared CSV/evaluator file. |
| P3 — Codex verification/review | Freeze P1/P2 sources, replay full capture and new holdout, paired controls of equal length; build/import/hash check; run stationary-camera identity integration and dropout/latency injection in Gazebo with position hold. Review raw frames, decisions, commands and measured joints. | New uniquely named evidence directory, no overwrites. Depends P1/P2. Update current progress/README/contracts to the tested snapshot; retain historical reports. |

Contracts to preserve: source seconds for motion/identity timeout; separate monotonic receipt age; raw TID may change while logical identity remains the selected person; confirmed original detection bbox or `None`; untrusted observations never drive the gimbal; position commands stay radians; no target input to flight control in 4A. Define behavior on clock reset and simulation pause explicitly. Keep bbox output in observed image coordinates; do not fabricate full-body boxes.

Parallel work is limited to independent labeling/evaluator review while a single writer owns guard/node. Runtime on a shared simulator and GPU benchmarks run sequentially after source freeze. No worker starts in this audit. After implementation final review, allow exactly one Antigravity correction round, review again, and report any remaining failure as requested.

## 5. Acceptance Criteria

Mandatory gates:

- Matching source/package/weights/config/input hashes; frozen raw-only holdout not used for tuning. D3 is regression. Include a fresh capture to avoid treating same-capture event separation as capture independence; user independently reviews ownership labels.
- Zero observed wrong-person confirmations and zero commands based on ambiguous/stale identity in **all labeled acceptance windows**; competing detection ownership checked with current IoU ≥0.5 and margin ≥0.05. Separate localization errors from physical person-switch events. No claim of universal accuracy from finite observations.
- Correct target before and after every crossing; no terminal LOST while a clear, confidently identifiable target returns within the short-term window. Same-TID hijack, new-TID return, no-feature input, overlap, isolated clipping, duplicate/backward stamps and full stream loss all covered. Tracker IDs for non-targets need not stay stable.
- Untrusted crops do not update logical-target anchors/gallery or trusted motion. Inspect DeepSORT gallery separately; add a targeted hook only if evidence establishes the need.
- Rejected/expired input emits no new tracking command; actual Gazebo joints hold the existing position setpoint within recorded tolerance. Startup/explicit initialization commands are distinguished from bbox-driven tracking commands.
- Relevant units/regressions, full replay and live stationary-camera integration pass; required checks cannot remain NOT RUN.

**Proposed numeric gates, not yet agreed:** correct target coverage ≥75% of clear target frames overall and ≥50% in each crossing; reacquire within 0.5 source seconds after reliable visibility returns, using two fresh consistent observations; full-stack processing ≥15 wall FPS and p95 capture-to-command age ≤200 ms in the specified load envelope. Report both all-visible and detector-present target coverage, and worst abstention duration. Lock these before opening new holdout outputs; do not tune them afterward to obtain PASS.

## 6. Next Phase Readiness

Advance to 4B only after the above 4A gates pass. The requested sequence fits the existing architecture:

- **4B:** gimbal control with explicit dt/rad/s, measured joint/camera pose, FOV behavior and camera-motion compensation justified by paired data. Keep UAV XY hold. Basic 4A freshness/command inhibition cannot be deferred to 4B.
- **5:** target-relative geometry/3D state and timestamped transforms, then bounded following/PX4 integration and a single setpoint authority. Resolve PX4 status/failsafe schema compatibility before relying on ROS feedback for flight acceptance. Reuse the current packages unless concrete boundaries require otherwise.
- **6:** disturbances, harder occlusion and sustained-loss behavior; minimum stale-target hold/failsafe must accompany Phase 5 from its first integration rather than wait for Phase 6.
- **7:** broaden end-to-end scenarios and regression benchmarks. Keep smaller end-to-end gates at every preceding phase.

Physical cameras, GPS, gimbals and UAV hardware remain outside this roadmap.

## 7. Open Decisions

Only numeric service-quality limits require user confirmation: coverage/per-event minimum and recovery deadline; full-stack FPS and p95 observation-age budget (proposals above). Independent review remains the user's existing selected responsibility. No decision to replace DeepSORT, add a package/framework, or start workers is needed now. **Stop here for the user's plan review.**
