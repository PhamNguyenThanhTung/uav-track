# Project Progress

Production is configured as YOLO26n → BoT-SORT + ORB GMC + pretrained CUDA ONNX Re-ID → TargetIdentityManager → gimbal. **Tracking acceptance remains FAIL.** The latest retained correction rejects results that expire during detector/tracker processing, keeps ROS source time updating during inference, and preserves GMC across rejected frames without identity learning. GPU access is restored and verified. Final suite: **175 PASS**, including CUDA and actual ROS timing integration. Full simulation completes and lands, but only **11/339 recorded guard events** accept a target. No body-following is implemented.

## Latest source-age correction and runtime — 2026-10-15

- Reproduced late-inference output: three component regressions failed before the fix. Actual ROS delivery also failed both advancing-clock cases because inference occupied the same callback group as `/clock`; paused-clock controls passed.
- Added an age-only recheck after detector/tracker work using the existing 0.75 s source-clock budget. Expired results emit no target bbox, appearance learning or new gimbal command. Two executor threads keep `/clock` responsive while images, reset and watchdog remain serialized. Diagnostic JPEG generation follows controller output.
- Late tracker frames still project historical motion with their GMC. The optional watchdog warp does not change the last real source timestamp or identity decision rules. SIGINT/SIGTERM drain workers before ROS teardown.
- **175 tests PASS** on the final source, including real CUDA Re-ID, delayed detector/tracker ROS delivery, paused clock, source resumption, clock rewind and non-learning GMC projection. Build, dependency check, syntax and whitespace checks PASS.
- Full final capture `tracking_3_people_20261015T065057Z`: **339 guard events, 11 accepted, zero wrong/unknown accepted labels**, source interval 46.156–235.028 s; clean landing/shutdown. Yaw **0/7**, crossing/occlusion/re-entry zero acceptance. Observed joined rate **1.23 frames/s**, processing p95 **780.1 ms**. These results **FAIL continuity**; joined FPS is not camera FPS. No new measured-joint closed-loop gimbal acceptance is claimed.
- Historical and retained policies replay the new inputs identically, with zero recorded output mismatches. Canonical replay outputs also stay identical (406/237/186 accepted; yaw 28/15/14). The timing correction does not solve the identity policy.
- Separately tested a corner-preserving affine-history candidate: geometry checks pass, but complete replay totals **413/165/179** and yaw **28/12/14** regress scenario/continuity gates. Candidate remains offline; no threshold search or promotion.
- Three known crop/GMC/weak-recovery reductions still FAIL retained production. Keep these and the new latency evidence as blockers before following. No dependency installation, body-following, worker or commit.

See [source-age verification and motion projection report](docs/tracking/source_age_verification_2026-10-15.md) for exact before/after evidence, final runtime limitations and commands.

## Latest redesign review — 2026-10-15

- Re-audited actual detector/adapter/guard/controller paths and all three complete captures; baseline counts and all-flags-off output equivalence match exactly. Frame evidence distinguishes geometry learning, motion, neural/HSV, crop/competition, confirmation and raw-ID/source changes.
- Tested H1–H4 separately, one H3 FIFO diagnostic control, then one final H1 + measured-exemplar/provenance composition. H2 failed partial-crop velocity tests; H4 supplied no trace benefit. No policy tuning followed final review.
- Final offline yaw **28/35, 16/36, 10/35** and totals **318/1510, 144/1508, 66/1467** FAIL the required floors; crossing/occlusion/re-entry also regress. All recorded accepted labels remain correct (zero wrong/unknown). Both known reductions and the added weak-recovery-chain test pass in the candidate; they still fail on retained baseline.
- Candidate non-GPU suite **175 PASS / 1 skipped**; retained production **163 PASS / 1 skipped**, including seven new native-BoT-SORT GMC cases. Build, dependencies and runner syntax PASS. CUDA smoke was attempted and FAILS because the device is unavailable; it is not counted as PASS.
- Ultralytics unexpectedly auto-installed CPU ONNX Runtime during the first GPU test. That side effect was undone: CPU distribution removed, original GPU 1.30.0 restored, all 327 package RECORD hashes verified. Subsequent checks disable auto-install. GPU availability remains an environment limitation.
- Retained changes: adapter determinant validation, seven regression cases, module documentation, this progress update and [redesign report](docs/tracking/target_guard_redesign_report.md). Guard/scenario/controller/watchdog behavior is unchanged. No delegation, orchestrator, body-following or commit.

## Previous geometry and appearance review — 2026-10-14

- Root verification confirms measured detector boxes are fresh, but the guard reuses GMC-transformed history for crop quality, motion and reference maintenance. Both reduced failures reproduce. The three captures also contain 7/7/9 singular GMC matrices accepted by baseline adapter validation; no stale observations, timestamp reversal or watchdog events occur in those traces.
- Initial candidate resolves both reductions and passes 66 adjacent tests, but complete acceptance totals regress to 396/188/115 versus baseline 406/237/186. One feedback round adds observed reference medoids, confidence provenance and separate motion proxies; **174 candidate tests PASS**, including real CUDA Re-ID and invalid-GMC BoT-SORT integration.
- Final candidate still FAILS: original/final/new-runtime yaw is **30/35, 12/36, 8/35**, total acceptance **281/1510, 90/1508, 69/1467**, with crossing/occlusion/re-entry regressions. Wrong/unknown acceptance remains zero; continuity does not meet acceptance.
- Stopped after that one feedback round. Guard, adapter and adapter tests match saved baseline bytes; installed perception sources also match. Restored production suite **157 PASS**, build/dependency/runner syntax PASS. Both reduced defects still FAIL on restored production.
- Only the new report, progress links and ignored review evidence remain from this task. No simulator/scenario/interface/body-following change, delegation, orchestrator or commit was introduced.

See [geometry and appearance robustness report](docs/tracking/geometry_appearance_robustness_report.md) for all scenario counts, visible-target interruption metrics, rejected sources/tests and exact verification commands. The yaw investigation below is preceding historical evidence.

## Current verified baseline

- Custom X500, 3-DOF gimbal, 1280x720 RGB camera; ROS 2 Jazzy/Gazebo/PX4 SITL under WSL2.
- Fresh measured accepted targets alone reach controller-facing detections. Identity ambiguity and pending recovery emit no target bbox; explicit reset permits new selection.
- Enrollment anchors remain protected, appearance memory bounded, and CUDA ONNX Re-ID/ORB GMC required at startup.
- Production flight control remains independent Offboard position/altitude hold. No autonomous horizontal body-following exists.
- Prior 2026-10-14 baseline: **157 tests PASS**. Current final suite: **175 PASS**, including real CUDA Re-ID and actual ROS source-clock timing checks. Perception build, dependency check and runner syntax PASS. Identity-policy acceptance is still FAIL.
- Prior reacquisition acceptance capture: 1,508 processed updates, 25 correct LOST recoveries, zero wrong/unknown acceptance, actual short/long selected-A re-entry, 0.620 m horizontal movement and clean landing. These remain historical evidence, not a PASS for continuity.

## Latest yaw investigation

- Exact replay confirms the extent guard compares detector geometry with cumulatively GMC-warped history, freezing trusted history/reference refresh and rejecting correct yaw candidates. All twelve lost-acceptance frames are traced individually.
- Candidate yaw reaches **33/35 original and 25/36 final**; feedback reaches 35/35 and 25/36. Both meet the prescribed yaw floors on those captures.
- Candidate still regresses original crossing **31→21/131** and long-stage coverage **104→82/387**. On a new complete runtime trace, baseline versus candidate acceptance is **186→112/1,467**, crossing **27→8/123**, occlusion **34→4/152**.
- One feedback trial leaves original crossing at **16/131** and new-runtime yaw at **9/35 versus baseline 14/35**. It fails acceptance and is not promoted. No further policy tuning follows.
- Candidate has **171 passing tests**, but runtime evidence rejects it. Candidate source/diffs/tests/replays are retained as ignored review artifacts; production policy changes are rolled back byte-for-byte.
- Two newly reduced GMC/crop counterexamples still FAIL on the restored baseline. Existing-suite success and distinct-shirt zero wrong-target observations do not establish complete identity safety.

See the current [camera yaw regression report](docs/tracking/camera_yaw_regression_report.md), including per-frame causes, full scenario comparisons, exact commands and final-state limitations. The [target reacquisition report](docs/tracking/target_reacquisition_report.md) and [migration report](docs/tracking/botsort_migration_report.md) preserve preceding task evidence.

## Runtime and retained runner fix

- A full candidate run after watchdog correction captures **48.808–235.952 s**, 1,467 processed guard updates, 15 correct LOST recoveries, zero wrong/unknown accepted identities and clean landing/shutdown.
- Selected A recovers after short re-entry at **181.404 s** and long re-entry at **222.456 s**. Actual lateral displacement is **0.624 m**, altitude 2.985–3.048 m.
- Joined observer: 5.09 observed frames/s, processing p95 225.5 ms. Candidate accepted coverage is only **7.6%**; same-input pre-task guard replay reaches 12.7%.
- First attempted run starts at 62.472 s and misses static/most yaw; retained but excluded from scenario acceptance. The runner now pauses source simulation time after hover during model startup, then resumes before recording the scenarios. Actor paths/source-time intervals and flight/ROS contracts are unchanged.
- The complete candidate simulator run contains no watchdog events; actual resumed-stream behavior remains node-integration evidence. Full simulation after rollback was not repeated. Exact replay and source-byte checks verify restoration to the previously verified guard.

## Remaining work before body-following

1. Production yaw remains **28/35 and 15/36** on the two canonical captures, below the required 33/35 and 22/36 floors.
2. Find a crop/ego-motion and view-memory policy that preserves identity safety and improves all scenario comparisons, including the newly reduced counterexamples and latest trace. Do not accept a yaw-only improvement.
3. Pre-register visible-target coverage/interruption budgets from the required control update rate; current simulation does not establish continuous following or a sufficient update rate. Do not invent a threshold to label these results PASS.
4. The latest complete runtime has only 11/339 accepted updates and processing p95 780.1 ms; investigate latency with a paired control and preserve the source-age gate. Similar-looking people and real-world identity safety remain unverified. No target 3D estimator, motion arbiter, horizontal following, search behavior or complete TF tree exists.

## Last updated

2026-10-15. **Source-age correction verified; tracking continuity FAIL.** Rejected identity-policy and corner-projection candidates remain offline. Current source/installed hashes match; final suite 175 PASS including CUDA, simulator landed cleanly. GMC validity and late-result projection retained independently of identity policy. No worker, orchestrator or commit.
