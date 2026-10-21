# Perception concurrency and ReID experiments — 2026-10-16

Only lazy annotation is retained as a runtime optimization. The separate perception worker, diagnostics worker and CUDA I/O binding did not show repeatable improvement in the required ROS metrics and were rolled back. Clock scheduling contention is measurable, but production clock frequency and executor architecture are unchanged. Identity continuity remains a separate acceptance gate.

## Evidence and protocol

All new evidence is under `evaluation/results/runs/concurrency_reid_20261016T075123Z/` ([directory](../../evaluation/results/runs/concurrency_reid_20261016T075123Z)). `manifest_before.json`, `working_tree_before.diff`, `original_source/` and `baseline_source/` preserve the starting state. `candidates_source/` preserves H2; `rejected_diagnostics_reid_source/` preserves the rejected H3/H4 implementation. No commit or delegation was used.

The controlled publisher reuses saved frame **400** of `evaluation/results/runs/tracking_3_people_20261009T084018Z/raw_camera.mp4`, RGB8 **1280×720**, three people, at **30 Hz**. Source timestamps follow a monotonic clock with an offset of 100 seconds. Baseline is the existing large Fast DDS profile, BEST_EFFORT KEEP_LAST depth 1, YOLO26n CUDA Graph and CUDA ONNX ReID, **FP32**, imgsz 640 (native detector tensor 384×640). Model weights, camera dimensions, BoT-SORT/ORB, guard thresholds, controller and scenario are unchanged.

Runs are sequential, never concurrent GPU workloads. Each ROS trial excludes ten completed warmup frames and then measures about 32 seconds. `controlled_case.py` records command arguments, publisher events, actual source-to-output **wall** age, raw ROS arrivals, completed inference events, native source-clock ages, stage timers, process/thread CPU, RSS and GPU samples. `performance.jsonl` records every observed processed source stamp; `processed_source_stamps.json` and `all_metrics.csv` provide ordered stamps and p50/p95/max metrics, including one-second FPS distributions. Aggregate FPS is completed frames divided by measured wall duration, never HUD/display FPS. Raw observer arrival rate and actual perception image callback rate are separate quantities.

The first 50 Hz trial overlaps a brief focused CPU-only test run and is not used alone as causal evidence; its clean repeat gives the same conclusion. A broad pytest collection initially encountered duplicate test modules in ignored source snapshots; the corrected command enumerates the actual evaluation test files and passes. No performance acceptance is based on that failed collection.

Full simulations use `target_reacquisition_test`, unchanged yaw/altitude schedule, guard debug enabled, no gimbal tracking, no lateral test, clean descent and shutdown. The original full baseline had an annotated-image observer. A second before/after pair uses **no annotated observer in either run**, because lazy rendering benefits that specific workload. Different simulation trajectories are observational corroboration; only the repeated fixed-image experiments establish causal runtime benefit.

## Required experiment summary

Times are milliseconds. Callback is the instrumented image callback work through completion/performance emission; early trials without `image_callback_ms` use `processing_ms` (within approximately 0.2 ms of the instrumented callback span). H2's tiny submission callback does **not** replace its 135.47 ms processing p95. Lazy rows are headless; other rows have an image subscriber.

| Experiment | AI FPS | Decision p95 | Callback p95 | ReID p95 | Tracker p95 | Verdict |
|---|---:|---:|---:|---:|---:|---|
| Baseline | 11.07 | 111.87 | 119.20 | 32.03 | 46.36 | Reference, viewer present |
| Clock test — 50 Hz repeat | 13.86 | 86.00 | 86.40 | 27.35 | 37.06 | Scheduling effect; production clock unchanged |
| Latest-frame worker | 8.77 | 153.77 | 0.09 | 41.62 | 54.77 | Reject: completed FPS and age regress |
| Diagnostics optimization — lazy | 15.18 | 108.96 | 81.90 | 31.61 | 44.10 | Retain for no-subscriber annotation |
| ReID candidate R3 | 10.61 | 118.49 | 121.81 | 30.23 | 42.83 | Reject: no repeatable ROS benefit |
| Final combination | 15.18 | 108.96 | 81.90 | 31.61 | 44.10 | Lazy only; all other candidates off |

The final combination contains no additional optimization beyond lazy annotation. This table must not be read as a viewer-on 11.07→15.18 FPS comparison: the matching headless baseline is 11.59 FPS (112.44 ms decision p95).

### All controlled trials

| Trial | AI FPS | Decision p95 | Callback p95 | ReID p95 | Tracker p95 | Context |
|---|---:|---:|---:|---:|---:|---|
| clock250_b1 | 10.93 | 130.54 | 129.09 | 35.14 | 47.01 | Viewer subscriber |
| clock50 | 13.79 | 81.55 | 87.98 | 27.29 | 38.36 | Viewer subscriber |
| clock100 | 12.37 | 96.06 | 97.86 | 28.83 | 39.63 | Viewer subscriber |
| clock250_b2 | 11.17 | 114.53 | 127.63 | 32.57 | 48.92 | Viewer subscriber |
| worker1 | 8.67 | 183.00 | 146.14 | 46.57 | 66.17 | Viewer subscriber |
| worker2 | 8.77 | 153.77 | 0.09 | 41.62 | 54.77 | Viewer subscriber |
| clock50_repeat | 13.86 | 86.00 | 86.40 | 27.35 | 37.06 | Viewer subscriber |
| no_view_baseline | 12.36 | 107.28 | 109.20 | 30.02 | 44.19 | Headless |
| lazy1 | 17.83 | 90.44 | 70.08 | 27.81 | 40.09 | Headless |
| diagnostics1 | 10.80 | 135.45 | 116.11 | 34.96 | 49.33 | Viewer subscriber |
| binding1 | 9.61 | 126.04 | 138.45 | 33.34 | 51.82 | Viewer subscriber |
| diagnostics2 | 12.07 | 122.07 | 100.23 | 31.38 | 44.33 | Viewer subscriber |
| binding2 | 10.61 | 118.49 | 121.81 | 30.23 | 42.83 | Viewer subscriber |
| baseline_final | 11.07 | 111.87 | 119.20 | 32.03 | 46.36 | Viewer subscriber |
| no_view_baseline2 | 11.59 | 112.44 | 112.66 | 31.48 | 43.45 | Headless |
| lazy_repeat | 15.18 | 108.96 | 81.90 | 31.61 | 44.10 | Headless |
| lazy_with_viewer | 10.94 | 110.08 | 110.74 | 31.49 | 43.90 | Viewer subscriber |

The original baseline repeats are 10.93/11.17/11.07 FPS. Headless baseline repeats are 12.36/11.59; lazy repeats are 17.83/15.18: a **30.99–44.21%** gain for the respective headless pairs, despite run-to-run variation. Decision p95 changes 107.28→90.44 and 112.44→108.96 ms. With a viewer, lazy gives **10.94 FPS**, within baseline variation: rendering remains enabled and this change promises no viewer-on gain.

## H1 — clock and executor

The installed ROS time-source subscription itself is wrapped per node only when `enable_performance_debug=true`; the native callback is still delegated to. No ROS installation file is modified. Its actual invocation count, CPU/wall time, RMW-to-callback delay and native thread IDs are recorded. Image receipts carry RMW times and callback/processing thread IDs. `/proc/<pid>/task/*/stat` supplies independent per-thread CPU samples. The passive profiler's clock receive rate is not treated as the publisher's rate: it also drops BEST_EFFORT samples under load.

At 250 Hz controlled source, perception services approximately **195–199 clock callbacks/s**, with native callback CPU only **0.64% of one core**. At 50 Hz it services about **49.6/s**, callback CPU about **0.20%**. The dispatch/main thread nevertheless consumes about **29–33% of one core** at 250 Hz versus **14.7%** at 50 Hz. Whole-process CPU can rise at 50 Hz because more inference work completes; lower dispatch load is not equivalent to lower total CPU.

The repeated 50 Hz trial improves completed FPS to 13.86 and actual source-to-output wall p95 to **92.44 ms**, compared with **118.43/133.03 ms** at 250 Hz. Thus the effect is not solely a stale `/clock` making source-age numbers look smaller. The 100 Hz trial is intermediate (12.37 FPS, wall-age p95 101.68 ms). Clock callback queue p95 is about 4 ms at 250 Hz and 7.95 ms in the 50 Hz repeat; reducing rate does not improve every responsiveness measure.

**Verdict:** frequency-associated executor/GIL scheduling contention is supported. Native clock callback computation is not itself the expensive stage. No GIL ownership profiler was used, so direct GIL contention is an inference from controlled scheduling and CPU evidence. Retain neither a production clock-rate change nor another executor redesign: the production source-time cadence and PX4 contract remain intact. The native Gazebo clock and full-run service rates are recorded separately below.

## H2 — one latest pending perception frame

The prototype used one dedicated perception thread and a condition-protected single replaceable pending item, no pool or multiprocessing. Source messages/stamps and receipt metadata were retained. Tracker, guard, reset and watchdog state were serialized by one state lock. Shutdown dropped pending work and joined the active call before node destruction; a failed frame was logged and the worker continued.

Focused tests passed for one-active/one-pending replacement, exception survival and shutdown. Actual ROS integration passed detector/ReID blocking with advancing and paused source clocks in both synchronous/worker modes: **11 focused tests PASS** including clock instrumentation. The final production restores the original synchronous path and its original source-clock tests.

DDS already discards unread old frames. H2 increases image callback delivery from about 11 to **28.7–29.0/s** and reduces RMW-to-callback queue p95 from about 33 to **21 ms**. But it adds a second wait before inference: receipt-to-processing p95 **62.10/49.06 ms**, versus under 0.1 ms synchronously. Its completed FPS is **8.67/8.77**, and decision p95 **183.00/153.77 ms**. Clock queue p95 remains about 4.3 ms.

**Reject.** A callback around 0.09 ms is not success when inference and age regress. Exact processed stamps are preserved in its logs. No full simulation was run for this rejected candidate, and no scenario/replay success is claimed for it. No perception worker, lock or queue remains in production.

## H3 — output, annotation and JPEG

The measured full baseline publishes target/controller output first, then guard JPEG/JSON, then annotation/HUD/image conversion and publication, then telemetry/performance. Full baseline p50/p95/max: guard diagnostics **6.45/11.88/48.97 ms**; annotation **19.96/36.68/83.72 ms**; total post-output work **27.18/46.24/119.34 ms**. These are stage distributions; their percentiles are not additive.

**E3a retained:** `enable_lazy_diagnostics=true` checks the annotated publisher's current subscriber count before rendering. It still creates the normal publisher, preserves camera source header/encoding/dimensions and renders normally when a subscriber appears. No viewer means no HUD frame copy, drawing, ROS byte conversion or image publish. The existing `enable_guard_debug=false` already disables JPEG entirely; that mechanism is unchanged. Explicit guard tracing continues normally even during subscriber discovery, preserving complete replay evidence.

Tests verify identical controller-facing target output with/without lazy annotation and no JPEG when guard diagnostics are disabled. The extracted experimental annotation renderer also produced an entire ROS Image identical byte-for-byte to the frozen baseline. With a subscriber the final code keeps the original renderer itself.

**E3b rejected:** one latest pending diagnostics job, detached header, owned decoded frame, copied boxes/scalar HUD values and deep-copied guard payload. The worker never dereferenced model/tracker/guard/current observations. Mutable-state snapshot and bounded-worker tests passed; no diagnostics frames were replaced in the two measured fixed-input runs. Its stage CPU work moved off the callback, but completed FPS varied **10.80/12.07** and decision p95 **135.45/122.07 ms**, against baseline 10.93–11.17 FPS and 111.87–130.54 ms. YOLO latency rose under competing Python work. No consistent primary-metric benefit justified retaining another thread. Its full diagnostic-job timings remain in the raw profiler evidence; all worker code was rolled back.

## GMC

Native Ultralytics ORB already converts to grayscale and downsamples by two, including masks/coordinates and translation scaling. The full baseline GMC p50/p95/max is **2.96/6.64/37.12 ms**; isolated long-tail samples do not make it the main callback cost. The exact share of measured callback work is included with the full-run results.

**GMC optimization rejected: not a material bottleneck.** No extra resize, mask change, ORB setting or affine-coordinate conversion was implemented.

## H4 — actual ReID transfers and separate probes

The current dynamic ONNX model disables the vendor's static-shape binding path. Native crop extraction constructs CPU RGB float tensors, bilinear-resizes each crop to 224×224, assembles a batch and copies it to CUDA. Native ONNX forward then calls **`im.cpu().numpy()`**, runs ORT CUDA with host input/output, AutoBackend copies output back to a CUDA tensor, and native ReID returns per-feature CPU NumPy arrays for association. The redundant device/host trips really occur; CUDA provider availability alone does not avoid them.

Project-owned instrumentation times crop, preprocessing (including its native H2D), backend entry-to-host-input, ORT run, output conversion, calls/frame and crops/frame. These instance wrappers only run when performance debugging is enabled. Fine transfer boundaries require an isolated probe using the same native operations; `reid_microbenchmark.json` includes p50/p95/max and exact CPU/batch construction checks. Host timings include synchronization and scheduling, not pure CUDA DMA bandwidth.

| Three-crop stage | p50 ms | p95 ms | max ms |
|---|---:|---:|---:|
| crop | 0.6020 | 0.9351 | 1.1791 |
| preprocess_cpu | 1.0761 | 1.6037 | 2.4761 |
| batch_to_cuda | 0.3989 | 0.5732 | 1.7704 |
| input_cuda_to_cpu | 0.2945 | 0.4856 | 0.8185 |
| ort | 6.9079 | 11.7635 | 17.5998 |
| output_cpu_to_cuda | 0.2634 | 0.3991 | 0.5247 |
| feature_cuda_to_cpu | 0.2579 | 0.5176 | 1.0327 |
| allocation | 0.0261 | 0.0569 | 0.1393 |

- **R1:** reject preprocessing reuse within a frame. Native confidence groups contain disjoint detections; there is no repeated crop to reuse. Reusing across source frames would change embeddings and is out of scope.
- **R2:** the fixed input already uses one batch of three crops per frame, so group consolidation cannot improve this benchmark. A separate two-group/one-batch microprobe is recorded below. The generic `feats` argument feeds arrays to the external image encoder rather than injecting final embeddings. A separate project-owned prototype therefore uses the public box parser and a frame-scoped encoder cache, retaining all native tracker update/association code. Its measured limits are below; it is not retained.
- **R3:** a project-owned per-instance forward uses public ORT `io_binding`, `bind_input`, `bind_output`, `run_with_iobinding`, and explicit input/output stream synchronization. It allocates owned FP32 CUDA outputs and removes intermediate host trips. Raw output is bit-identical for batches **1, 2, 3, 4, 6**. On **100 diverse saved video frames**, current detector outputs feed each tracker run; tracker IDs, embeddings, GMC, full guard inputs/similarities/decisions and accepted bboxes are exactly identical. Widely spaced video samples trigger the same existing GMC fallbacks on both paths. Microbenchmark p95 improves 19.30→15.97 ms initially but only 15.93→15.22 on repeat. Under ROS, FPS **9.61/10.61** and decision p95 **126.04/118.49** do not beat the repeated baseline. ReID p95 **33.34/30.23** versus baseline **35.14/32.57/32.03** is inconsistent and tracker tails do not improve consistently. **Rolled back**, despite correct outputs.
- **R4:** candidate CUDA feature-buffer allocation costs only **0.026/0.057 ms p50/p95**, below 1% of isolated ReID. No CPU batch-buffer rewrite is attempted: the native preprocessing helper owns its allocation, and there is no measured allocation bottleneck requiring a replacement. Reject persistent buffer/cache complexity without material measured savings. No aliased reusable output buffer is retained.

The material ReID cost is ORT execution plus CPU preprocessing and runtime scheduling, with a large full-system tail. Removing sub-millisecond transfers alone is insufficient here.

## Full simulation and replay

Both owned runs landed and exited cleanly. The original observer-on full baseline is **7.83 AI FPS**, decision-age **120/269/484 ms p50/p95/max**, callback **101.11/252.43/534.91 ms**, ReID **26.38/170.59/411.61 ms** and tracker **38.51/184.19/439.61 ms**. Native camera is 28.86 FPS; raw ROS observer 28.67 FPS. This is not the matching headless before/after comparison.

### Matching headless full-run comparison

| Metric | Before | After |
|---|---:|---:|
| AI completed FPS | 8.42 | 10.63 |
| Native camera FPS | 29.13 | 28.93 |
| Raw ROS receive FPS | 29.10 | 28.88 |
| Image callback FPS | 8.42 | 10.63 |

Each following cell is **p50 / p95 / max**; times are ms. All frame distributions and one-second FPS distributions are also in `all_metrics.csv`.

| Stage/age | Before | After |
|---|---:|---:|
| dds_transport_ms | 2.89 / 6.81 / 32.74 | 2.54 / 6.36 / 21.67 |
| queue_wait_wall_ms | 17.15 / 37.26 / 59.94 | 18.00 / 36.75 / 128.16 |
| callback_to_processing_ms | 0.05 / 0.11 / 6.61 | 0.05 / 0.10 / 2.08 |
| receive_age_source_ms | 44.00 / 64.00 / 68.00 | 48.00 / 64.00 / 80.00 |
| decision_age_source_ms | 112.00 / 256.00 / 572.00 | 112.00 / 240.00 / 432.00 |
| image_decode_ms | 0.44 / 2.22 / 15.21 | 0.40 / 2.28 / 12.04 |
| yolo_ms | 27.28 / 41.37 / 72.88 | 24.44 / 41.25 / 91.86 |
| tracker_ms | 36.87 / 176.30 / 522.61 | 37.16 / 167.99 / 427.45 |
| reid_crop_ms | 1.16 / 4.10 / 31.12 | 1.14 / 3.96 / 18.49 |
| reid_preprocess_ms | 6.87 / 15.03 / 36.89 | 6.40 / 15.62 / 41.90 |
| reid_input_copy_ms | 1.18 / 2.62 / 20.54 | 1.15 / 2.67 / 12.95 |
| reid_ort_ms | 13.56 / 147.82 / 444.47 | 14.01 / 141.88 / 362.50 |
| reid_output_copy_ms | 0.97 / 2.29 / 19.41 | 0.99 / 2.75 / 15.97 |
| reid_ms | 26.00 / 165.89 / 488.18 | 26.11 / 156.12 / 402.01 |
| gmc_ms | 2.90 / 5.98 / 25.06 | 3.07 / 6.53 / 25.02 |
| guard_ms | 0.74 / 3.04 / 15.63 | 0.74 / 2.68 / 15.50 |
| guard_diagnostics_ms | 6.31 / 12.24 / 23.65 | 6.48 / 12.20 / 37.67 |
| annotation_publish_ms | 17.10 / 32.89 / 81.37 | 0.01 / 0.03 / 1.65 |
| display_diagnostics_ms | 24.22 / 42.11 / 87.26 | 6.74 / 12.58 / 38.00 |
| processing_ms | 95.72 / 239.18 / 596.76 | 74.51 / 208.25 / 513.61 |
| processing_thread_cpu_ms | 67.67 / 204.83 / 468.80 | 49.34 / 176.08 / 395.93 |

| Resource | Before p50 / p95 / max | After p50 / p95 / max |
|---|---:|---:|
| perception_cpu_pct | 274.87 / 357.56 / 381.94 | 315.23 / 388.88 / 407.22 |
| Perception RSS MiB | 1978.58 / 1986.66 / 1989.09 | 1978.88 / 1980.04 / 1980.04 |
| gpu_util_pct | 34.00 / 41.00 / 50.00 | 35.00 / 41.00 / 44.00 |
| gpu_memory_mib | 726.00 / 726.00 / 726.00 | 729.00 / 761.00 / 761.00 |

CPU is percent of one core; GPU utilization/memory are device-wide `nvidia-smi` samples. More completed inference increases process CPU; memory stays bounded. The annotation check reduces measured p50 from **17.10 to 0.013 ms**. Completed FPS improves **26.22%** in this observational pair. Decision p95 **256→240 ms** remains above 200 ms. The ≥10 FPS reference is reached; ≥12 FPS and <200 ms are not met. This is a scoped performance improvement, not overall tracking acceptance.

- `full_headless_baseline`: source window 49.864–236.020 s; native clock **249.98/source-second**, **240.32/wall-second**; node clock callbacks **164.44/s**, clock callback CPU **0.72%**; GMC **2.95%** of summed callback work.

- `full_final`: source window 41.056–236.116 s; native clock **250.00/source-second**, **238.68/wall-second**; node clock callbacks **174.17/s**, clock callback CPU **0.73%**; GMC **3.95%** of summed callback work.

Clock-queue percentiles are the latest callback-delay samples attached to completed images, not every individual clock event. Counts/CPU use cumulative actual callback counters. Source windows and selected image stamps differ across full simulations; FPS/continuity deltas here are not asserted to be causal frame-for-frame comparisons.

### Guard replay and scenario results

| Capture | Recorded guard inputs | Accepted | Wrong | Unknown accepted | Runtime replay mismatches | Recoveries |
|---|---:|---:|---:|---:|---:|---:|
| full_baseline | 1484 | 183 | 0 | 0 | 0 | 23 |
| full_headless_baseline | 1641 | 187 | 0 | 0 | 0 | 16 |
| full_final | 2182 | 412 | 0 | 0 | 0 | 20 |

Frozen and current guard implementations give identical replay results. Exact processed source stamps are strictly increasing in each run. Replay covers recorded observer inputs: 1,641/1,642 processed headless-baseline events and 2,182/2,184 final events; the observer stops near source second 236 before the final output tail. No claim is made to replay the unrecorded final 1–2 events. There is no perception thread, diagnostics thread, new queue or concurrent tracker state in the retained runtime.

Independent observer acceptance counts (accepted / joined camera frames):

| Scenario | Before | After |
|---|---:|---:|
| A_static_camera_target_motion | 46/127 | 78/163 |
| B_gimbal_yaw_motion | 5/39 | 19/46 |
| C_person_crossing | 17/116 | 44/165 |
| D_three_person_occlusion_crossing | 25/155 | 41/204 |
| E_repeated_crossing | 59/411 | 139/569 |
| F_short_exit_reentry | 5/209 | 11/281 |
| G_long_exit_reentry | 11/482 | 11/535 |

All accepted labels are person A; there are no wrong/unknown accepted identities. Long re-entry remains weak (11 accepted frames in each run; its observed acceptance fraction is slightly lower after). Yaw/crossing/occlusion/re-entry continuity remains **FAIL** before and after. Same-input tests/sequence equivalence find no target-policy regression; changing processed timestamps prevents claiming exact scenario equivalence. The optimization does not authorize body-following or fix the existing identity-continuity problem.

### R2 follow-up: separate groups versus one batch

The explicit two-group probe uses the same three saved crops and original FP32 session. It measures a real shape/group interaction rather than just transfer overhead:

| Mode | ReID p50 / p95 / max ms |
|---|---:|
| separate | 180.27 / 229.34 / 256.47 |
| combined | 12.69 / 19.21 / 24.01 |
| separate_repeat | 171.76 / 220.11 / 258.90 |
| combined_repeat | 12.55 / 21.00 / 25.42 |

Combining 2+1 into 3 changes raw embeddings by up to **0.00056086**. The public encoder-cache prototype was tested on 100 saved frames with original detector outputs and torso descriptors: **3 mixed-group frames**, **0 ID/bbox differences**, **0 GMC differences**, **0 guard-decision differences**, normalized feature max difference **0.00018231**, max cosine distance **0.000000557**. Guard embedding inputs are **not bit-identical**. These observations do not establish an unsafe identity change, and do not qualify as complete production equivalence/gain evidence either.

In the final full run only **35/2,184 frames (1.60%)** invoke two ReID groups; 2,149 invoke one group. The standard fixed-image benchmark already has one batched call for three crops, so this cache cannot improve that input. The R2 probe is isolated evidence, not an AI FPS claim or an accepted ROS candidate. No full simulation is run for R2 because the required controlled primary-metric benefit has not been established. Fixed-batch padding, batching across timestamps, model/precision changes and another tracker schedule are not implemented. The ORT/group/shape interaction is recorded as a remaining runtime limitation rather than dismissed as negligible data movement.

## Final review and verification

- **PASS:** 187 final tests with real CUDA/ReID enabled, including new clock instrumentation and lazy-output checks; source-time pause/rewind, stale-result learning rejection, reset, watchdog, GMC and target outputs remain covered. Prototype-specific tests passed before rollback and are preserved with their candidate sources.
- **PASS:** all three full simulation captures land within about 0.06 m of ground and exit normally; guard replay has zero mismatches and zero wrong/unknown accepted targets. Final processed source stamps are monotonic. No added runtime queues remain.
- **PASS:** `colcon build --base-paths src --packages-select uav_perception`, 11 production Python source/install byte comparisons, Fast DDS XML parse, dependency check and whitespace review. Root-wide colcon initially discovered saved package copies; explicit `src` avoids historical evidence directories. This task's snapshot directory has standard collection/build ignore markers.
- **PASS:** 13,894 RECORD hashes across Ultralytics, ONNX Runtime GPU, torch, OpenCV and NumPy; active model SHA-256 values unchanged; ROS rclpy package checksum verification and external tracked PX4 diff are clean. No third-party source edit, dependency install, model replacement or commit.
- **PASS WITH NOTES:** scope/architecture review retains only the public subscriber check plus opt-in instrumentation. Native clock subscription/encoder hooks depend on the inspected installed API and remain debug-only. `/yolo/telemetry` retains its existing shape. GPU power samples are WSL/device telemetry, not a power-limit diagnosis.
- **FAIL:** decision-age p95 <200 ms, 12 FPS reference and existing tracking continuity; no GUI-painted-FPS improvement is claimed. These failures are reported separately from the measured headless gain.

Reproduce from the repository root after sourcing ROS and the external ROS workspace:

```bash
PYTHONDONTWRITEBYTECODE=1 YOLO_AUTOINSTALL=false RUN_BOTSORT_REID_RUNTIME=1 \
PYTHONPATH="$PWD/src/uav_perception:$PWD/evaluation:$PYTHONPATH" \
  ../software/venvs/yolo26/bin/python -m pytest -q -p no:cacheprovider \
  src/uav_perception/test evaluation/test_*.py src/uav_control/test
colcon build --base-paths src --packages-select uav_perception
```

Controlled reproduction uses `controlled_case.py --tag NEW_TAG --transport large --graph`, adding `--no-observer --lazy` for the retained candidate and `--no-observer` for its baseline. Use `--source-dir` with the saved H2/H3/H4 source snapshots to reproduce rolled-back flags. Standard runtime rollback is `-p enable_lazy_diagnostics:=false`.

Task-owned production changes: `yolo_detector.py` (subscriber check and opt-in stage/thread times), `botsort_adapter.py` (opt-in transfer/session times), new `clock_metrics.py`; profiler and two focused test files; current package README/system contract/progress and this report. Existing dirty changes from the prior environment/video audit are preserved. Guard/controller, model/config/DDS/scenario files are intentionally unchanged.

## Answers to the eleven questions

1. **Does `/clock` cause material contention?** Yes, rate-associated scheduling overhead is repeatable in the controlled harness; direct native callback CPU is below 1% of one core. Production clock remains unchanged.
2. **Is synchronous callback architecture the bottleneck?** Its post-output annotation costs time, but moving the whole perception path to another thread makes primary metrics worse; a general architecture replacement is not justified.
3. **Does a latest-frame worker help FPS or age?** Neither here. Reception is more responsive, but completed FPS falls and worker wait worsens decision age.
4. **What does DDS depth=1 already provide?** Old unread samples are discarded; the synchronous callback already takes a recent sample instead of draining a long queue.
5. **How much do diagnostics cost?** Original full baseline post-output p50/p95 is 27.18/46.24 ms, including annotation 19.96/36.68 and guard diagnostics 6.45/11.88. Matching headless lazy annotation drops 17.10 ms median to about 0.013 ms.
6. **Is GMC worth optimizing?** No. Native grayscale/downscale already exists; measured GMC is about 3 ms median and 6 ms p95.
7. **Where is ReID's bottleneck?** Native redundant transfers exist but individually cost sub-milliseconds; ORT execution, preprocessing/scheduling and group/shape interactions dominate the large tail. R2 microbenchmark exposes that interaction, without proving a qualified production fix.
8. **What is retained?** Lazy annotation with no subscribers, default-on but reversible; debug-only actual clock/thread, diagnostics and ReID timing.
9. **What is rolled back/rejected?** Perception worker, asynchronous diagnostics, I/O binding; R1/R4 caches and R2 promotion; clock-rate/executor changes and extra GMC resize. Experimental sources/evidence remain available.
10. **Full simulation before→after?** Matching headless pair: **8.42→10.63 AI FPS**, **256→240 ms decision p95**, clean landing/shutdown. Controlled headless repeats independently establish the annotation benefit. Viewer-on FPS is unchanged.
11. **Any tracking regression?** Same-input target/sequence checks and replay pass, with zero wrong/unknown acceptance. Full processed timestamps differ; continuity remains FAIL and long re-entry is still weak, so no overall tracking PASS is claimed.
