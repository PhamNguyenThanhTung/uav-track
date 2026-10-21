# External environment audit — 2026-10-16

**Verdict: PASS for a clean UAV runtime baseline, with the assurance limits below.** No remaining third-party source patch, active-model modification, unexpected package overwrite, editable install or candidate-path injection was detected. No package reinstall was needed. This audit adds dependency-installation protection in project code; it performs no FPS optimization or new benchmark.

Evidence: `evaluation/results/runs/external_environment_audit_20261016T070954Z/`. Before any production edit, this directory captured Git status, unstaged/staged binary diffs and a SHA-256 manifest of all 167 project files. `file_classification.json` classifies the existing files; `project_manifest_final.json` records the final state. No commit was created.

## State, action and ownership

| Item | Location | State before | Action | Final state |
|---|---|---|---|---|
| Ultralytics | `../software/venvs/yolo26/lib/python3.12/site-packages/ultralytics/` | 8.4.164; 382 RECORD hashes match; native auto-install defaults on | Preserve package; disable auto-install before vendor import in UAV package and test collection | Source unchanged; UAV startup reports `AUTOINSTALL=False` |
| ONNX Runtime | Same venv, `onnxruntime/` and `onnxruntime_gpu-1.30.0.dist-info/` | GPU 1.30.0 only; CPU distribution absent; all 327 RECORD hashes match | Verify again; run one ReID inference with provider profiling | Same version/files; CUDA execution confirmed |
| PyTorch | Same venv, `torch/` | Distribution 2.14.0; module build 2.14.0+cu130; 12,264 RECORD hashes match | CUDA/inference/equivalence checks; no package changes | RTX 4060 accessible; YOLO backend on `cuda:0`, FP32 |
| OpenCV | Same venv, `cv2/`, `opencv_python.libs/` | 4.10.0.84; 130 hashes match | Preserve and verify | Unchanged |
| ROS install | `/opt/ros/jazzy/` | Package-owned files match local dpkg receipts; no extra non-cache files | Verify installed ROS/runtime packages | No source/binary discrepancy found |
| PX4 and ROS dependencies | `../software/PX4-Autopilot/`, `../software/ros2_ws/src/` | Tracked trees clean; old additional PX4 checkouts present | Inspect main tree, initialized submodules and nested repositories | No tracked source edits; old checkouts preserved |
| Model weights | `~/.cache/ultralytics/yolo26n.pt`, `~/.cache/uav_perception/yolo26n-reid.onnx` | Active assets present; hashes below | Hash before/after; use existing files for smoke tests | No replacement, download or modification |
| Fast DDS | `src/uav_perception/config/fastdds_images.xml`, installed package config | Intentional project-owned 4 MiB message / 64 MiB shared segment profile | Review scope, schema and isolated startup | Retained; no system-wide profile found |
| CUDA Graph | `src/uav_perception/uav_perception/gpu_inference.py`, node integration | Project-owned option, default true for CUDA | Review capture/fallback and run existing tests | Retained without changes |
| Instrumentation | Node/adapter, `evaluation/profile_perception_pipeline.py` | Optional `/yolo/performance`, default false | Review branches/hooks; import and isolated publisher startup | Retained without changes |
| Ponytail | Codex plugin cache and skill directory outside project | Official 5.2.0 source and hooks | Compare installed bytes to pinned Git objects | 202 files match; no custom source patch or runtime dependency |
| PATH/env | Caller, ROS setup, project launchers | Expected ROS/venv paths; no candidate override active | Record base and production environments | No persistent environment edits |

The complete Python inventory contains **243 visible distributions** because this venv intentionally enables system site-packages. **28,712 recorded file hashes, totaling 6.45 GB, match their installation receipts.** Critical package trees contain no unrecorded non-cache files. The scan handles both standard wheel base64 hashes and Debian's hexadecimal RECORD entries; the initial 26 apparent mismatches were encoding differences, and all 26 match when decoded correctly. Both initial evidence and the corrected results are retained.

`dpkg --verify` produced no differences for 330 installed ROS/runtime packages, 203 Python system packages, and four Python runtime packages. ROS ownership enumeration found zero unowned files after excluding generated Python caches. These checks use trusted local installation metadata, not newly downloaded reference packages.

## Project file classification

| Class | Files or area | Disposition |
|---|---|---|
| A — project production code | `yolo_detector.py`, `botsort_adapter.py`, `gpu_inference.py`, package `__init__.py` | Preserve selected performance code. Only `__init__.py` gains the auto-install boundary in this audit. |
| B — tests/instrumentation | `evaluation/profile_perception_pipeline.py`, its test, perception GPU/pipeline timing tests | Retain. Add `test/conftest.py` and one subprocess regression for auto-install protection. |
| C — runtime/config | Fast DDS XML, `setup.py`, simulation launchers, rendering helper, pinned requirements and related current documentation | Retain. No transport, model, precision, queue or controller setting changed here. |
| D — experiment evidence | Ignored `evaluation/results/runs/` audit, latency and controlled-case artifacts | Preserve as evidence; none is imported by default runtime. |
| E — preceding work outside this audit | Other tracked UAV changes consolidated in existing HEAD; old external PX4 checkouts | Preserve. No reset, cleanup or deletion. |

The starting worktree had four modified documents: `PROGRESS.md`, the pipeline audit, system contract and perception README. They were saved before this audit and preserved. Current audit notes extend the appropriate progress/README files; no preceding performance result is reverted. Other existing project files are compared against the before manifest.

## Historical ONNX overwrite and current package proof

The earlier restore evidence records a real temporary environment mutation: Ultralytics auto-installed **`onnxruntime` CPU 1.31.0**, overwriting 23 files shared with the GPU distribution. It was restored with the original official **`onnxruntime-gpu==1.30.0`** wheel, without dependency updates. The stored wheel digest was `6acc0f4b3d5ac5f2984432faa59dd8fcce6775f8b43a7140afa0f322c7a703b1`. That digest still agrees with installed `direct_url.json` and the historical restore receipt. No evidence of a hand-edited vendor-source patch was found.

This audit independently verifies the current import path, installed distribution, version, all 327 GPU-package RECORD entries, provider list and actual inference. No `onnxruntime` CPU distribution, overlapping CPU `.dist-info`, missing GPU file or unrecorded file remains. `CPUExecutionProvider` in the provider list is a normal fallback provider shipped inside the GPU package; it is not evidence of a separately installed CPU wheel.

The installed ORT metadata points to the old, removed restore wheel in the evidence directory. It is a non-editable wheel installation receipt, not a runtime import or `PYTHONPATH` injection. Modules import from the venv's `site-packages`.

One profiled ReID invocation produced a finite `[1, 512]` output and **137 CUDA provider events plus 14 CPU provider events**. The native CUDA session remains first in provider order. ORT still emits its existing plugin-device-registration warning, but the execution profile confirms actual CUDA work. No provider configuration was optimized in this task.

`pip check` returns success before and after. The existing system `pybind11` metadata warning also appears in the earlier restore log; it is outside the active UAV package set and was preserved. Package names, versions and installation locations are unchanged across this audit.

## Auto-install hardening

Before the edit, the verification commands documented `YOLO_AUTOINSTALL=false`, but the production module and test collection did not enforce it themselves. Native Ultralytics 8.4.164 captures `AUTOINSTALL` from the environment during import and otherwise defaults to true.

The project package now sets `YOLO_AUTOINSTALL=false` before importing any Ultralytics consumer. This covers normal module/console entrypoints and the scenario runner without modifying a vendor file. Test `conftest.py` sets it before test modules that import Ultralytics directly. A fresh subprocess launched with `YOLO_AUTOINSTALL=true` confirms that importing the UAV package first results in `ultralytics.utils.AUTOINSTALL is False`. An embedding application that imports Ultralytics before the UAV package must set the variable before its own import; the standalone production startup has the required ordering.

This change prevents package installation during startup and inference. It does not disable explicit administrator/package-manager installation or change model download policy. Both active models already exist and no model download occurred during verification.

## Model files

| Active model | Bytes | SHA-256 | Comparison |
|---|---:|---|---|
| YOLO26n `.pt` | 5,544,453 | `9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef` | Matches historical October 12/13 manifests and the preceding benchmark copy; unchanged across audit |
| ReID `.onnx` | 9,873,245 | `8529c383197ae4c468eda535d1b165f8b4162cf17bf5fbcff49c7cb6455bc0bb` | Unchanged across audit; mtime October 14 precedes the performance task |

No historical SHA-256 manifest for this specific ReID ONNX file was found. Its current bytes are now pinned by the before/after manifests, but equivalence to an independently authenticated earlier upstream asset is not established here. The old `embedding_weights_sha256` entries belong to the preceding DeepSORT model and are not used as a comparison for the current ONNX asset. No redownload was justified.

The ignored `live_fps_view_20261015T072238Z` directory also contains a byte-identical YOLO `.pt` copy and an unused detector ONNX export from the earlier experiment. They are generated evidence, not selected production weights. Their hashes and inactive role are in `all_model_artifacts.json`; neither was removed or replaced.

## External settings, paths and tooling

- **Fast DDS:** XML is owned by the project and installed by its package. `launch_simulation.sh` and the tracking runner pass `FASTRTPS_DEFAULT_PROFILES_FILE` to the bridge; node `main()` applies the installed profile before `rclpy.init()` when neither supported profile variable is already set. Explicit `FASTDDS_DEFAULT_PROFILES_FILE` / `FASTRTPS_DEFAULT_PROFILES_FILE` overrides remain respected. No user/global default profile or shell export was found. These child-process settings do not alter other applications.
- **GPU rendering:** `config/sim/gazebo_rendering.sh` selects WSLg D3D12/NVIDIA in the launcher process tree. It does not patch Mesa, CUDA or a global library. WSL driver binaries are host-managed; runtime functionality is checked, but their vendor signatures were not independently revalidated.
- **Clock:** the earlier intentional WSL fix still leaves `systemd-timesyncd` disabled/inactive while `hv_utils.timesync_implicit=1` uses Windows time. This is documented in the existing WSL clock report. No service setting changed in this audit.
- **Ponytail:** installed files match Git object contents from official `DietrichGebert/ponytail`, commit `84740e3d344b69b4a133201fa519eb37a62a2b2a`. The standalone skill blob is `b76922f76ec6f4db532203c55d710b749fa1f6af`. Git metadata in the cache is excluded from content comparison. The installer moved its staging checkout, so that staging worktree reports deletions; comparison uses immutable Git blobs instead. Tool hooks and the existing `~/.local/bin/node` symlink to NVM Node 24.21.0 are tooling settings, not UAV dependencies.
- **Import paths:** normal production `PYTHONPATH` contains the installed project package, ROS dependency workspace and `/opt/ros` vendor paths. It contains no results directory, candidate source snapshot or editable wheel. The scenario runner prepends the actual project source directory by default; its existing explicit `TRACKING_PERCEPTION_SOURCE_DIR` override is unset in the captured environment. The audit script's own directory appears as `sys.path[0]` only when that script is executed directly; the separately captured production import path does not contain it.
- **Other Python hooks:** `.pth` files are the existing setuptools shim, namespace-package declarations and the conditional pytest-cov hook. `sitecustomize.py` is the dpkg-verified Ubuntu apport handler. No `.egg-link`, editable `direct_url`, custom startup hook or active test-only environment override was found.
- **PATH:** Codex argument/helper directories are per-session tool paths. The inherited Windows PATH includes unrelated existing entries, including a Conda DLL filename; these are not Linux Python/loader injection and were preserved. Base-shell CUDA/ORT/YOLO, RMW, ROS domain and Fast DDS override variables were unset. ROS setup/venv activation adds the expected process-local paths. The isolated startup smoke used domain 91 only in its child shell.

PX4's five top-level untracked paths contain six clean nested Git checkouts dating to September 23. Initialized registered submodules also report no changes. The three ROS dependency repositories are clean. Existing build products and parameter/log artifacts are generated state; they are not asserted to match an independently rebuilt official binary byte-for-byte.

## CUDA Graph and instrumentation review

CUDA Graph remains entirely in project-owned source. It wraps one warmed model instance in memory and does not edit Ultralytics/PyTorch files. Capture uses the actual preprocessed 384×640 camera tensor. Assignment occurs only after successful construction, so a capture exception leaves native eager forward in place. Unsupported shapes, devices, dtypes or prediction modes permanently return to native forward. Existing CPU-rejection, changing-image, worker-thread, fallback and exact-output ownership/equivalence tests remain present and pass. The default is true for CUDA; weights, FP32, thresholds and detector geometry are unchanged. Unavailable CUDA still fails the separate required ReID startup check, as intended.

Performance debug defaults to false. With it disabled there is no performance publisher, JSON event assembly or optional encoder-stage wrapper. The adapter retains its preceding lightweight ReID/GMC counters and a few conditional/timing checks; this audit reviews their code path and does not introduce an overhead benchmark. When enabled, per-instance timing wrappers measure existing crop/preprocess/backend methods without changing results. Generated experiment helpers are not imported. The five-value telemetry contract and source-clock semantics remain unchanged.

## Verification and assurance

- **PASS:** `pip check`, unchanged package inventory, all recorded Python hashes and unrecorded-file check; final critical-package hashes rechecked after runtime verification.
- **PASS:** system package verification, source Git/submodule checks, model before/after hashes and historical YOLO comparison.
- **PASS:** PyTorch CUDA, actual YOLO CUDA FP32 inference, native ReID feature/association test and independently profiled ORT CUDA execution.
- **PASS:** 183 tests with the real GPU/ReID suite enabled, including the new auto-install subprocess regression.
- **PASS:** project ROS package build, 12 source/install Python/config byte comparisons, Fast DDS XML schema and actual startup, profiler imports, isolated `/yolo/performance` discovery, clean SIGINT exit.
- **RETAINED NOTES:** receipts and Git object stores are local trust anchors; this is not a reauthentication of every wheel/system/driver against fresh upstream binaries. No earlier authenticated hash was available for the current ReID ONNX file. Existing system metadata warning and old unused external checkouts were documented and preserved.

The retained initial GPU smoke-check log contains an assertion against the original YOLO wrapper's CPU model copy. The corrected check inspects the executed predictor backend, its CUDA parameters and actual ORT provider events; all pass. This check correction required no package or model repair.

No source, package, model or system restoration was required during this audit. The only new runtime change is the project-owned auto-install guard; no optimization, tracking-policy edit, model replacement, dependency update, simulation scenario change or commit occurred.

```text
Third-party source modifications: NONE currently detected
Model modifications: NONE to active weights; historical inactive exports documented
Unexpected package changes: NONE detected; inventory unchanged
Intentional external/runtime config: project Fast DDS profile, process-local GPU rendering,
  preceding WSL time setting, official Ponytail tooling and Node PATH symlink
Project-owned performance changes: CUDA Graph and default-off instrumentation retained;
  auto-install protection added in project package/test setup
Environment health: PASS for continuing a separate ReID runtime task, within stated assurance
```

The existing target-identity continuity failure is independent of this environment audit and remains unresolved.
