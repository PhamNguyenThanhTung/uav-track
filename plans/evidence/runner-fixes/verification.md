# Runner fix verification

Date: 2026-11-16

## Static and focused checks

- `bash -n simulation/run_tracking_3_people_test.sh` — **PASS**.
- `shellcheck -S warning simulation/run_tracking_3_people_test.sh` — **PASS**. Default ShellCheck also reports INFO-only source and conditional-style notices.
- Python syntax compilation for the changed runner, audit, and focused tests — **PASS**.
- Focused pytest for `evaluation/test_camera_audit_export.py` and `evaluation/test_live_demo.py` — **PASS**, 14 tests and 6 subtests.
- `git diff --check` and `./launch_live_demo.sh --help` — **PASS**.

## Tracking runner

- A full 236-second SITL capture completed. This host's detector stopped at CUDA initialization, and the summary had `frames_observed=0` while the observer exited 0. The runner now rejects this missing tracking evidence.
- The first post-change landing attempt exposed two cleanup issues: the altitude controller kept publishing setpoints during Land, and three sequential ROS CLI reads made the earliest sample exceed the freshness window. The runner now stops its own setpoint publisher first and reads the three volatile telemetry topics concurrently.
- Short SITL verification: `TRACKING_STOP_SIM_TIME=5 ./simulation/run_tracking_3_people_test.sh` — **exit 9 as intended** because the summary contained zero tracked frames. Cleanup separately passed: Land command exit 0; `landed=true`; `arming_state=1`; NED z `0.02787 m`; all three PX4 timestamps were newer than the Land command and under 1.5 seconds old at the common check time. `cleanup_status.txt` preserved `exit_status=9`.
- Artifacts: `evaluation/results/runs/tracking_3_people_20261116T044150Z/`.

## Live demo

- Headless SITL reached armed hover at 1.80 m and verified stock `NAV_DLL_ACT=2` and `COM_RCL_EXCEPT=0`.
- The 11:43 run below happened before GPU recovery. CUDA was unavailable then; cleanup confirmed `landed=true`, `arming_state=1`, NED z `0.05091 m`.
- After the cold power cycle, `./launch_live_demo.sh --headless --hover-time 10` started YOLO and BoT-SORT on CUDA. The detector log reported roughly 29–30 FPS and 4–6 ms YOLO latency; the tracking interval observed 36 nonempty detection arrays.
- That run exited **1** because the primary 45-second landing confirmation timed out. Cleanup issued a second Land command and did confirm fresh `landed=true`, `arming_state=1`, NED z `-0.03716 m`; all owned process groups exited.
- A subsequent GUI run, `./launch_live_demo.sh --hover-time 20`, **passed**. The viewer process exited 0; tracking ran for 20 seconds, counted 1,346 camera frames and 63 nonempty detection arrays, and PX4 confirmed landed/disarmed at NED z `-0.06092 m`. Every owned process group exited.
- Artifacts: `evaluation/results/runs/live_demo_20261116T114310Z_125501/`, `evaluation/results/runs/live_demo_20261116T120718Z_5943/`, and `evaluation/results/runs/live_demo_20261116T125106Z_5681/`.
- The GUI viewer process started and exited cleanly. Its rendered window was not programmatically inspected.

## GPU diagnosis

- Hardware is a Lenovo LOQ 15APH8 (82XT), NVIDIA RTX 4060 Laptop GPU (PCI `10de:28e0`) plus AMD iGPU; kernel is `6.8.0-149-generic`, NVIDIA open module `580.178.04`.
- PyTorch is `2.6.0+cu124`, which is a CUDA build. `CUDA_VISIBLE_DEVICES` is unset. The failure is below PyTorch: `nvidia-smi` reports no devices and `/sys/bus/pci/devices/0000:01:00.0/driver` is absent while the NVIDIA module remains loaded.
- Kernel logs contain repeated NVRM `NV_ERR_NO_MEMORY` entries and at 11:04 record `nvidia-drm` removing the device followed by `Attempting to remove device 0000:01:00.0 with non-zero usage count!`.
- During the original failure, `lspci` still listed the RTX 4060 but the NVIDIA PCI driver link was absent. A root-only PCI rebind blocked in uninterruptible kernel `D` state.
- A firmware-setup reboot then stalled; the user forced power off and started the laptop again. After that cold power cycle, `lspci -nnk -s 01:00.0` reports `Kernel driver in use: nvidia`, `nvidia-smi -L` lists the RTX 4060, and `torch.cuda.is_available()` is true. No BIOS setting change was verified.
- A real PyTorch CUDA matrix multiplication completed on `cuda:0` using the RTX 4060.
- In the project launch environment (after sourcing the YOLO virtualenv and ROS overlays), ONNX Runtime 1.30.0 reports active providers `CUDAExecutionProvider`, `CPUExecutionProvider`. A warmed `yolo26n-reid.onnx` inference on a `1x3x256x128` tensor returned `(1, 512)` with median 0.90 ms. The bare venv without the ROS overlay library paths falls back to CPU because it cannot find `libcublasLt.so.13`; use the project's launch environment.
- One check immediately after the first post-recovery demo returned `nvidia-smi: No devices were found`. It was not reproduced on the next pre-run or post-run checks: the later GUI demo started and finished with `Kernel driver in use: nvidia`, `nvidia-smi -L` listing the RTX 4060, and `torch.cuda.is_available()` true. The intermittent result's cause is unconfirmed; keep monitoring if it recurs.
