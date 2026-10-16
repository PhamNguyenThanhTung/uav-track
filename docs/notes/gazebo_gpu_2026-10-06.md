# Gazebo GPU rendering verification — 2026-10-06

## Result and scope

Gazebo OGRE2 camera rendering now uses **D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)** on this WSL machine. The project launch helpers automatically configure it; the master launcher and Phase 4A launcher call these helpers. Native Linux selection and explicitly supplied Mesa overrides are preserved.

No resolution, camera update rate, world models, physics parameters, flight-control code, or perception code was changed. Physics continues to run on the CPU. PX4 flight, YOLO inference and concurrent full-stack performance were not exercised in these rendering checks.

## Root cause and fix

Before the fix, both `glxinfo -B` and `~/.gz/rendering/ogre2.log` reported `llvmpipe (LLVM 20.1.2, 256 bits)`, with `Accelerated: no`. NVIDIA CUDA was available, but OpenGL rendering was still using the CPU. WSLg's `stderr.log` also reported a glamor software fallback.

Setting only `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` or `MESA_LOADER_DRIVER_OVERRIDE=d3d12` did not change the renderer on this machine. The working combination was:

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

With this combination, `glxinfo` reports `Accelerated: yes`, OpenGL 4.6 and the RTX 4060 renderer. OGRE2 confirms the same renderer during actual camera generation. The exported variables are scoped to the Gazebo helper and its children; no global `.bashrc`, Windows driver or Linux driver installation was needed.

Mesa diagnostic binaries were downloaded and extracted under `/tmp/uav-gpu-check`, without installing system packages. The machine has `/dev/dxg`, WSLg 1.0.79, Mesa 25.2.8, Windows NVIDIA driver 581.86 and an RTX 4060 Laptop GPU with 8 GiB VRAM.

## Measurements

Every run used the same `gimbal_test.sdf`, the UAV camera at 1280×720 and the observer camera at 640×480, both configured at 30 Hz in simulation time. A Gazebo Transport subscriber recorded callback arrivals with `time.perf_counter` and image simulation timestamps. Each run waited for the first UAV image, warmed up for three wall seconds, then counted images in a 12-wall-second window. Gazebo partitions were isolated; temporary process groups were stopped after each run.

| Run | UAV Gazebo images | UAV wall FPS | Observer wall FPS | Renderer |
|---|---:|---:|---:|---|
| Original server-only configuration | 70 | 5.83 | 5.75 | llvmpipe |
| D3D12/NVIDIA, server only | 361 | 30.08 | 30.17 | RTX 4060 |
| Project automatic selection, world GUI open | 353 | 29.42 | 29.42 | RTX 4060 |
| Project automatic selection + ROS bridge | 341 | 28.42 | 28.50 | RTX 4060 |
| Project automatic selection + ROS bridge, reliable subscriber | 338 | 28.17 | 28.08 | RTX 4060 |

The isolated GPU server measurement is approximately **5.16×** the baseline wall-clock camera rate. All runs still observed approximately 30.3 camera frames per simulation second: the important improvement is simulation/rendering keeping pace with wall time. These are subscriber observations from short runs, not publisher-side loss-accounting benchmarks. The reported Gazebo `real_time_factor` is also recorded in JSON, but its rolling/startup behavior is separate from the measured image interval.

The ROS bridge check used the unchanged `config/sim/ros_gz_bridge.yaml`, an isolated ROS domain and a best-effort sensor-data subscriber. It received **242 images / 12 seconds = 20.17 wall FPS** while the Gazebo subscriber received 28.42 FPS. Repeating with a reliable subscriber at depth 1 received **337 images / 12 seconds = 28.08 wall FPS**, alongside 28.17 FPS at Gazebo. This demonstrates that receive QoS affects observed delivery on this machine; packet-level causes were not traced. Thus approximately 30 FPS rendering does not establish 30 FPS delivery to every ROS consumer. YOLO throughput and latency under either QoS were not measured; perception's existing default remains unchanged.

Raw per-run counts, durations, simulation spans and renderer log excerpts are stored in `evaluation/results/gpu_rendering_2026-10-06.json`.

## Running and checking

From the Git root:

```bash
# Full simulation launcher, with the world GUI:
./launch_simulation.sh

# Server only; camera still renders on the GPU via WSLg:
./launch_simulation.sh --headless

# Gazebo-only helper for rendering diagnostics:
./simulation/run_sim_phase3.sh -s -v 3

# Revert automatic selection for comparison (unset manual overrides first):
env -u GALLIUM_DRIVER -u MESA_D3D12_DEFAULT_ADAPTER_NAME \
    UAV_GAZEBO_GPU=0 ./simulation/run_sim_phase3.sh -s -v 3

# Inspect the actual renderer after Gazebo starts:
rg 'GL_RENDERER|Device Name' ~/.gz/rendering/ogre2.log

# Check camera arrivals at the Gazebo boundary:
gz topic -f -t /custom_x500/camera/image_raw
```

The existing perception node supports an optional reliable camera subscriber:

```bash
# After sourcing ROS/project setup and activating the YOLO virtualenv:
python3 -m uav_perception.yolo_detector --ros-args -p qos_reliability:=reliable
```

This is an optional experiment, not a verified YOLO throughput result. Reliable delivery can affect queueing/latency under load, so measure processed-frame FPS and image age before choosing it for control.

`--headless` in the master launcher means server-only (`-s`). The verified WSL configuration still uses WSLg's display for the OpenGL context. Display-free EGL (`--headless-rendering`) was not needed or validated; it should not be assumed to work identically on WSL. On this machine the EGL GBM probe failed despite successful D3D12 X11/Wayland contexts.

Existing `GALLIUM_DRIVER` and `MESA_D3D12_DEFAULT_ADAPTER_NAME` values take precedence. To choose another adapter, set its name before launch. If `LIBGL_ALWAYS_SOFTWARE` is set to `1` or `true`, the helper warns; unset it for GPU rendering.

## Validation

- PASS: actual GLX hardware renderer, OGRE2 GPU renderer, server-only camera generation, GUI world startup and camera generation, unchanged ROS bridge image delivery.
- PASS: representative observer-camera image inspected; world objects rendered rather than a blank frame.
- PASS: Bash syntax and ShellCheck warning checks for the configuration and launch helpers.
- PASS: automatic WSL selection, opt-out, existing overrides, and no selection change when WSL detection is absent.
- NOT RUN: full PX4/YOLO load, flight/gimbal behavior, display-free EGL and GUI display-refresh FPS. Camera FPS with the GUI open is measured; GUI display-refresh FPS is not.

## Sources

- [Microsoft WSLg GPU selection](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg): Mesa adapter selection and its independence from CUDA.
- [Mesa D3D12 driver](https://docs.mesa3d.org/drivers/d3d12.html): D3D12 renderer configuration.
- [Gazebo Sim 8 headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html): distinction between server-only mode and display-free EGL rendering.
