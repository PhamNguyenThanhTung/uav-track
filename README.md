# UAV Simulation & Target Tracking Project

Autonomous UAV simulation platform integrating ROS 2 Jazzy, Gazebo Harmonic, PX4 Autopilot SITL, real-time YOLO26n perception, BoT-SORT with ORB global motion compensation and pretrained ReID, and closed-loop gimbal visual servoing.

---

## Overview

This project provides an end-to-end simulation environment for testing aerial perception and gimbal visual servoing on a custom X500 multicopter equipped with a 3-DOF stabilized gimbal and downward-facing camera.

---

## Current Capabilities

- **High-Fidelity Simulation**: Custom X500 model in Gazebo Harmonic with realistic physical joint constraints and sensors.
- **Flight Control Integration**: PX4 SITL v1.17 connected via Micro XRCE-DDS Agent.
- **Autonomous Takeoff & Altitude Hold**: C++ offboard node (`altitude_control`) commanding position hold and target altitude.
- **Real-Time Perception**: GPU-accelerated YOLO26n person detection and BoT-SORT multi-object tracking with ORB GMC and ONNX ReID.
- **Closed-Loop Gimbal Servoing**: Real-time proportional visual servoing keeping ground targets centered in the camera frame.
- **Evaluation Framework**: Frame-accurate camera benchmark tools calculating latency, tracking retention, and angular rates.

---

## Architecture

Perception and flight control are currently decoupled:

```text
Gazebo Camera ──► ros_gz_bridge ──► yolo_detector ──► Gimbal Yaw & Pitch Cmds
                             (YOLO + BoT-SORT + ORB GMC + ReID)

PX4 SITL ◄────── DDS ──────► MicroXRCEAgent ◄────── altitude_control (Position Hold)
```

For complete architectural details, see [docs/project_overview.md](docs/project_overview.md) and [docs/system_contract.md](docs/system_contract.md).

---

## Repository Structure

```text
project/
├── README.md                           # Project entry point
├── AGENTS.md                           # Operating contract for AI coding agents
├── PROGRESS.md                         # Current verified progress & limitations
├── .gitignore                          # Build, cache, and artifact ignore rules
│
├── config/
│   └── sim/ros_gz_bridge.yaml          # Topic mappings for ROS 2 <-> Gazebo
│
├── docs/
│   ├── project_overview.md             # System overview & technology stack
│   ├── system_contract.md              # Active topics, QoS, and control contracts
│   └── simulation/                     # Historical milestone reports (Phases 1-3)
│
├── simulation/
│   ├── models/                         # Custom models (custom_x500_gimbal_down, targets)
│   ├── worlds/gimbal_test.sdf          # Simulation world definition
│   └── run_sim_phase3.sh               # Gazebo launch helper
│
├── src/
│   ├── uav_control/                    # ROS 2 C++ offboard altitude control package
│   └── uav_perception/                 # ROS 2 Python YOLO26n + BoT-SORT perception package
│
├── evaluation/                         # Metric calculations and benchmark suite
│   ├── metric_calculator.py            # Tracking error and retention math
│   ├── benchmark_evaluator.py          # Real-time frame-matching benchmark node
│   ├── test_metric_calculator.py       # Unit tests for metric calculator
│   └── test_evaluator_matching.py      # Unit tests for frame matcher
│
├── benchmark_evaluator.py              # Convenience symlink -> evaluation/benchmark_evaluator.py
├── metric_calculator.py                # Convenience symlink -> evaluation/metric_calculator.py
└── launch_simulation.sh                # Master simulation launcher script
```

---

## Environment

- **OS**: Ubuntu 24.04 LTS (under WSL2)
- **ROS 2**: Jazzy Jalisco (`/opt/ros/jazzy`)
- **Gazebo**: Harmonic (Gazebo Sim 8.15.0)
- **PX4**: v1.17.0 (`../software/PX4-Autopilot`)
- **Python Virtualenv**: `../software/venvs/yolo26` (PyTorch, Ultralytics, ONNX Runtime GPU; see [src/uav_perception/README.md](src/uav_perception/README.md) for install details)

---

## Running the System

### Option A: All-in-One Master Launcher (Recommended)

```bash
cd /home/tungt/UAV_Project/project

# Launch simulation with 3D Gazebo GUI:
./launch_simulation.sh

# Or launch Headless (saves CPU & RAM):
./launch_simulation.sh --headless

# Or launch Headless with automated Offboard Takeoff to 2.5 m:
./launch_simulation.sh --headless --offboard 2.5
```

### GPU Rendering on WSL

On WSL with `/dev/dxg`, the Gazebo helpers automatically select Mesa's D3D12
driver and the NVIDIA adapter for world and camera rendering. This applies to
both GUI and `--headless` runs. Existing `GALLIUM_DRIVER` and
`MESA_D3D12_DEFAULT_ADAPTER_NAME` overrides are respected. To disable this
automatic selection for troubleshooting, use
`UAV_GAZEBO_GPU=0 ./launch_simulation.sh --headless`.

GPU rendering was verified on RTX 4060 at 1280×720: approximately 30 camera
frames per wall-clock second with the server alone, and 29 with the world GUI.
Physics still runs on the CPU. These measurements exclude PX4 flight and YOLO
inference load; see [GPU verification](docs/notes/gazebo_gpu_2026-10-06.md).

### Option B: Launching Perception & Visual Servoing

In a new terminal:
```bash
source /opt/ros/jazzy/setup.bash
source /home/tungt/UAV_Project/software/ros2_ws/install/setup.bash
source /home/tungt/UAV_Project/software/venvs/yolo26/bin/activate
source /home/tungt/UAV_Project/project/install/setup.bash

python3 -m uav_perception.yolo_detector
```

---

## Verification

### Unit Tests
Run lightweight unit tests without launching the simulator:

```bash
cd /home/tungt/UAV_Project

# 1. Metric calculator unit tests:
python3 project/evaluation/test_metric_calculator.py

# 2. Sliding buffer frame matcher unit tests (requires ROS 2 environment):
bash -c "source /opt/ros/jazzy/setup.bash && source software/ros2_ws/install/setup.bash && python3 project/evaluation/test_evaluator_matching.py"

# 3. Perception package unit tests (TargetIdentityManager & GimbalController):
PYTHONPATH=project/src/uav_perception python3 -m pytest project/src/uav_perception/test/
```

### Runtime Benchmark
To record tracking performance metrics against a running simulation (outputs save safely to `evaluation/results/runs/<timestamp>/` by default or custom `--output-dir`):
```bash
python3 project/evaluation/benchmark_evaluator.py --verify-sec 25
```

---

## Documentation Links

- [AGENTS.md](AGENTS.md): Operating contract and safety rules for AI coding assistants.
- [PROGRESS.md](PROGRESS.md): Verified milestones, known limitations, and next architectural steps.
- [docs/project_overview.md](docs/project_overview.md): High-level system design and component overview.
- [docs/system_contract.md](docs/system_contract.md): Detailed ROS 2 topic, QoS, and interface specifications.
- [src/uav_perception/README.md](src/uav_perception/README.md): Perception package modules, parameters, and dependency tiers.
- [src/uav_control/README.md](src/uav_control/README.md): Offboard altitude control node usage.

---

## Current Limitations

- **Gimbal-Only Tracking**: The UAV body hovers at its takeoff position and does not chase targets horizontally.
- **No MotionArbiter**: No arbitration component exists to distribute errors between the gimbal and drone body.
- **Tracking Continuity**: Identity management and gimbal control have component and simulation evidence, but yaw/crossing/occlusion/re-entry continuity still fails acceptance. Source-age checks reject results that expire during inference; see [current progress](PROGRESS.md) and [timing verification](docs/tracking/source_age_verification_2026-10-15.md) for current test/runtime results.
