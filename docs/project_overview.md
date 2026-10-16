# Project Overview

This document provides a concise, high-level description of the UAV simulation platform, its verified technology stack, current operating architecture, and planned architectural progression.

---

## 1. Project Goal

The primary goal of this project is to develop and validate an autonomous UAV simulation platform capable of:
1. Simulating a multicopter with a 3-DOF stabilized gimbal and downward-facing camera.
2. Running real-time computer vision detection (YOLO26n) and tracking (BoT-SORT with ORB GMC and pretrained ReID).
3. Performing closed-loop visual servoing to maintain ground targets within camera view.
4. Progressing incrementally toward full autonomous target-following flight control.
5. Maintaining strict architectural boundaries to facilitate seamless transfer to physical drone hardware in future phases.

---

## 2. Verified Technology Stack

| Layer | Component | Version / Specification |
|---|---|---|
| **OS / Environment** | Ubuntu 24.04 LTS | Running on WSL2 (Linux kernel 6.6) |
| **Robotics Middleware** | ROS 2 Jazzy Jalisco | Standard desktop installation |
| **Physics Simulation** | Gazebo Harmonic | Gazebo Sim version 8.15.0 (`gz-sim8`) |
| **Flight Autopilot** | PX4 Autopilot SITL | v1.17.0 (standalone `gz_x500` airframe) |
| **DDS Bridge** | Micro XRCE-DDS Agent | UDP localhost port 8888 |
| **ROS-Gazebo Bridge** | `ros_gz_bridge` | Parameter bridge (`ros_gz_bridge.yaml`) |
| **Python Environment** | Python 3.12.3 virtualenv | Located at `software/venvs/yolo26` |
| **Deep Learning** | PyTorch 2.14.0+cu130 | CUDA acceleration on NVIDIA RTX 4060 GPU |
| **Object Detection** | Ultralytics YOLO26n | Model `yolo26n.pt` (Class 0: person, 640px) |
| **Multi-Object Tracker**| Ultralytics BoT-SORT 8.4.164 | ORB GMC, `yolo26n-reid.onnx` via ONNX Runtime CUDA, upstream association defaults in `src/uav_perception/config/botsort.yaml` |

---

## 3. Current Architecture

Perception and body flight control currently operate as **two decoupled subsystems**:

```text
[ Gazebo Harmonic Simulation ]
   │
   ├── Downward RGB Camera (1280x720) ───────► gz.msgs.Image
   └── Gimbal Position Actuators ◄─────────── gz.msgs.Double (Yaw, Pitch)
                           │                              ▲
                           ▼                              │
[ ros_gz_bridge ]                                         │
   ├── /custom_x500/camera/image_raw (sensor_msgs/Image)  │
   └── /custom_x500/gimbal/{yaw,pitch}_cmd ───────────────┘
                           │
                           ▼
[ uav_perception : yolo_detector ]
   ├── YOLO26n (GPU Detection)
   ├── BoT-SORT (Kalman + IoU / appearance association)
   │    ├── ORB global motion compensation
   │    └── pretrained ONNX Re-ID embeddings (CUDA)
   ├── TargetIdentityManager (Target Lifecycle & ID Switch Decoupling)
   └── GimbalController (Visual Servoing P-Control with Deadband & Rate Limits)
           │
           ├──► /yolo/detections (vision_msgs/Detection2DArray)
           └──► /yolo/annotated_image (sensor_msgs/Image)

─────────────────────────────────────────────────────────────────────────────
(NO CONNECTION / COMPLETELY INDEPENDENT)
─────────────────────────────────────────────────────────────────────────────

[ uav_control : altitude_control ]
   ├── Locks initial horizontal coordinates: (X_hold, Y_hold, Yaw_hold)
   ├── Streams Offboard position setpoint: (X_hold, Y_hold, -target_altitude_m)
   └── Sends Arming & Offboard mode transitions
           │
           ▼ (px4_msgs / DDS)
[ MicroXRCEAgent ] <───► [ PX4 SITL v1.17.0 ]
```

### Key Architectural Characteristics
- **Gimbal-Only Target Tracking**: When a target moves, only the gimbal pans and tilts to track it.
- **Stationary UAV Body**: The drone maintains its initial horizontal position $(X_{hold}, Y_{hold})$ and holds altitude. It does **not** follow the target horizontally.
- **Independent Safety**: Flight control does not depend on perception health; if perception crashes, the drone continues stable hovering.

---

## 4. Planned Architecture Progression

Progression from current implemented modules to future flight autonomy:

```text
Current Gimbal Tracker
       │
       ▼
[ Target Identity Layer ]          (IMPLEMENTED in source: Decouples track_id from target identity; unit-tested)
       │
       ▼
[ Target 3D State Estimator ]      (PROPOSED: Estimate ground position from pixel + gimbal + altitude)
       │
       ▼
[ MotionArbiter ]                  (PROPOSED: Distribute error between gimbal and UAV body)
       │
       ▼
[ UAV Body Following Controller ]  (PROPOSED: Velocity setpoints to chase target when gimbal saturates)
       │
       ▼
[ Target Loss Recovery ]           (PROPOSED: Search trajectory if target leaves view)
```
