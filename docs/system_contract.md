# System Interface Contract

This document specifies the exact active ROS 2 and Gazebo interfaces, message types, Quality of Service (QoS) profiles, and control contracts between subsystems.

---

## 1. Camera Sensor Interface

| Topic Name | Message Type | Direction | QoS Profile | Frequency |
|---|---|---|---|---|
| `/custom_x500/camera/image_raw` | `sensor_msgs/msg/Image` | GZ -> ROS 2 | Best Effort, Volatile, Depth 1 | ~30 Hz nominal (~2.5-3.0 Hz headless CPU) |
| `/custom_x500/camera/camera_info`| `sensor_msgs/msg/CameraInfo` | GZ -> ROS 2 | Best Effort, Volatile, Depth 1 | Matches image stream |

- **Image Dimensions**: 1280 (width) × 720 (height)
- **Pixel Encoding**: `rgb8`
- **Default Optical Axis**: Points straight down (pitch = 0°) along -Z of `camera_link`

---

## 2. Gimbal Actuation Interfaces

| Topic Name | Message Type | Direction | Bridged in config? | Actively Used in Control? | Range / Limits |
|---|---|---|---|---|---|
| `/custom_x500/gimbal/yaw_cmd` | `std_msgs/msg/Float64` | ROS 2 -> GZ | **YES** | **YES** (by `yolo_detector`) | $[-3.14159, +3.14159]\text{ rad}$ ($\pm 180^\circ$) |
| `/custom_x500/gimbal/pitch_cmd`| `std_msgs/msg/Float64` | ROS 2 -> GZ | **YES** | **YES** (by `yolo_detector`) | $[0.0, +1.0472]\text{ rad}$ ($0^\circ \text{ to } +60^\circ$) |
| `/custom_x500/gimbal/roll_cmd` | `std_msgs/msg/Float64` | ROS 2 -> GZ | **YES** | **NO** (idle / zero) | $[-0.7854, +0.7854]\text{ rad}$ ($\pm 45^\circ$) |

> **IMPORTANT**: Although `roll_cmd` is bridged in `ros_gz_bridge.yaml`, the current visual servoing algorithm in `yolo_detector.py` **only commands yaw and pitch**. Roll is left unactuated (held at 0.0 rad).

---

## 3. Perception Output Interfaces

| Topic Name | Message Type | Publisher Node | QoS Profile | Description |
|---|---|---|---|---|
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | `uav_perception/yolo_detector` | Reliable, Volatile, Depth 10 | 2D bounding boxes, active track ID, and detection confidence |
| `/yolo/annotated_image` | `sensor_msgs/msg/Image` | `uav_perception/yolo_detector` | Reliable, Volatile, Depth 10 | Rendered RGB image with HUD reticle, error vector, and tracking telemetry |

---

## 4. PX4 Autopilot Offboard Interfaces

Published and subscribed by `uav_control/altitude_control` via `MicroXRCEAgent` (DDS port 8888):

| Topic Name | Message Type | Direction | QoS Profile | Description |
|---|---|---|---|---|
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Node -> PX4 | Best Effort, Volatile, Depth 10 | Streams position mode enabled (`position = true`) at 10 Hz |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | Node -> PX4 | Best Effort, Volatile, Depth 10 | Desired NED position: $(X_{hold}, Y_{hold}, -Z_{target})$ and $Yaw_{hold}$ |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | Node -> PX4 | Best Effort, Volatile, Depth 10 | Sends Arming (`ARM_DISARM`) and Offboard mode (`DO_SET_MODE: 1, 6`) commands |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` | PX4 -> Node | Best Effort, Transient Local, Depth 10 | Vehicle NED coordinates, velocity, and heading. Node falls back to `/fmu/out/vehicle_local_position` if unversioned. |

---

## 5. Current Control Contract

1. **Perception controls the gimbal only**:
   - `uav_perception/yolo_detector` calculates pixel errors $(e_x, e_y)$ relative to the image center $(640, 360)$.
   - It directly updates and publishes `/custom_x500/gimbal/yaw_cmd` and `/custom_x500/gimbal/pitch_cmd`.
   - Perception does **not** command UAV body position, velocity, or attitude.

2. **Flight control maintains static position hold + target altitude**:
   - `uav_control/altitude_control` locks the horizontal coordinates $(X_{hold}, Y_{hold}, Yaw_{hold})$ observed at node startup.
   - It regulates the vertical axis to $-target\_altitude\_m$ in NED frame.
   - It is completely unaware of targets, detections, and gimbal orientation.

3. **Absence of MotionArbiter**:
   - There is **no arbitration layer** linking target position to vehicle movement.
   - Any future work introducing target-following flight must create this layer explicitly without altering `altitude_control` without an approved plan.
