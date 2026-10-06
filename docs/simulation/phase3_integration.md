# Phase 3 - Integration of X500 + Camera + Gimbal with PX4 SITL

## 1. System Architecture

Phase 3 unifies the physical model from Phase 1 (`custom_x500_gimbal_down` with 3-DOF gimbal and downward RGB camera) and the autonomous flight stack from Phase 2 (PX4 SITL + Micro XRCE-DDS Agent + ROS 2 `altitude_control` offboard node).

```text
+---------------------------------------------------------------------------------+
|                                 ROS 2 Jazzy                                     |
|  - Node: uav_control/altitude_control (Offboard flight controller)              |
|  - Gimbal Commands: /custom_x500/gimbal/{yaw,pitch,roll}_cmd (std_msgs/Float64) |
|  - Camera Subscriber: /custom_x500/camera/image_raw (sensor_msgs/Image)         |
+-------------------▲-----------------------------------------▲-------------------+
                    │ px4_msgs (DDS)                          │ ros_gz_bridge
                    ▼                                         ▼
+------------------------------------+      +-------------------------------------+
|        MicroXRCEAgent              |      |           ros_gz_bridge             |
|       (UDP 127.0.0.1:8888)         |      | (Camera Image + Gimbal Joint Cmds)  |
+-------------------▲----------------+      +------------------▲------------------+
                    │ uORB                                     │ Gazebo Transport
                    ▼                                          ▼
+------------------------------------+      +-------------------------------------+
|        PX4 Autopilot SITL          |      |          Gazebo Harmonic            |
|       (v1.17.0 / gz_x500)          |<====>|  World: gimbal_test.sdf             |
|  Attached to: custom_x500_...      | gz_  |  Model: custom_x500_gimbal_down     |
+------------------------------------+bridge+-------------------------------------+
```

### Integration Mechanism with PX4 v1.17
- The model `custom_x500_gimbal_down` includes `model://x500` with `merge="true"`, inheriting motor plugins (`MulticopterMotorModel` for rotors 0..3) and sensors (`imu_sensor`, `air_pressure_sensor`, `magnetometer_sensor`, `navsat_sensor`).
- PX4 SITL starts using the default `4001_gz_x500` airframe with `PX4_GZ_MODEL_NAME=custom_x500_gimbal_down`.
- PX4's `px4-rc.gzsim` detects the running Gazebo world `gimbal_test` and binds `gz_bridge` directly to `custom_x500_gimbal_down` without modifying any PX4 source files or CMake configurations.

---

## 2. Files Modified & Created

All files reside inside `/home/tungt/UAV_Project/project`:

- **Modified Files**:
  - `simulation/worlds/gimbal_test.sdf`: Added essential Gazebo simulation system plugins (`Imu`, `AirPressure`, `Magnetometer`, `NavSat`, `Contact`, `ApplyLinkWrench`), earth magnetic field, spherical coordinates, and adjusted model initial ground pose to `0 0 0.24 0 0 0`.
  - `simulation/models/custom_x500_gimbal_down/model.sdf`: Added `JointStatePublisher` system plugin for `/model/custom_x500_gimbal_down/joint_states`.
  - `config/sim/ros_gz_bridge.yaml`: Added ROS 2 -> Gazebo bridges for `yaw_cmd`, `pitch_cmd`, and `roll_cmd`.
  - `src/uav_control/src/altitude_control.cpp`: Added `has_taken_off_` check to prevent re-triggering arm/offboard during manual or autonomous landing.

- **Created Files**:
  - `launch_simulation.sh`: All-in-one launch script managing the lifecycle of Micro XRCE-DDS Agent, Gazebo Harmonic, ROS-Gazebo bridge, and PX4 SITL with automated cleanup on exit.
  - `simulation/run_sim_phase3.sh`: Environment configuration and launch script for Gazebo Harmonic with custom world and model paths.
  - `docs/simulation/phase3_integration.md`: This integration report.

---

## 3. How to Launch Simulation

### Option A: All-in-One Master Launcher (Recommended)
You can launch the entire simulation stack in a single command:

```bash
cd ~/UAV_Project/project

# Launch with Gazebo 3D GUI:
./launch_simulation.sh

# Or launch Headless (saves CPU / RAM):
./launch_simulation.sh --headless

# Or launch with automated Offboard Takeoff to 2.0 m:
./launch_simulation.sh --offboard 2.0
```
When you press `Ctrl+C` in this terminal, the script automatically shuts down all components cleanly.

### Option B: Manual Multi-Terminal Launch
If you prefer running each component in a separate terminal:

### Step 1: Start Micro XRCE-DDS Agent
```bash
source /opt/ros/jazzy/setup.bash
source ~/UAV_Project/software/ros2_ws/install/setup.bash
MicroXRCEAgent udp4 -p 8888
```

### Step 2: Start Gazebo Simulation
```bash
source /opt/ros/jazzy/setup.bash
~/UAV_Project/project/simulation/run_sim_phase3.sh -s
```
*(Omit `-s` if running with Gazebo GUI).*

### Step 3: Start ROS-Gazebo Bridge
```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/home/tungt/UAV_Project/project/config/sim/ros_gz_bridge.yaml
```

### Step 4: Start PX4 SITL attached to Custom Model
```bash
source /opt/ros/jazzy/setup.bash
cd ~/UAV_Project/software/PX4-Autopilot
PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_NAME=custom_x500_gimbal_down ./build/px4_sitl_default/bin/px4
```

### Step 5: Start Offboard Altitude Controller
```bash
source /opt/ros/jazzy/setup.bash
source ~/UAV_Project/software/ros2_ws/install/setup.bash
source ~/UAV_Project/project/install/setup.bash
ros2 run uav_control altitude_control --ros-args -p target_altitude_m:=2.0
```

---

## 4. Camera Topics & Performance

| Parameter | Value |
|---|---|
| Image Topic | `/custom_x500/camera/image_raw` |
| Camera Info Topic | `/custom_x500/camera/camera_info` |
| Message Type | `sensor_msgs/msg/Image` |
| Resolution | 1280 × 720 |
| Encoding | `rgb8` |
| Measured FPS | ~2.5 Hz (Headless software rendering with OGRE2 CPU rasterization) |
| Camera Orientation | Default optical axis points straight down (0° pitch) |

---

## 5. Gimbal Control Topics

| Axis | ROS 2 Topic | Type | Limits (rad) | Limits (deg) |
|---|---|---|---|---|
| Yaw | `/custom_x500/gimbal/yaw_cmd` | `std_msgs/msg/Float64` | `[-3.14159, +3.14159]` | `[-180°, +180°]` |
| Pitch | `/custom_x500/gimbal/pitch_cmd` | `std_msgs/msg/Float64` | `[0.0, +1.04720]` | `[0°, +60°]` |
| Roll | `/custom_x500/gimbal/roll_cmd` | `std_msgs/msg/Float64` | `[-0.78540, +0.78540]` | `[-45°, +45°]` |

---

## 6. Verification Results (Tests A – E)

### Test A — Startup & Discovery: PASS
- MicroXRCEAgent connected: `client_key: 0x00000001, session established`.
- PX4 attached to `custom_x500_gimbal_down` in world `gimbal_test`.
- EKF2 converged, sensors initialized, `Preflight check: OK`.

### Test B — Camera Verification: PASS
- `sensor_msgs/msg/Image` received continuously on `/custom_x500/camera/image_raw`.
- Verified width: 1280, height: 720, encoding: `rgb8`.
- Captured ground frame confirms downward orientation.

### Test C — Gimbal Motion While Landed: PASS
- Tested command sequence via ROS 2 topics:
  - Yaw = 0.3 rad, Pitch = 0.3 rad, Roll = 0.2 rad.
  - Joint positions reached `yaw: 0.289 rad`, `pitch: 0.278 rad`, `roll: 0.182 rad`.
  - Commanded all three back to 0.0 rad: returned to `~0.0 rad`.
  - Axes respond independently within defined limits without body collisions.

### Test D — In-Flight Altitude Control & Camera: PASS
- Offboard mode commanded, UAV armed and took off to target altitude 2.0 m.
- UAV held altitude at `1.99 m` and locked horizontal position `(X=-0.00, Y=0.02)`.
- Camera continuously published images while airborne. Aerial image captured at 2 m clearly resolved ground target markers (red box, green cylinder, blue sphere) and UAV shadow.
- Gimbal commanded in flight (pitch 0.3 rad, yaw 0.2 rad, roll 0.1 rad): joints responded in real time while maintaining hover.
- Tested climb to 3.0 m via parameter `target_altitude_m:=3.0`: UAV climbed smoothly and stabilized at `2.98 m`.

### Test E — Landing & Disarm: PASS
- Commanded `commander land`: PX4 switched to Land mode.
- UAV executed controlled descent, touched down at ground level, and auto-disarmed (`Disarmed by landing`).
- Node stopped cleanly without erratic behavior.

---

## 7. Known Limitations & Technical Constraints

1. **Software Rendering FPS**: In headless/virtualized environments without dedicated GPU passthrough, Gazebo Harmonic's OGRE2 engine uses LLVMpipe software rendering, limiting camera publish rate to ~2.5–3 Hz. Update rate increases to 30 FPS when running with hardware OpenGL acceleration.
2. **Gimbal Pitch Range**: By design, pitch is limited between 0° (straight down) and +60° (up towards horizon). Negative pitch (tilting backward) is mechanically restricted to avoid camera-to-skid collisions.
