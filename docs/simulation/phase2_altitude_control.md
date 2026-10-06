# Phase 2 - Altitude Control & Offboard Integration

> Status: HISTORICAL MILESTONE
>
> This document records the system state at the time of this milestone.
> It is not the current system source of truth.
> See `/PROGRESS.md`, `/docs/project_overview.md`, and `/docs/system_contract.md` for the current system state.



Phase 2 verifies the autonomous flight stack of PX4 in Gazebo Harmonic and establishes the project's first ROS 2 C++ offboard flight controller node:
1. **Part A — Baseline PX4 Flight Stack**: Validate takeoff to 2 m, stable hover, and safe landing using native PX4 commander commands on an X500 multicopter in Gazebo Harmonic.
2. **Part B — Custom ROS 2 Offboard Controller**: Build a ROS 2 C++ node (`altitude_control` in package `uav_control`) that communicates directly with PX4 via `px4_msgs` and `MicroXRCEAgent`. The node streams Offboard control setpoints, arms the vehicle, switches to Offboard mode, and commands arbitrary target altitudes (tested at 2 m, 3 m, and 5 m) while holding horizontal ($X, Y$) position without drift. Target altitude can be dynamically configured via ROS 2 parameters.

---

## System Architecture

```text
+------------------------------------+
|  uav_control (altitude_control)    |
|  - Streams OffboardControlMode     |
|  - Streams TrajectorySetpoint (NED)|
|  - Sends VehicleCommand (Arm/Mode) |
|  - Subscribes VehicleLocalPosition |
+------------------▲-----------------+
                   │ ROS 2 Jazzy (px4_msgs)
                   ▼
+------------------------------------+
|       Micro-XRCE-DDS-Agent         |
|       (UDP 127.0.0.1:8888)         |
+------------------▲-----------------+
                   │ uORB <-> DDS
                   ▼
+------------------------------------+
|       PX4 Autopilot SITL           |
|       (v1.17.0 / gz_x500)          |
+------------------------------------+
```

### Topic & QoS Details (PX4 v1.17 uORB)
- **FMU Publications**:
  - `/fmu/out/vehicle_local_position_v1` (`px4_msgs/msg/VehicleLocalPosition`): Used to monitor current NED position and altitude ($z = -altitude$). Subscribed with QoS `Best Effort`, `Transient Local` (depth 10).
  - Fallback topic: `/fmu/out/vehicle_local_position`.
- **FMU Subscriptions**:
  - `/fmu/in/offboard_control_mode` (`px4_msgs/msg/OffboardControlMode`): Published at 10 Hz with `position = true`. QoS `Best Effort`, `Volatile` (depth 10).
  - `/fmu/in/trajectory_setpoint` (`px4_msgs/msg/TrajectorySetpoint`): Published at 10 Hz with target NED position $(X_{ref}, Y_{ref}, -Z_{target})$ and yaw. QoS `Best Effort`, `Volatile` (depth 10).
  - `/fmu/in/vehicle_command` (`px4_msgs/msg/VehicleCommand`): Used to send arming commands (`VEHICLE_CMD_COMPONENT_ARM_DISARM`) and offboard mode switch commands (`VEHICLE_CMD_DO_SET_MODE`). QoS `Best Effort`, `Volatile` (depth 10).

---

## Files Created

- `project/src/uav_control/CMakeLists.txt`: Build configuration for `uav_control` package depending on `rclcpp` and `px4_msgs`.
- `project/src/uav_control/package.xml`: Package metadata and dependencies.
- `project/src/uav_control/src/altitude_control.cpp`: ROS 2 C++ node implementing offboard control, automatic arming, offboard switching, horizontal position locking, and altitude tracking.
- `project/src/uav_control/README.md`: Package documentation and usage instructions.
- `project/docs/simulation/phase2_altitude_control.md`: This report.

---

## How to Run

### Step 1: Start Micro XRCE-DDS Agent
```bash
source /opt/ros/jazzy/setup.bash
source ~/UAV_Project/software/ros2_ws/install/setup.bash
MicroXRCEAgent udp4 -p 8888
```

### Step 2: Start PX4 SITL with Gazebo Harmonic
```bash
source /opt/ros/jazzy/setup.bash
cd ~/UAV_Project/software/PX4-Autopilot
make px4_sitl gz_x500
```

### Step 3: Run the Altitude Control Node
In a new terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/UAV_Project/software/ros2_ws/install/setup.bash
source ~/UAV_Project/project/install/setup.bash

# Test 2.0 m
ros2 run uav_control altitude_control --ros-args -p target_altitude_m:=2.0

# Test 3.0 m
ros2 run uav_control altitude_control --ros-args -p target_altitude_m:=3.0

# Test 5.0 m
ros2 run uav_control altitude_control --ros-args -p target_altitude_m:=5.0
```

### Dynamic Altitude Reconfiguration:
While the node is running, altitude can be changed dynamically without rebuilding:
```bash
ros2 param set /altitude_control target_altitude_m 3.5
```

---

## Test Results

### Part A — PX4 Baseline Flight Stack

| Test Item | Command / Parameter | Result | Details |
|---|---|---|---|
| Takeoff 2 m | `param set MIS_TAKEOFF_ALT 2`<br>`commander takeoff` | **PASS** | Armed automatically, climbed smoothly to $z = -1.97\text{ m}$, hovered stably without drift. |
| Hold Altitude | Auto-loiter after takeoff | **PASS** | Held steady at altitude ~1.97 m with zero oscillations. |
| Land | `commander land` | **PASS** | Executed controlled descent, detected touchdown, auto-disarmed. |

### Part B — ROS 2 Offboard Controller (`altitude_control`)

| Test Item | Parameters / Command | Result | Measured Performance |
|---|---|---|---|
| Package Build | `colcon build --packages-select uav_control` | **PASS** | Compiled cleanly with 0 warnings and 0 errors. |
| Offboard & Arm | `altitude_control` node startup | **PASS** | Offboard setpoints pre-streamed >1 s, armed via `VEHICLE_CMD_COMPONENT_ARM_DISARM`, switched to custom mode 6 (Offboard). |
| Altitude 2 m | `-p target_altitude_m:=2.0` | **PASS** | Reached 2.01 m. Maintained stable hover at $X = 0.03\text{ m}, Y = -0.00\text{ m}$. |
| Altitude 3 m | `-p target_altitude_m:=3.0` | **PASS** | Reached 3.01 m. Maintained stable hover at $X = 0.01\text{ m}, Y = -0.02\text{ m}$. |
| Altitude 5 m | `-p target_altitude_m:=5.0` | **PASS** | Reached 5.00 m. Maintained stable hover at $X = 0.00\text{ m}, Y = -0.01\text{ m}$. |

---

## Key Technical Takeaways & SITL Tuning
1. **uORB Versioning**: In PX4 v1.17, publications from FMU have version suffixes (e.g. `/fmu/out/vehicle_local_position_v1`). The node was constructed with fallback subscription logic to support both versioned and unversioned uORB topic names seamlessly.
2. **QoS Profile Matching**: PX4 DDS publications require `QoS(10).best_effort().transient_local()`. Mismatched QoS will prevent `vehicle_local_position` messages from being received by ROS 2 nodes.
3. **Simulated Battery Drain in SITL**: Default Gazebo battery simulation drains within 60 s (`SIM_BAT_DRAIN 60`), which can trigger battery RTL failsafe during long offboard tests. Setting `param set SIM_BAT_ENABLE 0` in SITL ensures uninterrupted controller evaluation.
