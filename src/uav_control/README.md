# uav_control

ROS 2 package providing offboard altitude control for UAV via PX4 Autopilot (`px4_msgs`).

## Overview

The `altitude_control` node connects directly to PX4 via Micro XRCE-DDS Agent using `px4_msgs` and `rclcpp` (without `px4_ros_com`).
It continuously streams `OffboardControlMode` and `TrajectorySetpoint` messages, commands Offboard mode and vehicle arming, and maintains the current horizontal position (X/Y) while regulating altitude in NED coordinates ($z = -\text{target\_altitude}$).

## Parameters

- `target_altitude_m` (double, default: `2.0`): Target altitude in meters above origin.
- `publish_rate_hz` (double, default: `10.0`): Rate at which setpoints and offboard modes are published.

Parameters can be set at launch or dynamically updated via `ros2 param set`.

## Subscribed Topics

- `/fmu/out/vehicle_local_position_v1` (and `/fmu/out/vehicle_local_position`) [`px4_msgs/msg/VehicleLocalPosition`]: Drone position and heading in NED frame.
- `/fmu/out/vehicle_status_v1` (and `/fmu/out/vehicle_status`) [`px4_msgs/msg/VehicleStatus`]: Drone arming state and navigation mode.

## Published Topics

- `/fmu/in/offboard_control_mode` [`px4_msgs/msg/OffboardControlMode`]: Declares position control active.
- `/fmu/in/trajectory_setpoint` [`px4_msgs/msg/TrajectorySetpoint`]: Desired 3D position setpoint ($X_{hold}, Y_{hold}, -Z_{target}$) and yaw.
- `/fmu/in/vehicle_command` [`px4_msgs/msg/VehicleCommand`]: Commands for mode transitions (`VEHICLE_CMD_DO_SET_MODE`) and arming (`VEHICLE_CMD_COMPONENT_ARM_DISARM`).

## Build

```bash
cd /home/tungt/UAV_Project/project
source /opt/ros/jazzy/setup.bash
source /home/tungt/UAV_Project/software/ros2_ws/install/setup.bash
colcon build --packages-select uav_control
```

## Run

Default (2 m):
```bash
ros2 run uav_control altitude_control
```

Custom altitude (e.g. 3 m):
```bash
ros2 run uav_control altitude_control --ros-args -p target_altitude_m:=3.0
```

Custom altitude (e.g. 5 m):
```bash
ros2 run uav_control altitude_control --ros-args -p target_altitude_m:=5.0
```
