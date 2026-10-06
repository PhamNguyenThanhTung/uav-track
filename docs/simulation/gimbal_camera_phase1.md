# Phase 1 - X500 Camera + Gimbal

## Goal
Demonstrate physical/mechanical integration of a 3-DOF gimbal and downward-facing RGB camera on a custom X500 UAV model in Gazebo Harmonic. Verify joint motion limits and image publication to ROS 2.

## Model
Custom X500 model (`custom_x500_gimbal_down`) extending the base PX4 X500 without modifying upstream PX4 source code or files.
- **Base**: Included `model://x500`
- **Mount**: Attached underneath `base_link`
- **Hierarchy**:
  ```text
  X500 body (base_link)
     └── gimbal_mount_link
            └── gimbal_yaw_joint (Yaw: -180° -> +180°)
                   └── gimbal_yaw_link
                          └── gimbal_roll_joint (Roll: -45° -> +45°)
                                 └── gimbal_roll_link
                                        └── gimbal_pitch_joint (Pitch: 0° -> +60°)
                                               └── camera_link (RGB Camera)
  ```

## Camera
- **Type**: RGB Monocular Camera
- **Orientation**: Pointing STRAIGHT DOWN (pitch=0°)
- **Resolution**: 1280 × 720
- **FPS**: 30
- **Horizontal FOV**: 90° (1.5707963 rad)
- **Gazebo Topic**: `/custom_x500/camera/image_raw`
- **ROS 2 Topic**: `/custom_x500/camera/image_raw`

## Gimbal Joint Limits
- **Yaw**: `-180°` to `+180°` (`-3.14159 rad` to `+3.14159 rad`)
- **Pitch**: `0°` to `+60°` (`0.0 rad` to `+1.0472 rad`, 0° = straight down, +60° = tilted up towards horizon)
- **Roll**: `-45°` to `+45°` (`-0.7854 rad` to `+0.7854 rad`)

## Files
- `simulation/models/custom_x500_gimbal_down/model.config`: Gazebo model configuration
- `simulation/models/custom_x500_gimbal_down/model.sdf`: Model definition with 3-DOF gimbal & RGB camera
- `simulation/worlds/gimbal_test.sdf`: Test world with lighting, physics, ground plane, visual target markers
- `config/sim/ros_gz_bridge.yaml`: ROS-Gazebo bridge configuration
- `simulation/run_sim_phase1.sh`: Helper script to set resource paths and launch Gazebo simulation

## How to run

1. **Launch Gazebo Simulation**:
   ```bash
   cd ~/UAV_Project/project
   ./simulation/run_sim_phase1.sh
   ```

2. **Launch ROS-Gazebo Bridge (in another terminal)**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=~/UAV_Project/project/config/sim/ros_gz_bridge.yaml
   ```

3. **Verify ROS 2 Camera Image Topic**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic hz /custom_x500/camera/image_raw
   ```

4. **Test Gimbal Joint Positions**:
   ```bash
   # Yaw to +90 deg (1.57 rad)
   gz topic -t /model/custom_x500_gimbal_down/custom_x500/gimbal/yaw_cmd -m gz.msgs.Double -p "data: 1.57"

   # Pitch to +30 deg (0.52 rad)
   gz topic -t /model/custom_x500_gimbal_down/custom_x500/gimbal/pitch_cmd -m gz.msgs.Double -p "data: 0.52"

   # Roll to +20 deg (0.35 rad)
   gz topic -t /model/custom_x500_gimbal_down/custom_x500/gimbal/roll_cmd -m gz.msgs.Double -p "data: 0.35"
   ```

## Test result
- **Physics Stability**: PASS (No oscillations or physics explosions)
- **Joint Limits**: PASS (Yaw [-180°, +180°], Pitch [0°, +60°], Roll [-45°, +45°] strictly enforced)
- **Camera Orientation**: PASS (Default optical axis points straight down)
- **Image Publication**: PASS (Gazebo camera image successfully bridged to `/custom_x500/camera/image_raw` on ROS 2)

## Known limitations
- Primitive geometry (cylinder/box) used for gimbal visualization for Phase 1.
- Standalone physics verification; PX4 SITL flight controller integration for gimbal commands will be configured in Phase 2.
