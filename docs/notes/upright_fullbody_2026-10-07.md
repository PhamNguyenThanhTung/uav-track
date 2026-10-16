# Upright camera and full-body framing — 2026-10-07

The previous camera pose looked down with sensor up pointing toward the ground when tilted; raw images were inverted. At a 2 m hover the nearby walking actor also exceeded the useful vertical field of view.

## Changes

- Sensor pose roll changed from zero to pi, retaining pitch pi/2 and the optical ray direction. Raw images are upright without software image flipping.
- Yaw and pitch visual-servo error signs reversed to match the upright camera frame.
- UAV spawn moved from `(0, 0, 0.24)` to `(0, -2, 0.24)` m. Actor trajectory is unchanged; UAV holds starting XY.
- Initial pitch is 1.15 rad. Simulated pitch upper limit increased consistently in SDF/controller/node from 60 to 70 degrees: the first 60-degree run had 35/98 available boxes within 5 pixels of the top edge.
- Existing forward camera mount clearance fix retained. Newly observed target-facing pitch ~69.6 degrees shows no landing gear; unrestricted yaw/roll clearance remains unverified.

## Verification

PASS: `SDF_PATH="$PWD/simulation/models:$PWD/../software/PX4-Autopilot/Tools/simulation/gz/models" gz sdf -k simulation/models/custom_x500_gimbal_down/model.sdf`.

PASS: `colcon build --packages-select uav_perception` after sourcing ROS and workspace dependencies.

PASS: `PYTHONPATH="src/uav_perception:$PYTHONPATH" /usr/bin/python3 -m pytest src/uav_perception/test/ -q`: 33 tests, including projection through the actual SDF optical orientation and joint-limit consistency. Initial attempts overwrote ROS PYTHONPATH and failed collection; the correctly sourced invocation above passed.

PASS: `git diff --check`.

Runtime: `./launch_simulation.sh --offboard 2.0`, YOLO26 CUDA/DeepSORT, ROS-Gazebo bridge, PX4 SITL and gimbal tracking. A 25-second startup run lost its initial target during ascent and did not reacquire; its evidence is retained in `evaluation/results/runs/upright_fullbody_70deg_20261007/`. Perception was restarted after hover stabilized for the focused framing check; this does not resolve the startup failure.

PASS for observed upright framing at hover: `evaluation/benchmark_evaluator.py --verify-sec 25 --output-dir evaluation/results/runs/upright_fullbody_70deg_20261007/hover` produced 210 received camera frames, 147 available raw person detections, 118 available tracking outputs, zero observed tracking losses/ID switches. 92 tracking matches were incomplete, so the reported 100% retention applies only to available tracking outputs. Median processing 44.3 ms, p95 74.5 ms; mean radial error 39.2 pixels. All 118 available tracked boxes remained within image boundaries: minimum top margin 97.7 px and bottom margin 109.5 px. These box margins and inspected raw/annotated snapshots support full-body framing for this sampled hover interval, not every actor phase or flight altitude.

The independent 20-second camera monitor measured altitude 1.968–2.019 m, Gazebo camera 29.32 wall FPS, ROS observer 7.51 FPS, and a 7.03-second realtime clock jump. Raw snapshot shows the walking actor upright, head and feet visible, and no landing gear. See [raw image](../../evaluation/results/runs/upright_fullbody_70deg_20261007/hover/perception.png) and [metrics](../../evaluation/results/runs/upright_fullbody_70deg_20261007/hover/framing_metrics.json).

Full Phase 4A acceptance remains FAIL: PX4 failsafe/RTL and host clock jumps persist, and startup target reacquisition failed in the captured run. No clock-service or safety-threshold changes were made. Historical runs and external software were preserved.
