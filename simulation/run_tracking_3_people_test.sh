#!/usr/bin/env bash
# Isolated runtime launcher for the three-person tracking experiment.
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SOFTWARE_DIR="$(cd "${PROJECT_DIR}/.." && pwd)/software"
PX4_DIR="${SOFTWARE_DIR}/PX4-Autopilot"
ROS2_WS="${SOFTWARE_DIR}/ros2_ws"
WORLD_NAME="${TRACKING_WORLD:-tracking_3_people_reentry_test}"
STOP_SIM_TIME="${TRACKING_STOP_SIM_TIME:-236.0}"
WORLD_FILE="${PROJECT_DIR}/simulation/worlds/${WORLD_NAME}.sdf"
BRIDGE_CONFIG="${PROJECT_DIR}/config/sim/ros_gz_bridge.yaml"
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
MODEL_DIR="${PROJECT_DIR}/simulation/models"
PX4_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
PX4_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"
PX4_PLUGINS="${PX4_DIR}/build/px4_sitl_default/src/modules/simulation/gz_plugins"

for required in /opt/ros/jazzy/setup.bash "${ROS2_WS}/install/setup.bash" "${PROJECT_DIR}/install/setup.bash"; do
    [[ -f "${required}" ]] || { echo "Missing ${required}" >&2; exit 2; }
done
[[ -x "${PX4_BIN}" ]] || { echo "Missing PX4 SITL: ${PX4_BIN}" >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
source "${ROS2_WS}/install/setup.bash"
source "${PROJECT_DIR}/install/setup.bash"
source "${PROJECT_DIR}/config/sim/gazebo_rendering.sh"
export GZ_SIM_RESOURCE_PATH="${MODEL_DIR}:${PX4_MODELS}:${PX4_WORLDS}:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_PLUGINS}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export SDF_PATH="${MODEL_DIR}:${PX4_MODELS}:${SDF_PATH:-}"
unset PX4_SIM_MODEL
export PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD="${WORLD_NAME}" PX4_GZ_MODEL_NAME=custom_x500_gimbal_down

RUN_ID="tracking_3_people_$(date -u +%Y%m%dT%H%M%SZ)"
OUT="${PROJECT_DIR}/evaluation/results/runs/${RUN_ID}"
mkdir -p "${OUT}"
PERCEPTION_SOURCE_DIR="${TRACKING_PERCEPTION_SOURCE_DIR:-${PROJECT_DIR}/src/uav_perception}"
cp "${PERCEPTION_SOURCE_DIR}/uav_perception/target_identity.py" "${OUT}/target_identity_snapshot.py"
PIDS=()
CONTROL_PID=""
CONTROL_NODE="altitude_control"
OBSERVER_STATUS=0
LANDED=false

start() {
    local name="$1"; shift
    setsid "$@" >"${OUT}/${name}.log" 2>&1 &
    local pid=$!
    PIDS+=("${pid}")
    printf '%s\n' "${pid}" >"${OUT}/${name}.pid"
    echo "Started ${name}; log: ${OUT}/${name}.log"
}
local_z() {
    local message
    message="$(timeout 4 ros2 topic echo --once /fmu/out/vehicle_local_position_v1 2>/dev/null || true)"
    awk '/^[[:space:]]*z:/{print $2; exit}' <<<"${message}"
}
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    # A startup failure may leave physics paused; resume before requesting land.
    if [[ -f "${OUT}/startup_pause.log" ]] \
        && ! rg -q 'data: true' "${OUT}/startup_resume.log" 2>/dev/null; then
        gz service -s "/world/${WORLD_NAME}/control" --reqtype gz.msgs.WorldControl \
            --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false' \
            >"${OUT}/cleanup_resume.log" 2>&1 || true
    fi
    if [[ -n "${CONTROL_PID}" ]] && kill -0 "${CONTROL_PID}" 2>/dev/null; then
        echo "Requesting descent to 0 m and checking local-position telemetry..."
        timeout 8 ros2 param set "/${CONTROL_NODE}" target_altitude_m 0.0 >"${OUT}/landing_request.log" 2>&1 || true
        for _ in $(seq 1 18); do
            z="$(local_z || true)"
            if [[ -n "${z}" ]] && awk -v z="${z}" 'BEGIN{if ((z<0?-z:z)<0.16) exit 0; exit 1}'; then
                LANDED=true; echo "Ground altitude observed (local NED z=${z} m)."; break
            fi
            sleep 2
        done
        [[ "${LANDED}" == true ]] || echo "WARNING: ground altitude was not confirmed (last z=${z:-unknown})." >&2
    fi
    # Stop perception while /clock and ROS-Gazebo transport are still alive.
    # This lets rclpy finish its current callback before the bridge/simulator exit.
    if [[ -f "${OUT}/perception.pid" ]]; then
        local perception_pid
        perception_pid="$(cat "${OUT}/perception.pid")"
        if kill -0 "${perception_pid}" 2>/dev/null; then
            kill -INT -- "-${perception_pid}" 2>/dev/null || true
            sleep 1
        fi
    fi
    local pid
    for pid in "${PIDS[@]}"; do
        [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && kill -TERM -- "-${pid}" 2>/dev/null || true
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && kill -KILL -- "-${pid}" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    printf 'landed=%s\nobserver_exit=%s\n' "${LANDED}" "${OBSERVER_STATUS}" >"${OUT}/cleanup_status.txt"
    echo "Stopped this test's Gazebo/PX4/ROS processes. Artifacts: ${OUT}"
    exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

commit="$(git -C "${PROJECT_DIR}" rev-parse --short HEAD 2>/dev/null || echo unknown)"
cat >"${OUT}/run_manifest.json" <<EOF
{
  "experiment": "Production YOLO26 + BoT-SORT + ORB GMC + ONNX ReID runtime test",
  "run_id": "${RUN_ID}", "project_commit": "${commit}",
  "world": "${WORLD_FILE}", "bridge_config": "${OUT}/ros_gz_bridge.yaml",
  "command": "./simulation/run_tracking_3_people_test.sh",
  "stop_sim_time_s": ${STOP_SIM_TIME},
  "uav_spawn_enu_m": [0, -2, 0.24], "target_altitude_m": 3.0,
  "fixed_gimbal_yaw_pitch_rad": [-1.5708, 0.82],
  "test_intervals_world_sim_s": {"static_camera": [50,62], "gimbal_yaw": [62,66], "crossing": [66,82], "occlusion_crossing": [82,102], "repeated_crossing_with_vertical_translation": [102,162], "short_exit_reentry": [162,188], "long_exit_reentry": [188,236]}
}
EOF
echo "Artifact directory: ${OUT}"
sed "s|/world/gimbal_test/clock|/world/${WORLD_NAME}/clock|" \
    "${BRIDGE_CONFIG}" >"${OUT}/ros_gz_bridge.yaml"
BRIDGE_CONFIG="${OUT}/ros_gz_bridge.yaml"

start agent MicroXRCEAgent udp4 -p 8888
sleep 1
start gazebo gz sim -r -s "${WORLD_FILE}"
ready=false
for _ in $(seq 1 35); do
    pid="$(cat "${OUT}/gazebo.pid")"
    if ! kill -0 "${pid}" 2>/dev/null; then echo "Gazebo exited; see ${OUT}/gazebo.log" >&2; exit 3; fi
    if gz topic -l 2>/dev/null | rg -q "^/world/${WORLD_NAME}/clock$"; then ready=true; break; fi
    sleep 1
done
[[ "${ready}" == true ]] || { echo "Gazebo world clock not found." >&2; exit 4; }
POSE_TOPIC="/world/${WORLD_NAME}/pose/info"
if gz topic -l 2>/dev/null | rg -q "^${POSE_TOPIC}$"; then
    start actor_pose_trace gz topic -e -t "${POSE_TOPIC}"
fi
start bridge env FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${FASTRTPS_DEFAULT_PROFILES_FILE:-${PROJECT_DIR}/src/uav_perception/config/fastdds_images.xml}}" \
    ros2 run ros_gz_bridge parameter_bridge --ros-args -p "config_file:=${BRIDGE_CONFIG}"
ready=false
for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | rg -q '^/clock$' \
       && ros2 topic list 2>/dev/null | rg -q '^/custom_x500/camera/image_raw$'; then
        ready=true; break
    fi
    sleep 1
done
[[ "${ready}" == true ]] || { echo "ROS clock/camera topics were not bridged." >&2; exit 4; }
start px4 bash -c "cd '${PX4_DIR}' && exec '${PX4_BIN}' -d"
PX4_PID="$(cat "${OUT}/px4.pid")"
ready=false
for _ in $(seq 1 45); do
    if ! kill -0 "${PX4_PID}" 2>/dev/null; then echo "PX4 exited; see ${OUT}/px4.log" >&2; exit 5; fi
    if ros2 topic list 2>/dev/null | rg -q '^/fmu/out/vehicle_local_position_v1$'; then ready=true; break; fi
    sleep 1
done
[[ "${ready}" == true ]] || { echo "PX4 local-position topic was not found." >&2; exit 6; }

timeout 8 ros2 topic pub --once /custom_x500/gimbal/yaw_cmd std_msgs/msg/Float64 '{data: -1.5708}' >"${OUT}/gimbal_yaw.log" 2>&1 || true
timeout 8 ros2 topic pub --once /custom_x500/gimbal/pitch_cmd std_msgs/msg/Float64 '{data: 0.82}' >"${OUT}/gimbal_pitch.log" 2>&1 || true
start altitude_control bash -c "source /opt/ros/jazzy/setup.bash && source '${ROS2_WS}/install/setup.bash' && source '${PROJECT_DIR}/install/setup.bash' && exec ros2 run uav_control altitude_control --ros-args -p target_altitude_m:=3.0"
CONTROL_PID="$(cat "${OUT}/altitude_control.pid")"

echo "Waiting for three consecutive telemetry samples at 2.75–3.25 m..."
count=0
for _ in $(seq 1 75); do
    z="$(local_z || true)"
    if [[ -n "${z}" ]] && awk -v z="${z}" 'BEGIN{if (z < -2.75 && z > -3.25) exit 0; exit 1}'; then
        count=$((count + 1)); [[ ${count} -ge 3 ]] && break
    else count=0; fi
    sleep 1
done
if [[ ${count} -lt 3 ]]; then
    echo "Stable hover was not observed; see ${OUT}/altitude_control.log" >&2
    exit 7
fi
echo "Stable hover observed (NED z=${z} m); starting production BoT-SORT perception."
export PYTHONPATH="${PERCEPTION_SOURCE_DIR}${PYTHONPATH:+:${PYTHONPATH}}"
OBSERVER="${SOFTWARE_DIR}/venvs/yolo26/bin/python"
[[ -x "${OBSERVER}" ]] || { echo "YOLO26 virtualenv Python is missing." >&2; exit 8; }
if [[ "${TRACKING_LATERAL:-false}" == true ]]; then
    # Test-only handoff after stable hover. Both streams hold the same point
    # during startup; only the scripted stream remains for the actual maneuver.
    start lateral_control "${OBSERVER}" "${PROJECT_DIR}/evaluation/scripted_lateral_flight.py" \
        --output-dir "${OUT}" --ros-args -p use_sim_time:=true
    ready=false
    for _ in $(seq 1 40); do
        if [[ -s "${OUT}/lateral_ready.json" ]]; then ready=true; break; fi
        sleep .1
    done
    [[ "${ready}" == true ]] || { echo "Lateral test control did not start." >&2; exit 8; }
    kill -INT -- "-${CONTROL_PID}" 2>/dev/null || true
    CONTROL_PID="$(cat "${OUT}/lateral_control.pid")"
    CONTROL_NODE="lateral_test_control"
fi
# Hold scenario time during model startup so static/yaw intervals are captured.
# PX4 and existing Offboard streams retain their normal simulation-clock policy.
gz service -s "/world/${WORLD_NAME}/control" --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: true' \
    >"${OUT}/startup_pause.log" 2>&1
rg -q 'data: true' "${OUT}/startup_pause.log" \
    || { echo "Could not pause scenario during perception startup." >&2; exit 8; }
observer_args=(--output-dir "${OUT}" --world "${WORLD_NAME}" --stop-sim-time "${STOP_SIM_TIME}")
[[ "${TRACKING_GUARD_DEBUG:-false}" == true ]] && observer_args+=(--guard-debug)
[[ "${TRACKING_LATERAL:-false}" == true ]] && observer_args+=(--lateral-motion)
start observer "${OBSERVER}" "${PROJECT_DIR}/evaluation/observe_botsort_sim.py" "${observer_args[@]}"
OBSERVER_PID="$(cat "${OUT}/observer.pid")"
start perception bash -c "source '${PROJECT_DIR}/../software/venvs/yolo26/bin/activate' && exec '${OBSERVER}' -m uav_perception.yolo_detector --ros-args -p use_sim_time:=true -p enable_guard_debug:=${TRACKING_GUARD_DEBUG:-false} -p enable_performance_debug:=${TRACKING_PERFORMANCE_DEBUG:-false} -p enable_cuda_graph:=${TRACKING_CUDA_GRAPH:-true} -p enable_gimbal_tracking:=false -p initial_yaw:=-1.5708 -p initial_pitch:=0.82"
ready=false
for _ in $(seq 1 45); do
    perception_pid="$(cat "${OUT}/perception.pid")"
    if ! kill -0 "${perception_pid}" 2>/dev/null; then
        echo "Production perception exited; see ${OUT}/perception.log" >&2; exit 8
    fi
    if ros2 topic list 2>/dev/null | rg -q '^/yolo/telemetry$'; then ready=true; break; fi
    sleep 1
done
[[ "${ready}" == true ]] || { echo "Production perception topics were not created." >&2; exit 8; }
gz service -s "/world/${WORLD_NAME}/control" --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false' \
    >"${OUT}/startup_resume.log" 2>&1
rg -q 'data: true' "${OUT}/startup_resume.log" \
    || { echo "Could not resume scenario after perception startup." >&2; exit 8; }
if command -v nvidia-smi >/dev/null 2>&1; then
    start gpu_metrics nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used,memory.total \
        --format=csv,noheader,nounits --loop=1
fi
echo "Recording production ROS outputs through Gazebo sim time ${STOP_SIM_TIME} s (watchdog: 15 wall minutes)."
wait "${OBSERVER_PID}" || OBSERVER_STATUS=$?
if [[ ${OBSERVER_STATUS} -ne 0 ]]; then
    echo "Production observer exited ${OBSERVER_STATUS}; see ${OUT}/observer.log" >&2
    exit 9
fi
echo "Capture complete: ${OUT}/production_summary.json"
perception_pid="$(cat "${OUT}/perception.pid")"
ps -p "${perception_pid}" -o %cpu=,%mem=,rss=,etime=,args= \
    >"${OUT}/perception_resource_snapshot.txt" || true
