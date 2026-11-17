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
local_position_sample() {
    timeout 4 ros2 topic echo --once --qos-durability volatile \
        /fmu/out/vehicle_local_position_v1 2>/dev/null || true
}
local_z() {
    local message
    message="$(local_position_sample)"
    awk '/^[[:space:]]*z:/{print $2; exit}' <<<"${message}"
}
epoch_microseconds() {
    date +%s%6N
}
source_timestamp_is_fresh_after_land() {
    local timestamp="$1" requested_at="$2" now="$3"
    [[ "${timestamp}" =~ ^[0-9]+$ && "${requested_at}" =~ ^[0-9]+$ ]] || return 1
    [[ "${now}" =~ ^[0-9]+$ ]] || return 1
    awk -v timestamp="${timestamp}" -v requested="${requested_at}" -v now="${now}" \
        'BEGIN { if (timestamp > requested && timestamp <= now + 250000 && now - timestamp <= 1500000) exit 0; exit 1 }'
}
cleanup() {
    local status=$?
    local land_command_status=not_run
    local is_landed=unknown
    local arm_state=unknown
    local z=unknown
    local land_timestamp=unknown
    local status_timestamp=unknown
    local position_timestamp=unknown
    local telemetry_checked_at_us=unknown
    local land_request_epoch_us=unknown
    local land_echo_pid status_echo_pid position_echo_pid
    local land_message_file="${OUT}/cleanup_land_detected.yaml"
    local status_message_file="${OUT}/cleanup_vehicle_status.yaml"
    local position_message_file="${OUT}/cleanup_local_position.yaml"
    trap - EXIT INT TERM
    # A startup failure may leave physics paused; resume before requesting land.
    if [[ -f "${OUT}/startup_pause.log" ]] \
        && ! rg -q 'data: true' "${OUT}/startup_resume.log" 2>/dev/null; then
        gz service -s "/world/${WORLD_NAME}/control" --reqtype gz.msgs.WorldControl \
            --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: false' \
            >"${OUT}/cleanup_resume.log" 2>&1 || true
    fi
    if [[ -f "${OUT}/px4.pid" ]]; then
        local px4_pid
        px4_pid="$(cat "${OUT}/px4.pid" 2>/dev/null || true)"
        if [[ -n "${px4_pid}" ]] && kill -0 "${px4_pid}" 2>/dev/null; then
            if [[ -n "${CONTROL_PID}" ]]; then
                echo "Stopping this run's altitude setpoint publisher before PX4 Land..."
                kill -INT -- "-${CONTROL_PID}" 2>/dev/null || true
                sleep 1
                if kill -0 -- "-${CONTROL_PID}" 2>/dev/null; then
                    kill -TERM -- "-${CONTROL_PID}" 2>/dev/null || true
                    sleep 0.5
                fi
                if kill -0 -- "-${CONTROL_PID}" 2>/dev/null; then
                    kill -KILL -- "-${CONTROL_PID}" 2>/dev/null || true
                fi
                CONTROL_PID=""
            fi
            echo "Commanding PX4 Land..."
            timeout 8 "${PX4_DIR}/build/px4_sitl_default/bin/px4-commander" land >"${OUT}/land_command.log" 2>&1
            land_command_status=$?
            echo "PX4 Land command exit status: ${land_command_status}."
            land_request_epoch_us="$(epoch_microseconds)"
            echo "Waiting for telemetry to confirm landed state and motor disarm..."
            for _ in $(seq 1 25); do
                timeout 4 ros2 topic echo --once --qos-durability volatile \
                    /fmu/out/vehicle_land_detected >"${land_message_file}" 2>/dev/null &
                land_echo_pid=$!
                timeout 4 ros2 topic echo --once --qos-durability volatile \
                    /fmu/out/vehicle_status_v1 >"${status_message_file}" 2>/dev/null &
                status_echo_pid=$!
                local_position_sample >"${position_message_file}" 2>/dev/null &
                position_echo_pid=$!
                wait "${land_echo_pid}" 2>/dev/null || true
                wait "${status_echo_pid}" 2>/dev/null || true
                wait "${position_echo_pid}" 2>/dev/null || true
                telemetry="$(cat "${land_message_file}" 2>/dev/null || true)"
                status_msg="$(cat "${status_message_file}" 2>/dev/null || true)"
                position_msg="$(cat "${position_message_file}" 2>/dev/null || true)"
                is_landed="$(awk '/^[[:space:]]*landed:/{print $2; exit}' <<<"${telemetry}")"
                arm_state="$(awk '/^[[:space:]]*arming_state:/{print $2; exit}' <<<"${status_msg}")"
                land_timestamp="$(awk '/^[[:space:]]*timestamp:/{print $2; exit}' <<<"${telemetry}")"
                status_timestamp="$(awk '/^[[:space:]]*timestamp:/{print $2; exit}' <<<"${status_msg}")"
                position_timestamp="$(awk '/^[[:space:]]*timestamp:/{print $2; exit}' <<<"${position_msg}")"
                z="$(awk '/^[[:space:]]*z:/{print $2; exit}' <<<"${position_msg}")"
                telemetry_checked_at_us="$(epoch_microseconds)"
                if [[ "${land_command_status}" -eq 0 && "${is_landed}" == "true" && "${arm_state}" == "1" \
                    && "${z}" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]] \
                    && awk -v z="${z}" 'BEGIN{if ((z<0?-z:z)<0.25) exit 0; exit 1}' \
                    && source_timestamp_is_fresh_after_land "${land_timestamp}" "${land_request_epoch_us}" "${telemetry_checked_at_us}" \
                    && source_timestamp_is_fresh_after_land "${status_timestamp}" "${land_request_epoch_us}" "${telemetry_checked_at_us}" \
                    && source_timestamp_is_fresh_after_land "${position_timestamp}" "${land_request_epoch_us}" "${telemetry_checked_at_us}"; then
                    LANDED=true
                    echo "Landing and motor disarm confirmed by telemetry (landed=true, arming_state=1, NED z=${z} m)."
                    break
                fi
                sleep 2
            done
            if [[ "${LANDED}" != true ]]; then
                echo "WARNING: Land command and touchdown/disarm telemetry were not all confirmed (command_exit=${land_command_status}, landed=${is_landed:-unknown}, arm=${arm_state:-unknown}, last z=${z:-unknown})." >&2
                [[ "${status}" -ne 0 ]] || status=1
            fi
        else
            echo "WARNING: this run's PX4 process is no longer alive; landing cannot be confirmed." >&2
            land_command_status=px4_not_running
            [[ "${status}" -ne 0 ]] || status=1
        fi
    elif [[ -n "${CONTROL_PID}" ]] && kill -0 "${CONTROL_PID}" 2>/dev/null; then
        echo "Requesting best-effort descent to 0 m; PX4 landing telemetry is unavailable."
        timeout 8 ros2 param set "/${CONTROL_NODE}" target_altitude_m 0.0 >"${OUT}/landing_request.log" 2>&1 || true
        for _ in $(seq 1 18); do
            z="$(local_z || true)"
            if [[ -n "${z}" ]] && awk -v z="${z}" 'BEGIN{if ((z<0?-z:z)<0.16) exit 0; exit 1}'; then
                echo "Near-ground altitude observed (local NED z=${z} m); not sufficient to confirm PX4 landing."; break
            fi
            sleep 2
        done
        echo "WARNING: no live PX4 process was available to verify touchdown and disarm." >&2
        [[ "${status}" -ne 0 ]] || status=1
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
    if [[ "${status}" -eq 0 && "${LANDED}" != true ]]; then status=1; fi
    printf 'landed=%s\nland_command_exit=%s\nland_request_epoch_us=%s\ntelemetry_checked_at_us=%s\ntelemetry_landed=%s\narming_state=%s\nlocal_z_m=%s\nland_timestamp_us=%s\nstatus_timestamp_us=%s\nposition_timestamp_us=%s\nobserver_exit=%s\nexit_status=%s\n' \
        "${LANDED}" "${land_command_status}" "${land_request_epoch_us}" "${telemetry_checked_at_us}" "${is_landed}" "${arm_state}" "${z}" \
        "${land_timestamp}" "${status_timestamp}" "${position_timestamp}" "${OBSERVER_STATUS}" "${status}" \
        >"${OUT}/cleanup_status.txt"
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
if ! python3 -c '
import json, sys
with open(sys.argv[1], encoding="utf-8") as report_file:
    report = json.load(report_file)
frames = int(report.get("frames_observed", 0))
selected_actor = report.get("initial_selected_actor_by_costume")
selected_frames = sum(int(value) for value in report.get("selected_actor_observation_counts", {}).values())
sys.exit(0 if frames > 0 and selected_actor and selected_frames > 0 else 1)
' "${OUT}/production_summary.json"; then
    echo "Production capture completed without observed target-tracking evidence." >&2
    exit 9
fi
echo "Capture complete: ${OUT}/production_summary.json"
perception_pid="$(cat "${OUT}/perception.pid")"
ps -p "${perception_pid}" -o %cpu=,%mem=,rss=,etime=,args= \
    >"${OUT}/perception_resource_snapshot.txt" || true
