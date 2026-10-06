#!/usr/bin/env bash
# ==============================================================================
# All-in-One Simulation Launcher for Phase 3 (UAV Project)
# Integrates: Micro XRCE-DDS Agent + Gazebo Harmonic + ROS-GZ Bridge + PX4 SITL
#
# Supported Modes:
#   1. Default:
#      ./launch_simulation.sh
#      Starts Gazebo with 3D GUI, connects PX4 standalone, and keeps the
#      interactive PX4 console (pxh>) in foreground for manual flight control.
#
#   2. Headless:
#      ./launch_simulation.sh --headless
#      Runs simulation without opening the Gazebo 3D GUI window (saves CPU/RAM).
#
#   3. Offboard Altitude Control:
#      ./launch_simulation.sh --offboard <target_altitude>
#      Starts simulation and launches the ROS 2 altitude_control node with
#      the specified target altitude (default: 2.0 m).
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"
PX4_DIR="${PROJECT_DIR}/../software/PX4-Autopilot"
ROS2_WS="${PROJECT_DIR}/../software/ros2_ws"

GZ_WORLD="gimbal_test"
GZ_MODEL="custom_x500_gimbal_down"

# Source ROS 2 environments
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ERROR: ROS 2 Jazzy setup not found at /opt/ros/jazzy/setup.bash"
    exit 1
fi
source /opt/ros/jazzy/setup.bash

if [ -f "${ROS2_WS}/install/setup.bash" ]; then
    source "${ROS2_WS}/install/setup.bash"
else
    echo "ERROR: Dependency workspace setup not found at ${ROS2_WS}/install/setup.bash"
    exit 1
fi

if [ -f "${PROJECT_DIR}/install/setup.bash" ]; then
    source "${PROJECT_DIR}/install/setup.bash"
else
    echo "ERROR: Project workspace setup not found at ${PROJECT_DIR}/install/setup.bash"
    exit 1
fi

# Verify critical binaries and configuration files
if ! command -v MicroXRCEAgent &> /dev/null; then
    echo "ERROR: MicroXRCEAgent binary not found in PATH."
    exit 1
fi

if ! command -v gz &> /dev/null; then
    echo "ERROR: Gazebo (gz) binary not found in PATH."
    exit 1
fi

PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
if [ ! -x "${PX4_BIN}" ]; then
    echo "ERROR: PX4 SITL binary not found or not executable at ${PX4_BIN}"
    exit 1
fi

GZ_LAUNCH_SCRIPT="${PROJECT_DIR}/simulation/run_sim_phase3.sh"
if [ ! -x "${GZ_LAUNCH_SCRIPT}" ]; then
    echo "ERROR: Gazebo launch script not found or not executable at ${GZ_LAUNCH_SCRIPT}"
    exit 1
fi

BRIDGE_CONFIG="${PROJECT_DIR}/config/sim/ros_gz_bridge.yaml"
if [ ! -f "${BRIDGE_CONFIG}" ]; then
    echo "ERROR: Bridge config file not found at ${BRIDGE_CONFIG}"
    exit 1
fi

# Parse command line arguments
HEADLESS_MODE=false
AUTO_OFFBOARD=false
TARGET_ALT=2.0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless|-s)
            HEADLESS_MODE=true
            shift
            ;;
        --offboard)
            AUTO_OFFBOARD=true
            shift
            if [[ $# -gt 0 && "$1" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
                TARGET_ALT="$1"
                shift
            fi
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  (no args)             Default: Launch Gazebo 3D GUI + PX4 console (pxh>)"
            echo "  --headless, -s        Run Gazebo headless (no 3D GUI window)"
            echo "  --offboard [ALT]      Launch offboard altitude control node (default ALT: 2.0m)"
            echo "  -h, --help            Display this help message"
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $1"
            echo "Run '$0 --help' for usage."
            exit 1
            ;;
    esac
done

# Track process IDs for graceful and safe cleanup
PIDS=()

cleanup() {
    trap - EXIT INT TERM
    echo ""
    echo "========================================================="
    echo " Shutting down simulation components..."
    echo "========================================================="

    # Send SIGTERM to tracked parent PIDs and their immediate child processes
    for pid in "${PIDS[@]}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            pkill -TERM -P "$pid" 2>/dev/null || true
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    # Wait up to 3 seconds for graceful shutdown
    for _ in {1..30}; do
        any_alive=false
        for pid in "${PIDS[@]}"; do
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                any_alive=true
                break
            fi
        done
        if [ "$any_alive" = false ]; then
            break
        fi
        sleep 0.1
    done

    # Force kill any remaining tracked processes
    for pid in "${PIDS[@]}"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            pkill -KILL -P "$pid" 2>/dev/null || true
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done

    echo " All simulation processes stopped cleanly."
}

trap cleanup EXIT INT TERM

echo "========================================================="
echo " UAV PROJECT - SIMULATION LAUNCHER (PHASE 3)"
echo " Mode:         $( [ "$HEADLESS_MODE" = true ] && echo 'Headless (No GUI)' || echo 'GUI (3D Window)' )"
echo " Auto-Offboard: $( [ "$AUTO_OFFBOARD" = true ] && echo "YES (${TARGET_ALT} m)" || echo 'NO (Manual / Standby)' )"
echo " Target Model:  ${GZ_MODEL}"
echo " World:         ${GZ_WORLD}"
echo "========================================================="

# ------------------------------------------------------------------------------
# 1. Start Micro XRCE-DDS Agent
# ------------------------------------------------------------------------------
echo "[1/4] Starting Micro XRCE-DDS Agent (port 8888)..."
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
AGENT_PID=$!
PIDS+=("${AGENT_PID}")
sleep 1

if ! kill -0 "${AGENT_PID}" 2>/dev/null; then
    echo "ERROR: Micro XRCE-DDS Agent failed to start on UDP port 8888."
    exit 1
fi
echo "      Micro XRCE-DDS Agent is running (PID: ${AGENT_PID})."

# ------------------------------------------------------------------------------
# 2. Start Gazebo Harmonic Simulation
# ------------------------------------------------------------------------------
echo "[2/4] Starting Gazebo Harmonic simulation..."
GZ_ARGS=""
if [ "$HEADLESS_MODE" = true ]; then
    GZ_ARGS="-s"
fi

"${GZ_LAUNCH_SCRIPT}" ${GZ_ARGS} > /dev/null 2>&1 &
GZ_PID=$!
PIDS+=("${GZ_PID}")

echo "      Waiting for Gazebo simulation clock (/world/${GZ_WORLD}/clock)..."
GZ_TIMEOUT=25
while [ ${GZ_TIMEOUT} -gt 0 ]; do
    if ! kill -0 "${GZ_PID}" 2>/dev/null; then
        echo "ERROR: Gazebo process exited unexpectedly."
        exit 1
    fi
    if gz topic -l 2>/dev/null | grep -q "/world/${GZ_WORLD}/clock"; then
        break
    fi
    sleep 1
    GZ_TIMEOUT=$((GZ_TIMEOUT - 1))
done

if [ ${GZ_TIMEOUT} -eq 0 ]; then
    echo "ERROR: Timed out waiting for Gazebo clock."
    exit 1
fi
echo "      Gazebo world is ready."

# Verify that the model in Gazebo matches PX4's expected model name
echo "      Verifying model '${GZ_MODEL}' in Gazebo..."
if ! gz topic -l 2>/dev/null | grep -q "${GZ_MODEL}"; then
    if ! gz model --list 2>/dev/null | grep -q "${GZ_MODEL}"; then
        echo "ERROR: Model '${GZ_MODEL}' not found in Gazebo."
        exit 1
    fi
fi
echo "      Model '${GZ_MODEL}' verified."

# ------------------------------------------------------------------------------
# 3. Start ROS-Gazebo Parameter Bridge
# ------------------------------------------------------------------------------
echo "[3/4] Starting ROS-Gazebo parameter bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:="${BRIDGE_CONFIG}" > /dev/null 2>&1 &
BRIDGE_PID=$!
PIDS+=("${BRIDGE_PID}")
sleep 1

if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    echo "ERROR: ROS-Gazebo parameter bridge failed to start."
    exit 1
fi

echo "      Waiting for camera topic on ROS 2 (/custom_x500/camera/image_raw)..."
BRIDGE_TIMEOUT=15
while [ ${BRIDGE_TIMEOUT} -gt 0 ]; do
    if ! kill -0 "${BRIDGE_PID}" 2>/dev/null; then
        echo "ERROR: ROS-Gazebo bridge process exited unexpectedly."
        exit 1
    fi
    if ros2 topic list 2>/dev/null | grep -q "/custom_x500/camera/image_raw"; then
        break
    fi
    sleep 1
    BRIDGE_TIMEOUT=$((BRIDGE_TIMEOUT - 1))
done

if [ ${BRIDGE_TIMEOUT} -eq 0 ]; then
    echo "ERROR: Timed out waiting for ROS-Gazebo bridge topics."
    exit 1
fi
echo "      ROS-Gazebo bridge is ready."

# ------------------------------------------------------------------------------
# 4. Configure PX4 SITL Environment
# Standalone mode: PX4 attaches to existing Gazebo instance and model.
# Do NOT set PX4_SIM_MODEL when PX4_GZ_MODEL_NAME is used.
# ------------------------------------------------------------------------------
unset PX4_SIM_MODEL
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD="${GZ_WORLD}"
export PX4_GZ_MODEL_NAME="${GZ_MODEL}"

# ------------------------------------------------------------------------------
# 5. Launch PX4 SITL and Controllers
# ------------------------------------------------------------------------------
if [ "$AUTO_OFFBOARD" = true ]; then
    echo "[4/4] Starting PX4 SITL in background..."
    cd "${PX4_DIR}"
    "${PX4_BIN}" -d > /tmp/px4_sitl.log 2>&1 &
    PX4_PID=$!
    PIDS+=("${PX4_PID}")

    echo "      Waiting for PX4 telemetry on ROS 2 (/fmu/out/vehicle_local_position_v1)..."
    PX4_READY_TIMEOUT=30
    while [ ${PX4_READY_TIMEOUT} -gt 0 ]; do
        if ! kill -0 "${PX4_PID}" 2>/dev/null; then
            echo "ERROR: PX4 SITL process exited unexpectedly. Log output:"
            tail -n 25 /tmp/px4_sitl.log 2>/dev/null || true
            exit 1
        fi
        if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_local_position"; then
            break
        fi
        sleep 1
        PX4_READY_TIMEOUT=$((PX4_READY_TIMEOUT - 1))
    done

    if [ ${PX4_READY_TIMEOUT} -eq 0 ]; then
        echo "ERROR: Timed out waiting for PX4 uORB telemetry on ROS 2."
        exit 1
    fi
    echo "      PX4 SITL is ready."

    echo "========================================================="
    echo " Starting Offboard Altitude Control node (Target: ${TARGET_ALT} m)..."
    echo " Press Ctrl+C at any time to land and terminate."
    echo "========================================================="
    ros2 run uav_control altitude_control --ros-args -p target_altitude_m:="${TARGET_ALT}"

else
    echo "[4/4] Starting PX4 SITL in foreground attached to ${GZ_MODEL}..."
    echo "========================================================="
    echo " PX4 Console (pxh>) is active below."
    echo " Commands:"
    echo "   commander takeoff  -> Takeoff to 2m"
    echo "   commander land     -> Land smoothly"
    echo " Press Ctrl+C to terminate all simulation components."
    echo "========================================================="
    echo ""

    cd "${PX4_DIR}"
    "${PX4_BIN}"
fi
