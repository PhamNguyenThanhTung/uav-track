#!/usr/bin/env bash
# ==============================================================================
# UAV Track - Interactive Live Demo Launcher (with GUI)
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOFTWARE_DIR="${SCRIPT_DIR}/../software"

source "${SOFTWARE_DIR}/venvs/yolo26/bin/activate"
source "/opt/ros/jazzy/setup.bash"
source "${SOFTWARE_DIR}/ros2_ws/install/setup.bash"
source "${SCRIPT_DIR}/install/setup.bash"

export DISPLAY="${DISPLAY:-:1}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

exec python3 "${SCRIPT_DIR}/simulation/run_live_demo.py" "$@"
