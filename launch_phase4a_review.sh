#!/usr/bin/env bash
# Build Phase 4A, then open simulation, perception, and image-viewer tabs.
# Run this script from a WSL terminal. The current terminal is the build tab.

set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
ROS2_WS="$PROJECT_DIR/../software/ros2_ws"
YOLO_VENV="$PROJECT_DIR/../software/venvs/yolo26"

if [[ "${1:-}" == "--terminal" ]]; then
    mode="${2:-}"
    set +e

    case "$mode" in
        simulation)
            cd "$PROJECT_DIR"
            ./launch_simulation.sh --offboard 2.0
            result=$?
            ;;
        perception)
            source /opt/ros/jazzy/setup.bash
            source "$ROS2_WS/install/setup.bash"
            source "$YOLO_VENV/bin/activate"
            source "$PROJECT_DIR/install/setup.bash"
            cd "$PROJECT_DIR"
            "$YOLO_VENV/bin/python" -m uav_perception.yolo_detector --ros-args -p use_sim_time:=true
            result=$?
            ;;
        image-viewer)
            source /opt/ros/jazzy/setup.bash
            source "$ROS2_WS/install/setup.bash"
            source "$PROJECT_DIR/install/setup.bash"
            if ! ros2 pkg list | grep -qx rqt_image_view; then
                echo "rqt_image_view is not installed in this ROS 2 environment."
                echo "Install ros-jazzy-rqt-image-view, then run this launcher again."
                result=127
            else
                ros2 run rqt_image_view rqt_image_view
                result=$?
            fi
            ;;
        *)
            echo "Unknown terminal mode: $mode" >&2
            exit 2
            ;;
    esac

    echo
    echo "This terminal's process exited with status $result."
    read -r -p "Press Enter to close this terminal... " _ || true
    exit "$result"
fi

if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$ROS2_WS/install/setup.bash" ]]; then
    echo "ROS 2 Jazzy or the dependency workspace is missing." >&2
    exit 1
fi
if [[ ! -f "$YOLO_VENV/bin/activate" ]]; then
    echo "YOLO virtual environment not found at: $YOLO_VENV" >&2
    exit 1
fi
if ! command -v cmd.exe >/dev/null 2>&1 || ! command -v wsl.exe >/dev/null 2>&1; then
    echo "This launcher opens tabs through Windows Terminal from WSL." >&2
    exit 1
fi

echo "[1/4] Building uav_perception from the current workspace..."
source /opt/ros/jazzy/setup.bash
source "$ROS2_WS/install/setup.bash"
source "$YOLO_VENV/bin/activate"
cd "$PROJECT_DIR"
colcon build --packages-select uav_perception

if [[ -z "${WSL_DISTRO_NAME:-}" ]]; then
    echo "WSL_DISTRO_NAME is not set; cannot select the current WSL distro." >&2
    exit 1
fi

open_tab() {
    local title="$1"
    local mode="$2"
    local script_arg
    local terminal_command

    printf -v script_arg '%q' "$PROJECT_DIR/launch_phase4a_review.sh"
    terminal_command="exec $script_arg --terminal $mode"

    # -w 0 reuses the most recently active Windows Terminal window.
    local windows_command_dir
    windows_command_dir="$(dirname -- "$(command -v cmd.exe)")"
    (
        cd "$windows_command_dir"
        cmd.exe /c wt.exe -w 0 new-tab --title "$title" \
            wsl.exe -d "$WSL_DISTRO_NAME" -- bash -lc "$terminal_command"
    )
}

echo "[2/4] Opening the Gazebo + PX4 simulation tab..."
open_tab "02-Simulation" simulation
echo "[3/4] Opening the YOLO perception tab..."
open_tab "03-Perception" perception
echo "[4/4] Opening the annotated-image viewer tab..."
open_tab "04-Image-Viewer" image-viewer

echo
echo "All four terminals are ready: this tab is the build log."
echo "In the image viewer, choose /yolo/annotated_image from the topic list."
read -r -p "Press Enter to close the build terminal... " _ || true
