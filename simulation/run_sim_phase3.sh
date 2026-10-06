#!/usr/bin/env bash
# Phase 3 - Gazebo Harmonic Simulation Launch Script
# Usage: ./run_sim_phase3.sh [options to gz sim, e.g. -s for headless]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="${PROJECT_DIR}/../software/PX4-Autopilot"

PX4_MODELS="${PX4_DIR}/Tools/simulation/gz/models"
PX4_WORLDS="${PX4_DIR}/Tools/simulation/gz/worlds"
PX4_PLUGINS="${PX4_DIR}/build/px4_sitl_default/src/modules/simulation/gz_plugins"
CUSTOM_MODELS="${PROJECT_DIR}/simulation/models"

export GZ_SIM_RESOURCE_PATH="${CUSTOM_MODELS}:${PX4_MODELS}:${PX4_WORLDS}:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_PLUGINS}:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

WORLD_FILE="${PROJECT_DIR}/simulation/worlds/gimbal_test.sdf"

echo "========================================================="
echo " Starting Phase 3 Gazebo Simulation"
echo " World: ${WORLD_FILE}"
echo " Resource Path: ${GZ_SIM_RESOURCE_PATH}"
echo " Plugin Path: ${GZ_SIM_SYSTEM_PLUGIN_PATH}"
echo "========================================================="

gz sim -r "$@" "${WORLD_FILE}"
