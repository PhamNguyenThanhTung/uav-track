#!/usr/bin/env bash
# Phase 1 - Standalone Gazebo Simulation Launch Script
# Usage: ./run_sim_phase1.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${PROJECT_DIR}/config/sim/gazebo_rendering.sh"
PX4_MODELS="${PROJECT_DIR}/../software/PX4-Autopilot/Tools/simulation/gz/models"
CUSTOM_MODELS="${PROJECT_DIR}/simulation/models"

export GZ_SIM_RESOURCE_PATH="${CUSTOM_MODELS}:${PX4_MODELS}:${GZ_SIM_RESOURCE_PATH}"

WORLD_FILE="${PROJECT_DIR}/simulation/worlds/gimbal_test.sdf"

echo "========================================================="
echo " Starting Phase 1 Gazebo Simulation"
echo " World: ${WORLD_FILE}"
echo " Model Path: ${GZ_SIM_RESOURCE_PATH}"
echo "========================================================="

gz sim -r "${WORLD_FILE}"
