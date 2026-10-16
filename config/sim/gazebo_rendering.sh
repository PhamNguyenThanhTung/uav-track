#!/usr/bin/env bash
# Sourced by Gazebo launch helpers. Keep GPU selection local to Gazebo processes.
# WSLg can fall back to llvmpipe despite an available /dev/dxg GPU.
# Set UAV_GAZEBO_GPU=0 to keep Mesa's original selection for troubleshooting.

if [[ "${UAV_GAZEBO_GPU:-auto}" != "0" && -e /dev/dxg && -n "${WSL_DISTRO_NAME:-}" ]]; then
    export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
    export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
    echo " Gazebo WSL GPU: ${GALLIUM_DRIVER}; adapter: ${MESA_D3D12_DEFAULT_ADAPTER_NAME}"
    if [[ "${LIBGL_ALWAYS_SOFTWARE:-}" == "1" || "${LIBGL_ALWAYS_SOFTWARE:-}" == "true" ]]; then
        echo " WARNING: LIBGL_ALWAYS_SOFTWARE is set; unset it to enable GPU rendering." >&2
    fi
fi
