#!/usr/bin/env bash

# Exit immediately on error
set -e

# Resolve paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
MAPS_DIR="${PACKAGE_DIR}/maps"

# Ensure maps directory exists
mkdir -p "${MAPS_DIR}"

MAP_NAME="simple_maze"
MAP_PATH="${MAPS_DIR}/${MAP_NAME}"

echo "Saving map to: ${MAP_PATH}..."

# Run the nav2_map_server map_saver_cli
ros2 run nav2_map_server map_saver_cli -f "${MAP_PATH}" --ros-args -p use_sim_time:=true

echo "Map saved successfully!"
