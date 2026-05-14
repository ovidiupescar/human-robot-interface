#!/usr/bin/env bash
# Build the ROS2 workspace.
# Run from repo root: bash jetson/scripts/build_ws.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/../ros2_ws"

source /opt/ros/humble/setup.bash

cd "$WS"
rosdep install --from-paths src --ignore-src -y -r || true
colcon build --symlink-install

echo ""
echo "Build done. Source the workspace:"
echo "  source $WS/install/setup.bash"
