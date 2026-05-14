#!/usr/bin/env bash
# Launch the full robot ROS2 stack.

# NOTE: do NOT use `set -u` here — ROS2's /opt/ros/humble/setup.bash
# (and the workspace install/setup.bash) reference unset variables
# such as AMENT_TRACE_SETUP_FILES; nounset crashes the launch.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/../ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Adjust serial port + audio devices as needed. Empty overrides cannot be
# passed via the CLI (ROS2 rejects `name:=` with no value); rely on the
# launch file's defaults, set ENV vars to override if needed.
exec ros2 launch robot_bringup robot.launch.py \
    face_serial_port:=/dev/robot_face
