#!/usr/bin/env bash
# Launch the full robot ROS2 stack.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/../ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Adjust serial port + audio devices as needed
exec ros2 launch robot_bringup robot.launch.py \
    face_serial_port:=/dev/robot_face \
    audio_input_device:="" \
    audio_output_device:=""
