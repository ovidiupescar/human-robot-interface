#!/usr/bin/env bash
# Launch the Gemini Live realtime bridge node.
# Coexists with robot-stack; replaces hermes-gateway in the voice path.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/../ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# CUDA libs (matches robot-stack so any shared deps load identically).
export LD_LIBRARY_PATH="$HOME/.local/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
export PATH="/usr/local/cuda/bin:$PATH"

# CRITICAL: must match robot-stack's RMW implementation so the bridge
# sees /audio/chunk and /perception/wake_word. Default fastrtps will
# look isolated from a cyclonedds-running stack — DDS implementations
# do not interoperate at the wire level.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# GEMINI_API_KEY can be set via:
#   1. ~/.hermes/.env (preferred; one place for all keys)
#   2. environment passed in by systemd unit
#   3. ROS2 param api_key:= on the command line
# The node falls back through these in order.

exec ros2 run robot_realtime realtime_bridge
