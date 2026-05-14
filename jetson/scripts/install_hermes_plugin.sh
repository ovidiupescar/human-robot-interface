#!/usr/bin/env bash
# Install the ROS2 perception platform plugin and skills into Hermes.
#
# Assumes Hermes was installed via its own installer and lives at ~/hermes-agent
# (the install script default — adjust HERMES_HOME if not).

set -euo pipefail

HERMES_HOME="${HERMES_HOME:-$HOME/hermes-agent}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Private environment repo (SOUL.md, identity, deployment specifics) is
# expected as a sibling clone of the public repo. Override with HERMES_ENV_ROOT.
HERMES_ENV_ROOT="${HERMES_ENV_ROOT:-$(cd "$REPO_ROOT/../../human-robot-interface-environment/jetson" 2>/dev/null && pwd || true)}"
if [ -z "$HERMES_ENV_ROOT" ] || [ ! -d "$HERMES_ENV_ROOT" ]; then
    echo "ERROR: private env repo not found."
    echo "Set HERMES_ENV_ROOT to the path of human-robot-interface-environment/jetson"
    echo "  (default expects sibling clone at ../../human-robot-interface-environment/jetson)"
    exit 1
fi

if [ ! -d "$HERMES_HOME" ]; then
    echo "Hermes not found at $HERMES_HOME"
    echo "Install first: curl -fsSL https://raw.githubusercontent.com/NousResearch/hermes-agent/main/scripts/install.sh | bash"
    exit 1
fi

echo "=== Linking ROS2 platform plugin ==="
mkdir -p "$HERMES_HOME/plugins/platforms"
ln -sfn "$REPO_ROOT/hermes_plugin/platforms/ros2" "$HERMES_HOME/plugins/platforms/ros2"

echo "=== Linking skills ==="
mkdir -p "$HERMES_HOME/skills"
for skill in "$REPO_ROOT/hermes_skills"/*; do
    name=$(basename "$skill")
    ln -sfn "$skill" "$HERMES_HOME/skills/$name"
done

echo "=== Installing SOUL.md (Hermes auto-discovers in working tree) ==="
# Hermes scans the working dir for .hermes.md, SOUL.md, .cursorrules
# SOUL.md lives in the private env repo, not in this public source tree.
cp "$HERMES_ENV_ROOT/hermes_config/SOUL.md" "$HERMES_HOME/SOUL.md"

echo "=== Installing robot_bridge into Hermes python env ==="
# Hermes typically uses Python 3.11 in a venv at $HERMES_HOME/.venv
if [ -d "$HERMES_HOME/.venv" ]; then
    "$HERMES_HOME/.venv/bin/pip" install -e "$REPO_ROOT/robot_bridge"
else
    pip3.11 install -e "$REPO_ROOT/robot_bridge"
fi

echo "Done. Verify with: hermes gateway list"
