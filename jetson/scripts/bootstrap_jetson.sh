#!/usr/bin/env bash
# Bootstrap a fresh Jetson Orin Nano Super (JetPack 6 / Ubuntu 22.04)
# for the robot stack.
#
# Run once after first boot:
#   bash bootstrap_jetson.sh

set -euo pipefail

echo "=== System update ==="
sudo apt update
sudo apt -y upgrade
sudo apt -y install curl locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "=== ROS2 Humble apt source ==="
sudo apt -y install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "=== ROS2 Humble (ros-base + dev tools) ==="
sudo apt -y install ros-humble-ros-base ros-dev-tools

echo "=== System Python deps for nodes ==="
sudo apt -y install \
    python3-pip python3-rosdep python3-colcon-common-extensions \
    portaudio19-dev libsndfile1 \
    python3-serial

pip3 install --upgrade pip
pip3 install sounddevice numpy faster-whisper openwakeword kuzu

echo "=== rosdep init ==="
sudo rosdep init || true
rosdep update

echo "=== Python 3.11 (for Hermes) via deadsnakes ==="
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt -y install python3.11 python3.11-venv python3.11-dev

echo "=== udev rule for ESP32 face (CDC ACM) ==="
sudo tee /etc/udev/rules.d/99-robot-face.rules > /dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", SYMLINK+="robot_face"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "=== Done ==="
echo "Add to ~/.bashrc:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "Next: cd ros2_ws && bash ../scripts/build_ws.sh"
