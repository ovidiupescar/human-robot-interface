# Jetson Side — Robot Stack

Everything that runs on the **Jetson Orin Nano Super**. The ESP32 face firmware
lives in the repo root (`../src/`); this directory is for the host-side stack.

## Layout

```
jetson/
├── ros2_ws/                          # ROS2 colcon workspace
│   └── src/
│       ├── robot_face_msgs/          # custom msg/srv definitions
│       ├── robot_face_bridge/        # serial JSON ↔ ROS2 bridge node
│       ├── robot_audio/              # capture, playback, TTS
│       ├── robot_perception/         # VAD, Whisper STT
│       ├── robot_reflex/             # fast reactive face cmds
│       └── robot_bringup/            # launch files
├── robot_bridge/                     # Python lib used by skills (wraps rclpy)
├── hermes_plugin/
│   └── platforms/ros2/               # Hermes platform adapter (perception gateway)
├── hermes_skills/                    # agentskills.io-format skills
│   ├── set-face/
│   ├── speak/
│   └── listen/
└── scripts/                          # bootstrap, build, launch helpers
```

## Setup order (fresh Jetson)

1. `bash scripts/bootstrap_jetson.sh` — installs ROS2 Humble, Python 3.11, deps
2. `bash scripts/build_ws.sh` — builds the colcon workspace
3. Install Hermes:
   ```
   curl -fsSL https://raw.githubusercontent.com/NousResearch/hermes-agent/main/scripts/install.sh | bash
   hermes setup
   ```
4. `bash scripts/install_hermes_plugin.sh` — symlinks our plugin + skills into Hermes
5. Reboot or `source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash`

## Running

Three processes (use systemd or tmux):

```bash
# 1. Robot ROS2 stack
bash scripts/launch_robot.sh

# 2. Hermes agent (talks to ROS2 via our platform adapter)
hermes gateway start ros2-perception

# 3. (optional) Telegram gateway, etc.
hermes gateway start telegram
```

## How it talks to the face

```
Hermes skill (Python) → robot_bridge.RobotBridge → ROS2 topic /face/command
                                                          ↓
                                                  face_bridge node
                                                          ↓
                                                  USB serial / JSON
                                                          ↓
                                                    ESP32 face board
```

## How perception reaches Hermes

```
Audio → audio_capture → /audio/chunk
                            ↓
                     voice_activity → /perception/voice_active
                            ↓                    ↓
                  speech_recognizer       reflex_node → /face/command (instant)
                            ↓                    ↑
                /perception/transcript   (Hermes overrides reflex when it acts)
                            ↓
              ROS2 platform adapter (Hermes plugin)
                            ↓
        MessageEvent injected into "robot-main" Hermes session
                            ↓
                       Hermes LLM responds
                            ↓
                        send() → speak() → /speak service → TTS → speaker
```

## Single-session trick

All ROS2-sourced events use `chat_id="robot-main"`. Hermes's session manager
keys on `chat_id`, so every perception event lands in the same persistent
conversation — no fragmentation.

## State of things

- [x] ROS2 package skeletons compile and start
- [x] Hermes platform adapter skeleton
- [x] 3 skills (set-face, speak, listen)
- [ ] Validate adapter against real Hermes BasePlatformAdapter signature
- [ ] Wire Piper TTS in `tts_service_node._synthesize`
- [ ] Add vision_perception node
- [ ] Add RoArm-M2-S driver
- [ ] systemd units for auto-start
