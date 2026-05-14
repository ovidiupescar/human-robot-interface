# human-robot-interface

Open source robotics stack: ESP32 face firmware + Jetson-side ROS2 audio/perception/control pipeline + Hermes agent platform adapter.

## Layout

```
src/                    ESP32-S3 firmware (Waveshare AMOLED 1.75" face)
platformio.ini          PlatformIO build config
tools/                  Build / flash helpers
jetson/
├── ros2_ws/            ROS2 Humble colcon workspace
│   └── src/
│       ├── robot_audio/         Capture, playback, bilingual TTS (Piper RO + Kokoro EN)
│       ├── robot_face_bridge/   Serial JSON ↔ ROS2 bridge to ESP32 face
│       ├── robot_face_msgs/     Face command/state msgs + Speak.srv
│       ├── robot_control_msgs/  Interrupt + conversation state msgs
│       ├── robot_graph/         Context prefetch, identity fusion, knowledge graph
│       └── robot_bringup/       Launch files
├── robot_bridge/       Python lib (rclpy wrapper) used by Hermes skills
├── hermes_plugin/      Hermes platform adapter for ROS2
├── hermes_skills/      agentskills.io-format skill definitions
└── scripts/            Bootstrap, build, launch helpers
```

## What this repo does NOT contain

Personal robot identity (SOUL.md), memory data, architecture decisions, and deployment-specific configuration live in a separate private environment repo. The Hermes config path is resolved via the `HERMES_CONFIG_PATH` environment variable at runtime.

## Hardware

- **Brain:** NVIDIA Jetson Orin Nano Super (JetPack 6, Ubuntu 22.04, ROS2 Humble)
- **Face:** ESP32-S3 + Waveshare 1.75" AMOLED (CO5300 driver)
- **Audio:** USB conference puck (tested with Plantronics Poly Sync 20-M)

## Bilingual

Romanian (Piper ro_RO-mihai-medium) + English (Kokoro af_bella). Language resolver in `robot_audio/tts_service_node.py` selects engine per utterance.

## License

MIT — see [LICENSE](LICENSE).
