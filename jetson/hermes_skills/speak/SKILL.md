---
name: speak
description: "Speak text out loud through the robot's speaker. The robot's face automatically switches to the 'speaking' state during playback and returns to 'standby' after. Use whenever the robot should produce audible speech to a human in the room."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['speech', 'tts', 'output']
    related_skills: []
---

# speak

Synthesizes text-to-speech via the TTS service and plays it through the
USB conference speaker. Blocks until playback completes.

## Usage

```bash
python scripts/run.py --text "Hello there"
python scripts/run.py --text "$LONG_TEXT" --timeout 30
```

## Notes

- Face is driven automatically (`speaking` during playback, then `standby`).
- Long text → longer block. Use a generous `--timeout` for paragraphs.
- TTS engine: stub right now. Production: Piper running on Jetson GPU.
