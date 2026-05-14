---
name: listen
description: "Wait for the user to say something and return the transcription. Use when the robot has explicitly asked a question and needs to capture the answer before deciding what to do next. Returns the transcribed text or empty string on timeout."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['speech', 'stt', 'input']
    related_skills: []
---

# listen

Blocks until the next final transcript arrives from the STT pipeline
(faster-whisper). Useful when the conversation flow needs an explicit
synchronous answer.

## Usage

```bash
python scripts/run.py --timeout 10
```

Prints the captured text on stdout (empty line on timeout).

## Notes

- The face automatically shows `processing` while voice activity is detected
  (handled by the reflex node, not this skill).
- For continuous listening, you don't need this skill — the perception
  gateway already pushes every transcript into the Hermes session as
  a `[USER_VOICE]: ...` event. Use this skill only when you need a blocking
  Q&amp;A pattern.
