---
name: set-face
description: "Change the robot's facial expression on the front display. Use whenever the robot's emotional state should change visually — when starting to speak, thinking, returning to idle, or expressing anger. The face has four states: standby (calm breathing rings), processing (cyan thinking waveform), speaking (orange voice wave), aggressive (red spiky waveform)."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['face', 'expression', 'robot']
    related_skills: []
---

# set-face

Sets the robot's facial expression by writing to the ESP32 face board over ROS2.

## Usage

Invoke the helper script with a state name:

```bash
python scripts/run.py --state standby
python scripts/run.py --state processing
python scripts/run.py --state speaking --amplitude 0.7
python scripts/run.py --state aggressive
```

## States

| state        | meaning                            | when to use                              |
|--------------|------------------------------------|------------------------------------------|
| `standby`    | Calm cyan breathing rings          | Idle, listening passively, default       |
| `processing` | Cyan spiky waveform                | Robot is thinking / busy / loading       |
| `speaking`   | Orange voice wave                  | While speaking out loud                  |
| `aggressive` | Red spiky waveform with angry brows| Frustration, scolding, intense reaction  |

## Amplitude

Only used by `speaking`. Range 0.0–1.0. Higher = bigger waveform swings.
