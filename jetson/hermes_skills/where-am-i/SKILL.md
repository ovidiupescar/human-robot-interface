---
name: where-am-i
description: "Identify the robot's current physical location from visual scene recognition. Returns the location name and confidence, or 'unknown' if the scene hasn't been learned yet. Use whenever the robot needs to anchor an answer or memory to a place ('what was I doing in the kitchen?', 'am I home?')."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['location', 'vision', 'robot']
    related_skills: []
---

# where-am-i

Calls `/location/identify` ROS2 service. The scene_recognizer constantly
publishes the latest match on `/perception/current_location` — this skill
just returns the most recent identification.

```bash
python scripts/run.py
```
