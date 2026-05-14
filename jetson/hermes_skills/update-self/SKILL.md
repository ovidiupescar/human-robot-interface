---
name: update-self
description: "Modify the robot's evolving self-knowledge (its :Self node + Facts about itself). Use whenever the user expresses a preference about how the robot should behave ('be more concise', 'use less humor', 'speak slower') or when the agent realizes something true about itself worth recording."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['identity', 'self', 'preferences']
    related_skills: []
---

```bash
python scripts/run.py --set formality=formal,speak_speed=slow
python scripts/run.py --note "Owner prefers no jokes before 9am"
```
