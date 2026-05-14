---
name: set-location
description: "Manually declare the robot's current location by name (no camera needed). Use this when vision isn't running yet, or when the user explicitly tells the robot 'you are at <X>'. Creates the Location node if it doesn't exist."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['location', 'memory']
    related_skills: []
---

```bash
python scripts/run.py --name kitchen --parent home
```
