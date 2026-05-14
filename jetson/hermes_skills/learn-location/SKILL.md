---
name: learn-location
description: "Teach the robot the name of the place it's currently looking at. After this, it will visually recognize this location on its own. Optionally specify a parent (e.g. 'kitchen' is part of 'home') to build a place hierarchy."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['location', 'vision', 'memory', 'robot']
    related_skills: []
---

# learn-location

Captures the current scene embedding and binds it to a name in the knowledge
graph. Future scene matches against the same room return this name.

```bash
python scripts/run.py --name kitchen --parent home
```
