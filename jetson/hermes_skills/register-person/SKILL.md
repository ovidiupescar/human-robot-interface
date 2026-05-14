---
name: register-person
description: "Bind a name to the most-recent speaker (or to provided audio sample). After this the robot will recognize this voice and call this person by name. Use whenever someone introduces themselves ('I'm Maria') or you confirm who an unknown speaker is."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['identity', 'voice', 'memory']
    related_skills: []
---

```bash
python scripts/run.py --name Maria
```
