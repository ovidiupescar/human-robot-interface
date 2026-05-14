---
name: recall-memories
description: "Retrieve stored facts about a person, location, or event. Use whenever you need to ground a response in long-term memory ('what do I know about Maria?', 'what happens at the office?')."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['memory', 'graph', 'recall']
    related_skills: []
---

```bash
python scripts/run.py --subject-id p_abc --subject-type Person --limit 5
```
