---
name: relate-persons
description: "Add a typed relationship edge between two persons in the knowledge graph (e.g. parent_of, sibling_of, partner_of, friend_of, colleague_of). Use when conversation reveals a relationship ('Maria's my sister')."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['memory', 'graph', 'relationships']
    related_skills: []
---

```bash
python scripts/run.py --a p_abc --b p_def --relation sibling_of --bidirectional
```
