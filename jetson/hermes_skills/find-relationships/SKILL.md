---
name: find-relationships
description: "Traverse the relationship graph from a starting person. Useful for queries like 'who are Maria's siblings' or 'who has Maria mentioned recently'."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['memory', 'graph', 'relationships']
    related_skills: []
---

```bash
python scripts/run.py --subject-id p_abc --relation sibling_of --hops 1
```
