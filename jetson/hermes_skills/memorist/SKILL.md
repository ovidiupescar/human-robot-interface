---
name: memorist
description: "Reflect on recent events: read journal entries since last checkpoint, identify coherent windows, and distill them into structured graph entries (Episodes, Facts, Relationships)."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['memory', 'consolidation', 'background']
    related_skills: []
---

```bash
python scripts/run.py --mode incremental
python scripts/run.py --mode daily
```
