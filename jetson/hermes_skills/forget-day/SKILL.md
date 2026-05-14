---
name: forget-day
description: "Privacy operation: delete the raw journal file for a specific date AND remove all Episodes (and Facts derived solely from them) that occurred on that date. Use when the user explicitly asks 'forget today' / 'delete what happened yesterday'. Irreversible."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['memory', 'privacy']
    related_skills: []
---

```bash
python scripts/run.py --date 2026-05-11
```
