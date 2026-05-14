---
name: remember-about
description: "Store a fact about a person, location, or event in the long-term knowledge graph. Use whenever a user reveals something worth remembering ('my cat is named Whiskers', 'I work at Acme') or you yourself infer something high-confidence."
version: 0.1.0
platforms: ['linux']
metadata:
  hermes:
    tags: ['memory', 'graph']
    related_skills: []
---

```bash
python scripts/run.py --subject-id p_abc --subject-type Person \
    --content "Has a cat named Whiskers" --tags "pet,cat,name:Whiskers"
```
