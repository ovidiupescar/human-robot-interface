#!/usr/bin/env python3
"""Memorist runner — orchestrates journal scan + LLM distillation + graph writes.

The actual LLM-driven distillation is done by Hermes itself when this skill is
invoked from the agent loop. This script:
  1. Reads the unconsolidated journal slice via robot_bridge.
  2. Buckets into windows.
  3. Returns the windows as JSON for the agent to summarize.
  4. After agent emits structured ops, applies them via robot_bridge.

For now this prints the window list; real distillation prompt lives in
SKILL.md plus the agent's system prompt.
"""

import argparse
import json
import sys

from robot_bridge import RobotBridge


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--mode', choices=['incremental', 'daily'],
                   default='incremental')
    p.add_argument('--max-entries', type=int, default=500)
    args = p.parse_args()
    rb = RobotBridge()
    windows = rb.read_journal_windows(mode=args.mode,
                                       max_entries=args.max_entries)
    print(json.dumps(windows, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
