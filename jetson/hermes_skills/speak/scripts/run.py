#!/usr/bin/env python3
"""speak skill helper."""

import argparse
import sys

from robot_bridge import RobotBridge


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--text", required=True)
    p.add_argument("--timeout", type=float, default=30.0)
    args = p.parse_args()

    rb = RobotBridge()
    print(rb.speak(args.text, timeout_seconds=args.timeout))
    return 0


if __name__ == "__main__":
    sys.exit(main())
