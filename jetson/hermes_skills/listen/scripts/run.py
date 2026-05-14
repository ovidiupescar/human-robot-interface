#!/usr/bin/env python3
"""listen skill helper."""

import argparse
import sys

from robot_bridge import RobotBridge


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--timeout", type=float, default=15.0)
    args = p.parse_args()

    rb = RobotBridge()
    text = rb.listen(timeout_seconds=args.timeout)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
