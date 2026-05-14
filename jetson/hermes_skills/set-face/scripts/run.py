#!/usr/bin/env python3
"""set-face skill helper.

Usage:
    python run.py --state standby
    python run.py --state speaking --amplitude 0.7
"""

import argparse
import sys

from robot_bridge import RobotBridge, STATE_NAME_TO_INT


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--state", required=True, choices=list(STATE_NAME_TO_INT.keys()))
    p.add_argument("--amplitude", type=float, default=0.0)
    args = p.parse_args()

    rb = RobotBridge()
    result = rb.set_face(args.state, args.amplitude)
    print(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
