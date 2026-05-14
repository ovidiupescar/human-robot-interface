#!/usr/bin/env python3
"""where-am-i runner."""
import sys
from robot_bridge import RobotBridge


def main():
    rb = RobotBridge()
    info = rb.where_am_i()
    print(info)
    return 0


if __name__ == '__main__':
    sys.exit(main())
