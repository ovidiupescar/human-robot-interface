#!/usr/bin/env python3
import sys
from robot_bridge import RobotBridge


def main():
    rb = RobotBridge()
    for loc in rb.list_locations():
        parent = f" (in {loc['parent']})" if loc.get('parent') else ''
        print(f"- {loc['name']}{parent}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
