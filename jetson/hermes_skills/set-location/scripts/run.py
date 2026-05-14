#!/usr/bin/env python3
import argparse, sys
from robot_bridge import RobotBridge


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--name', required=True)
    p.add_argument('--parent', default='')
    args = p.parse_args()
    rb = RobotBridge()
    print(rb.set_current_location(args.name, parent=args.parent))
    return 0


if __name__ == '__main__':
    sys.exit(main())
