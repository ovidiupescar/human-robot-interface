#!/usr/bin/env python3
import argparse, sys
from robot_bridge import RobotBridge

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--name', required=True)
    args = p.parse_args()
    rb = RobotBridge()
    print(rb.register_person(args.name))
    return 0

if __name__ == '__main__':
    sys.exit(main())
