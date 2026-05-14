#!/usr/bin/env python3
import argparse, sys
from robot_bridge import RobotBridge

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--date', required=True, help='YYYY-MM-DD')
    args = p.parse_args()
    rb = RobotBridge()
    print(rb.forget_day(args.date))
    return 0

if __name__ == '__main__':
    sys.exit(main())
