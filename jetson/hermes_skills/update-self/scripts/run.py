#!/usr/bin/env python3
import argparse, sys
from robot_bridge import RobotBridge

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--set', dest='kv', default='',
                   help="comma-separated key=value pairs to update preferences")
    p.add_argument('--note', default='',
                   help="store a free-form fact about self")
    args = p.parse_args()
    rb = RobotBridge()
    if args.kv:
        prefs = dict(p.split('=', 1) for p in args.kv.split(',') if '=' in p)
        print(rb.update_self_preferences(prefs))
    if args.note:
        print(rb.add_self_fact(args.note))
    return 0

if __name__ == '__main__':
    sys.exit(main())
