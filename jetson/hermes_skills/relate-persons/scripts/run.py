#!/usr/bin/env python3
import argparse, sys
from robot_bridge import RobotBridge

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--a', required=True)
    p.add_argument('--b', required=True)
    p.add_argument('--relation', required=True)
    p.add_argument('--description', default='')
    p.add_argument('--bidirectional', action='store_true')
    args = p.parse_args()
    rb = RobotBridge()
    print(rb.relate_persons(args.a, args.b, args.relation,
                            description=args.description,
                            bidirectional=args.bidirectional))
    return 0

if __name__ == '__main__':
    sys.exit(main())
