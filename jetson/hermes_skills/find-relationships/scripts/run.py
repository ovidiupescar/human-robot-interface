#!/usr/bin/env python3
import argparse, sys
from robot_bridge import RobotBridge

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--subject-id', required=True)
    p.add_argument('--relation', default='')
    p.add_argument('--hops', type=int, default=1)
    args = p.parse_args()
    rb = RobotBridge()
    for r in rb.find_related(args.subject_id, relation=args.relation,
                             hops=args.hops):
        print(f"- {r['name']} (id={r['id']})")
    return 0

if __name__ == '__main__':
    sys.exit(main())
