#!/usr/bin/env python3
import argparse, sys
from robot_bridge import RobotBridge

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--subject-id', required=True)
    p.add_argument('--subject-type', default='Person')
    p.add_argument('--limit', type=int, default=10)
    p.add_argument('--query', default='')
    args = p.parse_args()
    rb = RobotBridge()
    for fact in rb.recall(args.subject_id, args.subject_type,
                          query=args.query, limit=args.limit):
        print(f"- {fact['content']} (conf {fact['score']:.2f})")
    return 0

if __name__ == '__main__':
    sys.exit(main())
