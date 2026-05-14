#!/usr/bin/env python3
import argparse, sys
from robot_bridge import RobotBridge

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--subject-id', required=True)
    p.add_argument('--subject-type', default='Person')
    p.add_argument('--content', required=True)
    p.add_argument('--tags', default='')
    p.add_argument('--source', default='manual')
    p.add_argument('--confidence', type=float, default=1.0)
    args = p.parse_args()
    rb = RobotBridge()
    print(rb.remember(args.subject_id, args.subject_type, args.content,
                      tags=args.tags, source=args.source,
                      confidence=args.confidence))
    return 0

if __name__ == '__main__':
    sys.exit(main())
