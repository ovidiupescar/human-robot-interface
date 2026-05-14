#!/usr/bin/env python3
import sys
from robot_bridge import RobotBridge

def main():
    rb = RobotBridge()
    print(rb.who_am_i())
    return 0

if __name__ == '__main__':
    sys.exit(main())
