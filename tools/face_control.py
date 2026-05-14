#!/usr/bin/env python3
"""Robot face controller — send state commands over serial."""

import serial
import sys
import time

STATES = {
    "1": ("STANDBY",    0, 0.0),
    "2": ("PROCESSING", 1, 0.0),
    "3": ("SPEAKING",   2, 0.5),
    "4": ("AGGRESSIVE", 3, 0.7),
}

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    baud = 115200

    print(f"Connecting to {port}...")
    ser = serial.Serial(port, baud, timeout=0.1)
    print(f"Connected.\n")

    while True:
        print("States:")
        print("  1 - STANDBY    (breathing rings)")
        print("  2 - PROCESSING (cyan waveform)")
        print("  3 - SPEAKING   (orange wave)")
        print("  4 - AGGRESSIVE (red spikes)")
        print("  a - auto cycle (1-2-3-4-1... every 3s)")
        print("  q - quit\n")

        choice = input("State> ").strip()

        if choice == "q":
            break

        if choice == "a":
            print("Auto-cycling (Ctrl+C to stop)...")
            try:
                while True:
                    for key in ["1", "2", "3", "4"]:
                        name, state, amp = STATES[key]
                        cmd = f'{{"state":{state},"amp":{amp}}}\n'
                        ser.write(cmd.encode())
                        print(f"  -> {name}")
                        time.sleep(3)
            except KeyboardInterrupt:
                print("\nStopped.\n")
            continue

        if choice not in STATES:
            print(f"Invalid: '{choice}'\n")
            continue

        name, state, amp = STATES[choice]
        cmd = f'{{"state":{state},"amp":{amp}}}\n'
        ser.write(cmd.encode())
        print(f"Sent: {cmd.strip()} -> {name}\n")

        # Read response
        while ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"  < {line}")
        print()

    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
