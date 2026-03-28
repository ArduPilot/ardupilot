#!/usr/bin/env python3
"""
ArduPilot Log Summary Tool

Supports:
1. MAVLink logs (.BIN / .tlog) using pymavlink
2. Simple structured text logs (fallback mode)

Outputs:
- Total message count
- Message type distribution
"""

import sys

from collections import Counter

from pymavlink import mavutil


def summarize_log(log_file):
    print(f"\nReading log: {log_file}\n")

    try:
        msg_types = Counter()
        total = 0

        log = mavutil.mavlink_connection(log_file)

        while True:
            msg = log.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            msg_types[msg_type] += 1
            total += 1

        if total == 0 or (len(msg_types) == 1 and "BAD_DATA" in msg_types):
            raise ValueError("Invalid MAVLink log")

    except ValueError:
        print("Falling back to text log parsing...\n")

        msg_types = Counter()
        total = 0

        with open(log_file, "r") as f:
            for line in f:
                if not line.strip():
                    continue

                msg_type = line.split(",")[0].strip()
                msg_types[msg_type] += 1
                total += 1

    print("===== LOG SUMMARY =====")
    print(f"Total Messages: {total}\n")

    print("Message Type Counts:")
    for msg_type, count in sorted(msg_types.items()):
        print(f"{msg_type}: {count}")

    if "ERR" in msg_types:
        print("\nWarning: Error messages detected in log (possible issue)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python log_summary.py <logfile>")
        sys.exit(1)

    summarize_log(sys.argv[1])
