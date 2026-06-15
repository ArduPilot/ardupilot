#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "modules" / "mavlink"))

from pymavlink import mavutil  # noqa: E402


def param_id_to_str(param_id):
    if isinstance(param_id, bytes):
        return param_id.split(b"\0", 1)[0].decode("ascii")
    return param_id.split("\0", 1)[0]


def collect_unsigned_list(endpoint, protected_names, timeout):
    mav = mavutil.mavlink_connection(endpoint, autoreconnect=False, robust_parsing=True)
    mav.wait_heartbeat(timeout=timeout)

    mav.mav.param_request_list_send(mav.target_system, mav.target_component)

    deadline = time.monotonic() + timeout
    advertised_count = None
    seen = {}

    while time.monotonic() < deadline:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            continue

        name = param_id_to_str(msg.param_id)
        seen[name] = msg
        advertised_count = msg.param_count
        if advertised_count is not None and len(seen) >= advertised_count:
            break

    if advertised_count is None:
        raise AssertionError("no PARAM_VALUE messages received")

    visible_protected = sorted(set(protected_names) & set(seen))
    if visible_protected:
        raise AssertionError(f"protected params visible in unsigned dump: {visible_protected}")

    if len(seen) != advertised_count:
        raise AssertionError(
            f"received {len(seen)} unique PARAM_VALUE names, advertised {advertised_count}"
        )

    print(f"unsigned-list ok: received={len(seen)} advertised={advertised_count}")


def main():
    parser = argparse.ArgumentParser(description="Verify ArduPilot parameter protection over MAVLink.")
    parser.add_argument("--endpoint", default="tcp:127.0.0.1:5760")
    parser.add_argument("--protected", action="append", required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("mode", choices=["unsigned-list"])
    args = parser.parse_args()

    if args.mode == "unsigned-list":
        collect_unsigned_list(args.endpoint, args.protected, args.timeout)


if __name__ == "__main__":
    main()
