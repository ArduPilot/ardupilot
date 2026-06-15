#!/usr/bin/env python3

import argparse
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "modules" / "mavlink"))

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402


def param_id_to_str(param_id):
    if isinstance(param_id, bytes):
        return param_id.split(b"\0", 1)[0].decode("ascii")
    return param_id.split("\0", 1)[0]


def collect_list(endpoint, protected_names, timeout, expect_protected, signing_key=None):
    mav = mavutil.mavlink_connection(endpoint, autoreconnect=False, robust_parsing=True)
    mav.wait_heartbeat(timeout=timeout)

    if signing_key is not None:
        timestamp = int((max(time.time(), 1420070400) - 1420070400) * 100 * 1000)
        mav.mav.setup_signing_send(
            mav.target_system,
            mav.target_component,
            signing_key,
            timestamp,
            force_mavlink1=False,
        )
        time.sleep(0.5)
        mav.setup_signing(
            signing_key,
            sign_outgoing=True,
            initial_timestamp=timestamp + 60 * 100 * 1000,
            link_id=42,
        )

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

    visible_protected = sorted(set(protected_names).intersection(seen))
    if expect_protected and visible_protected != sorted(protected_names):
        missing = sorted(set(protected_names) - set(seen))
        raise AssertionError(f"protected params missing from signed dump: {missing}")
    if not expect_protected and visible_protected:
        raise AssertionError(f"protected params visible in unsigned dump: {visible_protected}")

    if len(seen) != advertised_count:
        raise AssertionError(
            f"received {len(seen)} unique PARAM_VALUE names, advertised {advertised_count}"
        )

    mode = "signed-list" if expect_protected else "unsigned-list"
    print(f"{mode} ok: received={len(seen)} advertised={advertised_count}")


def main():
    parser = argparse.ArgumentParser(description="Verify ArduPilot parameter protection over MAVLink.")
    parser.add_argument("--endpoint", default="tcp:127.0.0.1:5760")
    parser.add_argument("--protected", action="append", required=True)
    parser.add_argument(
        "--signing-key-hex",
        default="00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff",
    )
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("mode", choices=["unsigned-list", "signed-list"])
    args = parser.parse_args()

    if args.mode == "unsigned-list":
        collect_list(args.endpoint, args.protected, args.timeout, expect_protected=False)
    elif args.mode == "signed-list":
        signing_key = bytes.fromhex(args.signing_key_hex)
        if len(signing_key) != 32:
            raise ValueError("--signing-key-hex must decode to 32 bytes")
        collect_list(
            args.endpoint,
            args.protected,
            args.timeout,
            expect_protected=True,
            signing_key=signing_key,
        )


if __name__ == "__main__":
    main()
