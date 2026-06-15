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

SIGNING_TIMESTAMP_OFFSET = 60 * 100 * 1000


def param_id_to_str(param_id):
    if isinstance(param_id, bytes):
        return param_id.split(b"\0", 1)[0].decode("ascii")
    return param_id.split("\0", 1)[0]


def signing_timestamp():
    return int((max(time.time(), 1420070400) - 1420070400) * 100 * 1000)


def enable_client_signing(mav, signing_key):
    mav.setup_signing(
        signing_key,
        sign_outgoing=True,
        initial_timestamp=signing_timestamp() + SIGNING_TIMESTAMP_OFFSET,
        link_id=42,
    )


def setup_vehicle_signing(mav, signing_key):
    timestamp = signing_timestamp()
    mav.mav.setup_signing_send(
        mav.target_system,
        mav.target_component,
        signing_key,
        timestamp,
        force_mavlink1=False,
    )
    time.sleep(0.5)
    enable_client_signing(mav, signing_key)


def read_param(mav, name, timeout):
    mav.mav.param_request_read_send(
        mav.target_system,
        mav.target_component,
        name.encode("ascii"),
        -1,
    )

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = mav.recv_match(blocking=True, timeout=1)
        if msg is None or msg.get_type() != "PARAM_VALUE":
            continue
        if param_id_to_str(msg.param_id) == name:
            return msg
    return None


def collect_list(endpoint, protected_names, timeout, expect_protected, signing_key=None):
    mav = mavutil.mavlink_connection(endpoint, autoreconnect=False, robust_parsing=True)
    mav.wait_heartbeat(timeout=timeout)

    if signing_key is not None:
        setup_vehicle_signing(mav, signing_key)

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

    if expect_protected:
        mode = "signed-list" if signing_key is not None else "unsigned-visible-list"
    else:
        mode = "unsigned-list"
    print(f"{mode} ok: received={len(seen)} advertised={advertised_count}")


def check_unsigned_read_set(endpoint, protected_name, timeout, signing_key):
    mav = mavutil.mavlink_connection(endpoint, autoreconnect=False, robust_parsing=True)
    mav.wait_heartbeat(timeout=timeout)

    unsigned_read = read_param(mav, protected_name, min(timeout, 5.0))
    if unsigned_read is not None:
        raise AssertionError(f"unsigned read returned protected param {protected_name}")

    setup_vehicle_signing(mav, signing_key)
    before = read_param(mav, protected_name, timeout)
    if before is None:
        raise AssertionError(f"signed read did not return protected param {protected_name}")

    old_value = before.param_value
    new_value = old_value + 1.0

    mav.disable_signing()
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        protected_name.encode("ascii"),
        new_value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    time.sleep(0.5)

    enable_client_signing(mav, signing_key)
    after = read_param(mav, protected_name, timeout)
    if after is None:
        raise AssertionError(f"signed re-read did not return protected param {protected_name}")
    if after.param_value != old_value:
        raise AssertionError(
            f"unsigned set changed {protected_name}: before={old_value} after={after.param_value}"
        )

    print(f"unsigned-read-set ok: {protected_name} unchanged={old_value} attempted={new_value}")


def main():
    parser = argparse.ArgumentParser(description="Verify ArduPilot parameter protection over MAVLink.")
    parser.add_argument("--endpoint", default="tcp:127.0.0.1:5760")
    parser.add_argument("--protected", action="append", required=True)
    parser.add_argument(
        "--signing-key-hex",
        default="00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff",
    )
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument(
        "mode",
        choices=["unsigned-list", "signed-list", "unsigned-visible-list", "unsigned-read-set"],
    )
    args = parser.parse_args()

    if args.mode == "unsigned-list":
        collect_list(args.endpoint, args.protected, args.timeout, expect_protected=False)
    elif args.mode == "unsigned-visible-list":
        collect_list(args.endpoint, args.protected, args.timeout, expect_protected=True)
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
    elif args.mode == "unsigned-read-set":
        signing_key = bytes.fromhex(args.signing_key_hex)
        if len(signing_key) != 32:
            raise ValueError("--signing-key-hex must decode to 32 bytes")
        if len(args.protected) != 1:
            raise ValueError("unsigned-read-set expects exactly one --protected parameter")
        check_unsigned_read_set(args.endpoint, args.protected[0], args.timeout, signing_key)


if __name__ == "__main__":
    main()
