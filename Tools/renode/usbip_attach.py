#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Keep a Renode USB/IP device attached through firmware re-enumeration.

Does the USB/IP import handshake itself and hands the connected socket
to vhci_hcd through sysfs, so the distribution usbip tool is not
needed. Root is only required for the sysfs attach/detach files; run
once with --install-rules (as root) to make those group-writable for
dialout, after which this helper never needs root again.
"""

import argparse
import grp
import os
import signal
import socket
import struct
import subprocess
import sys
import time

from pathlib import Path

VHCI = Path("/sys/devices/platform/vhci_hcd.0")
VDEV_ST_USED = "006"
VDEV_ST_NULL = "004"

USBIP_VERSION = 0x0111
OP_REQ_IMPORT = 0x8003
OP_REP_IMPORT = 0x0003

UDEV_RULE_PATH = Path("/etc/udev/rules.d/99-vhci-user.rules")
MODULES_LOAD_PATH = Path("/etc/modules-load.d/vhci-hcd.conf")
UDEV_RULE = (
    "# let members of dialout attach/detach USB/IP devices without root\n"
    'ACTION=="add", SUBSYSTEM=="platform", KERNEL=="vhci_hcd.0", '
    "RUN+=\"/bin/sh -c 'chgrp dialout /sys%p/attach /sys%p/detach; "
    "chmod g+w /sys%p/attach /sys%p/detach'\"\n"
)

stop_requested = False


def parse_args():
    parser = argparse.ArgumentParser(
        description="attach a Renode USB/IP device and reattach after disconnects"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=3240)
    parser.add_argument("--busid", default="1-0")
    parser.add_argument(
        "--install-rules",
        action="store_true",
        help="one-time root setup: load vhci_hcd at boot and make its "
        "attach/detach files group-writable (dialout), so this helper "
        "no longer needs root",
    )
    return parser.parse_args()


def install_rules():
    if os.geteuid() != 0:
        sys.exit("--install-rules must run as root (once)")
    try:
        gid = grp.getgrnam("dialout").gr_gid
    except KeyError:
        sys.exit("--install-rules requires a dialout group")
    UDEV_RULE_PATH.write_text(UDEV_RULE, encoding="ascii")
    MODULES_LOAD_PATH.write_text("vhci_hcd\n", encoding="ascii")
    try:
        subprocess.run(["udevadm", "control", "--reload"], check=True)
        subprocess.run(["modprobe", "vhci_hcd"], check=True)
    except (OSError, subprocess.CalledProcessError) as error:
        sys.exit("failed to load vhci_hcd rules: %s" % error)
    for name in ("attach", "detach"):
        path = VHCI / name
        if not path.exists():
            sys.exit("vhci_hcd did not create %s" % path)
        os.chown(path, 0, gid)
        os.chmod(path, 0o220)
    print(f"installed {UDEV_RULE_PATH} and {MODULES_LOAD_PATH}; "
          "vhci attach now needs no root")


def import_device(args):
    """connect to the exporter and import the device, returning the
    connected socket (owned by the kernel after a successful attach)
    with its devid and speed"""
    sock = socket.create_connection((args.host, args.port), timeout=5)
    try:
        sock.sendall(
            struct.pack(">HHI", USBIP_VERSION, OP_REQ_IMPORT, 0)
            + args.busid.encode().ljust(32, b"\0")
        )
        reply = b""
        while len(reply) < 8:
            chunk = sock.recv(8 - len(reply))
            if not chunk:
                raise OSError("connection closed during import")
            reply += chunk
        _version, code, status = struct.unpack(">HHI", reply)
        if code != OP_REP_IMPORT or status != 0:
            raise OSError(
                f"import of {args.busid} refused (code {code:#06x} status {status})"
            )
        dev = b""
        while len(dev) < 312:
            chunk = sock.recv(312 - len(dev))
            if not chunk:
                raise OSError("short device description")
            dev += chunk
        busnum, devnum, speed = struct.unpack(">III", dev[288:300])
        return sock, (busnum << 16) | devnum, speed
    except BaseException:
        sock.close()
        raise


def free_vhci_port(speed):
    """a free port on the right-speed hub, from the status table"""
    want_hs = speed != 5
    try:
        lines = (VHCI / "status").read_text(encoding="ascii").splitlines()[1:]
    except (OSError, UnicodeError):
        return None
    for line in lines:
        fields = line.split()
        if (
            len(fields) >= 3
            and fields[2] == VDEV_ST_NULL
            and (fields[0] == "hs") == want_hs
        ):
            return int(fields[1])
    return None


def attach(args):
    """import the device and hand its socket to vhci_hcd; returns
    (port, error) with exactly one of them set"""
    try:
        sock, devid, speed = import_device(args)
    except OSError as err:
        return None, str(err)
    try:
        port = free_vhci_port(speed)
        if port is None:
            return None, "no free vhci port"
        with open(VHCI / "attach", "w", encoding="ascii") as f:
            f.write(f"{port} {sock.fileno()} {devid} {speed}")
    except OSError as err:
        return None, str(err)
    finally:
        # the kernel holds its own reference once attached
        sock.close()

    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if port_attached(port):
            return port, ""
        time.sleep(0.1)
    return None, "attach accepted but no vhci port came up"


def port_attached(port):
    try:
        lines = (VHCI / "status").read_text(encoding="ascii").splitlines()[1:]
    except (OSError, UnicodeError, IndexError):
        return False
    for line in lines:
        fields = line.split()
        try:
            if len(fields) >= 3 and int(fields[1]) == port:
                return fields[2] == VDEV_ST_USED
        except ValueError:
            continue
    return False


def detach(port):
    try:
        with open(VHCI / "detach", "w", encoding="ascii") as f:
            f.write(str(port))
    except OSError:
        pass


def request_stop(_signum, _frame):
    global stop_requested
    stop_requested = True


def main():
    global stop_requested
    stop_requested = False
    args = parse_args()
    if args.install_rules:
        install_rules()
        return
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    if not VHCI.exists():
        if os.geteuid() == 0:
            subprocess.run(["modprobe", "vhci_hcd"], check=True)
        else:
            sys.exit(
                "vhci_hcd is not loaded; run once as root:\n"
                f"  sudo {sys.argv[0]} --install-rules"
            )
    if not os.access(VHCI / "attach", os.W_OK):
        sys.exit(
            "no write access to vhci attach; either run as root or, once:\n"
            f"  sudo {sys.argv[0]} --install-rules"
        )

    active_port = None
    last_error = None
    try:
        while not stop_requested:
            if active_port is not None and port_attached(active_port):
                time.sleep(0.2)
                continue

            if active_port is not None:
                print("USB/IP device disconnected; waiting to reattach", flush=True)
                active_port = None

            port, error = attach(args)
            if port is not None:
                print(f"attached {args.busid} on vhci port {port}", flush=True)
                active_port = port
                last_error = None
            else:
                if error != last_error:
                    print(f"waiting for device: {error}", flush=True)
                    last_error = error
                time.sleep(1.0)
    finally:
        if active_port is not None and port_attached(active_port):
            detach(active_port)
            print("detached", flush=True)


if __name__ == "__main__":
    main()
