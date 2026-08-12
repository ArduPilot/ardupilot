#!/usr/bin/env python3
"""
AP_SwarmMesh SITL Peer Table Scale Test

AP_SWARMMESH_MAX_PEERS now scales with the board's RAM class (HAL_MEM_CLASS)
instead of being hardcoded to 16 -- for SITL (HAL_MEM_CLASS_1000) that's the
full sysid range (255, sysid 0 is reserved for broadcast). Spinning up 255
real SITL vehicle processes to prove this is impractical (CPU/RAM cost scales
linearly, and ~250 processes will choke most machines). Instead, this
test launches a single real SITL receiver and injects synthetic HEARTBEAT
packets over raw UDP, one per distinct synthetic sysid, directly onto the
SwarmMesh multicast group (239.65.83.0:57733). This exercises exactly the
same AP_SwarmMesh_Backend::process_packet() / find_or_alloc_peer() path a
real peer's packet would, without needing hundreds of vehicle processes.

Verification: the receiver's own dataflash log (LOG_DISARMED=1) is parsed
for SMHB (heartbeat) messages after the run. Each successfully-tracked peer
logs exactly one distinct SysID there; if the peer table were still capped
at the old hardcoded 16, only the first 16 distinct sysids would ever appear
- everything after that would be dropped by find_or_alloc_peer() before
handle_mavlink() (and therefore the SMHB log write) is ever reached.

Usage:
    python3 swarm_sitl_scale_test.py
    python3 swarm_sitl_scale_test.py --count 250
"""

import argparse
import glob
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "modules" / "mavlink"))
from pymavlink import mavutil  # noqa: E402


MCAST_ADDR = "239.65.83.0"
MCAST_PORT = 57733
SYNC1, SYNC2 = 0xAD, 0xBC
SWARMMESH_VERSION_01 = 0x01
SWARMMESH_TYPE_MAVLINK = 0x00
SWARMMESH_NO_RTC = 0x01     # skip the deadline/staleness check: our synthetic senders have no real RTC to sync
SWARMMESH_BROADCAST = 0

RECEIVER_SYSID = 1
RECEIVER_TCP_PORT = 5760
BOOT_WAIT_S = 12


def find_sitl_binary():
    p = Path(__file__).resolve().parents[3] / "build" / "sitl" / "bin" / "arducopter"
    return str(p) if p.exists() else None


def write_param_file(path, log_hz):
    # log_hz is set high enough that log_rate_ok() never throttles at injection speed -
    # otherwise the combined-message-type RX log rate cap (default 50Hz) would make
    # SMHB entries an undercount of what the peer table actually holds.
    lines = [
        "P2P_TYPE 10",
        f"P2P_SYSID {RECEIVER_SYSID}",
        "P2P_DESTID 0",
        "P2P_TTL 1",
        "P2P_SR_POSITION 0",
        "P2P_SR_EXT_STAT 0",
        "P2P_SR_EXTRA1 0",
        "P2P_PRUNE_SECS 0",
        f"P2P_LOG_HZ {log_hz}",
        "LOG_DISARMED 1",
    ]
    Path(path).write_text("\n".join(lines) + "\n")


def make_heartbeat_payload(origin_id):
    mav = mavutil.mavlink.MAVLink(None, srcSystem=origin_id, srcComponent=1)
    msg = mav.heartbeat_encode(2, 3, 0, 0, 3, mavlink_version=3)
    return msg.pack(mav)


def make_packet(origin_id, seq, dest_id=SWARMMESH_BROADCAST, ttl=1):
    """Build one raw SwarmMesh p2p_header_t + MAVLink HEARTBEAT payload (see AP_SwarmMesh_packet.h)."""
    payload = make_heartbeat_payload(origin_id)
    hdr_no_crc = struct.pack(
        "<9BHQHB",
        SYNC1, SYNC2, SWARMMESH_VERSION_01, SWARMMESH_TYPE_MAVLINK,
        SWARMMESH_NO_RTC, origin_id, dest_id, origin_id, ttl,
        seq, 0, 0, len(payload),
    )
    crc = sum(hdr_no_crc) & 0xFF   # matches AP_SwarmMesh_Backend::send_mavlink()'s plain byte-sum CRC
    return hdr_no_crc + struct.pack("<B", crc) + payload


class Receiver:
    def __init__(self, sitl_bin, work_dir, speedup, log_hz):
        self.work_dir = work_dir
        os.makedirs(self.work_dir, exist_ok=True)
        self._param_file = os.path.join(self.work_dir, "receiver.parm")
        write_param_file(self._param_file, log_hz)
        self._log = open(os.path.join(self.work_dir, "console.log"), "w")
        self._proc = subprocess.Popen(
            [
                sitl_bin,
                "-w",
                "--model", "+",
                "--speedup", str(speedup),
                "--slave", "0",
                "--defaults", self._param_file,
                "--sim-address=127.0.0.1",
                "-I0",
            ],
            stdout=self._log, stderr=self._log, cwd=self.work_dir,
        )
        self.pid = self._proc.pid
        self._gcs_sock = None

    def connect_gcs(self, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                s = socket.socket()
                s.settimeout(1)
                s.connect(("127.0.0.1", RECEIVER_TCP_PORT))
                self._gcs_sock = s
                return True
            except (ConnectionRefusedError, TimeoutError, OSError):
                time.sleep(0.5)
        return False

    def stop(self):
        if self._gcs_sock:
            try:
                self._gcs_sock.close()
            except Exception:
                pass
        try:
            self._proc.terminate()
            self._proc.wait(timeout=3)
        except Exception:
            self._proc.kill()
        self._log.close()

    def bin_log_path(self):
        bins = sorted(glob.glob(os.path.join(self.work_dir, "logs", "*.BIN")))
        return bins[-1] if bins else None


def heard_sysids(bin_path):
    if bin_path is None:
        return set()
    mlog = mavutil.mavlink_connection(bin_path)
    seen = set()
    while True:
        m = mlog.recv_match(type="SMHB")
        if m is None:
            break
        seen.add(m.SysID)
    return seen


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--count", type=int, default=250, help="number of distinct synthetic peer sysids to inject (default: 250; valid sysids are 2..255 since 0=broadcast and 1=receiver)")
    ap.add_argument("--inject-delay-ms", type=float, default=5, help="delay between injected packets in ms (default: 5)")
    ap.add_argument("--settle", type=int, default=5, help="seconds to wait after injection for the receiver to finish processing/logging (default: 5)")
    ap.add_argument("--log-hz", type=int, default=2000, help="P2P_LOG_HZ on the receiver, high enough to not rate-limit SMHB writes at injection speed (default: 2000, the param's max)")
    ap.add_argument("--speedup", type=float, default=1, help="SITL speedup factor (default: 1)")
    ap.add_argument("--boot-wait", type=int, default=BOOT_WAIT_S, help=f"seconds to wait for boot (default: {BOOT_WAIT_S})")
    ap.add_argument("--sitl-bin", default=None, help="Path to arducopter SITL binary (auto-detected if omitted)")
    ap.add_argument("--work-dir", default=None, help="Working dir for logs/params (default: auto temp dir)")
    return ap.parse_args()


def print_sep(width=78):
    print("-" * width)


def main():
    args = parse_args()
    max_sysid = 255
    if args.count > max_sysid - 1:  # sysid range is 2..255 (254 slots), excluding 0 (broadcast) and 1 (receiver)
        print(f"ERROR: --count too large; at most {max_sysid - 1} distinct synthetic sysids are available (2..255)")
        sys.exit(1)

    sitl_bin = args.sitl_bin or find_sitl_binary()
    if not sitl_bin or not Path(sitl_bin).exists():
        print("ERROR: could not find arducopter SITL binary. Pass --sitl-bin.")
        sys.exit(1)

    work_dir = args.work_dir or tempfile.mkdtemp(prefix="swarm_scale_")

    print()
    print_sep()
    print("  AP_SwarmMesh SITL Peer Table Scale Test")
    print_sep()
    print(f"  Synthetic peers to inject : {args.count}  (sysids 2..{args.count + 1})")
    print(f"  Receiver sysid            : {RECEIVER_SYSID}")
    print(f"  SITL binary               : {sitl_bin}")
    print(f"  Work dir                  : {work_dir}")
    print_sep()

    subprocess.run(["pkill", "-9", "-f", r"arducopter.*-I\d"], capture_output=True)
    time.sleep(1)

    receiver = Receiver(sitl_bin, work_dir, args.speedup, args.log_hz)
    print(f"\n  Launched receiver: pid={receiver.pid}")

    def cleanup(sig=None, frame=None):
        print("\n  Stopping receiver...")
        receiver.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print(f"  Waiting for GCS TCP port (up to {args.boot_wait}s)...")
    ok = receiver.connect_gcs(timeout=args.boot_wait)
    print(f"    port {RECEIVER_TCP_PORT}: {'ok' if ok else 'TIMEOUT'}")
    if not ok:
        print("  ERROR: receiver never came up")
        receiver.stop()
        sys.exit(1)

    print(f"  Letting receiver finish boot + param load ({args.boot_wait}s)...")
    time.sleep(args.boot_wait)

    print(f"\n  Injecting {args.count} synthetic HEARTBEATs (one per distinct sysid)...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    injected_sysids = list(range(2, args.count + 2))
    t0 = time.monotonic()
    for sysid in injected_sysids:
        pkt = make_packet(origin_id=sysid, seq=1)
        sock.sendto(pkt, (MCAST_ADDR, MCAST_PORT))
        if args.inject_delay_ms > 0:
            time.sleep(args.inject_delay_ms / 1000.0)
    inject_elapsed = time.monotonic() - t0
    print(f"  Injected {len(injected_sysids)} packets in {inject_elapsed:.1f}s")

    print(f"  Waiting {args.settle}s for the receiver to finish processing + logging...")
    time.sleep(args.settle)

    print("  Stopping receiver...")
    receiver.stop()
    time.sleep(1)  # let dataflash finish flushing/closing

    print()
    print_sep()
    print("  RESULTS")
    print_sep()

    bin_path = receiver.bin_log_path()
    if bin_path is None:
        print("  FAIL: no dataflash log was produced")
        sys.exit(1)

    seen = heard_sysids(bin_path)
    seen.discard(RECEIVER_SYSID)  # multicast loopback: receiver may hear its own heartbeat
    expected = set(injected_sysids)
    missing = expected - seen
    unexpected = seen - expected

    print(f"  Injected   : {len(expected)} distinct sysids")
    print(f"  Tracked    : {len(seen)} distinct sysids seen in receiver's own SMHB log")
    if missing:
        shown = sorted(missing)[:20]
        print(f"  MISSING    : {len(missing)} sysid(s) never made it into the peer table, e.g. {shown}{' ...' if len(missing) > 20 else ''}")
    if unexpected:
        print(f"  UNEXPECTED : {sorted(unexpected)}")

    print_sep()
    passed = not missing and not unexpected
    print("  SCALE TEST:", "PASSED - peer table held every injected peer" if passed else "FAILED")
    print(f"  Logs written to: {work_dir}")
    print_sep()
    print()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
