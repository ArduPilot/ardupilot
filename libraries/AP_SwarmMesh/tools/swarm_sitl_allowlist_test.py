#!/usr/bin/env python3
"""
AP_SwarmMesh SITL Allowlist Test

Launches N ArduCopter SITL peers on the SwarmMesh multicast mesh
(239.65.83.0:57733). Every instance broadcasts to everyone (P2P_DESTID=0),
so all instances physically receive every peer's traffic regardless of
any allowlist -- this test proves the _PEER_* neighbourhood filter is
enforced in software (AP_SwarmMesh::peer_is_allowed()), not just an
artifact of who can hear whom.

One instance (the "observer") sets P2P_PEER_01, P2P_PEER_02, ... to
restrict itself to a specific subset of sysids. All other instances are
left with the default (all slots 0 => accept any peer).

Each instance logs to its own dataflash log (LOG_DISARMED=1, so logging
happens even though nothing ever arms) in its own working directory. This
test parses each instance's own log for SMHB (heartbeat) messages after
the run -- the SysID field there records which peers that specific
instance actually accepted into its table. The observer's log should
only ever contain SysIDs from its allowlist; every other instance's log
should contain every other peer's SysID.

Usage:
    python3 swarm_sitl_allowlist_test.py
    python3 swarm_sitl_allowlist_test.py --peers 8 --observer-sysid 1 --allow 2 3 4
"""

import argparse
import glob
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "modules" / "mavlink"))
from pymavlink import mavutil  # noqa: E402


SITL_BASE_TCP_PORT = 5760   # GCS port for instance i = base + 10*i
BOOT_WAIT_S = 12
RUN_S = 20                  # time to let heartbeats + logging accumulate


def find_sitl_binary():
    p = Path(__file__).resolve().parents[3] / "build" / "sitl" / "bin" / "arducopter"
    return str(p) if p.exists() else None


def write_param_file(path, sysid, allowed_peers, ttl):
    lines = [
        "P2P_TYPE 10",           # Type::SITL
        f"P2P_SYSID {sysid}",
        "P2P_DESTID 0",          # broadcast: deliver to all peers
        f"P2P_TTL {ttl}",
        "P2P_SR_POSITION 0",     # heartbeat only: keeps verification to a single log message type
        "P2P_SR_EXT_STAT 0",
        "P2P_SR_EXTRA1 0",
        "P2P_PRUNE_SECS 0",      # disable pruning during test
        "LOG_DISARMED 1",       # dataflash logging happens without ever arming
    ]
    for i, peer_sysid in enumerate(allowed_peers[:16]):
        lines.append(f"P2P_PEER_{i + 1:02d} {peer_sysid}")
    Path(path).write_text("\n".join(lines) + "\n")


class SITLInstance:
    """
    Unlike swarm_sitl_test.py's instance class, each of these runs in its own
    working directory: AP_Logger's default log path ("logs/NN.BIN") is relative
    to cwd, so instances sharing a directory would collide over the same log file.
    """

    def __init__(self, idx, sysid, sitl_bin, work_dir, speedup):
        self.idx = idx
        self.sysid = sysid
        self.tcp_port = SITL_BASE_TCP_PORT + 10 * idx
        self.work_dir = os.path.join(work_dir, f"peer_{sysid}")
        os.makedirs(self.work_dir, exist_ok=True)
        self._param_file = os.path.join(self.work_dir, "peer.parm")
        self._log = open(os.path.join(self.work_dir, "console.log"), "w")
        self._proc = None
        self._gcs_sock = None
        self._speedup = speedup
        self._sitl_bin = sitl_bin

    def write_params(self, allowed_peers, ttl):
        write_param_file(self._param_file, self.sysid, allowed_peers, ttl)

    def start(self):
        self._proc = subprocess.Popen(
            [
                self._sitl_bin,
                "-w",
                "--model", "+",
                "--speedup", str(self._speedup),
                "--slave", "0",
                "--defaults", self._param_file,
                "--sim-address=127.0.0.1",
                f"-I{self.idx}",
            ],
            stdout=self._log, stderr=self._log, cwd=self.work_dir,
        )
        self.pid = self._proc.pid

    def connect_gcs(self, timeout=10):
        """Open a TCP connection to unblock SITL's 'Waiting for connection'."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                s = socket.socket()
                s.settimeout(1)
                s.connect(("127.0.0.1", self.tcp_port))
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
    """Return the set of distinct SysID values seen in SMHB (heartbeat) messages in this dataflash log."""
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
    ap.add_argument("--peers", type=int, default=6, help="total number of SITL peers, sysids 1..N (default: 6)")
    ap.add_argument("--observer-sysid", type=int, default=1, help="sysid of the instance that sets the allowlist (default: 1)")
    ap.add_argument("--allow", type=int, nargs="+", default=[2, 3], help="sysids the observer allows (default: 2 3)")
    ap.add_argument("--duration", type=int, default=RUN_S, help=f"seconds to run after boot (default: {RUN_S})")
    ap.add_argument("--speedup", type=float, default=1, help="SITL speedup factor (default: 1)")
    ap.add_argument("--boot-wait", type=int, default=BOOT_WAIT_S, help=f"seconds to wait for boot (default: {BOOT_WAIT_S})")
    ap.add_argument("--sitl-bin", default=None, help="Path to arducopter SITL binary (auto-detected if omitted)")
    ap.add_argument("--work-dir", default=None, help="Working dir for logs/params (default: auto temp dir)")
    return ap.parse_args()


def print_sep(width=78):
    print("-" * width)


def main():
    args = parse_args()
    if args.observer_sysid not in range(1, args.peers + 1):
        print("ERROR: --observer-sysid must be one of the peer sysids (1..--peers)")
        sys.exit(1)

    sitl_bin = args.sitl_bin or find_sitl_binary()
    if not sitl_bin or not Path(sitl_bin).exists():
        print("ERROR: could not find arducopter SITL binary. Pass --sitl-bin.")
        sys.exit(1)

    work_dir = args.work_dir or tempfile.mkdtemp(prefix="swarm_allowlist_")
    os.makedirs(work_dir, exist_ok=True)

    print()
    print_sep()
    print("  AP_SwarmMesh SITL Allowlist Test")
    print_sep()
    print(f"  Peers          : {args.peers} (sysids 1..{args.peers})")
    print(f"  Observer sysid : {args.observer_sysid}")
    print(f"  Allowlist      : {sorted(args.allow)}")
    print(f"  SITL binary    : {sitl_bin}")
    print(f"  Work dir       : {work_dir}")
    print_sep()

    # kill any leftover arducopter SITL processes from previous runs (prevents port conflicts)
    subprocess.run(["pkill", "-9", "-f", r"arducopter.*-I\d"], capture_output=True)
    time.sleep(1)

    instances = []

    def cleanup(sig=None, frame=None):
        print("\n  Stopping all SITL instances...")
        for inst in instances:
            inst.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print(f"\n  Launching {args.peers} SITL instances...")
    for i in range(args.peers):
        sysid = i + 1
        inst = SITLInstance(i, sysid, sitl_bin, work_dir, args.speedup)
        allowed = list(args.allow) if sysid == args.observer_sysid else []
        inst.write_params(allowed, ttl=1)
        inst.start()
        instances.append(inst)
        filt_desc = f"allowlist={allowed}" if allowed else "no filter (accepts all)"
        print(f"    [{i:2d}] sysid={sysid}  pid={inst.pid}  {filt_desc}")

    print(f"\n  Waiting for SITL TCP ports (up to {args.boot_wait}s)...")
    for inst in instances:
        ok = inst.connect_gcs(timeout=args.boot_wait)
        print(f"    sysid={inst.sysid}  port {inst.tcp_port}: {'ok' if ok else 'TIMEOUT'}")

    print(f"\n  Running for {args.duration}s to let heartbeats accumulate in each instance's own log...")
    time.sleep(args.duration)

    print("  Stopping SITL instances...")
    for inst in instances:
        inst.stop()
    time.sleep(1)  # let dataflash finish flushing/closing

    print()
    print_sep()
    print("  RESULTS  (SysIDs recorded via SMHB heartbeats in each instance's OWN dataflash log)")
    print_sep()

    all_sysids = set(range(1, args.peers + 1))
    all_pass = True
    for inst in sorted(instances, key=lambda x: x.sysid):
        bin_path = inst.bin_log_path()
        seen = heard_sysids(bin_path)
        seen.discard(inst.sysid)  # multicast loopback: an instance may hear its own broadcast
        expected_others = all_sysids - {inst.sysid}

        if inst.sysid == args.observer_sysid:
            expected = set(args.allow) & expected_others
            label = "OBSERVER (filtered)"
        else:
            expected = expected_others
            label = "unfiltered"

        status = "PASS" if seen == expected else "FAIL"
        if status == "FAIL":
            all_pass = False
        log_note = "" if bin_path else "  (no dataflash log found!)"
        print(f"    sysid={inst.sysid:2d}  [{label:20s}]  expected={sorted(expected)!s:<20}  saw={sorted(seen)!s:<20}  {status}{log_note}")

    print_sep()
    print("  ALLOWLIST TEST:", "PASSED" if all_pass else "FAILED")
    print(f"  Logs written to: {work_dir}")
    print_sep()
    print()
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
