#!/usr/bin/env python3
"""
AP_SwarmMesh SITL Swarm Test
Launches N ArduCopter SITL instances connected over the SwarmMesh multicast
mesh (239.65.83.0:57733), exercises the full RX/TX stream pipeline, and
reports per-peer packet throughput and system resource usage.

All peers use P2P_DESTID=0 (broadcast) so every instance hears every other
instance without forwarding amplification.

Usage:
    python3 swarm_sitl_test.py [options]

Examples:
    # 4-peer quick smoke test
    python3 swarm_sitl_test.py --peers 4 --duration 15

    # full 16-peer stream test
    python3 swarm_sitl_test.py --peers 16 --duration 30 --sr-pos 5 --sr-ext 2 --sr-extra1 2

    # high-speed with speedup
    python3 swarm_sitl_test.py --peers 8 --duration 60 --speedup 5
"""

import argparse
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from collections import defaultdict
from pathlib import Path


# ---- constants ----------------------------------------------------------------

MCAST_ADDR = "239.65.83.0"
MCAST_PORT = 57733
SYNC1, SYNC2 = 0xAD, 0xBC
SITL_BASE_TCP_PORT  = 5760   # GCS port for instance i = base + 10*i
SITL_BASE_SIM_PORT  = 5501   # physics sim input port for instance i = base + 10*i
BOOT_WAIT_S         = 12     # seconds to wait for all instances to finish booting

# ---- helpers ------------------------------------------------------------------

def find_sitl_binary():
    candidates = [
        Path(__file__).resolve().parents[3] / "build" / "sitl" / "bin" / "arducopter",
        Path("/usr/local/bin/arducopter"),
    ]
    for p in candidates:
        if p.exists():
            return str(p)
    return None


def write_param_file(path, sysid, sr_pos, sr_ext, sr_extra1, ttl):
    lines = [
        f"P2P_TYPE 10",          # Type::SITL
        f"MAV_SYSID {sysid}",
        f"P2P_DESTID 0",         # broadcast: deliver to all peers
        f"P2P_TTL {ttl}",
        f"P2P_SR_POSITION {sr_pos}",
        f"P2P_SR_EXT_STAT {sr_ext}",
        f"P2P_SR_EXTRA1 {sr_extra1}",
        f"P2P_PRUNE_SECS 0",     # disable pruning during test
    ]
    Path(path).write_text("\n".join(lines) + "\n")


def gcs_port(instance_idx):
    return SITL_BASE_TCP_PORT + 10 * instance_idx


def sim_port(instance_idx):
    return SITL_BASE_SIM_PORT + 10 * instance_idx


# ---- packet monitor -----------------------------------------------------------

class PacketMonitor:
    """
    Background thread that joins the SwarmMesh multicast group and counts
    received packets per origin_id, along with total bytes.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.counts  = defaultdict(int)    # origin_id -> packet count
        self.bytes_  = defaultdict(int)    # origin_id -> total bytes
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3)

    def snapshot(self):
        with self.lock:
            return dict(self.counts), dict(self.bytes_)

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", MCAST_PORT))
        mreq = struct.pack("4sL", socket.inet_aton(MCAST_ADDR), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                data, _ = sock.recvfrom(4096)
                if len(data) >= 6 and data[0] == SYNC1 and data[1] == SYNC2:
                    origin_id = data[5]
                    with self.lock:
                        self.counts[origin_id]  += 1
                        self.bytes_[origin_id]  += len(data)
            except socket.timeout:
                pass
        sock.close()


# ---- perf monitor -------------------------------------------------------------

HZ = os.sysconf("SC_CLK_TCK")

def _read_proc_stat(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().split()
        utime  = int(fields[13])
        stime  = int(fields[14])
        return utime + stime
    except Exception:
        return None

def _read_rss_kb(pid):
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1])
    except Exception:
        return None
    return None


class PerfMonitor:
    """
    Background thread that samples CPU% and RSS (kB) for a list of PIDs.
    """

    def __init__(self, pids, interval_s=2.0):
        self.pids      = list(pids)
        self.interval  = interval_s
        self.lock      = threading.Lock()
        self._samples  = {pid: {"cpu": [], "rss": []} for pid in pids}
        self._stop     = threading.Event()
        self._thread   = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3)

    def summary(self):
        """Returns {pid: {cpu_avg, cpu_max, rss_avg_kb, rss_peak_kb}}."""
        with self.lock:
            result = {}
            for pid, s in self._samples.items():
                cpus = s["cpu"]
                rsss = s["rss"]
                result[pid] = {
                    "cpu_avg":    round(sum(cpus) / len(cpus), 1) if cpus else 0.0,
                    "cpu_max":    round(max(cpus), 1)             if cpus else 0.0,
                    "rss_avg_kb": int(sum(rsss) / len(rsss))      if rsss else 0,
                    "rss_peak_kb":int(max(rsss))                  if rsss else 0,
                }
            return result

    def _run(self):
        prev_ticks = {}
        prev_wall  = {}
        while not self._stop.is_set():
            now = time.monotonic()
            for pid in self.pids:
                ticks = _read_proc_stat(pid)
                rss   = _read_rss_kb(pid)
                if ticks is not None and pid in prev_ticks:
                    delta_ticks = ticks - prev_ticks[pid]
                    delta_wall  = now   - prev_wall[pid]
                    cpu = (delta_ticks / HZ) / delta_wall * 100.0
                    with self.lock:
                        self._samples[pid]["cpu"].append(cpu)
                if rss is not None:
                    with self.lock:
                        self._samples[pid]["rss"].append(rss)
                if ticks is not None:
                    prev_ticks[pid] = ticks
                    prev_wall[pid]  = now
            self._stop.wait(self.interval)


# ---- SITL process management --------------------------------------------------

class SITLInstance:
    def __init__(self, idx, sysid, sitl_bin, param_file, work_dir, speedup):
        self.idx       = idx
        self.sysid     = sysid
        self.tcp_port  = gcs_port(idx)
        self._log      = open(os.path.join(work_dir, f"sitl_{idx}.log"), "w")
        self._proc     = subprocess.Popen(
            [
                sitl_bin,
                "-w",                       # wipe params on start
                "--model", "+",
                "--speedup", str(speedup),
                "--slave", "0",
                "--defaults", param_file,
                "--sim-address=127.0.0.1",
                f"-I{idx}",
            ],
            stdout=self._log, stderr=self._log,
        )
        self.pid = self._proc.pid
        self._gcs_sock = None

    def connect_gcs(self, timeout=10):
        """Open a TCP connection to port 5760+10*idx to unblock SITL's 'Waiting for connection'."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                s = socket.socket()
                s.settimeout(1)
                s.connect(("127.0.0.1", self.tcp_port))
                self._gcs_sock = s
                return True
            except (ConnectionRefusedError, TimeoutError):
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


# ---- main ---------------------------------------------------------------------

def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--peers",     type=int, default=16,  help="Number of SITL peers (default: 16)")
    ap.add_argument("--duration",  type=int, default=30,  help="Test duration in seconds (default: 30)")
    ap.add_argument("--speedup",   type=float, default=1, help="SITL speedup factor (default: 1)")
    ap.add_argument("--sr-pos",    type=int, default=2,   help="P2P_SR_POSITION Hz (default: 2)")
    ap.add_argument("--sr-ext",    type=int, default=1,   help="P2P_SR_EXT_STAT Hz (default: 1)")
    ap.add_argument("--sr-extra1", type=int, default=1,   help="P2P_SR_EXTRA1 Hz (default: 1)")
    ap.add_argument("--ttl",       type=int, default=1,   help="P2P_TTL (default: 1; use 1 with broadcast to avoid forwarding loops)")
    ap.add_argument("--sitl-bin",  default=None,          help="Path to arducopter SITL binary (auto-detected if omitted)")
    ap.add_argument("--work-dir",  default=None,          help="Working dir for logs/params (default: auto temp dir)")
    ap.add_argument("--no-perf",   action="store_true",   help="Disable CPU/RAM sampling")
    ap.add_argument("--boot-wait", type=int, default=BOOT_WAIT_S, help=f"Seconds to wait for all instances to boot (default: {BOOT_WAIT_S})")
    return ap.parse_args()


def print_sep(width=80):
    print("─" * width)


def main():
    args = parse_args()

    sitl_bin = args.sitl_bin or find_sitl_binary()
    if not sitl_bin or not Path(sitl_bin).exists():
        print(f"ERROR: could not find arducopter SITL binary. Pass --sitl-bin.")
        sys.exit(1)

    work_dir = args.work_dir or tempfile.mkdtemp(prefix="swarm_sitl_")
    os.makedirs(work_dir, exist_ok=True)

    expected_stream_hz = args.sr_pos * 2 + args.sr_ext + args.sr_extra1 + 1  # +1 for heartbeat
    # each peer broadcasts to all (N-1) others; total expected rx rate per peer:
    expected_rx_hz = expected_stream_hz * (args.peers - 1)

    print()
    print_sep()
    print(f"  AP_SwarmMesh SITL Swarm Test")
    print_sep()
    print(f"  Peers      : {args.peers}  (sysids 1..{args.peers})")
    print(f"  Duration   : {args.duration}s  (speedup x{args.speedup})")
    print(f"  TX streams : heartbeat@1Hz  P2PSR_POSITION@{args.sr_pos}Hz  P2PSR_EXT_STAT@{args.sr_ext}Hz  P2PSR_EXTRA1@{args.sr_extra1}Hz")
    print(f"  Expected RX: ~{expected_rx_hz} pkt/s per peer  (all streams, {args.peers-1} senders)")
    print(f"  Broadcast  : dest_id=0 (no forwarding)")
    print(f"  TTL        : {args.ttl}")
    print(f"  SITL binary: {sitl_bin}")
    print(f"  Work dir   : {work_dir}")
    print_sep()

    # Kill any leftover arducopter SITL processes from previous runs (prevents port conflicts)
    subprocess.run(["pkill", "-9", "-f", r"arducopter.*-I\d"], capture_output=True)
    time.sleep(1)

    instances = []

    def cleanup(sig=None, frame=None):
        print("\n  Stopping all SITL instances...")
        for inst in instances:
            inst.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ---- launch all instances ----
    print(f"\n  Launching {args.peers} SITL instances...")
    for i in range(args.peers):
        sysid      = i + 1
        param_file = os.path.join(work_dir, f"peer_{sysid}.parm")
        write_param_file(param_file, sysid, args.sr_pos, args.sr_ext, args.sr_extra1, args.ttl)
        inst = SITLInstance(i, sysid, sitl_bin, param_file, work_dir, args.speedup)
        instances.append(inst)
        print(f"    [{i:2d}] sysid={sysid}  pid={inst.pid}  GCS port={inst.tcp_port}")

    # ---- unblock GCS wait ----
    print(f"\n  Waiting for SITL TCP ports (up to {args.boot_wait}s)...")
    for inst in instances:
        ok = inst.connect_gcs(timeout=args.boot_wait)
        status = "ok" if ok else "TIMEOUT"
        print(f"    [{inst.idx:2d}] sysid={inst.sysid}  port {inst.tcp_port}: {status}")

    print(f"\n  Letting instances finish boot + param load ({args.boot_wait}s)...")
    time.sleep(args.boot_wait)

    # ---- start monitors ----
    pkt_mon  = PacketMonitor()
    pkt_mon.start()

    pids = [inst.pid for inst in instances]
    perf_mon = None if args.no_perf else PerfMonitor(pids)
    if perf_mon:
        perf_mon.start()

    # ---- run the test ----
    print(f"\n  Running test for {args.duration}s...")
    t_start = time.monotonic()
    while True:
        elapsed = time.monotonic() - t_start
        if elapsed >= args.duration:
            break
        counts, _ = pkt_mon.snapshot()
        total = sum(counts.values())
        active_origins = len(counts)
        print(f"    t={elapsed:5.1f}s  total_rx={total:6d}  active_origins={active_origins}/{args.peers - 1}", end="\r")
        time.sleep(2)

    elapsed_actual = time.monotonic() - t_start
    print(f"\n\n  Test complete ({elapsed_actual:.1f}s wall-clock).")

    # ---- collect results ----
    counts, bytes_ = pkt_mon.snapshot()
    pkt_mon.stop()

    perf = {}
    if perf_mon:
        perf_mon.stop()
        perf = perf_mon.summary()

    # ---- stop instances ----
    print("  Stopping SITL instances...")
    for inst in instances:
        inst.stop()

    # ---- print summary ----
    print()
    print_sep()
    print("  PER-PEER PACKET RECEPTION  (packets received from each origin)")
    print_sep()

    header = f"  {'sysid':>5}  {'pid':>7}  {'rx_pkts':>8}  {'rx_kB':>7}  {'pkt_rate':>9}"
    if perf:
        header += f"  {'cpu_avg%':>8}  {'cpu_max%':>8}  {'rss_avg_MB':>10}  {'rss_peak_MB':>11}"
    print(header)
    print_sep()

    total_pkts = 0
    total_bytes = 0
    for inst in sorted(instances, key=lambda x: x.sysid):
        sysid = inst.sysid
        pkts  = counts.get(sysid, 0)
        byt   = bytes_.get(sysid, 0)
        rate  = pkts / elapsed_actual
        total_pkts  += pkts
        total_bytes += byt
        row = f"  {sysid:>5}  {inst.pid:>7}  {pkts:>8}  {byt/1024:>7.1f}  {rate:>8.1f}/s"
        if perf and inst.pid in perf:
            p = perf[inst.pid]
            row += f"  {p['cpu_avg']:>8.1f}  {p['cpu_max']:>8.1f}  {p['rss_avg_kb']/1024:>10.1f}  {p['rss_peak_kb']/1024:>11.1f}"
        print(row)

    print_sep()
    total_rate = total_pkts / elapsed_actual
    print(f"  {'TOTAL':>5}  {'':>7}  {total_pkts:>8}  {total_bytes/1024:>7.1f}  {total_rate:>8.1f}/s")

    if perf:
        all_cpu_avg  = [p['cpu_avg']     for p in perf.values() if p['cpu_avg'] > 0]
        all_rss_peak = [p['rss_peak_kb'] for p in perf.values() if p['rss_peak_kb'] > 0]
        if all_cpu_avg:
            print(f"\n  Fleet CPU:  avg {sum(all_cpu_avg)/len(all_cpu_avg):.1f}%/instance  "
                  f"total {sum(all_cpu_avg):.1f}%  "
                  f"peak-instance {max(all_cpu_avg):.1f}%")
        if all_rss_peak:
            total_rss_mb = sum(all_rss_peak) / 1024
            print(f"  Fleet RAM:  {total_rss_mb:.1f} MB total peak  "
                  f"avg {total_rss_mb/len(all_rss_peak):.1f} MB/instance")

    # ---- missing peer detection ----
    seen_origins = set(counts.keys())
    all_sysids   = set(range(1, args.peers + 1))
    missing = all_sysids - seen_origins
    if missing:
        print(f"\n  WARNING: no packets received from sysid(s): {sorted(missing)}")
    else:
        print(f"\n  All {args.peers} peers heard from. Full mesh confirmed.")

    print_sep()
    print(f"  Logs written to: {work_dir}")
    print_sep()
    print()


if __name__ == "__main__":
    main()
