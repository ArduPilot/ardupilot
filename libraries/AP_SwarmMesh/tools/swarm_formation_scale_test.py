#!/usr/bin/env python3
"""
AP_SwarmMesh SITL Leader-Follower Formation — SCALE variant
===========================================================

Same experiment as swarm_formation_test.py, but reworked to launch up to the
full sysid range (1 leader + 253 followers = 254 SITL ArduCopter processes) on
a single host. Scaling to hundreds of *flying* instances breaks every part of
the small-fleet orchestrator that assumed a handful of vehicles, so this file
changes the machinery, not just the geometry:

  * Geometry     : phyllotaxis (sunflower) packing instead of a single ring, so
                   253 followers spread out at a sane spacing (a 10 m ring would
                   put them 0.25 m apart).
  * Launch       : staggered in batches to avoid a boot-time CPU thundering herd.
  * Bring-up     : connect + wait-for-EKF + arm + takeoff run CONCURRENTLY across
                   a thread pool (done serially, 254 vehicles take tens of mins).
  * Logging      : ONE selectors-based reader drains all sockets, instead of one
                   Python thread per vehicle (254 threads starve on the GIL).
  * Defaults     : speedup 1 (254 sims oversubscribe any CPU), lower stream rate
                   (each node RXes ~ (N-1) x streams from the mesh).

Expect a heavy, slow run: with 254 processes, boot + EKF settle alone is minutes,
and wall-clock runs slower than real time. Validate at a smaller --followers
first (e.g. 24) before committing to the full 253.

Requires pymavlink:  pip install pymavlink
"""

import argparse
import csv
import math
import os
import selectors
import shutil
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("ERROR: pymavlink is required. Install with: pip install pymavlink")


# ---- constants ----------------------------------------------------------------

DEFAULT_SITL_BASE_TCP_PORT = 5760      # GCS/MAVLink port for instance i = base + 10*i
COPTER_GUIDED_MODE = 4
GOLDEN_ANGLE = math.pi * (3.0 - math.sqrt(5.0))   # ~137.5 deg

BASE_LAT, BASE_LON, BASE_ALT, BASE_YAW = -35.363261, 149.165230, 584.0, 0.0

# velocity-only type_mask for SET_POSITION_TARGET_LOCAL_NED
VEL_ONLY_TYPEMASK = 0b0000110111000111


# ---- geometry -----------------------------------------------------------------

def meters_to_latlon(lat0, lon0, north_m, east_m):
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def phyllotaxis_offsets(n, spacing):
    """N (north, east) offsets on a sunflower spiral; ~`spacing` m between neighbours."""
    offs = []
    for i in range(1, n + 1):        # start at 1 so the leader (center) isn't shadowed
        r = spacing * math.sqrt(i)
        th = i * GOLDEN_ANGLE
        offs.append((r * math.cos(th), r * math.sin(th)))   # (north, east)
    return offs


# ---- SITL process -------------------------------------------------------------

def find_sitl_binary():
    for p in [Path(__file__).resolve().parents[3] / "build" / "sitl" / "bin" / "arducopter",
              Path("/usr/local/bin/arducopter")]:
        if p.exists():
            return str(p)
    return None


class SITLInstance:
    def __init__(self, idx, mesh_sysid, sitl_bin, work_dir, home, speedup, base_port):
        self.idx = idx
        self.mesh_sysid = mesh_sysid
        self.tcp_port = base_port + 10 * idx
        self.work_dir = work_dir
        self._log = open(os.path.join(work_dir, "sitl.log"), "w")
        home_str = f"{home[0]},{home[1]},{home[2]},{home[3]}"
        self._proc = subprocess.Popen(
            [sitl_bin, "-w", "--model", "+", "--speedup", str(speedup),
             "--home", home_str, "--defaults", os.path.join(work_dir, "defaults.parm"),
             "--base-port", str(self.tcp_port),
             f"-I{idx}"],
            stdout=self._log, stderr=self._log, cwd=work_dir,
        )
        self.pid = self._proc.pid

    def stop(self):
        try:
            self._proc.terminate()
            self._proc.wait(timeout=5)
        except Exception:
            self._proc.kill()
        self._log.close()


def write_param_file(path, mesh_sysid, swarm_size, sr_pos, is_follower, cfg):
    lines = [
        "P2P_TYPE 10", f"P2P_SYSID {mesh_sysid}", "P2P_DESTID 0", "P2P_TTL 1",
        # position stream only: the formation needs the leader's position/velocity;
        # EXT_STAT/EXTRA1 just multiply the (N-1)x per-node RX load at scale.
        f"P2P_SR_POSITION {sr_pos}", "P2P_SR_EXT_STAT 0", "P2P_SR_EXTRA1 0",
        "P2P_PRUNE_SECS 0", f"P2P_SWARM_SIZE {swarm_size}",
        # bring-up of hundreds of sims takes 15+ min of hover; don't let the battery
        # failsafe pull vehicles out of GUIDED mid-experiment.
        "BATT_LOW_VOLT 0", "BATT_CRT_VOLT 0", "BATT_FS_LOW_ACT 0", "BATT_FS_CRT_ACT 0",
    ]
    if is_follower:
        lines += [
            "SCR_ENABLE 1",
            f"SCR_USER1 {cfg['leader_sysid']}",
            f"SCR_USER2 {cfg['north']:.3f}",
            f"SCR_USER3 {cfg['east']:.3f}",
            f"SCR_USER4 {cfg['formation_alt']:.3f}",
            f"SCR_USER5 {cfg['expected_peers']}",
            "SCR_USER6 0",
        ]
    Path(path).write_text("\n".join(lines) + "\n")


# ---- MAVLink vehicle control --------------------------------------------------

class Vehicle:
    def __init__(self, name, tcp_port, mesh_sysid):
        self.name = name
        self.mesh_sysid = mesh_sysid
        self.tcp_port = tcp_port
        self.mav = None
        self.lock = threading.Lock()     # guards track/last_pos
        self.iolock = threading.Lock()   # serializes send/recv on the shared mav object
        self.last_pos = None       # (t, lat, lon, rel_alt_m)
        self.track = []
        self.dbg = ""              # last SFDBG statustext from the follower
        self.armed_ok = False
        self.fail_reason = ""

    # --- bring-up (each runs in its own worker thread; touches only its own socket) ---
    def connect(self, timeout=90):
        """(Re)open the link and (re)request the streams we depend on. Safe to call
        again after SITL's -w wipe-reboot drops the initial connection."""
        if self.mav is not None:
            try:
                self.mav.close()
            except Exception:
                pass
        self.mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.tcp_port}")
        self.mav.wait_heartbeat(timeout=timeout)
        for mid in (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
            self._set_msg_interval(mid, 2)
        return True

    def _set_msg_interval(self, msg_id, hz):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def wait_ready_to_arm(self, timeout=180):
        """Wait for a 3D GPS fix, reconnecting if the link goes silent (the -w
        wipe-reboot drops the first connection a few seconds into boot)."""
        deadline = time.monotonic() + timeout
        last_rx = time.monotonic()
        while time.monotonic() < deadline:
            msg = self.mav.recv_match(type=["GPS_RAW_INT", "SYS_STATUS"], blocking=True, timeout=2)
            if msg is None:
                if time.monotonic() - last_rx > 8:   # link likely dropped by the reboot
                    try:
                        self.connect()
                    except Exception:
                        pass
                    last_rx = time.monotonic()
                continue
            last_rx = time.monotonic()
            if msg.get_type() == "GPS_RAW_INT" and msg.fix_type >= 3:
                return True
        return False

    def arm(self, timeout=60):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            ack = self.mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                    and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return True
            hb = self.mav.recv_match(type="HEARTBEAT", blocking=False)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        return False

    def set_mode(self, mode_num):
        self.mav.set_mode(mode_num)

    def takeoff(self, alt_m):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m)

    def bringup(self, alt_m, ready_timeout=180):
        """connect -> ready -> GUIDED -> arm -> takeoff, with one full retry on any
        failure (a fresh connection recovers from SITL dropping the TCP link under
        load). Sets fail_reason for the post-run report."""
        for _round in range(2):
            try:
                self.connect()
                if not self.wait_ready_to_arm(timeout=ready_timeout):
                    self.fail_reason = "gps-ready timeout"
                    continue
                self.set_mode(COPTER_GUIDED_MODE)
                time.sleep(0.5)
                # arm + takeoff, verifying the climb actually starts: a takeoff command
                # can be lost right after arming, and the copter then auto-disarms on
                # the ground and never rejoins (a permanently idle follower at scale).
                for _attempt in range(2):
                    if not self.arm():
                        self.fail_reason = "arm timeout"
                        continue
                    self.takeoff(alt_m)
                    deadline = time.monotonic() + 30
                    while time.monotonic() < deadline:
                        msg = self.mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
                        if msg is not None and msg.relative_alt * 1e-3 > 1.0:
                            self.armed_ok = True
                            self.fail_reason = ""
                            return True
                    self.fail_reason = "no climb after takeoff"
            except Exception as e:
                self.fail_reason = f"{type(e).__name__}: {e}"
        return False

    # --- runtime commands (main thread) ---
    def send_velocity_ned(self, vn, ve, vd):
        with self.iolock:
            self.mav.mav.set_position_target_local_ned_send(
                0, self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_ONLY_TYPEMASK,
                0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)

    def set_param(self, name, value):
        with self.iolock:
            self.mav.mav.param_set_send(
                self.mav.target_system, self.mav.target_component,
                name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def rel_alt(self):
        with self.lock:
            return self.last_pos[3] if self.last_pos else None


# ---- selectors-based logger: ONE thread drains every vehicle socket ------------

class FleetLogger:
    """Vehicles are registered incrementally, the moment they finish bring-up: a
    socket left unread while later waves boot fills its TCP buffer and SITL drops
    the connection (this cost an entire run's data at 254 vehicles). Any socket
    that errors afterwards is unregistered rather than killing the thread."""

    def __init__(self, t0):
        self.t0 = t0
        self.sel = selectors.DefaultSelector()
        self._pending = []
        self._plock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def add(self, v):
        """Thread-safe: called from bring-up workers as each vehicle arms."""
        with self._plock:
            self._pending.append(v)

    def start(self):
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            with self._plock:
                pend, self._pending = self._pending, []
            for v in pend:
                try:
                    self.sel.register(v.mav.port, selectors.EVENT_READ, v)
                except Exception:
                    pass
            if not self.sel.get_map():
                time.sleep(0.2)
                continue
            try:
                events = self.sel.select(timeout=0.5)
            except Exception:
                continue
            for key, _ in events:
                v = key.data
                try:
                    # drain everything buffered on this socket
                    while True:
                        with v.iolock:
                            m = v.mav.recv_match(type=["GLOBAL_POSITION_INT", "STATUSTEXT"], blocking=False)
                        if m is None:
                            break
                        mt = m.get_type()
                        if mt == "STATUSTEXT":
                            txt = m.text.decode(errors="replace") if isinstance(m.text, bytes) else m.text
                            if txt.startswith("SFDBG"):
                                v.dbg = txt
                            continue
                        t = time.monotonic() - self.t0
                        rec = (t, m.lat * 1e-7, m.lon * 1e-7, m.relative_alt * 1e-3)
                        with v.lock:
                            v.last_pos = rec
                            v.track.append(rec)
                except Exception:
                    # any failure on one vehicle drops just that vehicle, never the
                    # whole fleet's logging thread (a dead socket lost a full run before)
                    try:
                        self.sel.unregister(key.fileobj)
                    except Exception:
                        pass

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=4)


# ---- main ---------------------------------------------------------------------

def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--followers", type=int, default=253, help="Number of followers (default: 253 -> 254 total)")
    ap.add_argument("--spacing", type=float, default=4.0, help="Nearest-neighbour spacing, m (default: 4)")
    ap.add_argument("--alt", type=float, default=15.0, help="Formation altitude AGL, m (default: 15)")
    ap.add_argument("--speedup", type=float, default=1.0, help="SITL speedup (default: 1; >1 is unrealistic at scale)")
    ap.add_argument("--sr-pos", type=int, default=0, help="follower P2P_SR_POSITION Hz (default: 0). In a leader->follower experiment followers only CONSUME the leader; broadcasting their own position just floods the mesh and, at scale, delays the leader's updates enough that followers steer to a stale position. 0 keeps the leader fresh.")
    ap.add_argument("--leader-sr", type=int, default=5, help="leader P2P_SR_POSITION Hz -- higher so its slot stays fresh through mesh loss (default: 5)")
    ap.add_argument("--leader-speed", type=float, default=2.0, help="Leader speed, m/s (default: 2)")
    ap.add_argument("--move-time", type=float, default=60.0, help="Total leader flight time, s (default: 60)")
    ap.add_argument("--leader-path", choices=["north", "L", "box"], default="box")
    ap.add_argument("--converge-wait", type=float, default=45.0, help="Mesh-convergence hold, s (default: 45)")
    ap.add_argument("--batch-size", type=int, default=16, help="Instances launched per batch (default: 16)")
    ap.add_argument("--batch-delay", type=float, default=4.0, help="Seconds between launch batches (default: 4)")
    ap.add_argument("--workers", type=int, default=24, help="Concurrent bring-up workers (default: 24)")
    ap.add_argument("--arm-timeout", type=float, default=240, help="Per-vehicle wait-for-GPS timeout, s. Raise at scale: sims run slower than real time (default: 240)")
    ap.add_argument("--climb-timeout", type=float, default=180, help="Wait for the fleet to reach altitude, s (default: 180)")
    ap.add_argument("--ready-peers", type=int, default=0, help="Peers a follower must hear before engaging (0 = leader-fresh only; the full-swarm gate is fragile at scale)")
    ap.add_argument("--base-port", type=int, default=DEFAULT_SITL_BASE_TCP_PORT, help=f"Base MAVLink TCP port for instance 0 (default: {DEFAULT_SITL_BASE_TCP_PORT}; instance i uses base+10*i)")
    ap.add_argument("--sitl-bin", default=None)
    ap.add_argument("--work-dir", default=None, help="Working dir (keep OUTSIDE any cloud-synced folder)")
    ap.add_argument("--csv", default="formation_scale_track.csv")
    return ap.parse_args()


def main():
    args = parse_args()
    sitl_bin = args.sitl_bin or find_sitl_binary()
    if not sitl_bin or not Path(sitl_bin).exists():
        sys.exit("ERROR: could not find arducopter SITL binary. Build it or pass --sitl-bin.")

    script_src = (Path(__file__).resolve().parents[1] / ".." / "AP_Scripting" / "applets" / "swarm_follower.lua").resolve()
    if not script_src.exists():
        sys.exit(f"ERROR: follower script not found at {script_src}")

    work_root = Path(args.work_dir or "./swarm_scale_run").resolve()
    work_root.mkdir(parents=True, exist_ok=True)

    n = args.followers
    total = n + 1
    leader_sysid = 1
    offsets = phyllotaxis_offsets(n, args.spacing)
    outer_r = max(math.hypot(no, ea) for no, ea in offsets) if offsets else 0.0

    print(f"\n  AP_SwarmMesh Formation SCALE test")
    print(f"  Leader + {n} followers ({total} SITL processes)")
    print(f"  Phyllotaxis spacing {args.spacing} m, outer radius {outer_r:.0f} m, alt {args.alt} m, speedup x{args.speedup}")
    print(f"  Launch: batches of {args.batch_size} every {args.batch_delay}s; bring-up workers {args.workers}")
    print(f"  MAVLink TCP ports: {args.base_port}..{args.base_port + 10 * (total - 1)}")
    print(f"  Work dir: {work_root}\n")

    subprocess.run(["pkill", "-9", "-f", r"arducopter.*-I\d"], capture_output=True)
    time.sleep(1)

    instances, vehicles = [], []

    def cleanup(*_):
        print("\n  Stopping SITL instances...")
        for inst in instances:
            inst.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ---- staggered launch ----
    print("  Launching (staggered)...")
    for idx in range(total):
        mesh_sysid = idx + 1
        is_follower = mesh_sysid != leader_sysid
        wd = work_root / f"inst_{idx}"
        wd.mkdir(parents=True, exist_ok=True)
        if is_follower:
            north, east = offsets[mesh_sysid - 2]
            # readiness gate: at scale, requiring the FULL peer table (total-1) is
            # fragile -- a node that misses even one peer never engages. A follower
            # only needs a fresh LEADER fix to hold formation, so gate on that alone
            # (SCR_USER5=0 disables the peer-count requirement in swarm_follower.lua).
            cfg = dict(leader_sysid=leader_sysid, north=north, east=east,
                       formation_alt=args.alt, expected_peers=args.ready_peers)
            home = (*meters_to_latlon(BASE_LAT, BASE_LON, north, east), BASE_ALT, BASE_YAW)
            (wd / "scripts").mkdir(exist_ok=True)
            shutil.copy(script_src, wd / "scripts" / "swarm_follower.lua")
        else:
            cfg = {}
            home = (BASE_LAT, BASE_LON, BASE_ALT, BASE_YAW)
        sr = args.sr_pos if is_follower else args.leader_sr
        write_param_file(wd / "defaults.parm", mesh_sysid, total, sr, is_follower, cfg)
        instances.append(SITLInstance(idx, mesh_sysid, sitl_bin, str(wd), home, args.speedup, args.base_port))
        if (idx + 1) % args.batch_size == 0 and (idx + 1) < total:
            print(f"    launched {idx + 1}/{total}...")
            time.sleep(args.batch_delay)
    print(f"    launched {total}/{total}.")

    # ---- concurrent bring-up (connect + ready + arm + takeoff) ----
    for inst in instances:
        name = "leader" if inst.mesh_sysid == leader_sysid else f"f{inst.mesh_sysid}"
        vehicles.append(Vehicle(name, inst.tcp_port, inst.mesh_sysid))
    leader = next(v for v in vehicles if v.mesh_sysid == leader_sysid)

    print(f"\n  Bringing up {total} vehicles (connect/arm/takeoff, {args.workers} workers)...")
    done = {"ok": 0, "fail": 0}
    dlock = threading.Lock()
    t_bu = time.monotonic()

    # logger starts NOW and vehicles join it as they arm, so no armed vehicle's
    # socket ever sits unread while the rest of the fleet boots.
    t0 = time.monotonic()
    logger = FleetLogger(t0)
    logger.start()

    def _bring(v):
        ok = v.bringup(args.alt, ready_timeout=args.arm_timeout)
        if ok:
            logger.add(v)
        with dlock:
            done["ok" if ok else "fail"] += 1
            tot = done["ok"] + done["fail"]
            if tot % args.batch_size == 0 or tot == total:
                print(f"    bring-up {tot}/{total}  (ok={done['ok']} fail={done['fail']})  {time.monotonic()-t_bu:.0f}s")
        return ok

    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        list(ex.map(_bring, vehicles))
    armed = [v for v in vehicles if v.armed_ok]
    print(f"  Armed/airborne: {len(armed)}/{total}  in {time.monotonic()-t_bu:.0f}s")
    failed = [v for v in vehicles if not v.armed_ok]
    if failed:
        from collections import Counter
        reasons = Counter(v.fail_reason or "unknown" for v in failed)
        print(f"  Bring-up failures by reason: {dict(reasons)}")
        print(f"  Failed sysids: {sorted(v.mesh_sysid for v in failed)[:20]}{'...' if len(failed) > 20 else ''}")
    if not leader.armed_ok:
        print("  WARNING: leader failed bring-up; aborting.")
        cleanup()

    # ---- wait for the fleet to reach altitude ----
    print(f"  Climbing to {args.alt} m...")
    deadline = time.monotonic() + args.climb_timeout
    while time.monotonic() < deadline:
        at_alt = sum(1 for v in armed if (v.rel_alt() or 0) >= args.alt * 0.9)
        print(f"    at altitude: {at_alt}/{len(armed)}   ", end="\r")
        if at_alt >= len(armed) * 0.95:
            break
        time.sleep(2)
    print()

    # Followers self-engage once they've climbed (swarm_follower.lua gates on its own
    # AGL), so there's no runtime "go" flag to deliver to hundreds of vehicles.
    print("  Followers self-engage on altitude (no runtime flag needed).")

    print(f"\n  Holding {args.converge_wait}s for mesh convergence at {total}-node scale...")
    time.sleep(args.converge_wait)

    # ---- move the leader ----
    s = args.leader_speed
    legs = {"north": [(s, 0.0)],
            "L": [(s, 0.0), (0.0, s)],
            "box": [(s, 0.0), (0.0, s), (-s, 0.0), (0.0, -s)]}[args.leader_path]
    leg_time = args.move_time / len(legs)
    print(f"  Flying leader '{args.leader_path}' ({len(legs)} leg(s) x {leg_time:.0f}s)...")
    sample = [v for v in armed if v.mesh_sysid in (2, 3, 4)]
    last_print = 0.0
    for (vn, ve) in legs:
        leg_deadline = time.monotonic() + leg_time
        while time.monotonic() < leg_deadline:
            leader.send_velocity_ned(vn, ve, 0.0)
            if time.monotonic() - last_print > 5:
                last_print = time.monotonic()
                print(f"    [move] leader_alt={leader.rel_alt():.0f}  " +
                      "  ".join(f"f{v.mesh_sysid}:{v.dbg.replace('SFDBG ','')}" for v in sample))
            time.sleep(0.2)
    for _ in range(10):
        leader.send_velocity_ned(0.0, 0.0, 0.0)
        time.sleep(0.2)

    print("  Settling 5s...")
    time.sleep(5)

    # ---- debug: sample follower states (why some may not be tracking) ----
    fol = [v for v in armed if v.mesh_sysid != leader_sysid]
    print("  Follower debug (last SFDBG):")
    for v in fol[:6] + fol[-6:]:
        print(f"    sysid {v.mesh_sysid:>3}: {v.dbg or '(no debug seen)'}")

    # ---- write CSV ----
    print("\n  Writing tracks...")
    logger.stop()
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "mesh_sysid", "role", "lat", "lon", "rel_alt_m"])
        for v in vehicles:
            role = "leader" if v.mesh_sysid == leader_sysid else "follower"
            with v.lock:
                for (t, lat, lon, alt) in v.track:
                    w.writerow([f"{t:.2f}", v.mesh_sysid, role, f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}"])
    print(f"    wrote {args.csv}  ({sum(len(v.track) for v in vehicles)} samples)")

    for inst in instances:
        inst.stop()
    print("\n  Done.\n")


if __name__ == "__main__":
    main()
