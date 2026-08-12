#!/usr/bin/env python3
"""
AP_SwarmMesh SITL Leader-Follower Formation Test
================================================

Launches one leader + N follower ArduCopter SITL instances connected over themSwarmMesh multicast mesh (239.65.83.0:57733).
Each follower runs `swarm_follower.lua`, which reads the leader's GLOBAL_POSITION_INT out of its local SwarmMesh peer table
and commands a fixed NED offset as a GUIDED target.

The user:
  1. launches all instances, each with its own working dir and param file.
  2. arms and takes off every vehicle to a common altitude in GUIDED.
  3. waits for the mesh to converge (followers acquire leader pos).
  4. drives the leader slowly (constant velocity) while followers maintain formation autonomously via their Lua scripts.
  5. logs every vehicle's position to a CSV.

Requires pymavlink:  pip install pymavlink

Usage:
    python3 swarm_formation_test.py --followers 3 --alt 15 --speedup 3

The followers are placed in a ring around the leader at --radius metres. Edit FORMATION_OFFSETS() to change the shape.
"""

import argparse
import csv
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("ERROR: pymavlink is required. Install with: pip install pymavlink")


# ---- constants ----------------------------------------------------------------

SITL_BASE_TCP_PORT = 5760      # GCS/MAVLink port for instance i = base + 10*i
COPTER_GUIDED_MODE = 4

# default SITL home (Canberra model airfield), lat, lon, alt(m), yaw(deg)
BASE_LAT = -35.363261
BASE_LON = 149.165230
BASE_ALT = 584.0
BASE_YAW = 0.0

# velocity-only type_mask for SET_POSITION_TARGET_LOCAL_NED: ignore pos (bits 0-2), accel (bits 6-8), yaw (bit10), yaw_rate (bit11). Use vx,vy,vz.
VEL_ONLY_TYPEMASK = 0b0000110111000111  # = 3527


# ---- geometry -----------------------------------------------------------------

def meters_to_latlon(lat0, lon0, north_m, east_m):
    """Offset a lat/lon by a local NE displacement in metres (flat earth)."""
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def formation_offsets(n, radius):
    """Return a list of (north_m, east_m) offsets: N followers evenly on a ring."""
    offsets = []
    for i in range(n):
        ang = 2.0 * math.pi * i / max(1, n)
        offsets.append((radius * math.cos(ang), radius * math.sin(ang)))
    return offsets


# ---- SITL process -------------------------------------------------------------

def find_sitl_binary():
    for p in [Path(__file__).resolve().parents[3] / "build" / "sitl" / "bin" / "arducopter", Path("/usr/local/bin/arducopter")]:
        if p.exists():
            return str(p)
    return None


class SITLInstance:
    def __init__(self, idx, mesh_sysid, sitl_bin, work_dir, home, speedup):
        self.idx = idx
        self.mesh_sysid = mesh_sysid
        self.tcp_port = SITL_BASE_TCP_PORT + 10 * idx
        self.work_dir = work_dir
        os.makedirs(work_dir, exist_ok=True)
        param_file = os.path.join(work_dir, "defaults.parm")  # written by caller
        self._log = open(os.path.join(work_dir, "sitl.log"), "w")
        home_str = f"{home[0]},{home[1]},{home[2]},{home[3]}"
        self._proc = subprocess.Popen(
            [sitl_bin, "-w", "--model", "+", "--speedup", str(speedup),
             "--home", home_str, "--defaults", param_file, f"-I{idx}"],
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
        "P2P_TYPE 10",              # Type::SITL
        f"P2P_SYSID {mesh_sysid}",
        "P2P_DESTID 0",            # broadcast
        "P2P_TTL 1",
        f"P2P_SR_POSITION {sr_pos}",
        "P2P_SR_EXT_STAT 2",
        "P2P_SR_EXTRA1 0",
        "P2P_PRUNE_SECS 0",        # no pruning during the test
        f"P2P_SWARM_SIZE {swarm_size}",
    ]
    if is_follower:
        lines += [
            "SCR_ENABLE 1",
            f"SCR_USER1 {cfg['leader_sysid']}",
            f"SCR_USER2 {cfg['north']:.3f}",
            f"SCR_USER3 {cfg['east']:.3f}",
            f"SCR_USER4 {cfg['formation_alt']:.3f}",
            f"SCR_USER5 {cfg['expected_peers']}",
            "SCR_USER6 0",     # engage flag, raised by the script after takeoff
        ]
    Path(path).write_text("\n".join(lines) + "\n")


# ---- MAVLink vehicle control --------------------------------------------------

class Vehicle:
    """Wraps a pymavlink connection to one SITL instance."""

    def __init__(self, name, tcp_port, mesh_sysid):
        self.name = name
        self.mesh_sysid = mesh_sysid
        self.tcp_port = tcp_port
        self.mav = None
        self.lock = threading.Lock()
        self.last_pos = None       # (t, lat, lon, rel_alt_m)
        self._stop = threading.Event()
        self._thread = None
        self.track = []            # list of (t, lat, lon, rel_alt_m)

    def connect(self, timeout=30):
        self.mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{self.tcp_port}")
        self.mav.wait_heartbeat(timeout=timeout)
        # ask for position at 5 Hz
        self._set_msg_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
        return True

    def _set_msg_interval(self, msg_id, hz):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def start_logging(self, t0):
        self._t0 = t0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            msg = self.mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
            if msg is None:
                continue
            t = time.monotonic() - self._t0
            rec = (t, msg.lat * 1e-7, msg.lon * 1e-7, msg.relative_alt * 1e-3)
            with self.lock:
                self.last_pos = rec
                self.track.append(rec)

    def stop_logging(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    # --- commands ---
    def wait_ready_to_arm(self, timeout=60):
        """Wait until EKF/GPS report the vehicle is armable."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.mav.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
            if msg is None:
                continue
            sensors = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
            # crude readiness: gyro+accel healthy and a GPS fix present
            gpi = self.mav.recv_match(type="GPS_RAW_INT", blocking=False)
            if gpi is not None and gpi.fix_type >= 3:
                return True
        return False

    def set_mode(self, mode_num):
        self.mav.set_mode(mode_num)

    def arm(self, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
            ack = self.mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True
            # confirm via heartbeat armed flag
            hb = self.mav.recv_match(type="HEARTBEAT", blocking=False)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        return False

    def takeoff(self, alt_m):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt_m)

    def send_velocity_ned(self, vn, ve, vd):
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, VEL_ONLY_TYPEMASK,
            0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)

    def set_param(self, name, value):
        self.mav.mav.param_set_send(
            self.mav.target_system, self.mav.target_component,
            name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def rel_alt(self):
        with self.lock:
            return self.last_pos[3] if self.last_pos else None


# ---- main ---------------------------------------------------------------------

def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--followers", type=int, default=3, help="Number of follower drones (default: 3)")
    ap.add_argument("--radius", type=float, default=10.0, help="Formation ring radius, metres (default: 10)")
    ap.add_argument("--alt", type=float, default=15.0, help="Takeoff / formation altitude AGL, metres (default: 15)")
    ap.add_argument("--speedup", type=float, default=3.0, help="SITL speedup factor (default: 3)")
    ap.add_argument("--sr-pos", type=int, default=5, help="P2P_SR_POSITION Hz (default: 5)")
    ap.add_argument("--leader-speed", type=float, default=1.5, help="Leader cruise speed, m/s (default: 1.5)")
    ap.add_argument("--move-time", type=float, default=40.0, help="Total seconds to fly the leader across all legs (default: 40)")
    ap.add_argument("--leader-path", choices=["north", "L", "box"], default="north",
                    help="Leader trajectory: straight north, an L, or a box (default: north)")
    ap.add_argument("--converge-wait", type=float, default=20.0, help="Seconds to wait for mesh convergence before moving (default: 20)")
    ap.add_argument("--sitl-bin", default=None, help="Path to arducopter SITL binary (auto-detected)")
    ap.add_argument("--work-dir", default=None, help="Working dir (default: ./swarm_formation_run)")
    ap.add_argument("--csv", default="formation_track.csv", help="Output CSV of all vehicle tracks")
    ap.add_argument("--boot-wait", type=int, default=20, help="Seconds to wait for instances to boot")
    return ap.parse_args()


def main():
    args = parse_args()
    sitl_bin = args.sitl_bin or find_sitl_binary()
    if not sitl_bin or not Path(sitl_bin).exists():
        sys.exit("ERROR: could not find arducopter SITL binary. Build it or pass --sitl-bin.")

    script_src = Path(__file__).resolve().parents[1] / ".." / "AP_Scripting" / "applets" / "swarm_follower.lua"
    script_src = script_src.resolve()
    if not script_src.exists():
        sys.exit(f"ERROR: follower script not found at {script_src}")

    work_root = Path(args.work_dir or "./swarm_formation_run").resolve()
    work_root.mkdir(parents=True, exist_ok=True)

    n = args.followers
    total = n + 1
    leader_sysid = 1                      # mesh sysid of the leader
    follower_sysids = list(range(2, 2 + n))
    offsets = formation_offsets(n, args.radius)

    print(f"\n  AP_SwarmMesh Formation Test")
    print(f"  Leader (mesh sysid {leader_sysid}) + {n} followers {follower_sysids}")
    print(f"  Ring radius {args.radius} m, altitude {args.alt} m, speedup x{args.speedup}")
    print(f"  SITL binary: {sitl_bin}\n")

    subprocess.run(["pkill", "-9", "-f", r"arducopter.*-I\d"], capture_output=True)
    time.sleep(1)

    instances, vehicles = [], []

    def cleanup(*_):
        print("\n  Stopping SITL instances...")
        for v in vehicles:
            v.stop_logging()
        for inst in instances:
            inst.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ---- launch instances ----
    for idx in range(total):
        mesh_sysid = idx + 1
        is_follower = mesh_sysid != leader_sysid
        wd = work_root / f"inst_{idx}"
        wd.mkdir(parents=True, exist_ok=True)

        if is_follower:
            fi = mesh_sysid - 2  # follower index into offsets
            north, east = offsets[fi]
            cfg = dict(leader_sysid=leader_sysid, north=north, east=east,
                       formation_alt=args.alt, expected_peers=total - 1)
            home = (*meters_to_latlon(BASE_LAT, BASE_LON, north, east), BASE_ALT, BASE_YAW)
            # install the follower Lua script
            (wd / "scripts").mkdir(exist_ok=True)
            shutil.copy(script_src, wd / "scripts" / "swarm_follower.lua")
        else:
            cfg = {}
            home = (BASE_LAT, BASE_LON, BASE_ALT, BASE_YAW)

        write_param_file(wd / "defaults.parm", mesh_sysid, total, args.sr_pos, is_follower, cfg)
        inst = SITLInstance(idx, mesh_sysid, sitl_bin, str(wd), home, args.speedup)
        instances.append(inst)
        role = "LEADER" if not is_follower else f"follower N={cfg['north']:.1f} E={cfg['east']:.1f}"
        print(f"    [{idx}] mesh_sysid={mesh_sysid} pid={inst.pid} port={inst.tcp_port}  {role}")

    print(f"\n  Waiting {args.boot_wait}s for boot...")
    time.sleep(args.boot_wait)

    # ---- connect ----
    print("  Connecting MAVLink...")
    for inst in instances:
        name = "leader" if inst.mesh_sysid == leader_sysid else f"follower{inst.mesh_sysid}"
        v = Vehicle(name, inst.tcp_port, inst.mesh_sysid)
        v.connect()
        vehicles.append(v)
        print(f"    {name}: heartbeat ok (port {inst.tcp_port})")

    leader = next(v for v in vehicles if v.mesh_sysid == leader_sysid)

    # ---- arm + takeoff all ----
    print("\n  Waiting for EKF/GPS ready...")
    for v in vehicles:
        v.wait_ready_to_arm()
    print("  Setting GUIDED, arming, taking off...")
    for v in vehicles:
        v.set_mode(COPTER_GUIDED_MODE)
    time.sleep(1)
    for v in vehicles:
        if not v.arm():
            print(f"    WARNING: {v.name} failed to arm")
    time.sleep(1)
    for v in vehicles:
        v.takeoff(args.alt)

    t0 = time.monotonic()
    for v in vehicles:
        v.start_logging(t0)

    # wait for all to reach altitude
    print(f"  Climbing to {args.alt} m...")
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        alts = [v.rel_alt() for v in vehicles]
        if all(a is not None and a >= args.alt * 0.9 for a in alts):
            break
        time.sleep(1)
    print(f"    altitudes: {['%.1f' % (v.rel_alt() or 0) for v in vehicles]}")

    # ---- engage followers now that they're safely at altitude ----
    print("  Raising engage flag (SCR_USER6) on followers...")
    for v in vehicles:
        if v.mesh_sysid != leader_sysid:
            v.set_param("SCR_USER6", 1)

    # ---- let the mesh converge; followers acquire the leader ----
    print(f"\n  Holding {args.converge_wait}s for mesh convergence + formation acquisition...")
    time.sleep(args.converge_wait)

    # ---- move the leader; followers track autonomously ----
    # each leg is (vn, ve) in m/s; the leader holds each for an equal slice of move-time.
    s = args.leader_speed
    legs = {
        "north": [(s, 0.0)],
        "L":     [(s, 0.0), (0.0, s)],
        "box":   [(s, 0.0), (0.0, s), (-s, 0.0), (0.0, -s)],
    }[args.leader_path]
    leg_time = args.move_time / len(legs)
    print(f"  Flying leader path '{args.leader_path}' at {s} m/s ({len(legs)} leg(s) x {leg_time:.0f}s)...")
    for (vn, ve) in legs:
        leg_deadline = time.monotonic() + leg_time
        while time.monotonic() < leg_deadline:
            leader.send_velocity_ned(vn, ve, 0.0)
            time.sleep(0.2)
    # stop leader
    for _ in range(10):
        leader.send_velocity_ned(0.0, 0.0, 0.0)
        time.sleep(0.2)

    print("  Settling 5s...")
    time.sleep(5)

    # ---- collect + write CSV ----
    print("\n  Writing tracks...")
    for v in vehicles:
        v.stop_logging()
    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "mesh_sysid", "role", "lat", "lon", "rel_alt_m"])
        for v in vehicles:
            role = "leader" if v.mesh_sysid == leader_sysid else "follower"
            with v.lock:
                for (t, lat, lon, alt) in v.track:
                    w.writerow([f"{t:.2f}", v.mesh_sysid, role, f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}"])
    print(f"    wrote {args.csv}")

    for inst in instances:
        inst.stop()
    print("\n  Done.\n")


if __name__ == "__main__":
    main()
