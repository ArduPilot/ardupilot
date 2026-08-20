#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Decentralized glyph formation with onboard CBF avoidance over AP_SwarmMesh.

The leader publishes only which word is active and where it is anchored, in its
coordination basket. Every other vehicle runs swarm_speller.lua, derives its
cell from a persistent formation number, and filters its own velocity against
peer state received at 5 Hz from its bounded transit cohort.
Nothing in this harness assigns a vehicle or sends a formation flight setpoint.

The font and word list are parsed out of the Lua script, so the fleet size and the
target glyph this script plots can never drift from what the vehicles fly.

Usage:
    python3 swarm_spell_test.py --word GSoC --work-dir ~/spell_run --csv ~/spell_run/track.csv
"""

import argparse
import bisect
import csv
import math
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))

from swarm_formation_scale_test import (      # noqa: E402  (path set above)
    BASE_ALT,
    BASE_LAT,
    BASE_LON,
    BASE_YAW,
    FleetLogger,
    SITLInstance,
    Vehicle,
    find_sitl_binary,
    meters_to_latlon,
    phyllotaxis_offsets,
)

GLYPH_W, GLYPH_H, GLYPH_GAP = 5, 7, 1
COHORT_SIZE = 2
COHORTS_PER_WAVE = 4
APPLET = (Path(__file__).resolve().parents[2] / "AP_Scripting" / "applets" / "swarm_speller.lua")


# ---- font, shared with the applet ---------------------------------------------

def load_font(applet_path):
    """Parse the 5x7 font and the word list out of swarm_speller.lua.

    The script is the benchmark. The vehicles fly whats in there,
    so this script reads it rather than keeping a second copy that could drift.
    """
    text = applet_path.read_text()
    font = {}
    for m in re.finditer(r'^\s{2}(\S) = \{((?:"[.#]{%d}",?\s*)+)\},' % GLYPH_W, text, re.M):
        rows = re.findall(r'"([.#]{%d})"' % GLYPH_W, m.group(2))
        if len(rows) == GLYPH_H:
            font[m.group(1)] = rows
    words = re.search(r'local WORDS = \{(.*?)\}', text)
    word_list = re.findall(r'"([^"]+)"', words.group(1)) if words else []
    if not font or not word_list:
        sys.exit(f"ERROR: could not parse font/words out of {applet_path}")
    return font, word_list


def build_slots(word, font):
    """(north, east) cell offsets for a word, in cells. Must match build_slots()
    in the Lua script exactly: character, then row, then column."""
    out = []
    nchars = len(word)
    total_w = nchars * GLYPH_W + (nchars - 1) * GLYPH_GAP
    x0 = -(total_w - 1) * 0.5
    y0 = (GLYPH_H - 1) * 0.5
    for ci, ch in enumerate(word):
        bmp = font.get(ch)
        if bmp is None:
            sys.exit(f"ERROR: no glyph for '{ch}' in the Lua script font")
        cx = ci * (GLYPH_W + GLYPH_GAP)
        for row in range(GLYPH_H):
            for col in range(GLYPH_W):
                if bmp[row][col] == "#":
                    out.append((y0 - row, x0 + cx + col))
    return out


def render_word(word, font):
    """ASCII preview of what the swarm will fly, for the console banner."""
    lines = []
    for row in range(GLYPH_H):
        line = ""
        for ch in word:
            line += font[ch][row].replace(".", " ").replace("#", "O") + " "
        lines.append("    " + line)
    return "\n".join(lines)


# ---- SITL parameters ----------------------------------------------------------

def write_param_file(path, mesh_sysid, swarm_size, is_leader, cfg):
    lines = [
        "P2P_TYPE 10", f"MAV_SYSID {mesh_sysid}", "P2P_DESTID 0", "P2P_TTL 1",
        # Lua immediately silences this stream, then enables it only while this
        # vehicle's bounded cohort is transiting. The value remains the active rate.
        f"P2P_SR_POSITION {cfg['sr_pos']}", "P2P_SR_EXT_STAT 0", "P2P_SR_EXTRA1 0",
        f"P2P_SR_COORD {cfg['sr_coord']}",
        "P2P_PRUNE_SECS 0", f"P2P_SWARM_SIZE {swarm_size}",
        # bring up of a large fleet takes a long hover; don't let a battery failsafe
        # pull vehicles out of GUIDED mid flight
        "BATT_LOW_VOLT 0", "BATT_CRT_VOLT 0", "BATT_FS_LOW_ACT 0", "BATT_FS_CRT_ACT 0",
        "SCR_ENABLE 1",
        "SCR_HEAP_SIZE 200000", "SCR_VM_I_COUNT 100000",
        "WPNAV_SPEED 200", "WPNAV_ACCEL 200",
        f"SCR_USER1 {cfg['leader_sysid']}",
        f"SCR_USER2 {cfg['alt']:.3f}",
        f"SCR_USER3 {cfg['spacing']:.3f}",
        # the leader starts idle: the swarm is given its task only once it is airborne
        "SCR_USER4 0",
        f"SCR_USER5 {cfg['formation_number'] if not is_leader else 0}",
        f"SCR_USER6 {cfg['safe_distance']:.3f}",
    ]
    Path(path).write_text("\n".join(lines) + "\n")


# ---- main ---------------------------------------------------------------------

def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--word", default="GSoC", help="word to spell (must be in the Luas WORDS list)")
    ap.add_argument("--then", default=None, help="second word to morph into, to show reassignment")
    ap.add_argument("--spacing", type=float, default=4.0, help="glyph cell spacing, m (default: 4)")
    ap.add_argument("--alt", type=float, default=20.0, help="formation altitude AGL, m (default: 20)")
    # a compact cloud, staged clear of the word by --standoff, so the swarm visibly flies
    # into the letters rather than materialising on top of them
    ap.add_argument("--scatter", type=float, default=4.0, help="takeoff scatter spacing, m (default: 4)")
    # how far south of the word the fleet takes off. -1 sizes it from the glyph and the
    # cloud so the two never overlap; 0 puts the fleet back inside the word.
    ap.add_argument("--standoff", type=float, default=-1.0,
                    help="stage the fleet this far south of the word, m (default: auto)")
    # claims have to propagate before a contest can resolve, so a low rate makes the
    # swarm converge slowly and leaves stale claims sitting in every peer's map
    ap.add_argument("--sr-coord", type=int, default=2,
                    help="leader and active wave task/intent rate, Hz (default: 2)")
    # Peer positions let a vehicle tell a claim that was flown to from one that was only
    # asserted, and the Lua scrpt switches those rules on whenever position data is available.
    ap.add_argument("--sr-pos", type=int, default=5,
                    help="peer state rate used by the onboard CBF, Hz (default: 5)")
    ap.add_argument("--speedup", type=float, default=1.0, help="SITL speedup (default: 1)")
    ap.add_argument("--form-timeout", type=float, default=1000.0,
                    help="wait for the scheduled cohort waves to form the word, s (default: 1000)")
    ap.add_argument("--hold", type=float, default=30.0, help="hold the finished word, s (default: 30)")
    ap.add_argument("--tol", type=float, default=2.5, help="a speller counts as on its cell within this, m")
    ap.add_argument("--safe-distance", type=float, default=3.0,
                    help="minimum commanded 3D centre separation, m (default: 3)")
    ap.add_argument("--overlap-distance", type=float, default=1.0,
                    help="sampled centre distance counted as physical overlap, m (default: 1)")
    ap.add_argument("--log-rate", type=int, default=10,
                    help="GCS track sampling rate used for separation checks, Hz (default: 10)")
    ap.add_argument("--followers", type=int, default=None,
                    help="override fleet size (default: one per cell plus --spare)")
    ap.add_argument("--spare", type=int, default=0,
                    help="extra idle vehicles beyond one per cell (default: 0)")
    ap.add_argument("--batch-size", type=int, default=16)
    ap.add_argument("--batch-delay", type=float, default=4.0)
    ap.add_argument("--workers", type=int, default=24)
    ap.add_argument("--arm-timeout", type=float, default=240)
    ap.add_argument("--base-port", type=int, default=5760)
    ap.add_argument("--sitl-bin", default=None)
    ap.add_argument("--work-dir", default="./swarm_spell_run")
    ap.add_argument("--csv", default="spell_track.csv")
    ap.add_argument("--glyph-csv", default=None, help="write the target cells here (default: alongside --csv)")
    return ap.parse_args()


def latlon_targets(cells, spacing):
    return [meters_to_latlon(BASE_LAT, BASE_LON, nc * spacing, ec * spacing) for nc, ec in cells]


def placement(vehicles, leader_sysid, targets, target_alt_m, tol_m):
    """Measure the formation ourselves from logged positions rather than trusting the
    swarms own account of it. Returns (spellers placed, cells occupied, error list)."""
    errs, occupied = [], set()
    for v in vehicles:
        if v.mesh_sysid == leader_sysid or not v.armed_ok:
            continue
        with v.lock:
            pos = v.last_pos
        if pos is None:
            continue
        best_i, best_d = None, None
        for i, (tlat, tlon) in enumerate(targets):
            north = (pos[1] - tlat) * 111320.0
            east = (pos[2] - tlon) * 111320.0 * math.cos(math.radians(tlat))
            vertical = pos[3] - target_alt_m
            d = math.sqrt(north * north + east * east + vertical * vertical)
            if best_d is None or d < best_d:
                best_i, best_d = i, d
        errs.append(best_d)
        if best_d <= tol_m:
            occupied.add(best_i)
    return sum(1 for e in errs if e <= tol_m), len(occupied), errs


def interpolate_track(track, times, sample_time, max_gap_s=0.6):
    """Interpolate one vehicle's track at sample_time, rejecting logging gaps."""
    if len(track) < 2:
        return None
    right = bisect.bisect_left(times, sample_time)
    if right == 0 or right >= len(track):
        return None
    before, after = track[right - 1], track[right]
    gap = after[0] - before[0]
    if gap <= 0 or gap > max_gap_s:
        return None
    fraction = (sample_time - before[0]) / gap
    return (
        before[1] + (after[1] - before[1]) * fraction,
        before[2] + (after[2] - before[2]) * fraction,
        before[3] + (after[3] - before[3]) * fraction,
    )


def separation_metrics(vehicles, start_s, end_s, sample_rate_hz, safe_distance, overlap_distance):
    """Measure pairwise 3D separation from independently logged tracks."""
    tracks = {}
    for vehicle in vehicles:
        if not vehicle.armed_ok:
            continue
        with vehicle.lock:
            track = list(vehicle.track)
            tracks[vehicle.mesh_sysid] = (track, [point[0] for point in track])

    minimum = math.inf
    safe_violations = 0
    overlaps = 0
    evaluated_pairs = 0
    step = 1.0 / sample_rate_hz
    sample_time = start_s
    while sample_time <= end_s:
        positions = []
        for sysid, (track, times) in tracks.items():
            position = interpolate_track(track, times, sample_time)
            if position is not None:
                positions.append((sysid, position))
        for first in range(len(positions)):
            for second in range(first + 1, len(positions)):
                first_pos = positions[first][1]
                second_pos = positions[second][1]
                mean_lat = math.radians((first_pos[0] + second_pos[0]) * 0.5)
                north = (first_pos[0] - second_pos[0]) * 111320.0
                east = (first_pos[1] - second_pos[1]) * 111320.0 * math.cos(mean_lat)
                vertical = first_pos[2] - second_pos[2]
                distance = math.sqrt(north * north + east * east + vertical * vertical)
                minimum = min(minimum, distance)
                safe_violations += distance < safe_distance
                overlaps += distance < overlap_distance
                evaluated_pairs += 1
        sample_time += step

    return minimum, safe_violations, overlaps, evaluated_pairs


def main():
    args = parse_args()
    if args.safe_distance >= args.spacing:
        sys.exit("ERROR: --safe-distance must be smaller than --spacing")
    if args.scatter <= args.overlap_distance:
        sys.exit("ERROR: --scatter must exceed --overlap-distance")
    if args.sr_pos != 5:
        print("WARNING: the Lua uncertainty bound assumes 5 Hz peer position updates")
    sitl_bin = args.sitl_bin or find_sitl_binary()
    if not sitl_bin or not Path(sitl_bin).exists():
        sys.exit("ERROR: could not find arducopter SITL binary. Build it or pass --sitl-bin.")
    if not APPLET.exists():
        sys.exit(f"ERROR: speller applet not found at {APPLET}")

    font, words = load_font(APPLET)
    if args.word not in words:
        sys.exit(f"ERROR: '{args.word}' is not in the applet's WORDS list {words}")
    if args.then is not None and args.then not in words:
        sys.exit(f"ERROR: '{args.then}' is not in the applet's WORDS list {words}")

    word_idx = words.index(args.word) + 1
    then_idx = words.index(args.then) + 1 if args.then else None

    cells = build_slots(args.word, font)
    if then_idx:
        cells = max(cells, build_slots(args.then, font), key=len)
    n = args.followers if args.followers is not None else len(cells) + args.spare
    total = n + 1
    leader_sysid = 1

    work_root = Path(args.work_dir).resolve()
    work_root.mkdir(parents=True, exist_ok=True)
    glyph_csv = Path(args.glyph_csv) if args.glyph_csv else Path(args.csv).with_name(
        Path(args.csv).stem + "_glyph.csv")

    print("\n  AP_SwarmMesh decentralized glyph formation")
    print(f"  Spelling '{args.word}'" + (f" then '{args.then}'" if args.then else ""))
    print(render_word(args.word, font))
    print(f"  {len(build_slots(args.word, font))} cells, {n} spellers + 1 leader "
          f"({total} SITL processes)")
    print(f"  Cell spacing {args.spacing} m, alt {args.alt} m, coordination at {args.sr_coord} Hz")
    print(f"  Position traffic: {args.sr_pos} Hz for at most "
          f"{COHORT_SIZE * COHORTS_PER_WAVE} vehicles in four layered cohorts")
    print(f"  Work dir: {work_root}\n")

    subprocess.run(["pkill", "-9", "-f", r"arducopter.*-I\d"], capture_output=True)
    time.sleep(1)

    instances, vehicles = [], []
    logger = None
    cleaning_up = False

    def cleanup(_signum=None, _frame=None, exit_code=0):
        nonlocal cleaning_up
        if cleaning_up:
            return
        cleaning_up = True
        print("\n  Stopping SITL instances...")
        # Stop socket polling before terminating SITL. Otherwise every closed TCP
        # socket remains selector readable and pymavlink emits an EOF line forever.
        if logger is not None:
            logger.stop()
        for inst in instances:
            inst.stop()
        raise SystemExit(exit_code)
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ---- staggered launch: spellers staged in a cloud clear of the word ----
    # Taking off inside the glyph means every vehicle starts within a cell or two of some
    # cell it has no claim to, which poisons the presence rules and lets nearest free cell
    # assign most vehicles without anybody really moving. Staging the cloud off to one side
    # makes the swarm fly into the word and gives presence an honest starting point.
    offsets = phyllotaxis_offsets(n, args.scatter)
    cloud_radius = max((math.hypot(no, ea) for no, ea in offsets), default=0.0)
    glyph_half_height = (GLYPH_H - 1) * 0.5 * args.spacing
    standoff = args.standoff
    if standoff < 0:
        standoff = glyph_half_height + cloud_radius + 20.0
    print(f"  Launching (staggered), staging {standoff:.0f} m south of the word "
          f"(cloud radius {cloud_radius:.0f} m)...")
    for idx in range(total):
        mesh_sysid = idx + 1
        is_leader = mesh_sysid == leader_sysid
        wd = work_root / f"inst_{idx}"
        wd.mkdir(parents=True, exist_ok=True)
        (wd / "scripts").mkdir(exist_ok=True)
        shutil.copy(APPLET, wd / "scripts" / "swarm_speller.lua")
        if is_leader:
            home = (BASE_LAT, BASE_LON, BASE_ALT, BASE_YAW)
        else:
            north, east = offsets[mesh_sysid - 2]
            home = (*meters_to_latlon(BASE_LAT, BASE_LON, north - standoff, east),
                    BASE_ALT, BASE_YAW)
        cfg = dict(leader_sysid=leader_sysid, alt=args.alt, spacing=args.spacing,
                   sr_coord=args.sr_coord, sr_pos=args.sr_pos, formation_number=idx,
                   safe_distance=args.safe_distance)
        write_param_file(wd / "defaults.parm", mesh_sysid, total, is_leader, cfg)
        instances.append(SITLInstance(idx, mesh_sysid, sitl_bin, str(wd), home,
                                      args.speedup, args.base_port))
        if (idx + 1) % args.batch_size == 0 and (idx + 1) < total:
            print(f"    launched {idx + 1}/{total}...")
            time.sleep(args.batch_delay)
    print(f"    launched {total}/{total}.")

    # ---- concurrent bring-up ----
    for inst in instances:
        name = "leader" if inst.mesh_sysid == leader_sysid else f"s{inst.mesh_sysid}"
        vehicles.append(Vehicle(name, inst.tcp_port, inst.mesh_sysid))
    leader = next(v for v in vehicles if v.mesh_sysid == leader_sysid)

    print(f"\n  Bringing up {total} vehicles ({args.workers} workers)...")
    done = {"ok": 0, "fail": 0}
    dlock = threading.Lock()
    t_bu = time.monotonic()
    logger = FleetLogger(time.monotonic())
    logger.start()

    def _bring(v):
        ok = v.bringup(args.alt, ready_timeout=args.arm_timeout)
        if ok:
            v._set_msg_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                args.log_rate)
            logger.add(v)
        with dlock:
            done["ok" if ok else "fail"] += 1
            tot = done["ok"] + done["fail"]
            if tot % args.batch_size == 0 or tot == total:
                print(f"    bring-up {tot}/{total}  (ok={done['ok']} fail={done['fail']})  "
                      f"{time.monotonic() - t_bu:.0f}s")
        return ok

    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        list(ex.map(_bring, vehicles))

    armed = [v for v in vehicles if v.armed_ok]
    print(f"  Airborne: {len(armed)}/{total}")
    if not leader.armed_ok:
        print("  ERROR: the leader never got airborne; nothing will be published.")
        cleanup(exit_code=1)

    print("\n  Letting the mesh converge (15s)...")
    time.sleep(15)

    # ---- the demo: hand the swarm a task and watch it organise itself ----
    spellers = sum(1 for v in vehicles if v.armed_ok and v.mesh_sysid != leader_sysid)

    task_windows = []

    def run_task(idx, name):
        cells_for_word = build_slots(name, font)
        targets = latlon_targets(cells_for_word, args.spacing)
        print(f"\n  Leader announces task {idx} ('{name}') -- {len(cells_for_word)} cells, "
              f"{spellers} spellers airborne")
        t_task = time.monotonic()
        track_start = t_task - logger.t0
        leader.set_param("SCR_USER4", idx)
        deadline = time.monotonic() + args.form_timeout
        t_formed, placed, filled, settled = None, 0, 0, 0
        while time.monotonic() < deadline:
            time.sleep(3)
            placed, filled, _ = placement(vehicles, leader_sysid, targets, args.alt, args.tol)
            elapsed = time.monotonic() - t_task
            print(f"    {placed:>3}/{spellers} spellers on a cell, "
                  f"{filled:>3}/{len(cells_for_word)} cells filled   {elapsed:5.0f}s")
            # judge on cells occupied, not vehicles placed: a vehicle sharing a cell
            # with another is "placed" but leaves a hole in the word. require two
            # consecutive polls so a swarm still shuffling is not called finished
            if filled >= min(spellers, len(cells_for_word)):
                settled += 1
                if settled >= 2:
                    t_formed = elapsed
                    break
            else:
                settled = 0
        if t_formed is not None:
            print(f"    '{name}' formed in {t_formed:.0f}s")
        else:
            print(f"    '{name}' settled at {placed}/{spellers} after {args.form_timeout:.0f}s")
        print(f"  Holding {args.hold:.0f}s...")
        time.sleep(args.hold)
        task_windows.append((name, track_start, time.monotonic() - logger.t0))
        return (placed, filled, len(cells_for_word)), t_formed

    results = [(args.word, *run_task(word_idx, args.word))]
    if then_idx:
        results.append((args.then, *run_task(then_idx, args.then)))

    # ---- write the flown tracks and the target glyph ----
    print("\n  Writing tracks...")
    logger.stop()
    separation_results = []
    for name, start_s, end_s in task_windows:
        metrics = separation_metrics(
            vehicles, start_s, end_s, args.log_rate,
            args.safe_distance, args.overlap_distance,
        )
        separation_results.append((name, metrics))
        minimum, safe_violations, overlaps, evaluated_pairs = metrics
        minimum_text = f"{minimum:.2f} m" if math.isfinite(minimum) else "unavailable"
        print(f"    '{name}' separation: minimum {minimum_text}, "
              f"CBF-bound samples {safe_violations}/{evaluated_pairs}, overlaps {overlaps}")

    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "mesh_sysid", "role", "lat", "lon", "rel_alt_m"])
        for v in vehicles:
            role = "leader" if v.mesh_sysid == leader_sysid else "speller"
            with v.lock:
                for (t, lat, lon, alt) in v.track:
                    w.writerow([f"{t:.2f}", v.mesh_sysid, role,
                                f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}"])
    print(f"    wrote {args.csv}  ({sum(len(v.track) for v in vehicles)} samples)")

    with open(glyph_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["word", "slot", "lat", "lon"])
        for name in [args.word] + ([args.then] if args.then else []):
            for i, (north_cells, east_cells) in enumerate(build_slots(name, font), start=1):
                lat, lon = meters_to_latlon(BASE_LAT, BASE_LON,
                                            north_cells * args.spacing, east_cells * args.spacing)
                w.writerow([name, i, f"{lat:.7f}", f"{lon:.7f}"])
    print(f"    wrote {glyph_csv} (target cells)")

    # ---- how well did the swarm actually place itself? ----
    final_word = args.then or args.word
    targets = latlon_targets(build_slots(final_word, font), args.spacing)
    _, _, errs = placement(vehicles, leader_sysid, targets, args.alt, args.tol)
    if errs:
        errs.sort()
        print(f"\n  Cell-keeping error ('{final_word}'): "
              f"median {errs[len(errs) // 2]:.2f} m, p90 {errs[int(len(errs) * 0.9)]:.2f} m, "
              f"worst {errs[-1]:.2f} m")

    print()
    for name, (placed, filled, cells_n), t_formed in results:
        status = f"formed in {t_formed:.0f}s" if t_formed is not None else "did not complete"
        print(f"  '{name}': {placed}/{spellers} spellers placed, "
              f"{filled}/{cells_n} cells filled, {status}")
        # placed > filled means two vehicles settled on the same cell, i.e. the
        # claim negotiation failed to separate them
        if placed > filled:
            print(f"    WARNING: {placed - filled} vehicle(s) doubled up on a cell")

    unsafe = False
    for name, (minimum, safe_violations, overlaps, _) in separation_results:
        if overlaps > 0:
            unsafe = True
            print(f"    ERROR: '{name}' recorded {overlaps} sampled physical-overlap pair(s)")
        elif safe_violations > 0:
            print(f"    WARNING: '{name}' entered the {args.safe_distance:.1f} m CBF buffer; "
                  f"minimum {minimum:.2f} m")

    debug_states = [f"{vehicle.name}: {vehicle.dbg}" for vehicle in vehicles if vehicle.dbg]
    if debug_states:
        print("\n  Final onboard controller states:")
        for state in debug_states:
            print(f"    {state}")

    for inst in instances:
        inst.stop()
    print("\n  Done.\n")
    if unsafe:
        sys.exit(1)


if __name__ == "__main__":
    main()
