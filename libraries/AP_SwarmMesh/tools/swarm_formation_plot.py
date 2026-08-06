#!/usr/bin/env python3
"""
Plot the output of swarm_formation_test.py.

Reads the track CSV (t_s, mesh_sysid, role, lat, lon, rel_alt_m) and produces:
  1. a top-down XY plot of every vehicle's path (metres, local ENU about the
     leader's start), leader vs followers; and
  2. per-follower horizontal offset-from-leader error over time, which shows how
     tightly formation was held as the leader moved.

Requires matplotlib:  pip install matplotlib

Usage:
    python3 swarm_formation_plot.py formation_track.csv -o formation.png
"""

import argparse
import csv
import math
from collections import defaultdict

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    raise SystemExit("matplotlib required: pip install matplotlib")


def latlon_to_local(lat, lon, lat0, lon0):
    """Flat-earth ENU metres about (lat0, lon0). Returns (east, north)."""
    north = (lat - lat0) * 111320.0
    east = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
    return east, north


def load(csv_path):
    rows = defaultdict(list)   # mesh_sysid -> list of (t, lat, lon, alt)
    roles = {}
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            sid = int(r["mesh_sysid"])
            roles[sid] = r["role"]
            rows[sid].append((float(r["t_s"]), float(r["lat"]), float(r["lon"]), float(r["rel_alt_m"])))
    for sid in rows:
        rows[sid].sort()
    return rows, roles


def leader_pos_at(leader_track, t):
    """Linear-interpolate the leader (t,east,north) track at time t."""
    if t <= leader_track[0][0]:
        return leader_track[0][1], leader_track[0][2]
    if t >= leader_track[-1][0]:
        return leader_track[-1][1], leader_track[-1][2]
    for i in range(1, len(leader_track)):
        t1, e1, n1 = leader_track[i]
        if t1 >= t:
            t0, e0, n0 = leader_track[i - 1]
            f = (t - t0) / (t1 - t0) if t1 != t0 else 0.0
            return e0 + f * (e1 - e0), n0 + f * (n1 - n0)
    return leader_track[-1][1], leader_track[-1][2]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("-o", "--out", default="formation.png")
    args = ap.parse_args()

    rows, roles = load(args.csv)
    leader_sid = next((s for s, r in roles.items() if r == "leader"), None)
    if leader_sid is None:
        raise SystemExit("no leader row in CSV")

    # reference: leader's first fix
    lat0, lon0 = rows[leader_sid][0][1], rows[leader_sid][0][2]

    # convert everything to local ENU
    local = {}   # sid -> list of (t, east, north)
    for sid, tr in rows.items():
        local[sid] = [(t, *latlon_to_local(lat, lon, lat0, lon0)) for (t, lat, lon, _a) in tr]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # --- panel 1: XY tracks ---
    for sid, tr in sorted(local.items()):
        e = [p[1] for p in tr]
        n = [p[2] for p in tr]
        is_leader = sid == leader_sid
        ax1.plot(e, n, "-", lw=2.5 if is_leader else 1.5,
                 color="black" if is_leader else None,
                 label=f"{'LEADER' if is_leader else 'follower'} {sid}", zorder=3 if is_leader else 2)
        ax1.plot(e[0], n[0], "o", ms=8, color="tab:green")   # start
        ax1.plot(e[-1], n[-1], "s", ms=8, color="tab:red")   # end
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.set_title("Formation tracks (green=start, red=end)")
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)

    # --- panel 2: follower offset error vs time ---
    lead = local[leader_sid]
    for sid, tr in sorted(local.items()):
        if sid == leader_sid:
            continue
        # per-sample horizontal distance to leader (relative to the mean, so we
        # measure how *constant* the offset is, not its magnitude)
        errs, ts = [], []
        rels = []
        for (t, e, n) in tr:
            le, ln = leader_pos_at(lead, t)
            rels.append((t, e - le, n - ln))
        if not rels:
            continue
        # baseline = median offset once settled (second half of the run)
        mid = rels[len(rels) // 2:]
        be = sum(r[1] for r in mid) / len(mid)
        bn = sum(r[2] for r in mid) / len(mid)
        for (t, de, dn) in rels:
            ts.append(t)
            errs.append(math.hypot(de - be, dn - bn))
        ax2.plot(ts, errs, lw=1.5, label=f"follower {sid}")
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("offset deviation from held slot (m)")
    ax2.set_title("Formation-keeping error vs time")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
