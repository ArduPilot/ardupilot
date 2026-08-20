#!/usr/bin/env python3
"""
Turn a swarm_spell_test.py run into a self-contained animated HTML replay
(no external libraries, opens in any browser).

Top-down view of the swarm resolving from its takeoff cloud into the word: target
cells are drawn as outlines that light up as they are occupied, vehicles as dots
with fading trails, plus a running count of cells filled.

Usage:
    python3 swarm_spell_viz.py spell_track.csv spell_track_glyph.csv -o spell.html
"""

import argparse
import csv
import json
import math
from collections import defaultdict


def latlon_to_en(lat, lon, lat0, lon0):
    east = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
    north = (lat - lat0) * 111320.0
    return east, north


def load_tracks(csv_path):
    rows = defaultdict(list)
    roles = {}
    for r in csv.DictReader(open(csv_path)):
        sid = int(r["mesh_sysid"])
        roles[sid] = r["role"]
        rows[sid].append((float(r["t_s"]), float(r["lat"]), float(r["lon"])))
    for sid in rows:
        rows[sid].sort()
    return rows, roles


def load_glyph(csv_path, word=None):
    cells = []
    for r in csv.DictReader(open(csv_path)):
        if word is not None and r["word"] != word:
            continue
        cells.append((r["word"], float(r["lat"]), float(r["lon"])))
    return cells


def resample(series, grid):
    """Linear-interpolate (t, e, n) onto the time grid."""
    out = []
    i = 0
    for t in grid:
        while i + 1 < len(series) and series[i + 1][0] < t:
            i += 1
        if t <= series[0][0]:
            out.append([series[0][1], series[0][2]])
        elif t >= series[-1][0]:
            out.append([series[-1][1], series[-1][2]])
        else:
            t0, e0, n0 = series[i]
            t1, e1, n1 = series[min(i + 1, len(series) - 1)]
            f = (t - t0) / (t1 - t0) if t1 != t0 else 0.0
            out.append([e0 + f * (e1 - e0), n0 + f * (n1 - n0)])
    return out


HTML = """<!doctype html>
<meta charset="utf-8">
<title>SwarmMesh — spelling __WORD__</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; background:#0a0d12; color:#c8d2e0;
         font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace; }
  header { padding:10px 16px; border-bottom:1px solid #1b2430; display:flex;
           gap:20px; align-items:baseline; flex-wrap:wrap; }
  h1 { font-size:14px; margin:0; font-weight:600; letter-spacing:.06em; }
  .stat b { color:#5ee6c6; font-weight:600; }
  canvas { display:block; width:100%; height:auto; }
  footer { padding:10px 16px; display:flex; gap:14px; align-items:center;
           border-top:1px solid #1b2430; }
  button { background:#16202c; color:#c8d2e0; border:1px solid #263444;
           padding:5px 14px; border-radius:5px; cursor:pointer; font:inherit; }
  button:hover { background:#1d2a38; }
  input[type=range] { flex:1; accent-color:#5ee6c6; }
</style>
<header>
  <h1>AP_SwarmMesh — decentralized glyph formation</h1>
  <div class="stat">t <b id="clock">0.0</b> s</div>
  <div class="stat">cells filled <b id="filled">0</b>/<span id="total"></span></div>
  <div class="stat">vehicles <b id="nveh"></b></div>
</header>
<canvas id="c" width="1600" height="900"></canvas>
<footer>
  <button id="play">pause</button>
  <input type="range" id="scrub" min="0" value="0" step="1">
  <span id="speed-l">speed 1x</span>
  <button id="speed">faster</button>
</footer>
<script>
const D = __DATA__;
const cv = document.getElementById('c'), cx = cv.getContext('2d');
const TOL = D.tol;
document.getElementById('total').textContent = D.cells.length;
document.getElementById('nveh').textContent = D.sids.length;

// World bounds from cells and tracks, with a margin. Accumulated in a loop rather than
// with Math.min(...xs): a big swarm over a long run is hundreds of thousands of points
// and spreading that into arguments overflows the stack.
let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
function extend(x, y) {
  if (x < minX) minX = x;
  if (x > maxX) maxX = x;
  if (y < minY) minY = y;
  if (y > maxY) maxY = y;
}
for (const c of D.cells) extend(c[0], c[1]);
for (const s of D.sids) for (const p of D.tracks[s]) extend(p[0], p[1]);
const pad = 12;
minX -= pad; maxX += pad; minY -= pad; maxY += pad;
const sc = Math.min(cv.width / (maxX - minX), cv.height / (maxY - minY));
const ox = (cv.width - (maxX - minX) * sc) / 2, oy = (cv.height - (maxY - minY) * sc) / 2;
const X = e => ox + (e - minX) * sc;
const Y = n => cv.height - oy - (n - minY) * sc;   // north up

let f = 0, playing = true, speed = 1;
const scrub = document.getElementById('scrub');
scrub.max = D.grid.length - 1;

function draw() {
  cx.fillStyle = '#0a0d12';
  cx.fillRect(0, 0, cv.width, cv.height);

  // which cells are occupied this frame
  const occupied = new Array(D.cells.length).fill(false);
  const pos = D.sids.map(s => D.tracks[s][f]);
  let filled = 0;
  for (let ci = 0; ci < D.cells.length; ci++) {
    const c = D.cells[ci];
    for (const p of pos) {
      if (Math.hypot(p[0] - c[0], p[1] - c[1]) <= TOL) { occupied[ci] = true; break; }
    }
    if (occupied[ci]) filled++;
  }

  // target cells
  const r = Math.max(3, D.spacing * sc * 0.38);
  for (let ci = 0; ci < D.cells.length; ci++) {
    const c = D.cells[ci];
    cx.beginPath();
    cx.rect(X(c[0]) - r, Y(c[1]) - r, r * 2, r * 2);
    if (occupied[ci]) { cx.fillStyle = 'rgba(94,230,198,.16)'; cx.fill(); }
    cx.strokeStyle = occupied[ci] ? 'rgba(94,230,198,.55)' : 'rgba(120,140,165,.22)';
    cx.lineWidth = 1;
    cx.stroke();
  }

  // trails
  const TRAIL = 28;
  cx.lineWidth = 1.2;
  for (const s of D.sids) {
    const tr = D.tracks[s];
    const start = Math.max(0, f - TRAIL);
    if (f - start < 2) continue;
    cx.beginPath();
    cx.moveTo(X(tr[start][0]), Y(tr[start][1]));
    for (let k = start + 1; k <= f; k++) cx.lineTo(X(tr[k][0]), Y(tr[k][1]));
    cx.strokeStyle = s === D.leader ? 'rgba(255,196,92,.30)' : 'rgba(110,170,255,.28)';
    cx.stroke();
  }

  // vehicles
  for (let i = 0; i < D.sids.length; i++) {
    const s = D.sids[i], p = pos[i];
    const isLeader = s === D.leader;
    cx.beginPath();
    cx.arc(X(p[0]), Y(p[1]), isLeader ? 5 : 3.2, 0, Math.PI * 2);
    cx.fillStyle = isLeader ? '#ffc45c' : '#6eaaff';
    cx.fill();
  }

  document.getElementById('clock').textContent = D.grid[f].toFixed(1);
  document.getElementById('filled').textContent = filled;
  scrub.value = f;
}

function tick() {
  if (playing) { f = (f + speed) % D.grid.length; draw(); }
  requestAnimationFrame(tick);
}
document.getElementById('play').onclick = e => {
  playing = !playing; e.target.textContent = playing ? 'pause' : 'play';
};
document.getElementById('speed').onclick = () => {
  speed = speed >= 8 ? 1 : speed * 2;
  document.getElementById('speed-l').textContent = 'speed ' + speed + 'x';
};
scrub.oninput = () => { f = +scrub.value; playing = false;
  document.getElementById('play').textContent = 'play'; draw(); };
draw(); tick();
</script>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="track CSV from swarm_spell_test.py")
    ap.add_argument("glyph_csv", help="target-cell CSV written alongside it")
    ap.add_argument("-o", "--out", default="spell.html")
    ap.add_argument("--word", default=None, help="which word's cells to draw (default: the last one)")
    ap.add_argument("--fps", type=int, default=10, help="animation frames per second (default 10)")
    ap.add_argument("--spacing", type=float, default=4.0, help="cell spacing used in the run, m")
    ap.add_argument("--tol", type=float, default=2.5, help="occupied-cell radius, m")
    args = ap.parse_args()

    rows, roles = load_tracks(args.csv)
    leader = next((s for s, r in roles.items() if r == "leader"), min(rows))
    lat0, lon0 = rows[leader][0][1], rows[leader][0][2]

    all_cells = load_glyph(args.glyph_csv)
    word = args.word or all_cells[-1][0]
    cells = [latlon_to_en(la, lo, lat0, lon0) for (w, la, lo) in all_cells if w == word]
    if not cells:
        raise SystemExit(f"no cells for word '{word}' in {args.glyph_csv}")

    local = {sid: [(t, *latlon_to_en(la, lo, lat0, lon0)) for (t, la, lo) in tr]
             for sid, tr in rows.items()}
    t_min = min(tr[0][0] for tr in local.values())
    t_max = max(tr[-1][0] for tr in local.values())
    dt = 1.0 / args.fps
    grid = [round(t_min + i * dt, 3) for i in range(int((t_max - t_min) / dt) + 1)]

    sids = sorted(local)
    data = {
        "grid": [round(t, 2) for t in grid],
        "leader": leader,
        "sids": sids,
        "tracks": {s: [[round(p[0], 2), round(p[1], 2)] for p in resample(local[s], grid)]
                   for s in sids},
        "cells": [[round(e, 2), round(n, 2)] for e, n in cells],
        "spacing": args.spacing,
        "tol": args.tol,
    }
    with open(args.out, "w") as f:
        f.write(HTML.replace("__DATA__", json.dumps(data, separators=(",", ":")))
                    .replace("__WORD__", word))
    print(f"wrote {args.out}  ({len(sids)} vehicles, {len(cells)} cells, {len(grid)} frames)")


if __name__ == "__main__":
    main()
