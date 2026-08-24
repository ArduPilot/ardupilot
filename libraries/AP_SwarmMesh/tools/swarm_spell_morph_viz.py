#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Build a self-contained GSoC-to-CoSG HTML replay from a morph SITL run.

Usage:
    python3 swarm_spell_morph_viz.py track.csv glyph.csv tasks.csv -o replay.html
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


def load_tracks(path):
    rows = defaultdict(list)
    roles = {}
    with open(path, newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            sysid = int(row["mesh_sysid"])
            roles[sysid] = row["role"]
            rows[sysid].append((float(row["t_s"]), float(row["lat"]),
                                float(row["lon"]), float(row["rel_alt_m"])))
    return rows, roles


def load_glyphs(path):
    glyphs = defaultdict(list)
    with open(path, newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            glyphs[row["word"]].append((float(row["lat"]), float(row["lon"])))
    return glyphs


def load_tasks(path):
    with open(path, newline="") as csv_file:
        return [(row["word"], float(row["start_s"]), float(row["end_s"]))
                for row in csv.DictReader(csv_file)]


def resample(series, grid, lat0, lon0):
    out = []
    index = 0
    local = [(t, *latlon_to_en(lat, lon, lat0, lon0), alt)
             for t, lat, lon, alt in series]
    for sample_time in grid:
        while index + 1 < len(local) and local[index + 1][0] < sample_time:
            index += 1
        if sample_time <= local[0][0]:
            point = local[0][1:]
        elif sample_time >= local[-1][0]:
            point = local[-1][1:]
        else:
            before = local[index]
            after = local[min(index + 1, len(local) - 1)]
            fraction = ((sample_time - before[0]) / (after[0] - before[0])
                        if after[0] != before[0] else 0.0)
            point = tuple(before[i] + fraction * (after[i] - before[i])
                          for i in range(1, 4))
        out.append([round(value, 1) for value in point])
    return out


FRAGMENT = r"""<div id="swarm-morph-replay">
  <h2>AP_SwarmMesh · GSoC → CoSG</h2>
  <div class="viz-row" aria-live="polite">
    <span>task <strong data-phase></strong></span>
    <span>elapsed <strong data-time></strong> s</span>
    <span>cells <strong data-filled></strong>/56</span>
    <span>minimum separation <strong data-separation></strong> m</span>
  </div>
  <canvas data-swarm-canvas role="img"
          aria-label="Top-down replay of 56 decentralized vehicles forming GSoC and morphing into CoSG"></canvas>
  <div class="viz-controls">
    <button type="button" class="btn btn-primary" data-play>Pause</button>
    <label class="form-label">Timeline
      <span class="sr-only">experiment time</span>
      <input class="form-range" data-scrub type="range" min="0" value="0" step="1">
    </label>
    <label class="form-label">Replay rate
      <select class="form-select" data-rate>
        <option value="10">10×</option>
        <option value="30" selected>30×</option>
        <option value="100">100×</option>
      </select>
    </label>
  </div>
</div>
<style>
  #swarm-morph-replay { width: 100%; }
  #swarm-morph-replay h2 { margin-bottom: .5rem; }
  #swarm-morph-replay [data-swarm-canvas] {
    display: block;
    width: 100%;
    aspect-ratio: 16 / 9;
    margin: .75rem 0;
  }
  #swarm-morph-replay .viz-controls .form-label:first-of-type {
    flex: 1 1 18rem;
  }
</style>
<script>
(() => {
  const root = document.getElementById('swarm-morph-replay');
  const D = __DATA__;
  const canvas = root.querySelector('[data-swarm-canvas]');
  const ctx = canvas.getContext('2d');
  const scrub = root.querySelector('[data-scrub]');
  const play = root.querySelector('[data-play]');
  const rate = root.querySelector('[data-rate]');
  scrub.max = D.grid.length - 1;

  let frame = 0;
  let playing = true;
  let lastTick = performance.now();
  let replayTime = D.grid[0];
  let bounds;

  function color(name) {
    return getComputedStyle(root).getPropertyValue(name).trim();
  }

  function phaseAt(time) {
    return D.tasks.find(task => time >= task.start && time <= task.end) || D.tasks[D.tasks.length - 1];
  }

  function updateBounds() {
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    const extend = p => {
      minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
      minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    };
    Object.values(D.cells).flat().forEach(extend);
    for (const sid of D.sids) D.tracks[sid].forEach(extend);
    const pad = 8;
    bounds = {minX: minX - pad, maxX: maxX + pad, minY: minY - pad, maxY: maxY + pad};
  }

  function resize() {
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(320, canvas.clientWidth);
    const height = width * 9 / 16;
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    draw();
  }

  function draw() {
    if (!bounds || !D.grid.length) return;
    const width = canvas.clientWidth;
    const height = width * 9 / 16;
    ctx.clearRect(0, 0, width, height);
    const scale = Math.min(width / (bounds.maxX - bounds.minX), height / (bounds.maxY - bounds.minY));
    const ox = (width - (bounds.maxX - bounds.minX) * scale) / 2;
    const oy = (height - (bounds.maxY - bounds.minY) * scale) / 2;
    const x = east => ox + (east - bounds.minX) * scale;
    const y = north => height - oy - (north - bounds.minY) * scale;
    const task = phaseAt(D.grid[frame]);
    const cells = D.cells[task.word];
    const positions = D.sids.map(sid => D.tracks[sid][frame]);
    const occupied = cells.map(cell => positions.some(p =>
      Math.hypot(p[0] - cell[0], p[1] - cell[1], p[2] - D.alt) <= D.tol));

    const neutral = color('--border');
    const active = color('--viz-series-1');
    const vehicle = color('--viz-series-2');
    const leader = color('--viz-series-3');
    const radius = Math.max(3, D.spacing * scale * .36);
    cells.forEach((cell, index) => {
      ctx.beginPath();
      ctx.rect(x(cell[0]) - radius, y(cell[1]) - radius, radius * 2, radius * 2);
      ctx.globalAlpha = occupied[index] ? .28 : .45;
      ctx.fillStyle = occupied[index] ? active : color('--muted');
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = occupied[index] ? active : neutral;
      ctx.lineWidth = 1;
      ctx.stroke();
    });

    const trailStart = Math.max(0, frame - 8);
    D.sids.forEach(sid => {
      const points = D.tracks[sid];
      ctx.beginPath();
      ctx.moveTo(x(points[trailStart][0]), y(points[trailStart][1]));
      for (let index = trailStart + 1; index <= frame; index++) {
        ctx.lineTo(x(points[index][0]), y(points[index][1]));
      }
      ctx.globalAlpha = .32;
      ctx.strokeStyle = sid === D.leader ? leader : vehicle;
      ctx.stroke();
      ctx.globalAlpha = 1;
    });

    positions.forEach((point, index) => {
      const sid = D.sids[index];
      ctx.beginPath();
      ctx.arc(x(point[0]), y(point[1]), sid === D.leader ? 5 : 3.2, 0, Math.PI * 2);
      ctx.fillStyle = sid === D.leader ? leader : vehicle;
      ctx.fill();
    });

    let minimum = Infinity;
    for (let first = 0; first < positions.length; first++) {
      for (let second = first + 1; second < positions.length; second++) {
        minimum = Math.min(minimum, Math.hypot(
          positions[first][0] - positions[second][0],
          positions[first][1] - positions[second][1],
          positions[first][2] - positions[second][2]));
      }
    }
    root.querySelector('[data-phase]').textContent = task.word;
    root.querySelector('[data-time]').textContent = Math.max(0, D.grid[frame] - task.start).toFixed(0);
    root.querySelector('[data-filled]').textContent = occupied.filter(Boolean).length;
    root.querySelector('[data-separation]').textContent = minimum.toFixed(1);
    scrub.value = frame;
  }

  function tick(now) {
    if (playing) {
      replayTime += (now - lastTick) * .001 * Number(rate.value);
      if (replayTime >= D.grid[D.grid.length - 1]) {
        replayTime = D.grid[D.grid.length - 1];
        playing = false;
        play.textContent = 'Play';
      }
      frame = Math.min(D.grid.length - 1,
                       Math.round((replayTime - D.grid[0]) / D.samplePeriod));
      draw();
    }
    lastTick = now;
    requestAnimationFrame(tick);
  }

  play.addEventListener('click', () => {
    if (!playing && frame === D.grid.length - 1) {
      frame = 0; replayTime = D.grid[0];
    }
    playing = !playing;
    play.textContent = playing ? 'Pause' : 'Play';
  });
  scrub.addEventListener('input', () => {
    frame = Number(scrub.value);
    replayTime = D.grid[frame];
    playing = false;
    play.textContent = 'Play';
    draw();
  });
  updateBounds();
  new ResizeObserver(resize).observe(canvas);
  requestAnimationFrame(tick);
})();
</script>
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="track CSV from swarm_spell_morph_test.py")
    parser.add_argument("glyph_csv", help="target-cell CSV written alongside the track")
    parser.add_argument("task_csv", help="task-window CSV written alongside the track")
    parser.add_argument("-o", "--out", default="swarm-morph-replay.html")
    parser.add_argument("--sample-period", type=float, default=3.0,
                        help="seconds between replay frames (default: 3)")
    parser.add_argument("--spacing", type=float, default=4.0)
    parser.add_argument("--alt", type=float, default=20.0)
    parser.add_argument("--tol", type=float, default=2.5)
    args = parser.parse_args()

    tracks, roles = load_tracks(args.csv)
    glyphs = load_glyphs(args.glyph_csv)
    tasks = load_tasks(args.task_csv)
    leader = next((sysid for sysid, role in roles.items() if role == "leader"), min(tracks))
    lat0, lon0 = tracks[leader][0][1:3]
    start = tasks[0][1]
    end = tasks[-1][2]
    frame_count = int((end - start) / args.sample_period) + 1
    grid = [round(start + index * args.sample_period, 2) for index in range(frame_count)]
    if grid[-1] < end:
        grid.append(round(end, 2))
    sysids = sorted(tracks)
    data = {
        "grid": grid,
        "samplePeriod": args.sample_period,
        "leader": leader,
        "sids": sysids,
        "tracks": {sysid: resample(tracks[sysid], grid, lat0, lon0) for sysid in sysids},
        "cells": {
            word: [[round(value, 1) for value in latlon_to_en(lat, lon, lat0, lon0)]
                   for lat, lon in cells]
            for word, cells in glyphs.items()
        },
        "tasks": [{"word": word, "start": start_s, "end": end_s}
                  for word, start_s, end_s in tasks],
        "spacing": args.spacing,
        "alt": args.alt,
        "tol": args.tol,
    }
    with open(args.out, "w") as output:
        output.write(FRAGMENT.replace("__DATA__", json.dumps(data, separators=(",", ":"))))
    print(f"wrote {args.out} ({len(sysids)} vehicles, {len(grid)} frames)")


if __name__ == "__main__":
    main()
