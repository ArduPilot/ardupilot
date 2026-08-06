#!/usr/bin/env python3
"""
Turn a swarm_formation_test.py track CSV into a self-contained, animated HTML
visualization (no external libraries, opens in any browser).

Renders a tactical top-down replay of the swarm — leader with range rings,
followers connected by formation spokes with fading trails — plus a live
formation-keeping error trace, with play/pause and a mission-clock scrubber.

Usage:
    python3 swarm_formation_viz.py formation_track.csv -o formation.html
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


def load(csv_path):
    rows = defaultdict(list)
    roles = {}
    for r in csv.DictReader(open(csv_path)):
        sid = int(r["mesh_sysid"])
        roles[sid] = r["role"]
        rows[sid].append((float(r["t_s"]), float(r["lat"]), float(r["lon"]), float(r["rel_alt_m"])))
    for sid in rows:
        rows[sid].sort()
    return rows, roles


def resample(series, grid):
    """Linear-interpolate (t, e, n) series onto the time grid."""
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("-o", "--out", default="formation.html")
    ap.add_argument("--fps", type=int, default=20, help="animation frames per second (default 20)")
    ap.add_argument("--from", dest="from_t", type=float, default=None, help="crop replay to start at this time (s)")
    ap.add_argument("--to", dest="to_t", type=float, default=None, help="crop replay to end at this time (s)")
    args = ap.parse_args()

    rows, roles = load(args.csv)
    leader = next(s for s, r in roles.items() if r == "leader")
    lat0, lon0 = rows[leader][0][1], rows[leader][0][2]

    local = {sid: [(t, *latlon_to_en(la, lo, lat0, lon0)) for (t, la, lo, _a) in tr]
             for sid, tr in rows.items()}

    t_min = min(tr[0][0] for tr in local.values())
    t_max = max(tr[-1][0] for tr in local.values())
    lo = args.from_t if args.from_t is not None else t_min
    hi = args.to_t if args.to_t is not None else t_max
    dt = 1.0 / args.fps
    grid = [round(lo + i * dt, 3) for i in range(int((hi - lo) / dt) + 1)]

    sids = sorted(local.keys())
    tracks = {sid: resample(local[sid], grid) for sid in sids}

    # per-follower held offset (median of second half) -> formation error series
    errors = {}
    lead_res = tracks[leader]
    for sid in sids:
        if sid == leader:
            continue
        rel = [[tracks[sid][k][0] - lead_res[k][0], tracks[sid][k][1] - lead_res[k][1]]
               for k in range(len(grid))]
        half = rel[len(rel) // 2:]
        be = sum(r[0] for r in half) / len(half)
        bn = sum(r[1] for r in half) / len(half)
        errors[sid] = [round(math.hypot(r[0] - be, r[1] - bn), 3) for r in rel]

    # For big swarms, per-follower error lines (and JSON) don't scale: replace them
    # with a per-frame percentile envelope (p10 / median / p90) across all followers.
    big = (len(sids) - 1) > 24
    err_env = None
    if big:
        fs = [s for s in sids if s != leader]
        p10, p50, p90 = [], [], []
        for k in range(len(grid)):
            vals = sorted(errors[s][k] for s in fs)
            n = len(vals)
            p10.append(round(vals[int(n * 0.10)], 2))
            p50.append(round(vals[int(n * 0.50)], 2))
            p90.append(round(vals[min(n - 1, int(n * 0.90))], 2))
        err_env = {"p10": p10, "p50": p50, "p90": p90}

    data = {
        "grid": [round(t, 2) for t in grid],
        "leader": leader,
        "roles": roles,
        "tracks": {s: [[round(p[0], 2), round(p[1], 2)] for p in tr] for s, tr in tracks.items()},
        "errors": {} if big else errors,
        "err_env": err_env,
        "big": big,
        "sids": sids,
    }

    html = HTML_TEMPLATE.replace("__DATA__", json.dumps(data)).replace("__FPS__", str(args.fps))
    with open(args.out, "w") as f:
        f.write(html)
    print(f"wrote {args.out}  ({len(grid)} frames, {t_max:.0f}s, {len(sids)} vehicles)")


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SwarmMesh formation replay</title>
<style>
  /* Committed dark "tactical telemetry" display — a single deliberate visual world. */
  :root{
    --ground:#080d12; --panel:#0f171e; --panel-2:#0c141b;
    --hair:#16232c; --hair-strong:#22333f;
    --ink:#cdd9e1; --muted:#6b7d8a; --dim:#43535e;
    --leader:#ffb547;
    --mono:ui-monospace,"SF Mono",SFMono-Regular,Menlo,Consolas,monospace;
    --sans:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--ground);color:var(--ink);font-family:var(--sans);
       -webkit-font-smoothing:antialiased;}
  .wrap{max-width:1080px;margin:0 auto;padding:22px 18px 30px;}
  header{display:flex;align-items:baseline;justify-content:space-between;gap:16px;
         border-bottom:1px solid var(--hair-strong);padding-bottom:12px;margin-bottom:18px;flex-wrap:wrap;}
  h1{font-size:16px;font-weight:600;letter-spacing:.02em;margin:0;}
  .eyebrow{font-family:var(--mono);font-size:11px;letter-spacing:.18em;text-transform:uppercase;color:var(--muted);}
  .stat{font-family:var(--mono);font-size:12px;color:var(--muted);}
  .stat b{color:var(--ink);font-weight:600;}
  .panels{display:flex;gap:16px;flex-wrap:wrap;}
  .card{background:var(--panel);border:1px solid var(--hair-strong);border-radius:12px;
        padding:14px;flex:1 1 400px;min-width:300px;}
  .cap{font-family:var(--mono);font-size:11px;letter-spacing:.14em;text-transform:uppercase;
       color:var(--muted);margin:0 0 10px;}
  canvas{width:100%;height:auto;display:block;background:var(--panel-2);border-radius:8px;}
  .legend{display:flex;gap:16px;flex-wrap:wrap;font-family:var(--mono);font-size:12px;margin-top:12px;}
  .legend span{display:inline-flex;align-items:center;gap:7px;color:var(--muted);}
  .legend b{color:var(--ink);font-weight:500;}
  .dot{width:9px;height:9px;border-radius:50%;box-shadow:0 0 6px currentColor;}
  .transport{display:flex;align-items:center;gap:14px;margin-top:18px;
             background:var(--panel);border:1px solid var(--hair-strong);border-radius:12px;padding:12px 16px;}
  button{background:transparent;color:var(--leader);border:1px solid var(--leader);border-radius:6px;
         padding:7px 18px;font-family:var(--mono);font-size:13px;letter-spacing:.06em;cursor:pointer;
         min-width:92px;transition:background .12s,color .12s;}
  button:hover{background:var(--leader);color:var(--ground);}
  button:focus-visible{outline:2px solid var(--leader);outline-offset:2px;}
  input[type=range]{flex:1;accent-color:var(--leader);height:4px;cursor:pointer;}
  input[type=range]:focus-visible{outline:2px solid var(--leader);outline-offset:4px;}
  .clock{font-family:var(--mono);font-variant-numeric:tabular-nums;font-size:14px;
         color:var(--leader);min-width:88px;text-align:right;}
  @media (prefers-reduced-motion:reduce){button{transition:none}}
</style></head><body><div class="wrap">
<header>
  <div>
    <div class="eyebrow">AP_SwarmMesh · SITL replay</div>
    <h1>Leader–follower formation over the mesh</h1>
  </div>
  <div class="stat" id="hud"></div>
</header>
<div class="panels">
  <div class="card">
    <p class="cap">Overhead view · East–North (m) · range rings (10 m)</p>
    <canvas id="map" width="540" height="540"></canvas>
    <div class="legend" id="legend"></div>
  </div>
  <div class="card">
    <p class="cap">Formation error · deviation from held slot (m)</p>
    <canvas id="err" width="540" height="540"></canvas>
  </div>
</div>
<div class="transport">
  <button id="play">Pause</button>
  <input type="range" id="scrub" min="0" value="0" step="1" aria-label="mission time">
  <span class="clock" id="clock">T+00.0s</span>
</div>
</div>
<script>
const D = __DATA__;
const BIG = D.big;   // large swarm: drop spokes/legend, use a percentile error band
// leader = amber; followers on a cool sequence
const FOLLOW = ["#38bdf8","#2dd4bf","#4ade80","#a78bfa","#f472b6","#22d3ee"];
const followIdx = {};
{let i=0; for(const s of D.sids){ if(s!==D.leader){ followIdx[s]=i++; } }}
const colorFor = s => s===D.leader ? "#ffb547" : FOLLOW[followIdx[s]%FOLLOW.length];

// square bounds over all tracks
let minE=1e9,maxE=-1e9,minN=1e9,maxN=-1e9;
for(const s of D.sids) for(const p of D.tracks[s]){minE=Math.min(minE,p[0]);maxE=Math.max(maxE,p[0]);minN=Math.min(minN,p[1]);maxN=Math.max(maxN,p[1]);}
const pad=Math.max(6,Math.max(maxE-minE,maxN-minN)*0.10);
minE-=pad;maxE+=pad;minN-=pad;maxN+=pad;
const span=Math.max(maxE-minE,maxN-minN), cE=(minE+maxE)/2, cN=(minN+maxN)/2;
minE=cE-span/2;maxE=cE+span/2;minN=cN-span/2;maxN=cN+span/2;

const map=document.getElementById('map'), mx=map.getContext('2d');
const err=document.getElementById('err'), ex=err.getContext('2d');
const W=map.width,H=map.height,M=26;
const sx=e=>(e-minE)/(maxE-minE)*(W-2*M)+M;
const sy=n=>H-M-((n-minN)/(maxN-minN)*(H-2*M));
const mPerPx=(maxE-minE)/(W-2*M);

let maxErr=0.1;
if(BIG){ for(const v of D.err_env.p90) maxErr=Math.max(maxErr,v); }
else { for(const s in D.errors) for(const v of D.errors[s]) maxErr=Math.max(maxErr,v); }
maxErr=Math.max(0.5,Math.ceil(maxErr*5)/5);

function drawMap(f){
  mx.clearRect(0,0,W,H);
  // fine grid every 10 m
  mx.strokeStyle="#16232c";mx.lineWidth=1;
  const step=10/mPerPx;
  for(let x=sx(Math.ceil(minE/10)*10);x<W-M;x+=step){mx.beginPath();mx.moveTo(x,M);mx.lineTo(x,H-M);mx.stroke();}
  for(let y=sy(Math.ceil(minN/10)*10);y>M;y-=step){mx.beginPath();mx.moveTo(M,y);mx.lineTo(W-M,y);mx.stroke();}
  // range rings around leader
  const L=D.tracks[D.leader][f];const lx=sx(L[0]),ly=sy(L[1]);
  mx.strokeStyle="#22333f";
  for(const r of [10,20,30]){mx.beginPath();mx.arc(lx,ly,r/mPerPx,0,7);mx.stroke();}
  // formation spokes leader -> followers (only legible for small swarms)
  if(!BIG){
    mx.strokeStyle="rgba(255,181,71,0.22)";mx.lineWidth=1;
    for(const s of D.sids){ if(s===D.leader)continue; const p=D.tracks[s][f];
      mx.beginPath();mx.moveTo(lx,ly);mx.lineTo(sx(p[0]),sy(p[1]));mx.stroke();}
  }
  // trails + markers
  const trailWin=BIG?28:90, fr=BIG?2.6:5;
  for(const s of D.sids){
    const tr=D.tracks[s],c=colorFor(s),lead=s===D.leader;
    const start=Math.max(0,f-trailWin);
    mx.lineWidth=lead?2.4:(BIG?0.9:1.6);mx.strokeStyle=c;mx.globalAlpha=lead?0.9:(BIG?0.5:0.75);
    mx.beginPath();for(let k=start;k<=f;k++){const p=tr[k];k===start?mx.moveTo(sx(p[0]),sy(p[1])):mx.lineTo(sx(p[0]),sy(p[1]));}
    mx.stroke();mx.globalAlpha=1;
    const p=tr[f],X=sx(p[0]),Y=sy(p[1]);
    mx.beginPath();mx.arc(X,Y,lead?7:fr,0,7);mx.fillStyle=c;
    if(!BIG||lead){mx.shadowColor=c;mx.shadowBlur=lead?12:8;}
    mx.fill();mx.shadowBlur=0;
    if(lead){mx.lineWidth=1.5;mx.strokeStyle="rgba(0,0,0,.55)";mx.stroke();}
  }
  // scale bar
  mx.strokeStyle="#6b7d8a";mx.lineWidth=2;mx.beginPath();mx.moveTo(M,H-12);mx.lineTo(M+10/mPerPx,H-12);mx.stroke();
  mx.fillStyle="#6b7d8a";mx.font="11px ui-monospace,monospace";mx.fillText("10 m",M,H-16);
}

function drawErr(f){
  ex.clearRect(0,0,W,H);
  const x0=46,y0=H-30,x1=W-14,y1=16;
  ex.fillStyle="#6b7d8a";ex.font="11px ui-monospace,monospace";
  for(let g=0;g<=4;g++){const v=maxErr*g/4,y=y0-(y0-y1)*g/4;
    ex.strokeStyle="#16232c";ex.lineWidth=1;ex.beginPath();ex.moveTo(x0,y);ex.lineTo(x1,y);ex.stroke();
    ex.fillText(v.toFixed(1),8,y+3);}
  ex.strokeStyle="#22333f";ex.beginPath();ex.moveTo(x0,y0);ex.lineTo(x0,y1);ex.stroke();
  const N=D.grid.length,px=k=>x0+(x1-x0)*k/(N-1),py=v=>y0-(y0-y1)*Math.min(v,maxErr)/maxErr;
  if(BIG){
    // p10-p90 band + median line across all followers
    const E=D.err_env;
    ex.fillStyle="rgba(56,189,248,0.18)";ex.beginPath();
    for(let k=0;k<=f;k++){k===0?ex.moveTo(px(k),py(E.p10[k])):ex.lineTo(px(k),py(E.p10[k]));}
    for(let k=f;k>=0;k--){ex.lineTo(px(k),py(E.p90[k]));}
    ex.closePath();ex.fill();
    ex.strokeStyle="#38bdf8";ex.lineWidth=2;ex.beginPath();
    for(let k=0;k<=f;k++){k===0?ex.moveTo(px(k),py(E.p50[k])):ex.lineTo(px(k),py(E.p50[k]));}
    ex.stroke();
    const y=py(E.p50[f]);ex.beginPath();ex.arc(px(f),y,3,0,7);ex.fillStyle="#38bdf8";ex.fill();
    ex.fillStyle="#6b7d8a";ex.fillText("median + p10–p90 band, "+(D.sids.length-1)+" followers",x0+6,y1+4);
  } else {
    for(const s of D.sids){ if(s===D.leader)continue;
      const c=colorFor(s),e=D.errors[s];
      ex.strokeStyle=c;ex.lineWidth=1.6;ex.globalAlpha=.9;ex.beginPath();
      for(let k=0;k<=f;k++){k===0?ex.moveTo(px(k),py(e[k])):ex.lineTo(px(k),py(e[k]));}
      ex.stroke();ex.globalAlpha=1;
      const y=py(e[f]);ex.beginPath();ex.arc(px(f),y,3,0,7);ex.fillStyle=c;ex.fill();
    }
  }
  ex.strokeStyle="#43535e";ex.setLineDash([3,4]);ex.beginPath();ex.moveTo(px(f),y1);ex.lineTo(px(f),y0);ex.stroke();ex.setLineDash([]);
  ex.fillStyle="#6b7d8a";ex.fillText("m",8,y1-2);ex.fillText("T+"+D.grid[f].toFixed(0)+"s",x1-52,y0+20);
}

// legend + hud
const leg=document.getElementById('legend');
if(BIG){
  const el=document.createElement('span');
  el.innerHTML=`<span class="dot" style="background:#ffb547;color:#ffb547"></span><b>leader</b>`+
    `&nbsp;&nbsp;<span class="dot" style="background:#38bdf8;color:#38bdf8"></span>${D.sids.length-1} followers`;
  leg.appendChild(el);
} else {
  for(const s of D.sids){const c=colorFor(s);const el=document.createElement('span');
    el.innerHTML=`<span class="dot" style="background:${c};color:${c}"></span>`+
      (s===D.leader?`<b>leader</b> ${s}`:`follower ${s}`);leg.appendChild(el);}
}
document.getElementById('hud').innerHTML =
  `<b>${D.sids.length}</b> vehicles &nbsp;·&nbsp; <b>${(D.sids.length-1)}</b> followers &nbsp;·&nbsp; <b>${D.grid[D.grid.length-1].toFixed(0)}s</b> replay`;

const scrub=document.getElementById('scrub'),clock=document.getElementById('clock'),playBtn=document.getElementById('play');
scrub.max=D.grid.length-1;
let f=0,playing=true;
function render(){drawMap(f);drawErr(f);scrub.value=f;clock.textContent="T+"+D.grid[f].toFixed(1)+"s";}
scrub.addEventListener('input',()=>{f=+scrub.value;playing=false;playBtn.textContent="Play";render();});
playBtn.addEventListener('click',()=>{playing=!playing;playBtn.textContent=playing?"Pause":"Play";});
setInterval(()=>{if(playing){f=(f+1)%D.grid.length;render();}},1000/__FPS__);
render();
</script></body></html>
"""


if __name__ == "__main__":
    main()
