# AP_SwarmMesh Leader-Follower Formation Demo (SITL)

A leader + N follower ArduCopter SITL instances connect over the AP_SwarmMesh multicast mesh. Each follower reads the leaders position out of its own peer table (**populated over the decentralized mesh, not from the GCS**) and holds a fixed NED offset in `GUIDED` mode as the leader flies in a set trajectory.

> **Scaling to the full 254-node sysid range** (bugs fixed, lessons, and the
> `swarm_formation_scale_test.py` usage) is documented in **[SCALING.md](SCALING.md)**.
> Headline: 253/254 sitl drones held **0.87 m median** formation off a moving leader.

## Pieces

| File | Role |
|------|------|
| `../../AP_Scripting/applets/swarm_follower.lua` | Onboard follower logic: reads `swarm:get_peer_location(leader_sysid)`, commands a fixed NED offset target with the leaders velocity FF. |
| `swarm_formation_test.py` | launches the formation, arms/takeoff, engages followers, flies the leader, logs tracks to CSV. |
| `swarm_formation_scale_test.py` | Scale variant (up to 253 followers): staggered launch, concurrent bring up, single thread logger. See [SCALING.md](SCALING.md). |
| `swarm_formation_plot.py` | Static matplotlib plot of the track CSV (overhead paths + formation error). |
| `swarm_formation_viz.py` | Animated HTML replay of the CSV (large swarm mode + `--from/--to` cropping). |

The follower logic depends on the `swarm:` Lua binding
(`swarm:count()`, `swarm:get_peer_location(sysid)`, `swarm:get_peer_velocity_NED(sysid)`),
added in `libraries/AP_Scripting/generator/description/bindings.desc` and backed
by `AP_SwarmMesh::get_peer_location()` / `get_peer_velocity_NED()`.

## Follower parameters (SCR_USER*)

The script sets the following parameters per follower:

| Param | Meaning |
|-------|---------|
| `SCR_USER1` | Leader mesh sysid (the leader's `MAV_SYSID`) |
| `SCR_USER2` | Offset North of the leader (m) |
| `SCR_USER3` | Offset East of the leader (m) |
| `SCR_USER4` | Formation altitude above home (m) |
| `SCR_USER5` | Expected peer count for "ready" (leader + other followers) |
| `SCR_USER6` | Engage flag: 0 = hold, ≥1 = keep formation (set after takeoff so the script doesn't fight the climb) |

Readiness is determined locally (peer count + a fresh leader fix) rather than signalled over the mesh for now (TODO).

## Setup

**1. Python deps.** The script needs `pymavlink`. the static PNG plot needs `matplotlib`

```bash
python3 -m venv ~/swarm-venv
~/swarm-venv/bin/pip install pymavlink matplotlib
```

**2. SITL binary.** Build once with scripting enabled:

```bash
./waf configure --board sitl && ./waf copter
```

The script auto-detects `build/sitl/bin/arducopter`. You only need to rebuild after changing C++ or `bindings.desc` edits to `swarm_follower.lua` load at runtime.

## Run

Run from the repo root. Point `--work-dir`/`--csv`. The script kills stale `arducopter` SITL processes,
launches the fleet, arms/takeoff, engages followers, flies the leader, and
logs every vehicle's track to the CSV.

```bash
# 4-drone diamond flying a box trajectory
~/swarm-venv/bin/python libraries/AP_SwarmMesh/tools/swarm_formation_test.py \
    --followers 4 --radius 10 --alt 15 --speedup 2 \
    --leader-path box --leader-speed 1.5 --move-time 40 --converge-wait 15 \
    --work-dir ~/swarm_run --csv ~/swarm_run/track.csv

# animated HTML replay (open in any browser)
~/swarm-venv/bin/python libraries/AP_SwarmMesh/tools/swarm_formation_viz.py \
    ~/swarm_run/track.csv -o ~/swarm_run/formation.html

# or a static PNG
~/swarm-venv/bin/python libraries/AP_SwarmMesh/tools/swarm_formation_plot.py \
    ~/swarm_run/track.csv -o ~/swarm_run/formation.png
```

Key options: `--followers N`, `--radius` (ring radius, m), `--alt`, `--leader-path {north,L,box}`, `--leader-speed` (m/s), `--move-time` (s total, split across path legs), `--speedup` (SITL time multiplier. Note wall clock waits still apply, so the leader covers speedup × speed × time).

**Quick verification test:** `--followers 2 --speedup 5 --leader-path north --move-time 20`.

**Watch it live (optional):** while a run is going, attach a GCS to any instance's
MAVLink port — leader = `tcp:127.0.0.1:5760`, followers = 5770/5780/5790/5800
(`+10` per instance) — e.g. `mavproxy.py --master tcp:127.0.0.1:5760`. Per-instance
SITL logs are under `<work-dir>/inst_*/sitl.log`.

## Results

With velocity FF, a 4 drone diamond holds its formation to within ~0.2 m (lag bias ~0.17 m, jitter rms ~0.12 m) while the leader manoeuvres.

## Notes / next steps

- Velocity FF (`swarm:get_peer_velocity_NED`) is on by default. It falls back to a position target if the EKF origin or leader velocity is unavailable. Could add accel FF.
- Altitude is held fixed (ABOVE_HOME). Should extend to full 3D by tracking the leades altitude instead.
- Coordination fields (`role`, `formation_slot`) are not yet on the TX path. Leader identity and readiness are currently determined locally. The sender should set a readiness variable itself for robustness.
