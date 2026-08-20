# Scaling the Leader-Follower Formation to the Full Sysid Range (254 nodes)

This document records what it took to scale the SITL leader-follower formation demo (see [README.md](README.md)) from a handful of drones to the full sysid range (1 leader + 253 followers = 254 ArduCopter SITL processes on a single host) the bugs that surfaced, what was learned, and how to drive the scale test.

## Result

On one 14-core / 48 GB laptop:

| Followers | Armed & airborne | Tracking a moving leader | Formation error (median) |
|----------:|-----------------:|-------------------------:|-------------------------:|
| 6         | 100%             | 100%                     | ~0.5 m |
| 40        | 100%             | 100%                     | **0.81 m** |
| 253       | 253/254          | 236/252 (94%)            | **0.87 m** (p90 5.3 m) |

A near full sysid range swarm of real ArduCopter instances holds sub-meter leader-follower formation off a moving leader over the decentralized mesh. Getting there required fixing several bugs.

---

## The bugs

### 1. Mesh partition by RTC clock skew — `deadline_ms == 0`

**Symptom:** followers split into "tracking" and "idle" groups by boot order (about half never saw the leader).

**Cause:** `AP_SwarmMesh_Backend::process_packet()` rejected a packet as stale when `origin_time_us + deadline_ms < now`. Every TX path sends with `deadline_ms = 0`, so the budget was zero. This means a packet was "stale" the instant the receiver's RTC read ahead of the sender's. With each SITL instance booting at a slightly different times, a node silently dropped all traffic from any peer whose clock was behind its own.

**Fix** ([AP_SwarmMesh_Backend.cpp](../AP_SwarmMesh_Backend.cpp), ~line 282): `deadline_ms == 0` now means *"no freshness budget"* and skips the staleness check entirely.

### 2. RX drain starvation — fixed per-tick byte budget

**Symptom:** at scale, peer tables never filled (nodes stopped hearing peers).

**Cause:** the backend drained at most `1024` bytes/tick through the parser. At *N* peers the mesh delivers ~*N* × stream-rate packets/second. Once that exceeds the drain budget, the transport buffer backs up.

**Fix:** the budget is now `AP_SWARMMESH_RX_BUDGET_BYTES` ([AP_SwarmMesh_config.h](../AP_SwarmMesh_config.h)), scaled by board type (16384 on SITL, 1024 elsewhere) so it can exceed the arrival rate of a large swarm.

### 3. SITL multicast socket buffer — `SO_RCVBUF`

**Symptom:** at 254 nodes, peer tables converged to only ~half the swarm even with (2) fixed (the kernel was dropping ~50% of multicast traffic).

**Cause:** the SITL mesh socket used the default ~256 KB receive buffer. At 254 senders (~600+ pkt/s per node) on a heavily flooded host, the buffer overflowed between drains and the kernel discarded datagrams.

**Fix:** ([AP_SwarmMesh_SITL.cpp](../AP_SwarmMesh_SITL.cpp)): `setsockopt(SO_RCVBUF, 4 MB)` on the multicast socket. This was the single biggest help for table convergence at scale.

### 4. Freshness / position **decoupling**

**Symptom:** at 40+ followers, followers froze at their slots while the leader flew then jumped to catch up, yet the follower's telemetry read `lf=1 age=0` ("leader fresh, zero age") the entire time.

**Cause:** `get_peer_location()` gates on `PeerState.freshness`, which is `(now - last_heard) <= 1 s`. But `last_heard` is bumped by any packet from that peer (including its 1 Hz heartbeat) while `global_pos` only updates on a `GLOBAL_POSITION_INT`. Under mesh load these decouple, the heartbeat keeps the freshness flag true while the position value goes stale by tens of seconds, so the follower goes to where the leader was.

**Fix:**
leader→follower formation, followers never need to broadcast their own position, each only consumes the leader. Yet 40 followers each broadcasting position at 2 Hz flooded the mesh and delayed the leader's updates past the point of useful tracking. Silencing follower position broadcasts (`--sr-pos 0`, now the scale test default) restored real time tracking: **40 followers went from ~27 m / freezing to 0.81 m; the full 254 went from 19 m to 0.87 m — a 22× improvement.**

**Two takeaways for the library:** (a) mesh freshness should be *per-message-class* (or messages should carry their own recency), so a consumer can tell a fresh flag from a fresh value; (b) a node should only produce the streams the swarm actually consumes.

---

## Orchestration lessons (in `swarm_formation_scale_test.py`)

Hundreds of flying SITL instances break every assumption of the small fleet orchestrator. What was needed:

- **Phyllotaxis (sunflower) packing** instead of a single ring — 253 followers on a 10 m ring would sit 0.25 m apart; the spiral spreads them at a set nearest-neighbour spacing (~6 m, outer radius ~64 m for 253).
- **Staggered launch** in batches to avoid a boot time CPU overload.
- **Concurrent bringup** (thread pool): connect + wait-for-EKF + arm + takeoff run in parallel. Done serially, 254 vehicles take tens of minutes.
- **Reconnect across the `-w` wipe reboot**: SITL reboots once after loading defaults, dropping the first TCP connection; bringup must reconnect.
- **One `selectors`-based logger** registering vehicles incrementally as they arm. one python thread per vehicle starves on the GIL, and an unread socket fills its TCP buffer and gets dropped by SITL.
- **Follower self-gate on altitude** instead of a runtime "go" flag, a flag broadcast to hundreds of vehicles is silently lost for some. Each follower's own AGL is always available onboard.
- **Follower side leader cache** (`LEADER_HOLD_MS`) so a momentary freshness gap doesn't drop a follower out of formation.

---

## Using `swarm_formation_scale_test.py`

### Setup

```bash
python3 -m venv ~/swarm-venv
~/swarm-venv/bin/pip install pymavlink matplotlib
./waf configure --board sitl && ./waf copter        # build the SITL binary once
```

(If a rebuild seems to do nothing, see the iCloud/`waf` note in the parent repo memory — install `empy`/`pexpect`, put that python on `PATH`, reconfigure.)

### Run

Point `--work-dir`/`--csv` **outside** any cloud-synced folder, and wrap in `caffeinate -i` so closing the lid doesn't kill a long run.

```bash
# Full 254-node run (~35-40 min). --sr-pos 0 is the default and is essential: followers stay silent so the leader's position stays fresh.
caffeinate -i ~/swarm-venv/bin/python \
  libraries/AP_SwarmMesh/tools/swarm_formation_scale_test.py \
    --followers 253 --spacing 4 --alt 15 --speedup 1 \
    --leader-sr 8 --leader-path box --leader-speed 2 \
    --move-time 120 --converge-wait 120 \
    --work-dir ~/swarm_run --csv ~/swarm_run/track.csv

# A fast, tight sanity run (~8 min): 40 followers
caffeinate -i ~/swarm-venv/bin/python \
  libraries/AP_SwarmMesh/tools/swarm_formation_scale_test.py \
    --followers 40 --spacing 4 --alt 15 --leader-path box --move-time 60 \
    --work-dir ~/swarm_run --csv ~/swarm_run/track.csv
```

### Key options

| Option | Meaning |
|--------|---------|
| `--followers N` | number of followers (leader is added automatically) |
| `--spacing M` | phyllotaxis nearest-neighbour spacing, metres |
| `--sr-pos HZ` | **follower** position broadcast rate. **Default 0** — keep it there for leader-follower; non-zero floods the mesh and reintroduces the freshness/staleness bug at scale |
| `--leader-sr HZ` | **leader** position broadcast rate (default 5; 8 at 254) |
| `--leader-path {north,L,box}` | leader trajectory |
| `--leader-speed`, `--move-time` | leader velocity (m/s) and total flight time (split across legs) |
| `--converge-wait` | seconds to let the mesh settle before moving (longer at scale) |
| `--batch-size`, `--batch-delay` | launch stagger |
| `--workers` | concurrent bring-up threads |
| `--arm-timeout`, `--climb-timeout` | per-vehicle waits (raise at scale — slow sim-time) |
| `--speedup` | keep at 1 at scale; >1 oversubscribes the host |

### Visualize

`swarm_formation_viz.py` auto-switches to a large-swarm mode (percentile error band instead of per-follower lines, no spokes/legend clutter). Crop to the flight window with `--from/--to` so the file isn't dominated by the long hover.

```bash
# find the leader's first move time in the run's stdout, or just crop the tail
~/swarm-venv/bin/python libraries/AP_SwarmMesh/tools/swarm_formation_viz.py \
    ~/swarm_run/track.csv -o ~/swarm_run/formation.html --from <t0> --to <t1> --fps 12
open ~/swarm_run/formation.html
```

### Reading the results

The orchestrator prints per-follower `SFDBG` state near the end: `eng` (self-engaged / climbed), `cnt` (peers in table), `lf` (leader fix this tick), `age` (ms since the cached leader fix), `mode` (4 = GUIDED), `armed`. During the move it also prints a live `[move]` sample of a few followers — watch `age`: if it climbs, the leader is going stale (too much mesh traffic — check `--sr-pos`).