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

## Coordination hook

The mesh carries vehicle telemetry by itself, but it assigns nothing to `role`, `task_id`, `formation_slot` or `priority` -- those belong to whatever is coordinating the swarm. This hook lets an onboard Lua script or a companion computer fill them in and read back what every peer published. See `../../AP_Scripting/examples/swarm_coordination.lua`.

```lua
local state = SwarmCoordState()
state:role(2)
state:formation_slot(3)
state:user(0, 1)      -- bytes the library never interprets
state:user_len(1)
swarm:set_coord_state(state)

local peer = swarm:get_peer_coord_state(swarm:get_peer_sysid(0))
if peer then
  gcs:send_text(6, string.format('peer slot %u', peer:formation_slot()))
end
```

The published state is rebroadcast at `P2P_SR_COORD` Hz until it is replaced, so a script only calls `set_coord_state()` when something changes. Nothing is transmitted until the first call, so a vehicle with no coordination logic does not put an empty basket on the mesh. Received state is logged as `SMCO` and expires from `get_peer_coord_state()` like any other message type.

Alongside the named fields, `AP_SWARMMESH_COORD_USER_MAX` bytes (32 by default) travel opaquely for coordination the wire format does not name. Vehicles built with different max sizes are compatible: a receiver keeps what fits and reports how much it kept through `user_len()`.

A basket travels as a MAVLink `TUNNEL` message, which is what lets a **companion computer** use the same hook. It publishes by sending that TUNNEL over its ordinary telemetry link, AP_SwarmMesh intercepts it and broadcasts it to the swarm, and it receives the peers' baskets as `P2P_FWD_PORT` forwards peer traffic to the companion unmodified.

## Packet loss and relay

`SIM_SWARM_LOSS` sets the percentage of incoming mesh packets each vehicle discards, simulating an unreliable radio link. The draw is made per receiver, so a packet lost by one vehicle is still heard by the others, which is what a real radio link does and what a shared multicast bus on its own does not. The parameter is read live, so loss can be introduced and cleared mid-flight without a reboot:

```bash
# in a GCS attached to one instance
param set SIM_SWARM_LOSS 30
```

Relaying is observable from outside the fleet: a packet whose `dest_id` is neither broadcast (0) nor the receiving vehicle's own sysid is forwarded back onto the group with `ttl` decremented and `prev_id` set to the forwarding vehicle, so joining the multicast group from a script is enough to watch a packet travel.

Both behaviours are covered by the `Copter.SwarmMesh` autotest, which joins the mesh as an extra node, injects packets addressed to a third-party sysid, and checks that they come back relayed, that `ttl=0` packets do not, and that `SIM_SWARM_LOSS=50` removes roughly half of them:

```bash
./Tools/autotest/autotest.py test.Copter.SwarmMesh
```

Note the multicast group is fixed, so that test assumes no other SwarmMesh SITL instance is running on the same host.

## Notes / next steps

- Velocity FF (`swarm:get_peer_velocity_NED`) is on by default. It falls back to a position target if the EKF origin or leader velocity is unavailable. Could add accel FF.
- Altitude is held fixed (ABOVE_HOME). Should extend to full 3D by tracking the leades altitude instead.
- The follower applet still derives leader identity and readiness locally. Now that coordination state is on the TX path (see above), it could publish readiness in its own basket and let followers wait on the leader's `role` instead of counting peers.
