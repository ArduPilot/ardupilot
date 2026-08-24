# AP_SwarmMesh

A decentralized peer-to-peer (P2P) state dissemination layer for ArduPilot.

AP_SwarmMesh gives every vehicle in a swarm its own view of every other vehicle, maintained onboard, without a Ground Control Station (GCS) in the middle. Each node broadcasts a small, user-selected stream of MAVLink messages over a P2P radio, and each node keeps a peer state table built from what it hear from its peers. Onboard Lua scripts or a companion computer, as well as `AP_LocationDB` and the dataflash log read that table instead of needing to talk to a central GCS.

The library is transport-agnostic, meaning framing, deduplication, freshness, forwarding, and peer table maintenance live in a shared backend, and a light transport driver. Currenlty, that driver is a UART serial port (`AP_SwarmMesh_Serial`) or a UDP multicast socket for simulation (`AP_SwarmMesh_SITL`).

**Start here for quick usage:** [Usage](#4-usage).

---

## Contents

1. [Background](#1-background)
2. [Architecture](#2-architecture)
3. [Implementation notes](#3-implementation-notes)
4. [Usage](#4-usage)
5. [Limitations](#5-limitations)
6. [Testing status](#6-testing-status)
7. [Future work](#7-future-work)
8. [Reference](#8-reference)

---

## 1. Background

AP_SwarmMesh was written for Google Summer of Code 2026 (mentors: Nate Mailhot and Asif Khan).

Swarm coordination in ArduPilot has historically been centralized. A GCS holds a link to every vehicle and coordination happens on the ground. This approach has a single point of failure, needs external computation, and in practice is finitely scalable.

The goal with AP_SwarmMesh is that each vehicle carries enough of the swarm's state to make its own decisions:

- **Peer state table.** A bounded table stored in memory of every peer's identity, kinematics, vehicle state, and coordination state which is updated as packets arrive.
- **Peer stream.** A user-configurable set of MAVLink messages broadcast to the swarm that's rate-limited per hardware class.
- **Peer frame header.** A fixed 23 byte routing header prepended to each MAVLink frame, carrying everything needed to deduplicate, expire, and forward a packet without parsing its payload.
- **Logging and snapshots.** Received peer telemetry is written to the dataflash log, and the peer table is periodically snapshotted to the onboard SD card to protect against reboots or lost connections.

See the GSoC development blog [here](https://discuss.ardupilot.org/t/gsoc-2026-ap-swarmmesh-resilient-mavlink-ad-hoc-swarm-networking-for-ardupilot/144105) for more details such as the wire format rationale and the full sysid range SITL results.

---

## 2. Architecture

```
                 ┌──────────────────────────────────────────────┐
   Lua script ──▶│  AP_SwarmMesh          (frontend, singleton) │
   Companion  ──▶│    • peer_state[] table                      │
                 │    • parameters, pruning, snapshots          │
                 │    • get_peer_location() / _velocity_NED()   │
                 │    • set_coord_state() / get_peer_coord_state│
                 └───────────────────┬──────────────────────────┘
                                     │
                 ┌───────────────────▼──────────────────────────┐
                 │  AP_SwarmMesh_Backend                        │
                 │    • RX: parse_byte() → process_packet()     │
                 │      → handle_mavlink() → peer table         │
                 │    • TX: stream buckets → send_mavlink()     │
                 │    • dedup, freshness, TTL, forwarding       │
                 └───────┬───────────────────────────┬──────────┘
                         │                           │
        ┌────────────────▼────────┐   ┌──────────────▼──────────────┐
        │ AP_SwarmMesh_Serial     │   │ AP_SwarmMesh_SITL           │
        │ UART → P2P radio        │   │ UDP multicast 239.65.83.0   │
        └─────────────────────────┘   └─────────────────────────────┘
```

The frontend/backend split follows `AP_Beacon`. Everything protocol related lives in `AP_SwarmMesh_Backend` where a transport implements five virtual methods (`transport_ready/available/read/txspace/write`). Adding a new radio requires writing those five methods.

### Wire format

Each mesh packet is a fixed 23 byte header followed by an unmodified MAVLink frame ([AP_SwarmMesh_packet.h](AP_SwarmMesh_packet.h)):

| Field | Type | Purpose |
|---|---|---|
| `stx1`, `stx2` | `uint8_t` (×2) | Sync bytes `0xAD 0xBC` |
| `version` | `uint8_t` | Header version (currently 1) - mismatches are dropped |
| `type` | `uint8_t` | Payload type; `0` = MAVLink. Lets a packet be routed without parsing it |
| `flags` | `uint8_t` | Bit 0 (`SWARMMESH_NO_RTC`): sender had no GPS synced clock |
| `origin_id` | `uint8_t` | System ID of the node that created the packet |
| `dest_id` | `uint8_t` | `0` = broadcast, otherwise targeted delivery |
| `prev_id` | `uint8_t` | System ID of the most recent forwarding node |
| `ttl` | `uint8_t` | Hop budget - decremented on each relay |
| `seq` | `uint16_t` | Per-origin sequence number, used for deduplication |
| `origin_time_us` | `uint64_t` | Creation time - GPS UTC microseconds |
| `deadline_ms` | `uint16_t` | Freshness budget - `0` means no budget |
| `payload_len` | `uint8_t` | Length of the MAVLink frame that follows |
| `crc` | `uint8_t` | Byte sum over `stx1`…`payload_len` |

The header is fixed length and self-validating, so the receive path can reject a bad, duplicate, stale, or packets not meant for us before handing bytes to the MAVLink parser. The payload keeps its own MAVLink CRC.

### TX path

`AP_SwarmMesh::update()` runs at 100 Hz from the vehicle scheduler. Each tick:

1. A `HEARTBEAT` goes out at a fixed 1 Hz, independent of every stream rate.
2. Each stream bucket whose interval has elapsed is sent out. Bucket intervals are clamped to the hardware profile (`P2P_HW_MASK`): 200 Hz for a 'Full' radio, 10 Hz for 'Lite' (default).
3. `send_mavlink()` serializes the message, checks `transport_txspace()` (dropping and counting the packet if the transport is backed up), stamps the header, and writes header+payload as a single buffer so datagram transports see exactly one packet.

The vehicle's `MAV_SYSID` is its mesh identity. It is the `origin_id`, the sysid the peer stream is packed with, and the key of its peer table entry. There is no separate mesh ID parameter.

### RX path

Bytes are drained from the transport through `parse_byte()`, a state machine (`WAIT_SYNC1 → WAIT_SYNC2 → HEADER → PAYLOAD`), bounded by `AP_SWARMMESH_RX_BUDGET_BYTES` per tick. A complete packet goes to `process_packet()`, which applies, in order:

1. **Version check** — drop unknown header versions.
2. **Self origin check** — drop our own packets echoed back by a shared bus.
3. **Peer lookup / allocation** — subject to the neighbourhood allowlist and table size.
4. **Deduplication** — a sliding sequence window (32 entries) (`last_seq` + `seq_seen_mask`).
5. **Staleness** — only if the sender was GPS synced, `deadline_ms != 0`, and we are GPS synced.
6. **TTL** — drop `ttl == 0`.
7. **Routing** — `dest_id == 0` is consumed locally; `dest_id == our sysid` is consumed locally; anything else is relayed with `ttl-1` and `prev_id` rewritten, and is not consumed.
8. **Delivery** — the raw frame is optionally mirrored to `P2P_FWD_PORT`, then fed to a MAVLink parser, and each decoded message updates the peer entry.

---

## 3. Implementation notes

Things that are not obvious from the headers, and that were mostly learned from trial and error.

### Freshness is per message type, not per peer

The first design had a single `last_heard` timestamp per peer. This resulted in the 1 Hz heartbeat keeping a peer marked "fresh" while its position, for example, is tens of seconds old. In the leader-follower experiment this showed up as followers flying to where the leader used to be, while reporting a fresh leader the whole time.

`PeerState` now carries `last_heard_ms[NUM_FRESH_TYPES]` and a `freshness` bitmask, one bit per tracked message type, with per type budgets in `FRESH_BUDGET_MS` ([AP_SwarmMesh_Backend.cpp](AP_SwarmMesh_Backend.cpp)):

| Type | Budget | Type | Budget |
|---|---|---|---|
| `HEARTBEAT` | 3 s | `EXTENDED_SYS_STATE` | 5 s |
| `SYS_STATUS` | 3 s | `ATTITUDE` | 2 s |
| `GLOBAL_POSITION_INT` | 10 s | `EKF_STATUS_REPORT` | 5 s |
| `LOCAL_POSITION_NED` | 2 s | `SCALED_IMU` | 2 s |
| `POSITION_TARGET_GLOBAL_INT` | 3 s | `COORDINATION` | 3 s |

Accessors gate on the right bit (`get_peer_location()` requires the `GLOBAL_POSITION_INT` bit, `get_peer_velocity_NED()` accepts either position bit, `get_peer_coord_state()` requires the `COORDINATION` bit). `freshness == 0` means the peer is dead, and pruning removes it.

### `deadline_ms == 0` means no budget

Every built in stream currently sends `deadline_ms = 0`. Originally a zero budget meant "expires immediately", so a packet was stale the instant the receiver's clock read ahead of the sender's. Since every vehicle boots with a slightly different clock, this silently partitioned the mesh into nodes that could hear each other and nodes that could not, by boot order. `0` now means no freshness budget and skips the check entirely.

The staleness check additionally requires both ends to have a GPS RTC -- the sender stamps `SWARMMESH_NO_RTC` in `flags` when it does not, and the receiver skips the check when it does not. GPS UTC is the only clock shared across the mesh. As a result the check is effectively inert in SITL (each instance runs its own sim clock) and inert for all stock streams. It is infrastructure for sensitive latency traffic that a future sender opts into.

### Broadcast is single hop. Only directed traffic is relayed

This is a deliberate design point. Only a packet addressed to somebody else gets relayed.

The reason is that the primary mode is state dissemination over a shared broadcast medium, where every node already hears every other node in range, and relaying broadcasts would multiply the offered load by the swarm size. Multi-hop is an additional feature to address a packet to a specific `dest_id`, and intermediate nodes relay it.

Note that with *N* nodes in range, a directed packet produces up to *N-1* relays. Keep `P2P_TTL` small for directed traffic.

### Memory scales with the board

`sizeof(PeerState)` is ~168 bytes. `AP_SWARMMESH_MAX_PEERS` is chosen from `HAL_MEM_CLASS` ([AP_SwarmMesh_config.h](AP_SwarmMesh_config.h)) and a board can override it in hwdef:

| `HAL_MEM_CLASS` | Max peers | Table size |
|---|---:|---:|
| ≥ 1000 (H7, SITL, Linux, QURT) | 255 | ~43 KB |
| ≥ 500 | 128 | ~21 KB |
| ≥ 300 | 64 | ~11 KB |
| ≥ 192 | 18 | ~3 KB |
| below that | 8 | ~1.3 KB |

The table is allocated statically at its compile time max. `P2P_SWARM_SIZE` bounds how many entries are used, but does not reduce the footprint.

### The RX budget is the practical load limit

`AP_SWARMMESH_RX_BUDGET_BYTES` caps how many bytes the parser drains per tick: 1024 on hardware, 16384 in SITL. At the 100 Hz task rate that is ~100 kB/s on hardware and ~1.6 MB/s in SITL. If the mesh delivers more than that, the transport's buffer backs up and the peer table starves.

Note that the Copter scheduler entry declares a 50 µs expected time for `update()`, which is optimistic for a full drain. If you raise the RX budget on a real board, check the `PM` log for scheduler overruns.

### SITL transport

`AP_SwarmMesh_SITL` puts every instance on UDP multicast group `239.65.83.0:57733`, which is an ideal shared bus meaning no loss, no range. However:

- The socket needs a 4 MB `SO_RCVBUF`. The default ~256 KB overflows between drains at high node counts and the kernel silently discards roughly half the traffic, so peer tables converge to about half the swarm. This was the main fix for large scale convergence.
- Because the bus is lossless, `SIM_SWARM_LOSS` exists to model an unreliable link. The draw is made per receiver, so a packet lost by one vehicle is still heard by the others (which is what an actual radio does).

The group address and port are fixed, so only one SwarmMesh SITL swarm can run per host.

### Coordination hook

The library carries `role`, `task_id`, `formation_slot`, `priority`, target position/velocity/accel, and up to `AP_SWARMMESH_COORD_USER_MAX` (32) opaque user bytes (assigning no default meaning to them). Whatever coordinates the swarm onbaord (Lua script or companion computer) must assign these values.

The basket travels as a MAVLink `TUNNEL` message ([AP_SwarmMesh_coord.h](AP_SwarmMesh_coord.h)), which is what makes it work from both directions: an onboard Lua script publishes via `swarm:set_coord_state()`, and a companion computer publishes by sending the same `TUNNEL` over its ordinary telemetry link, where `GCS_MAVLink` routes it into the mesh. Received baskets come back out through `P2P_FWD_PORT` unmodified.

Vehicles built with different `AP_SWARMMESH_COORD_USER_MAX` values are still compatible. A receiver keeps what fits and reports how much it kept in `user_len`.

> The `TUNNEL` payload type is currently `32768`, in the experimental range. See [Future work](#7-future-work).

### AP_LocationDB integration

When `AP_LOCATIONDB_KEYDOMAIN_MAVLINK_ENABLED` is set, every received `GLOBAL_POSITION_INT` is also pushed into `AP_LocationDB` under a MAVLink domain key built from the peer's sysid, compid, and msgid, populating position, velocity, and heading. That makes mesh peers visible to any LocationDB consumer (following, tracking, avoidance) without those consumers knowing about the mesh.

Only `GLOBAL_POSITION_INT` is published currently, and the item is dropped if the vehicle has no EKF origin (`get_vector_from_origin_NEU()` fails). Covered by the `Copter.SwarmMeshLocationDB` autotest.

### Persistence

With filesystem write support, the peer table is snapshotted to `/APM/PEERS/peers.dat` at `P2P_SAVE_HZ` (capped to 10 Hz) and restored at `init()`. Only alive entries are written, and restored entries come back with `freshness == 0` identity and last known state (ie. the snapshots are automatically considered stale). Because pruning deletes entries with `freshness == 0`, a restored peer is dropped again within `P2P_PRUNE_SECS` unless it is actually heard from.

The save path is not atomic so power loss while writing can leave a partial file, which the magic/version/size check rejects on the next boot.

---

## 4. Usage

### 4.1 What the system is for

The primary and hardware tested mode is P2P state broadcast dissemination. Every node broadcasts a small stream (`P2P_DESTID = 0`) and every node builds a table of every peer it can hear. Formation flight, collision awareness, distributed task allocation, and swarm health monitoring are examples of possible applications of this system.

Two secondary modes are supported:

- **Targeted delivery** — set `P2P_DESTID` to a peer's sysid and your streams go to that peer alone.
- **Multi-hop forwarding** — directed packets are relayed by intermediate nodes until `ttl` runs out, so a node can reach a peer that is out of direct radio range.

### 4.2 IMPORTANT

> **Broadcast only what the swarm actually consumes.**

Critically, every node's transmissions are received and parsed by every other node, so total mesh load grows as *N* × *N* × rate. At 40 followers, leaving each follower's position stream on at 2 Hz pushed formation error from **0.81 m to ~27 m**, with followers freezing and snapping. At 254 nodes it was 0.87 m versus 19 m. Silencing one unused stream was a 22× improvement.

The default for every `P2P_SR_*` parameter is therefore `0`. Turn on one stream, confirm you need it, then consider the next.

### 4.3 Build and enable

AP_SwarmMesh is compiled in when `HAL_PROGRAM_SIZE_LIMIT_KB > 2048` (any 2 MB+ board). It is currently wired into ArduCopter only (`ArduCopter/Copter.cpp` schedules `update()` at 100 Hz and `ArduCopter/system.cpp` calls `init()`). Other vehicles will need to be added see [Future Work](#7-future-work).

```bash
./waf configure --board sitl && ./waf copter
```

Parameters use the `P2P_` prefix on Copter.

### 4.4 Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `P2P_TYPE` | 0 | Backend. `0` = off, `1` = Serial, `10` = SITL multicast |
| `P2P_SR_POSITION` | 0 | Hz — `GLOBAL_POSITION_INT`, `LOCAL_POSITION_NED` |
| `P2P_SR_EXT_STAT` | 0 | Hz — `SYS_STATUS`, `NAV_CONTROLLER_OUTPUT`, `POSITION_TARGET_GLOBAL_INT` |
| `P2P_SR_EXTRA1` | 0 | Hz — `ATTITUDE`, `EKF_STATUS_REPORT`, `SCALED_IMU`, `EXTENDED_SYS_STATE` |
| `P2P_SR_COORD` | 0 | Hz — the coordination basket (nothing sent until first published) |
| `P2P_SWARM_SIZE` | 0 | Max peer table entries used. `0` = the board's compile time max |
| `P2P_DESTID` | 0 | Destination for our streams. `0` = broadcast |
| `P2P_TTL` | 255 | Hop budget stamped on our packets |
| `P2P_HW_MASK` | 0 | Bit 0 = 'Full' radio (200 Hz cap). Clear = 'Lite' (10 Hz cap) |
| `P2P_LOG_HZ` | 50 | Combined RX dataflash write rate. `0` disables RX logging |
| `P2P_LOG_MASK` | 0x3FF | Which RX message types are logged |
| `P2P_SAVE_HZ` | 1 | Peer snapshot rate to `/APM/PEERS/peers.dat`. `0` disables |
| `P2P_PRUNE_SECS` | 10 | Stale entry prune interval. `0` disables |
| `P2P_PEER_01`…`_16` | 0 | Neighbourhood allowlist. All zero = accept any peer |
| `P2P_FWD_PORT` | -1 | Serial port to mirror received peer MAVLink to. `-1` disables |

`SIM_SWARM_LOSS` (SITL only) sets the percentage of incoming mesh packets each vehicle discards. It is read live, so loss can be introduced and cleared mid flight.

### 4.5 SITL

SITL is where the library has been tested at scale, up to the full sysid range (`P2P_TYPE = 10`).

**Minimum working setup** — a leader broadcasting position, followers listening:

```bash
# on every instance
param set P2P_TYPE 10
param set MAV_SYSID <unique per vehicle>

# on the leader only
param set P2P_SR_POSITION 5
```

Followers set `P2P_SR_POSITION 0`. They still emit the 1 Hz heartbeat, so every peer table still populates but the mesh isn't flooded.

**The orchestrated experiments** live in [tools/](tools/) and are documented in [tools/README.md](tools/README.md) and [tools/SCALING.md](tools/SCALING.md):

```bash
python3 -m venv ~/swarm-venv
~/swarm-venv/bin/pip install pymavlink matplotlib
```

```bash
# 40 followers, ~8 minutes (recommended)
caffeinate -i ~/swarm-venv/bin/python \
  libraries/AP_SwarmMesh/tools/swarm_formation_scale_test.py \
    --followers 40 --spacing 4 --alt 15 --leader-path box --move-time 60 \
    --work-dir ~/swarm_run --csv ~/swarm_run/track.csv
```

```bash
# full 254 node run, ~35-40 minutes
caffeinate -i ~/swarm-venv/bin/python \
  libraries/AP_SwarmMesh/tools/swarm_formation_scale_test.py \
    --followers 253 --spacing 4 --alt 15 --speedup 1 \
    --leader-sr 8 --leader-path box --leader-speed 2 \
    --move-time 120 --converge-wait 120 \
    --work-dir ~/swarm_run --csv ~/swarm_run/track.csv
```

```bash
# animated HTML replay of either run
~/swarm-venv/bin/python libraries/AP_SwarmMesh/tools/swarm_formation_viz.py \
    ~/swarm_run/track.csv -o ~/swarm_run/formation.html
```

Practical notes for large runs:

- Keep `--sr-pos 0` (the default). This is the rule from section 4.2.
- Keep `--speedup 1` past a few dozen instances, flooding the host changes the effective timing of state delivery and vehicle dynamics, so results are poor.
- The practical ceiling in a flying experiment is the simulation host, not the mesh. The routing and peer table layer was verified separately to the full 253 peers by injecting synthetic peers over UDP.

The functional tests do not need the orchestrator:

```bash
./Tools/autotest/autotest.py test.Copter.SwarmMesh test.Copter.SwarmMeshLocationDB
```

### 4.6 Hardware

Real deployments are markedly different. The constraint is now radio bandwidth and not the host CPU.

**Wiring.** Attach the P2P radio to a spare UART and set:

```
SERIALn_PROTOCOL = 51      # SwarmMesh
SERIALn_BAUD     = <your radio's rate>
P2P_TYPE         = 1       # Serial backend
P2P_HW_MASK      = 0       # Lite: caps stream rates at 10 Hz
MAV_SYSID        = <unique per vehicle>
```

> Protocol 51 is not yet listed in the `SERIALn_PROTOCOL` parameter metadata, so most GCS dropdowns will not offer it by name. Set the raw value 51.

Use your standard telemetry link for your GCS as usual, the mesh is a separate port.

**Sizing the streams.** A mesh packet is 23 bytes of header plus the MAVLink frame. Useful numbers:

| Message | On the wire |
|---|---:|
| `HEARTBEAT` | ~46 B |
| `GLOBAL_POSITION_INT` | ~65 B |
| `LOCAL_POSITION_NED` | ~65 B |
| `POSITION` bucket tick (both of the above) | ~130 B |

Note that the `POSITION` bucket currently sends both `LOCAL_POSITION_NED` and `GLOBAL_POSITION_INT` on every tick, so one tick costs ~130 B, not ~65 B. Load on the shared medium is roughly:

```
bytes/s  ≈  N_transmitting × Σ(bucket_rate_hz × bucket_bytes)  +  N_nodes × 46      # heartbeats
```

On a 57600 baud link (~5.7 kB/s of usable throughput), that means:

| Swarm | Streams on | Offered load | Verdict |
|---|---|---:|---|
| 5 nodes, leader broadcasts only | leader position @ 2 Hz | ~0.5 kB/s | comfortable |
| 5 nodes, all broadcasting | position @ 2 Hz each | ~1.5 kB/s | fine |
| 10 nodes, all broadcasting | position @ 2 Hz each | ~3.1 kB/s | near the limit |
| 10 nodes, all broadcasting | position @ 5 Hz each | ~7.0 kB/s | over budget - will drop |
| 20 nodes, all broadcasting | position @ 5 Hz each | ~13.9 kB/s | far over budget |

So on real radios, plan for single digit vehicle counts with one or two streams, and lean on `P2P_DESTID`/asymmetric rates so only the nodes whose state is consumed transmit it. Scale beyond that requires increasing the baudrate over the serial link or a higher bandwidth radio, and `P2P_HW_MASK` bit 0 to unlock rates above 10 Hz.

**Bounding the table on small boards.** On an F4-class board the table holds 18 peers, filled first come, first serve, which on a busy channel may not be the 18 you want. Pin them by setting`P2P_PEER_XX` to the sysids you want:

```
P2P_PEER_01 = 2
P2P_PEER_02 = 3
P2P_PEER_03 = 7
```

Any non zero slot switches the filter on so only listed sysids are tracked. Note that F4 boards are also forced to the Lite 10 Hz cap regardless of `P2P_HW_MASK`.

**Checking it works.** `SMST` in the dataflash log carries the backend counters (`CRCFail`, `Stale`, `TTL`, `Dedup`, `Drop`, `TXseq`, `TXfwd`, `TXdrop`). A healthy link shows `TXseq` climbing with `TXdrop` near zero and low `CRCFail`. Rising `TXdrop` means the radio cannot keep up with your stream rates. Rising `CRCFail` means line noise or a baud mismatch. Received peer telemetry is logged as `SMHB`, `SMGP`, `SMAT`, `SMCO`, etc.

### 4.7 Consuming the peer table

**From Lua** (`libraries/AP_Scripting/applets/swarm_follower.lua` is a complete worked example):

```lua
local leader = 1
local loc = swarm:get_peer_location(leader)          -- nil if unknown or position stale
local vel = swarm:get_peer_velocity_NED(leader)      -- nil if unknown or velocity stale
local age = swarm:get_peer_position_last_update_ms(leader)
local n   = swarm:count()
local sid = swarm:get_peer_sysid(0)                  -- iterate the table by index
```

Both accessors return `nil` rather than a stale value, so a script should cache the last good fix and fly it for a bounded hold time rather than dropping out of formation on a single missed update. Using the peer's velocity FF is best for performance, cutting steady state lag against a moving leader from ~1.4 m to ~0.17 m.

**Publishing coordination state** (`libraries/AP_Scripting/examples/swarm_coordination.lua`):

```lua
local state = SwarmCoordState()
state:role(2)
state:formation_slot(3)
state:user(0, 1)        -- opaque bytes, the library never interprets them
state:user_len(1)
swarm:set_coord_state(state)

local peer = swarm:get_peer_coord_state(swarm:get_peer_sysid(0))
if peer then
  gcs:send_text(6, string.format('peer slot %u', peer:formation_slot()))
end
```

Published state is rebroadcast at `P2P_SR_COORD` until replaced, so call `set_coord_state()` only when something changes. Nothing is transmitted until the first call.

**From a companion computer.** Set `P2P_FWD_PORT` to a MAVLink serial port. Every accepted peer frame is written out unmodified, preserving each peer's sysid, compid, sequence, and CRC, so the companion's parser sees the swarm as an ordinary multi-vehicle MAVLink stream. This is why every vehicle needs a distinct `MAV_SYSID`. The companion publishes coordination state back by sending the `TUNNEL` message over its normal telemetry link.

---

## 5. Limitations

Known and deliberate, in rough order of how likely they are to bite:

1. **Bandwidth is the binding constraint on hardware.** See section 4.6. Everything else is downstream of this.
2. **No delivery guarantee.** MAVLink is inherently lossy and there is currently no ACK/NACK, no retransmission, and no way to know whether a critical message arrived.
3. **Broadcast is single hop.** Relaying applies only to directed packets. A node out of direct range contributes nothing to a broadcaster's table.
4. **Relay is not currenlty routed.** Every node that hears a directed packet relays its first copy. Keep `P2P_TTL` low.
5. **No authentication or encryption.** Any node on the radio channel can inject packets under any `origin_id`. Do not deploy on a shared or contested channel.
6. **Peer allocation is first come, first serve.** Once the table is full, new peers are rejected until pruning frees a slot. Use the allowlist on smaller boards.
7. **Copter only currently.** Other vehicles need the scheduler/init/parameter integration.
8. **The staleness mechanism is inert currently.** All stock streams send `deadline_ms = 0`, and the check needs GPS UTC on both ends.
9. **One SwarmMesh swarm per host in SITL.** The multicast group and port are fixed.
10. **Snapshot writes are not atomic.** Power loss while writing leaves a partial file, rejected on the next boot.

---

## 6. Testing status

What has currently been validated:

| Layer | SITL | Hardware |
|---|---|---|
| Framing, CRC, parser state machine | ✅ up to 254 nodes | ✅ |
| Deduplication, TTL, relay | ✅ autotest + 254 node runs | ✅ |
| Peer table, per type freshness, pruning | ✅ | ✅ |
| Broadcast state dissemination | ✅ 0.87 m median formation error @ 253 followers | ✅ |
| Lua bindings, formation control | ✅ | ✅ |
| Coordination hook (`TUNNEL` basket) | ✅ | ✅ two vehicle coordinated trajectory |
| `AP_LocationDB` publishing | ✅ autotest | ⚠️ not hardware tested |
| `P2P_FWD_PORT` companion forwarding | ✅ | ✅ |
| **`AP_SwarmMesh_Serial` (UART backend)** | n/a | ❌ untested |
| Simulated packet loss (`SIM_SWARM_LOSS`) | ✅ autotest | n/a |

### SITL results

Leader flying a box trajectory, followers holding a phyllotaxis formation, all coordination over the mesh:

| Followers | Armed & airborne | Tracking a moving leader | Median formation error |
|---:|---:|---:|---:|
| 40 | 100% | 100% | 0.36 m |
| 253 | 253 / 254 | 236 / 252 (94%) | 0.87 m (p90 5.3 m) |

A separate decentralized glyph formation experiment (57 instances spelling `GSoC`, with onboard CBF separation filtering) held 56/56 cells at a median 0.02 m cell error with zero sampled separations below 1 m across 15 million pair samples. See [tools/Glyph_Formation_Experiment.md](tools/Glyph_Formation_Experiment.md).

### Hardware validation

AP_SwarmMesh has flown on real hardware as part of [ArduSwarm](https://github.com/kwakurichter/ArduSwarm) v2.1.1, a port of ArduPilot 4.7.0 for Bitcraze Crazyflie hardware ([ArduPilot_cus](https://github.com/kwakurichter/ArduPilot_cus)). That validated the whole protocol stack on real radios and real flight controller timing: framing, CRC, deduplication, freshness, forwarding, peer table maintenance, and companion computer coordination.

Two demos were flown: a leader-follower (a hovering leader relays movement commands to listening followers) and a two vehicle coordinated trajectory driven by exchanged coordination baskets.

Note that on ArduSwarm the mesh rides the Crazyflie's nRF51 P2P radio through a custom backend built on that project's `AP_Syslink` library (not `AP_SwarmMesh_Serial`). It implements the same five transport methods, but sends each packet as one nRF51 P2P broadcast (up to 251 bytes, immediate, unacked, never split) rather than a byte stream. So:

- Everything above the transport driver is hardware proven.
- `AP_SwarmMesh_Serial` itself is technically not flight tested, but its a thin wrapper over `uart->available/read/txspace/write` and should work. For first bring up on a UART radio, check `SMST` counters on the bench with two vehicles before trusting it in the air.
- Crazyflie hardware is not representative of a typical ArduPilot vehicle, so these results do not transfer directly to a standard flight controller with a standard telemetry radio.

---

## 7. Future work

### ACK/NACK synchronization

There is currently no way to know whether a critical message reached its target. The header already reserves what is needed (`type` can carry an ack packet, `seq` identifies the packet, `dest_id` identifies the sender to reply to). The plan is to let a message type be flagged critical, have the receiver return an ack, and let the origin track and report loss.

### ESP32 radio firmware

The library defines the flight controller half of the link while the radio half is currently under development. A reference ESP32 firmware (for `AP_SwarmMesh_Serial`), using the 23-byte header over UART protocol and an ESP-NOW/mesh channel over the air is still required.

### AP_LocationDB integration

The current integration is deliberately minimal: `GLOBAL_POSITION_INT` in, one MAVLink domain LocationDB item out, covered by `Copter.SwarmMeshLocationDB`. Open items:

- `AP_LocationDB` is not yet in ArduPilot master. This branch carries it, so the integration will need revalidation when the upstream library is merged and its APIs settle.
- Only position, velocity, and heading are published. Acceleration and the EKF variances the peer table still require integration.
- Items are dropped when the vehicle has no EKF origin, since keys are stored in NEU relative to it. A peer heard before origin is set is not recorded.
- Interaction with other LocationDB producers (ADS-B, `FOLLOW_TARGET`) has not yet been tested.
- Hardware testing of the LocationDB path is outstanding.

### Smaller items

- Register a real `MAV_TUNNEL_PAYLOAD_TYPE` block for the coordination basket and move off the experimental value `32768`.
- Other vehicles need to register `update()` in their respective main loops to use them.
- Add `51:SwarmMesh` to the `SERIALn_PROTOCOL` parameter metadata so it appears in GCS dropdowns.
- Extend the integration beyond Copter.
- `send_stream()` sends both `LOCAL_POSITION_NED` and `GLOBAL_POSITION_INT` in the `POSITION` bucket; it should skip the local one when a global fix is available.
- Make the peer snapshot write atomic (write then rename).
- Single source for the `MsgFresh` / `LogMsg` enum pair.

---

## 8. Reference

### Files

| File | Role |
|---|---|
| [AP_SwarmMesh.h](AP_SwarmMesh.h) / [.cpp](AP_SwarmMesh.cpp) | Frontend: peer table, parameters, accessors, snapshots, pruning |
| [AP_SwarmMesh_Backend.h](AP_SwarmMesh_Backend.h) / [.cpp](AP_SwarmMesh_Backend.cpp) | Protocol: parser, routing, freshness, streams, logging |
| [AP_SwarmMesh_packet.h](AP_SwarmMesh_packet.h) | The 23 byte peer header definition |
| [AP_SwarmMesh_coord.h](AP_SwarmMesh_coord.h) | Coordination basket wire format |
| [AP_SwarmMesh_Serial.h](AP_SwarmMesh_Serial.h) / [.cpp](AP_SwarmMesh_Serial.cpp) | UART transport |
| [AP_SwarmMesh_SITL.h](AP_SwarmMesh_SITL.h) / [.cpp](AP_SwarmMesh_SITL.cpp) | UDP multicast transport |
| [AP_SwarmMesh_PeerStorage.h](AP_SwarmMesh_PeerStorage.h) | Snapshot file format |
| [AP_SwarmMesh_config.h](AP_SwarmMesh_config.h) | Compile time sizing |
| [LogStructure.h](LogStructure.h) | `SMST`, `SMHB`, `SMGP`, `SMCO`, … log messages |
| [tools/](tools/) | SITL experiments, orchestrators, visualizers |

### Log messages

`SMST` (backend counters), `SMHB` (heartbeat), `SMSS` (sys status), `SMGP` (global position),
`SMLP` (local position), `SMPT` (position target), `SMES` (extended sys state), `SMAT` (attitude),
`SMEK` (EKF status), `SMIM` (scaled IMU), `SMCO` (coordination).

### Further reading

- [GSoC Post](https://discuss.ardupilot.org/t/gsoc-2026-ap-swarmmesh-resilient-mavlink-ad-hoc-swarm-networking-for-ardupilot/144105) — project blog: proposal, wire-format rationale, midterm/final results
- [tools/README.md](tools/README.md) — running the SITL experiments
- [tools/SCALING.md](tools/SCALING.md) — scaling to the full sysid range, and the bugs found doing it
- [tools/Glyph_Formation_Experiment.md](tools/Glyph_Formation_Experiment.md) — decentralized glyph formation, CBF separation filtering
- [ArduSwarm](https://github.com/kwakurichter/ArduSwarm) — the hardware platform AP_SwarmMesh was tested on