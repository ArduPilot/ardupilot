# Hi-Link HLK-LD2451 Proximity Radar Driver

ArduPilot `AP_Proximity` backend for the Hi-Link HLK-LD2451 24GHz FMCW
motion/speed radar (vehicle/intruder detection radar, UART output), plus
two different ways to exercise it in SITL without hardware.

See [HLK-LD2451_data.md](HLK-LD2451_data.md) for the sensor's UART protocol
as reverse-engineered from the datasheet and live capture (the *hardware's*
ICD), and the bundled manufacturer PDF/DXF for electrical specs. **Note:**
the actual driver/simulator pair in this repo uses a slightly different
frame layout than that ICD document (5-byte target blocks with one shared
`alarm` byte, not 6-byte blocks with a per-target alarm) — see "How it
works" below. The driver's own header comment
(`AP_Proximity_HLK_LD2451.h`) is the authoritative protocol reference.

## How it works

**On real hardware:** the sensor streams frames over UART (115200 8N1):
a fixed header, a little-endian length field, a payload (`count`, a shared
`alarm` byte, then up to 3 target blocks of `angle, distance, direction,
speed, snr`), and a fixed tail. `AP_Proximity_HLK_LD2451::read_sensor_data()`
byte-scans for the header, reads the length, then buffers exactly that many
payload bytes before validating the tail — malformed frames are discarded
and the parser resyncs on the next header. `parse_frame()` only actually
*uses* `angle` and `distance` per target (converted to a vehicle-frame
angle via `PRX_ORIENT`/yaw-correction and pushed into `AP_Proximity`'s
obstacle database); `direction`/`speed`/`snr` are parsed for frame validity
but not consumed by avoidance logic today.

```
HLK-LD2451 (or a simulator) --UART/TCP--> AP_Proximity_HLK_LD2451 --> AP_Proximity database --> AVOID_*/OA_* (avoidance)
```

**In SITL**, the same driver code runs unmodified — only what's on the
other end of the serial port changes. Two options, in increasing order
of physical realism:

1. **Built-in native simulator** (`libraries/SITL/SIM_HLK_LD2451.cpp`) —
   generates real, correctly-framed HLK-LD2451 packets *inside* the SITL
   process itself, at two fixed body-frame probe angles (+45°/-30°),
   using SITL's own obstacle/fence map for distance (falls back to an
   oscillating synthetic distance if no obstacle is in range, so there's
   always a "moving" target to exercise the motion-detection logic). No
   second process to run.
2. **External Python frame generator** (`Tools/autotest/hlk_ld2451_sim.py`)
   — connects to SITL over TCP and streams scripted scenarios (a target
   approaching, a static target, multiple targets). Good for repeatable,
   deterministic test scenarios (this is what `test_hlk_ld2451.sh` uses).

A full Webots simulation (physically modelled rover, 4 corner radars,
real sensor-model physics) also exists, developed separately on Windows —
see `webots_rover.parm` below for the tuned parameter set it uses; it's
not wired into this repo as a submodule.

## Relevant files

| Path | Purpose |
|---|---|
| `libraries/AP_Proximity/AP_Proximity_HLK_LD2451.cpp/.h` | Flight-code proximity backend — the real driver, runs unchanged on hardware and in SITL |
| `libraries/SITL/SIM_HLK_LD2451.cpp/.h` | Built-in SITL simulator (option 1 above): generates frames from SITL's obstacle map, no external process |
| `Tools/autotest/hlk_ld2451_sim.py` | External scripted frame generator (option 2 above), speaks the UART protocol over TCP |
| `Tools/autotest/test_hlk_ld2451.sh` | Automated end-to-end SITL test using option 2 |
| `webots_rover.parm` | Tuned `rover.parm` for the separate (Windows-only) Webots rover project — see below |

## Parameters

| Param | Value | Meaning |
|---|---|---|
| `PRX1_TYPE` | `19` | Selects the HLK-LD2451 backend |
| `SERIALx_PROTOCOL` | `11` | Proximity protocol on whichever `SERIALx` the sensor is wired to |
| `SERIALx_BAUD` | `115` | 115200 baud, matching the radar's UART config |

Any free `SERIALx` works — the examples below use SERIAL4/5/2 in different
places for historical reasons; what matters is that `PRX1_TYPE=19` and the
protocol/baud line up with whichever port the frames actually arrive on.

## Running in SITL

### Option 1 — built-in simulator (fastest, single process)

```bash
cd /path/to/ardupilot
./waf configure --board sitl && ./waf rover   # build ArduRover if not already built
Tools/autotest/sim_vehicle.py -v Rover --map --console \
  -A "--serial5=sim:hlk-ld2451" \
  --add-param-file=<(echo -e "PRX1_TYPE 19\nSERIAL5_PROTOCOL 11\nSERIAL5_BAUD 115")
```

No second terminal needed — SITL generates its own valid frames internally.
Watch `param show PRX1_TYPE` and `SYS_STATUS` in MAVProxy as usual.

### Option 2a — automated test script (recommended for CI/regression)

```bash
Tools/autotest/test_hlk_ld2451.sh
```

Starts ArduRover SITL with the radar wired to SERIAL4, launches
`hlk_ld2451_sim.py` to feed synthetic target frames over TCP, then uses
pymavlink to confirm `PRX1_TYPE` reads back correctly and the proximity
sensor shows up healthy in `SYS_STATUS`. Prints `[PASS]`/`[FAIL]` and
cleans up all background processes on exit.

### Option 2b — interactive (MAVProxy map/console)

Terminal 1 — start SITL with the radar wired to SERIAL4:

```bash
sim_vehicle.py -v Rover --map --console \
  -A "--serial4=tcp:5765" \
  --add-param-file=<(echo -e "PRX1_TYPE 19\nSERIAL4_PROTOCOL 11\nSERIAL4_BAUD 115")
```

Terminal 2 — feed it simulated radar targets:

```bash
python3 Tools/autotest/hlk_ld2451_sim.py --host 127.0.0.1 --port 5765 --scenario approach
```

`--scenario` options (see `hlk_ld2451_sim.py`):
- `approach` (default) — a target closing in on the vehicle
- `static` — a stationary target (constant distance; still motion-eligible per the frame protocol, unlike real hardware which wouldn't report it at all — see the Doppler note below)
- `multi` — multiple simultaneous targets

In MAVProxy: check `param show PRX1_TYPE`, watch `SYS_STATUS` for the
proximity bit, or use the map's proximity/object-avoidance overlay to see
targets reported by the sim.

### Testing against the Webots rover (developed separately on Windows)

A full Webots simulation (physically modelled rover, 4 corner radars, real
sensor-model physics) is developed and run directly on Windows — it's not
wired into this repo. [`webots_rover.parm`](webots_rover.parm) is a copy of
its tuned `rover.parm` (navigation gains, servo mapping, `PRXn_TYPE 19` +
`PRXn_YAW_CORR` for the radars) kept here for reference; sync it manually
if the Windows-side config changes. From `ashoka`, SITL is started the same
way regardless of what's driving the FDM on the other end:

```bash
python3 Tools/autotest/sim_vehicle.py -v Rover --model webots-python \
  --sim-address=<windows-host-ip> \
  --custom-location=28.5016472,77.3921611,0,0 \
  --add-param-file=docs/HLK-LD2451/webots_rover.parm \
  --out=<windows-host-ip>:14550 \
  -w --console --map
```

- `--out=<windows-host-ip>:14550` adds a link for a GCS — point QGroundControl /
  Mission Planner at UDP 14550.
- `-w` wipes and reloads params cleanly from the file — without it, stale
  params from a previous run can silently override what's in the file
  (this is exactly what caused the "no vehicle on GCS" GPS investigation:
  a stale saved param dump had `GPS1_TYPE`/`PRX1-4_TYPE`/`ARMING_SKIPCHK`
  all reverted to broken defaults).

The radar bridge (Webots → real HLK-LD2451 UART frames → SITL) is a
Windows-side script, part of that separate project — not in this repo.

**Gotcha:** if arming fails with `PreArm: 3D Accel calibration needed` even
though you set `ARMING_CHECK 0`, that's because ArduPilot 4.7 renamed
`ARMING_CHECK` to `ARMING_SKIPCHK` with **inverted** bitmask semantics (bits
= checks to skip; default `0` = skip nothing). The old name silently no-ops
now. Use `ARMING_SKIPCHK -1` instead (already set correctly in
`webots_rover.parm`), or live-fix a running SITL with
`param set ARMING_SKIPCHK -1` in MAVProxy.

## Motion-only behaviour

The real HLK-LD2451 is Doppler/FMCW-based and **only reports moving
targets** — a stationary object never appears in a frame at all, regardless
of distance. This matters for all three SITL options: the built-in
simulator's obstacles must actually move (or use its oscillating fallback);
`hlk_ld2451_sim.py`'s `static` scenario is a simplification (constant
distance, still framed as a target — real hardware wouldn't report it);
and in Webots, only `target_car` (a Solid with `radarCrossSection` set,
scripted to sweep back and forth) is ever detected — static geometry like
walls or rocks isn't, by design, matching the real sensor.

## Requirements

- ArduRover SITL binary built at `build/sitl/bin/ardurover`
- Python 3 with pymavlink (`modules/mavlink/pymavlink`)
