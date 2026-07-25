# Hi-Link HLK-LD2451 Proximity Radar Driver

ArduPilot `AP_Proximity` backend for the Hi-Link HLK-LD2451 24GHz FMCW
motion/speed radar, plus a SITL simulator so the driver can be exercised
without hardware.

See [HLK-LD2451_data.md](HLK-LD2451_data.md) for the UART frame protocol
(ICD), and the bundled manufacturer PDF/DXF for electrical specs.

## Relevant files

| Path | Purpose |
|---|---|
| `libraries/AP_Proximity/AP_Proximity_HLK_LD2451.cpp/.h` | Flight-code proximity backend |
| `libraries/SITL/SIM_HLK_LD2451.cpp/.h` | SITL model of the sensor |
| `Tools/autotest/hlk_ld2451_sim.py` | Standalone Python simulator that speaks the radar's UART protocol over TCP |
| `Tools/autotest/test_hlk_ld2451.sh` | Automated end-to-end SITL test |

## Parameters

| Param | Value | Meaning |
|---|---|---|
| `PRX1_TYPE` | `19` | Selects the HLK-LD2451 backend |
| `SERIAL4_PROTOCOL` | `11` | Proximity protocol on SERIAL4 |
| `SERIAL4_BAUD` | `115` | 115200 baud, matching the radar's default UART config |

(Any free `SERIALx` can be used — the test setup below uses SERIAL4.)

## Running in SITL

### Option A — automated test script (recommended)

```bash
cd /path/to/ardupilot
./waf configure --board sitl && ./waf rover   # build ArduRover if not already built
Tools/autotest/test_hlk_ld2451.sh
```

This starts ArduRover SITL with the radar wired to SERIAL4, launches
`hlk_ld2451_sim.py` to feed synthetic target frames over TCP, then uses
pymavlink to confirm `PRX1_TYPE` reads back correctly and the proximity
sensor shows up healthy in `SYS_STATUS`. Prints `[PASS]`/`[FAIL]` and
cleans up all background processes on exit.

### Option B — interactive (MAVProxy map/console)

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
- `static` — no motion targets (radar is Doppler-only, so this exercises the "no target" frame)
- `multi` — multiple simultaneous targets

In MAVProxy: check `param show PRX1_TYPE`, watch `SYS_STATUS` for the
proximity bit, or use the map's proximity/object-avoidance overlay to see
targets reported by the sim.

### Option C — full Webots simulation (real rover model + 4-radar rig)

For visual, physics-driven testing instead of synthetic frames, see the
companion Webots project
([kumarnitish378/ardupilot-webots-rover](https://github.com/kumarnitish378/ardupilot-webots-rover)):
a modelled Ackermann UGV with 4 corner radars feeding this exact driver
through `tools/radar_mav.py`, which speaks the real HLK-LD2451 UART protocol
(not a MAVLink stand-in). [`webots_rover.parm`](webots_rover.parm) in this
directory is a synced copy of that project's tuned `rover.parm`
(navigation gains, servo mapping, `PRX1_TYPE 19` on SERIAL2, etc.) so you
can use it directly without cross-referencing the other checkout:

```bash
Tools/autotest/sim_vehicle.py -v Rover --model webots-python \
  --sim-address=<windows-host-ip> \
  --custom-location=28.5016472,77.3921611,0,0 \
  --add-param-file=docs/HLK-LD2451/webots_rover.parm \
  --console --map
```

For a quick param-only smoke test without Webots (no `--model`/`--sim-address`,
just the internal SITL physics model):

```bash
Tools/autotest/sim_vehicle.py -v Rover -w \
  -P PRX1_TYPE=19 -P SERIAL2_PROTOCOL=11 -P SERIAL2_BAUD=115 -P SERIAL4_PROTOCOL=0
```

**Gotcha:** if arming fails with `PreArm: 3D Accel calibration needed` even
though you set `ARMING_CHECK 0`, that's because ArduPilot 4.7 renamed
`ARMING_CHECK` to `ARMING_SKIPCHK` with **inverted** bitmask semantics (bits
= checks to skip; default `0` = skip nothing). The old name silently no-ops
now. Use `ARMING_SKIPCHK -1` instead (already set correctly in
`webots_rover.parm`), or live-fix a running SITL with `param set
ARMING_SKIPCHK -1` in MAVProxy.

## Requirements

- ArduRover SITL binary built at `build/sitl/bin/ardurover`
- Python 3 with pymavlink (`modules/mavlink/pymavlink`)

