# ArduPlane fmuv3-hil — HIL Simulation with X-Plane

Hardware-In-the-Loop simulation using **fmuv3-hil** firmware and the
`mavlink_xplane.py` bridge script.  X-Plane provides full flight dynamics;
the bridge translates X-Plane data into MAVLink sensor injection messages
that drive the flight controller exactly as a real flight would.

---

## Architecture

```
X-Plane (192.168.1.10)           mavlink_xplane.py (Mac)           fmuv3-hil (USB)
────────────────────              ──────────────────────            ─────────────────
flight dynamics     DATA@ ──────► parse rows                       ArduPlane
  lat/lon/alt       49005         convert coords   HIL_SENSOR ────► SITL INS backend
  pitch/roll/hdg                  baro synthesis   GPS_INPUT  ────► GPS_MAV backend
  gyro / accel                    alt→pressure     VISION_POS ────► EKF3 attitude
  velocity NED                    joystick→RC      RC_OVERRIDE────► RC input
  joystick axes
                   ◄── DREF ────  servo→DREF     ◄── SERVO_OUTPUT_RAW
  control surfaces  49000
```

### What each MAVLink message does

| Message | Rate | Purpose | Requires |
|---------|------|---------|---------|
| `HIL_SENSOR` | 50 Hz | Gyro, accel, baro, diff-pressure → SITL INS backend | `AP_SIM_ENABLED=1` (`env SIM_ENABLED 1` in hwdef) |
| `GPS_INPUT` | 5 Hz | Lat/lon/alt/vel NED → GPS_MAV backend | `GPS1_TYPE=14`, `AP_GPS_MAV_ENABLED=1` in hwdef |
| `VISION_POSITION_ESTIMATE` | 50 Hz | Roll/pitch/yaw → EKF3 attitude | `VISO_TYPE=1`, `EK3_SRC1_YAW=6` |
| `RC_CHANNELS_OVERRIDE` | 20 Hz | X-Plane joystick axes → RC channels 1–4 | _(standard MAVLink)_ |
| `SERVO_OUTPUT_RAW` | 50 Hz | Servo PWM → X-Plane DREFs (ailerons, elevator, throttle, rudder, flaps) | _(standard MAVLink)_ |

### HIL_SENSOR — how it works

`HIL_SENSOR` (MAVLink message 107) feeds simulated IMU and barometer data
directly to the SITL INS backend compiled into fmuv3-hil.

Fields sent per packet (`fields_updated` bitmask):

| Field group | Fields | Source |
|-------------|--------|--------|
| Accelerometer | `xacc, yacc, zacc` (m/s²) | X-Plane Gload row 4, body frame |
| Gyroscope | `xgyro, ygyro, zgyro` (rad/s) | X-Plane AngularVelocities row 16 |
| Barometer | `abs_pressure` (hPa) | ISA model from altitude |
| Airspeed | `diff_pressure` (hPa) | `0.5 × ρ × v²` from IAS row 3 |
| Altitude | `pressure_alt` (m) | Same as alt from row 20 |
| Temperature | `temperature` (°C) | ISA lapse rate from altitude |

Magnetometer fields are **not sent** — the firmware uses the real onboard mag
(or it can be disabled via `COMPASS_ENABLE=0`).

### GPS_INPUT — how it works

`GPS_INPUT` (MAVLink message 232) injects position and velocity into the
`AP_GPS_MAV` backend, activated by `GPS1_TYPE=14`.

Fields sent:

| Field | Source | Units |
|-------|--------|-------|
| `lat`, `lon` | X-Plane LatLonAlt row 20 `data[1,2]` | degE7 |
| `alt` | row 20 `data[3]` × 0.3048 | m MSL |
| `vn`, `ve`, `vd` | row 21 `data[−6,4,−5]` (NED) | m/s |
| `yaw` | Row 17 heading | cdeg |
| `fix_type` | constant 3 | 3D fix |
| `satellites_visible` | constant 10 | — |

### Coordinate transforms (match SIM_XPlane.cpp exactly)

| Quantity | X-Plane row | C++ `data[N]` | Python `values[N-1]` | Transform |
|----------|-------------|---------------|----------------------|-----------|
| Pitch (°) | 17 | `data[1]` | `d[0]` | `× π/180` |
| Roll (°) | 17 | `data[2]` | `d[1]` | `× π/180` |
| Heading (°) | 17 | `data[3]` | `d[2]` | `× π/180` |
| vN (m/s) | 21 | `data[6]` | `d[5]` | `× −1` (−South) |
| vE (m/s) | 21 | `data[4]` | `d[3]` | direct |
| vD (m/s) | 21 | `data[5]` | `d[4]` | `× −1` (−Up) |
| accel x (m/s²) | 4 | `data[6]` | `d[5]` | `× g` |
| accel y (m/s²) | 4 | `data[7]` | `d[6]` | `× g` |
| accel z (m/s²) | 4 | `data[5]` | `d[4]` | `× −g` (normal load inverted) |
| gyro XP12 (rad/s) | 16 | `data[1,2,3]` | `d[0,1,2]` | `× π/180` |
| gyro XP11 (rad/s) | 16 | `data[2,1,3]` | `d[1,0,2]` | already rad/s, axes swapped |

---

## Step 1 — Build fmuv3-hil Firmware

The `fmuv3-hil` board variant extends `fmuv3` with three compile-time changes:

| hwdef change | Effect |
|---|---|
| `env SIM_ENABLED 1` | `AP_SIM_ENABLED=1` — compiles in SITL INS + SITL GPS backends |
| `define AP_GPS_MAV_ENABLED 1` | Re-enables MAVLink GPS backend (GPS1_TYPE=14) alongside SITL GPS |
| `define HAL_HAVE_SAFETY_SWITCH 0` | Safety switch disabled — servos active immediately on boot |
| `define AP_SIM_INS_FILE_ENABLED 0` | Disables file-replay INS (not needed for live HIL) |

```bash
# Set up toolchain path
ARM_BIN="$HOME/toolchain/gcc-arm-none-eabi-10-2020-q4-major/bin"
export PATH="$ARM_BIN:$HOME/.pyenv/shims:$HOME/.pyenv/bin:/usr/local/bin:/usr/bin:/bin:$PATH"

# Configure and build
./waf configure --board fmuv3-hil
./waf plane

# Flash (with board connected via USB)
./waf plane --upload
```

The configure step prints `Adding environment SIM_ENABLED 1` to confirm the
SITL framework is active.

---

## Step 2 — Set ArduPlane Parameters

Set these once via MAVProxy or QGC.  They persist across reboots.

### Core HIL parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `GPS1_TYPE` | `14` | Accept position from MAVLink `GPS_INPUT` |
| `AHRS_EKF_TYPE` | `3` | Use EKF3 (fuses GPS_INPUT + VISION_POSITION_ESTIMATE) |
| `EK3_SRC1_POSXY` | `3` | GPS as horizontal position source |
| `EK3_SRC1_VELXY` | `3` | GPS as horizontal velocity source |
| `EK3_SRC1_POSZ` | `1` | Barometric altitude source |
| `EK3_SRC1_YAW` | `6` | ExternalNav (VISION_POSITION_ESTIMATE) as yaw source |
| `VISO_TYPE` | `1` | Enable visual odometry / external attitude input |
| `BRD_SAFETY_DEFLT` | `0` | No safety switch required |
| `ARMING_SKIPCHK` | `-1` | Skip pre-arm checks during simulation |

### Compass (optional — disable if using simulated mag)

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `COMPASS_ENABLE` | `0` | Disable onboard compass if yaw from GPS/ExternalNav is sufficient |

### Runway origin (WICC)

| Parameter | Value |
|-----------|-------|
| `SIM_OPOS_LAT` | `-6.900300` |
| `SIM_OPOS_LNG` | `107.575200` |
| `SIM_OPOS_ALT` | `738.0` |
| `SIM_OPOS_HDG` | `131.0` |

Reboot after changing `GPS1_TYPE`, `AHRS_EKF_TYPE`, or `VISO_TYPE`.

---

## Step 3 — X-Plane Setup

No plugin required. X-Plane sends data via its built-in UDP Data Output;
the bridge auto-subscribes using `DSEL` packets.

**Settings → Net Connections → Data tab:**

```
☑ Send data to an IP
  IP:   192.168.1.20    ← Mac's LAN IP (find with: ipconfig getifaddr en0)
  Port: 49005
```

**Press `P`** to confirm X-Plane is **not paused** before starting the bridge.

No manual row selection needed — the bridge sends `DSEL` packets to subscribe
to the 8 required rows automatically (rows 1, 3, 4, 16, 17, 20, 21, 136).

### DREF overrides sent to X-Plane on startup

| Dataref | Value | Purpose |
|---------|-------|---------|
| `sim/operation/override/override_joystick` | `1.0` | Allow DREF control of surfaces |
| `sim/operation/override/override_throttles` | `1.0` | Allow DREF control of throttle |
| `sim/flightmodel/controls/parkbrake` | `1.0` | Hold aircraft until armed |

### Control surface DREFs (from `xplane_plane.json`)

| Channel | Dataref | Type |
|---------|---------|------|
| ch1 aileron | `sim/joystick/yoke_roll_ratio` | ANGLE |
| ch2 elevator | `sim/joystick/yoke_pitch_ratio` | ANGLE |
| ch3 throttle | `sim/flightmodel/engine/ENGN_thro_use[0..3]` | RANGE |
| ch4 rudder | `sim/joystick/yoke_heading_ratio` | ANGLE |
| ch5 flaps | `sim/cockpit2/controls/flap_ratio` | RANGE |

`ANGLE`: `value = (pwm − 1500) / 500`
`RANGE`: `value = (pwm − 1000) / 1000`

---

## Step 4 — Connect and Run

### Start MAVProxy (serial owner + UDP forwarder)

```bash
export PATH="$HOME/.pyenv/shims:$HOME/.pyenv/bin:/usr/local/bin:/usr/bin:/bin:$PATH"

mavproxy.py \
  --master /dev/tty.usbmodem1101 \
  --baudrate 115200 \
  --out udp:127.0.0.1:14550 \
  --out udp:127.0.0.1:14560
```

`14550` → QGC
`14560` → bridge script

### Start the bridge

```bash
python3 mavlink_xplane.py \
  --pixhawk udpin:0.0.0.0:14560 \
  --xplane-host 192.168.1.10 \
  --debug
```

Full argument reference:

| Argument | Default | Notes |
|----------|---------|-------|
| `--pixhawk` | `udpin:0.0.0.0:14560` | MAVLink connection (serial or UDP) |
| `--xplane-host` | `127.0.0.1` | X-Plane machine IP |
| `--xplane-port` | `49000` | X-Plane receives DREF/DSEL on this port |
| `--bind-port` | `49005` | Local port to receive DATA@ from X-Plane |
| `--hil-rate` | `50` | Hz for HIL_SENSOR + VISION_POSITION_ESTIMATE |
| `--gps-rate` | `5` | Hz for GPS_INPUT |
| `--rc-rate` | `20` | Hz for RC_CHANNELS_OVERRIDE |
| `--no-servos` | off | Skip servo DREF forwarding to X-Plane |
| `--debug` | off | Print state every 5 s |

### Connect QGC

1. **Disable auto-connect:** QGC → Application Settings → General → Auto Connect → uncheck all
2. **Add UDP link:** Application Settings → Comm Links → Add → Type: UDP → Port: `14550`

---

## Step 5 — Verify Sensor Flow

With `--debug`, the bridge prints every 5 s:

```
[xp]   lat=-6.90030  lon=107.57520  alt=738.0 m  t=12.4 s
[att]  pitch=+2.1°  roll=-0.3°  hdg=131.0°
[vel]  N=0.0  E=0.0  D=0.0 m/s  airspeed=0.0 m/s
[gyro] [0.001, -0.002, 0.000] rad/s
[acc]  [0.04, -0.01, -9.81] m/s²
[baro] 932.45 hPa  diff=0.0000 hPa
[joy]  [0.5, 0.5, 0.0, 0.5, 0.5, 0.5]
```

In QGC / MAVProxy, verify:

```
# MAVProxy console
status GPS1        → should show 3D fix, 10 satellites
status EKF3        → should show variance OK after ~30 s
```

---

## Step 6 — Arm and Fly

```bash
# In MAVProxy console
arm throttle       # arms if pre-arm checks pass (ARMING_SKIPCHK=-1 skips them)
mode auto          # activate AUTO mission
disarm             # after landing
```

Or use QGC arm button / RC switch.

---

## Troubleshoot

### No X-Plane DATA@ received

```bash
# Verify X-Plane is sending to the Mac (stop bridge first, then run):
python3 xp_sniff.py 49005
```

Expected output:
```
[192.168.1.10:49001]  DATA  len=293  total=1
  row   1: [0.0, 0.0, 12.4, ...]
  row  17: [2.1, -0.3, 131.0, ...]
  row  20: [-6.9003, 107.5752, 2421.0, ...]
```

If nothing appears: check X-Plane IP setting is `192.168.1.20` (Mac), not `127.0.0.1`.

### GPS1_TYPE=14 not in parameter list

The build did not include `AP_GPS_MAV_ENABLED=1`.  Confirm the hwdef has it:

```bash
grep AP_GPS_MAV build/fmuv3-hil/ap_config.h
# Should print: #define AP_GPS_MAV_ENABLED 1
```

If not, rebuild after verifying [libraries/AP_HAL_ChibiOS/hwdef/fmuv3-hil/hwdef.dat](libraries/AP_HAL_ChibiOS/hwdef/fmuv3-hil/hwdef.dat).

### EKF3 not converging

Check that:
1. `GPS_INPUT` is arriving — MAVProxy: `message GPS_RAW_INT` should show `fix_type=3`
2. `VISION_POSITION_ESTIMATE` is accepted — MAVProxy: `message VISION_POSITION_DELTA`
3. `EK3_SRC1_YAW=6` and `VISO_TYPE=1` are set

### Waf configuration error: stale `ap_config.h`

```bash
./waf distclean
./waf configure --board fmuv3-hil
./waf plane
```

---

## Notes

- `env SIM_ENABLED 1` is a **compile-time** hwdef flag — it cannot be set via parameters
- `HAL_HAVE_SAFETY_SWITCH=0` means servos output immediately on boot with no button press
- `AP_GPS_MAV_ENABLED=1` must be explicit because `env SIM_ENABLED 1` replaces the default
  GPS backend with the SITL GPS (type 100); without this define, GPS1_TYPE=14 is not compiled in
- The bridge uses the exact same X-Plane DATA@ rows and coordinate transforms as
  `libraries/SITL/SIM_XPlane.cpp` — see comments in `mavlink_xplane.py` for per-field mapping
- ARM toolchain at `~/toolchain/gcc-arm-none-eabi-10-2020-q4-major/`
- Python packages (pymavlink) installed in pyenv 3.10.18
