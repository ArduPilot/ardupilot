# Skybrush fork feature map

Reference: skybrush-io/ardupilot branch `CMCopter-4.7` (`b66c323a2d9`) compared
with tag `Copter-4.7.2`. This map lists what the fork adds and is the checklist
for the SkyBrush native module.

Upstream already reserves mode number 127, RC aux function 248-249 and G2
parameter index 62 for Skybrush.

The diff also contains changes that are not Skybrush features, and the module
does not reproduce them:
- ArduPilot-4.7 stable-branch merges: atomic pin mode in AP_HAL_ChibiOS,
  AP_BoardConfig, and the AEDROXH7 hwdef/bootloader.
- An AP_Param flush fix cherry-picked from master.
- The CollMot `cmcopter` PPMSum receiver hack.
- Build tweaks.

Tags: **T** = exercised by the SITL tester, **P** = needed for parity only.

## 1. Drone show flight mode (mode 127 `DRONE_SHOW`/`SHOW`) — T

- `ModeDroneShow` runs an 11-stage state machine at main-loop rate:
  - Stages: Off 0, Init 1, WaitForStartTime 2, Takeoff 3, Performing 4, RTL 5,
    Loiter 6, Landing 7, Landed 8, Error 9, TestingLights 10.
  - The stage number appears on the wire and in the `SHOW` log.
- **Wait for start**:
  - Calibrates the baro at T-10 s.
  - Sets home to the current location while on the ground.
  - Arms itself (method SCRIPTING) 8 s before takeoff, or 0.8×DISARM_DELAY
    if that is smaller.
  - Retries arming every 1 s until T+5 s, then gives up and goes to Error.
  - If Copter auto-disarms while waiting, it re-arms.
  - Disarms when authorization is revoked, but only if the mode armed the
    vehicle itself.
  - More than 6 s late leads to Landed.
- **Takeoff** to `SHOW_TAKEOFF_ALT` above the current position:
  - Uses `auto_takeoff`, holding yaw at the initial armed bearing.
  - Completes at 70% of the takeoff altitude once the trajectory is at least
    that high, or when the waypoint is reached plus 10 s.
- **Performing**:
  - At `SHOW_CTRL_RATE` (default 10 Hz) sends pos/vel/accel and yaw/yaw-rate
    targets to guided via `set_pos_vel_accel_NED_m`. `mode_guided.run()` runs
    every loop.
  - Velocity feed-forward is scaled by `SHOW_VEL_FF_GAIN` when `SHOW_CTRL_MODE`
    bit0 is set.
  - Optional altitude lock above the takeoff altitude.
  - If a target can't be sent, it holds position and goes to Loiter.
- **End of show**: `SHOW_POST_ACTION` selects 0 Loiter, 1 Land, 2 RTL, or
  3 RTL-if-circular-and-near-takeoff-else-Land. Land is forced after a
  collective RTH, or below min(1 m, 0.5×takeoff alt).
  - Land, RTL and Loiter run *inside* mode 127: the mode calls
    `mode_land/mode_rtl/mode_loiter.init()/run()` directly. RTL runs with no
    disarm on landing.
  - Landed disarms. A new start time 10 s to 12 h ahead re-initialises the mode.
  - Error and TestingLights disarm.
- **Mode properties**:
  - Requires position (so the EKF failsafe applies), is an autopilot mode, no
    manual throttle.
  - Not `in_guided_mode()`, so GCS guided commands are ignored.
  - MAVLink arming is always allowed. Other arming methods need a show loaded,
    a plausible trajectory, and origin and orientation set.
  - GCS takeoff starts a test takeoff to Loiter.
- **Boot and switching**:
  - `SHOW_MODE_BOOT` bit0 replaces INITIAL_MODE at boot.
  - Bit1 switches to show mode (reason SCRIPTING, only when disarmed) when
    authorization is granted.
- **Radio failsafe**: with FS_OPTIONS bit2 the show continues ("Continuing
  Show"). The GCS failsafe is unchanged.

## 2. Show data and the libskybrush library — T

- **Show file**: `/COLLMOT/show.skyb` (`./collmot/show.skyb` on SITL).
  - Uploaded by MAVFTP. The tester copies it into place before boot instead.
  - Loaded at boot and on `MAV_CMD_USER_1` param1=0. param1=1 deletes the file
    and clears the show. Both are refused while armed.
  - The whole file stays resident in memory.
- **Container**:
  - Starts with `"skyb"`, a version byte and a feature byte.
  - Optional CRC32 using the same algorithm as MAVFTP, computed with bytes 6..9
    zeroed.
  - Then TLV blocks: `type u8, len u16`.
- **Blocks**:
  - 1 trajectory: int16×scale mm Bezier segments of degree 0/1/3/7, durations
    u16 ms.
  - 2 light bytecode: 14 opcodes, LEB128 durations in 20 ms units, loop
    depth 4.
  - 4 RTH plan.
  - 5 yaw control.
  - 6 events: pyro and light-source/pixel events.
  - 7 GCS light-control setup: pixel coordinates and palette.
- **Trajectory statistics**:
  - Takeoff time = first z crossing of the takeoff altitude, minus the
    calculated climb time (WPNAV climb speed and acceleration), minus
    MOT_SPOOL_TIME, clamped to ≥ 0. The arming lead time is measured against
    this, not against show-clock zero.
  - Also computes landing time, duration, start position and circularity
    (start-to-end distance ≤ 0.5 m).
  - The root finder handles degree ≤ 3 only, as in libskybrush.
- **Landing correction**: with `SHOW_OPTIONS` bit1 on a circular trajectory,
  the trajectory end is replaced at takeoff
  (`replace_end_to_land_at_with_terminal_velocity`):
  - end point = *actual* takeoff XY at half the takeoff height
  - terminal descent velocity = `LAND_SPD_MS`
  - the duration of the last dynamic scene is recomputed
- **Screenplay**: a list of scenes, each with a time axis (piecewise-linear
  rate warping) and a show controller that maps wall time to scene time, then
  to position/velocity/yaw/colour and due events.
- **Collective RTH** = a TIME_AXIS_CONFIG packet with a second scene built from
  the RTH plan.
- libskybrush is GPL-3.0-or-later (CollMot Robotics). A Rust port keeps that
  licence and attribution.

## 3. Timing and authorization — T

- **`SHOW_SYNC_MODE`**:
  - 1 GPS (default): the start time is GPS time-of-week `SHOW_START_TIME`
    seconds plus `SHOW_START_MSEC`. The GPS week is the one nearest now.
    "Now" is `AP_GPS::time_epoch_usec()`, clamped so it never goes backwards.
  - 0 countdown: uses `micros64()`.
- `SHOW_START_TIME/MSEC/AUTH` are volatile: they are reset to unset/revoked at
  boot.
- The start time can only change in stages Off, Init, WaitForStartTime and
  Landed.
- **Authorization scope** (`SHOW_START_AUTH`): 0 revoked, 1 live, 2 rehearsal,
  3 lights only. Only 1 may start motors. 3 goes to TestingLights at the start
  time.
- **RC aux 248**: starts the show 10 s later, only in show mode and >5 s after
  boot. Logs LogEvent 248. **P**

## 4. MAVLink protocol

**GCS→drone DATA16/32/64/96** with `type 0x5C`; `data[0]` is the inner type:

| Inner | Name | Notes | Use |
|---|---|---|---|
| 1 | START_CONFIG | `i32 tow_s, u8 auth, [i32 countdown_ms], [u16 msec]`. Writes `SHOW_START_TIME/MSEC/AUTH` | T |
| 2 | deprecated CRTH | ignored | P |
| 3 | SIMPLE_GEOFENCE_SETUP | `u8 n, u16 alt_dm, u16 radius_dm, u16 token, u8 flags, 3×pad`, n×`i16 x_dm, y_dm` in the show frame | T |
| 4 | ACK | ignored | P |
| 5 | TIME_AXIS_CONFIG | seq, start ms, scenes and segments; CRTH scenes `0xC000\|t` | P |

Geofence setup:
- Validates the polygon first, then writes `FENCE_ALT_MAX/RADIUS/ACTION` with
  save and PARM logging.
- Writes a POLYGON_INCLUSION polyfence. AC_PolyFence then reports "Fence
  indexed OK".
- Writes the type mask to `FENCE_TYPE`, then `FENCE_ENABLE` 1 (0 if no types).
  The fork writes the mask into `FENCE_ENABLE`, which is only an on/off switch
  here.
- Replies with DATA16 `type 0x5D` `[4, token(2), MAV_RESULT]`.

**Drone→GCS**:
- DATA16 0x5B: 14-byte status at 2 Hz (standard profile).
- DATA96 0x5B: 48-byte extended status at 1 Hz (compact profile,
  `SHOW_TEL_PROFILE=1`).
- The tester accepts only DATA96. It reads `data[7]` = stage (with the
  preflight nibble) and `data[10:12]` = elapsed seconds.
- The status packet also carries:
  - start time, RGB565 colour and flags
  - GPS health
  - boot count (`STAT_BOOTCNT`)
  - ESC error rate flag and RTCM message counts on TELEM1/TELEM2 over 5 s
  - position, velocity, heading, hdop and vdop
- Default stream intervals depend on the profile: standard adds
  GLOBAL_POSITION_INT 2 Hz, SYS_STATUS 1 Hz and GPS_RAW 1 Hz. **T**

**COMMAND_INT**:

| Command | param1 | Action | Use |
|---|---|---|---|
| `MAV_CMD_USER_1` | 0 | reload | P |
| | 1 | clear | P |
| | 2 | pyro test (no interlocks in the fork) | P |
| | 3 | group-filtered command re-injection via `inject_command_int_packet` | P |
| | 4 | set light source | P |
| `MAV_CMD_USER_2` | 0 | set origin/orientation/AMSL at full int32 precision | P |

**Other inbound** (P):
- `LED_CONTROL` with instance=pattern=42: "where are you" flashes, colours and
  effects, optional group filter.
- `TUNNEL` with payload type 42421: pixel-grid frames.

**Capability bit** `0x4000000`, set when the module starts, so a GCS that asks
for AUTOPILOT_VERSION during the first seconds of boot does not see it. **P**

## 5. Fences and failsafes — T

- **Hard fence** (`SHOW_HFENCE_EN/DIST/TO`) runs at 25 Hz after `fence_check()`:
  - Any geofence breach → SOFT, and the breach point is recorded.
  - More than DIST from the breach point → HARD.
  - HARD for TO seconds → "Hard fence breached, disarming" and a forced
    disarm with method FENCEBREACH.
  - It always tracks HardB for the log, even when disabled.
- **Bubble fence** (`SHOW_BFENCE_EN/DXY/DZ/TO/ACT`) watches deviation from the
  last setpoint while Performing:
  - After TO s it acts once per second, with the reply rate-limited to one
    every 5 s. Actions:
    - 1 report: "Bubble fence breached"
    - 2 flash lights
    - 3 RTL with reason FENCE_BREACHED, falling back to Land: "…, return to
      launch"
    - 4 Land: "…, landing"
    - 5 forced disarm with method FENCEBREACH: "…, disarming"
- **`FS_EKF_ACTION=42`**: a forced disarm with method EKFFAILSAFE and the text
  "EKF Failsafe: forced disarm". This is a fork-only vehicle change; here the
  module sets a custom mode option that applies in every flight mode while
  the module runs.
- **Forced disarm** = `arming.disarm(method, false)`, falling back to
  `motors->armed(false)`.

## 6. Lights — P (the tester does not check LEDs)

Colour priority, at 25 Hz with repeats on the odd 50 Hz ticks:
1. compass cal running
2. LED_CONTROL signal
3. failsafes, red blink
4. bubble flash
5. flying (by mode or stage)
6. spooling
7. pre-arm
8. standby states

Blink and breathe phases lock to GPS milliseconds. Output processing:
- dimmed by `SHOW_PRE_LIGHTS`
- gamma correction, `SHOW_LED0_MINBRI`
- RGBW conversion (`SHOW_LED0_WTEMP`)

Backends (`SHOW_LED0_TYPE`):
- MAVLink DEBUG_VECT
- NeoPixel, ProfiLED, NeoPixel-RGBW (SerialLED)
- debug text
- servo PWM on SERVOn_FUNCTION 106-109 (scripting13-16, W/R/G/B): normal,
  inverted and limited variants
- I2C RGB/RGBW
- WGDrones UART
- AP_Notify `handle_rgb`

Compass-cal result flashes use the AP_Notify event counters.

## 7. Pyro — P (the tester rejects pyro shows)

- Events come from the show (block 6), shifted at load by `SHOW_PYRO_TCOMP`.
- They fire at 25 Hz in Performing only.
- Safety gate: armed, flying, have absolute position, no GPS glitch, EKF OK,
  no battery failsafe, no fence breach, within MAX_XY/Z_ERR of the setpoint,
  and above `SHOW_PYRO_MINALT`.
- More than 3 s late counts as missed. Outputs turn off automatically after
  `SHOW_PYRO_ITIME`. Every event is logged as `SBEV`.
- Backends (`SHOW_PYRO_TYPE`):
  - debug
  - single servo: function 105 (scripting12), 85%/100%
  - multiple servos. All function 105 outputs must share `SERVOn_REVERSED`,
    as one scaled value holds them off.
  - Cobra 6M on a UART, which takes over SERIALx_PROTOCOL 14 (Volz)
  - relays, switched on with a host timeout renewed every second, so they
    turn off if the module stops

## 8. Logging — T (SHOW, FNCS)

| Name | Format | Fields | Rate |
|---|---|---|---|
| SHOW | `QiBBifffBBBff` | TimeUS,ClockMS,Stage,Scene,SceneMS,X,Y,Z,R,G,B,HDist,VDist | 25 Hz in the fork; the tester uses TimeUS, ClockMS, Stage, and ClockMS must be consistent with TimeUS within 50 ms |
| FNCS | `QBBHBBH` | TimeUS,GeoEn,GeoB,GeoCnt,HardB,BubbleB,BubbleCnt | 1 Hz |
| SBEV | `QiBiBBIB` | show events | P |
| SBSP | `QBBBQIff` | screenplay segments | P |
| CRTH | `QiIfff` | collective RTH trigger | P |

PARM entries are written for origin, orientation and fence changes. The
tester reads 12 `SHOW_*` values from PARM.

## 9. Parameters (`SHOW_`, 40 entries, indices 1-40)

| Parameters | Indices |
|---|---|
| START_TIME, ORIGIN_LAT, ORIGIN_LNG, ORIENTATION, START_AUTH | 1-5 |
| LED0_TYPE, LED0_COUNT, LED0_CHAN | 6-8 |
| MODE_BOOT, PRE_LIGHTS, CTRL_MODE, ORIGIN_AMSL, GROUP, CTRL_RATE | 9-14 |
| TAKEOFF_ERR, VEL_FF_GAIN, TAKEOFF_ALT, SYNC_MODE, LED0_GAMMA | 15-19 |
| HFENCE_EN, HFENCE_DIST, HFENCE_TO | 20-22 |
| LED0_WTEMP, MAX_XY_ERR, MAX_Z_ERR, POST_ACTION | 23-26 |
| BFENCE_EN, BFENCE_DXY, BFENCE_DZ, BFENCE_TO, BFENCE_ACT | 27-31 |
| LED0_MINBRI | 32 |
| PYRO_MINALT, PYRO_TYPE, PYRO_TCOMP, PYRO_ITIME | 33-36 |
| OPTIONS, TEL_PROFILE, MAX_ESC_ERR, START_MSEC | 37-40 |

Types:
- Integers: START_TIME, ORIGIN_LAT/LNG/AMSL and PYRO_TCOMP/ITIME are int32;
  START_MSEC is int16; most of the rest are int8.
- Floats: the distances, gains and times.

Other parameters:
- **Read**: `MOT_SPOOL_TIME`, `LAND_SPD_MS`, `STAT_BOOTCNT`, `DISARM_DELAY`,
  `WP_SPD_UP`, `WP_ACC_Z`.
- **Written**: `FENCE_*`.
- `CM_FLOCK_OPTIONS` is legacy CollMot flock control, not show logic, and is
  out of scope.

## 10. Small library patches in the fork

| Patch | Purpose | Module treatment |
|---|---|---|
| GCS: `ap_message` 101/102, custom default intervals, protected `set_ap_message_interval` | status streams | the module sends its own packets; stream rates set with `set_message_interval` |
| GCS: `inject_command_int_packet` | group commands | existing `run_command_int` binding |
| GCS/AP_GPS: per-link RTCM `MessageCounter`, RTCM functions take a channel | RTCM counters in status | `gps:rtcm_message_count()` per channel; the module keeps the 5 s window |
| `AP_ESC_Telem::has_high_error_rate` | status flag | `esc_telem:get_raw_rpm_and_error_rate()`; the module keeps each ESC's last rate |
| `AP_Relay::get_index_mask` | relay pyro | module reads `RELAYn_FUNCTION` |
| AP_Notify `DroneShowNotificationBackend`, OreoLED `rgb_control` | compass-cal flashes, OreoLED | `notify.get_event_counts()`; OreoLED not supported |
| AP_Logger static SHOW/FNCS/SBEV/SBSP/CRTH, LogEvent 248 | logs | dynamic `logger.write` |
| AP_Common FWVersion not PACKED on SITL | macOS link | n/a |
