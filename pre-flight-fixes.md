# mr_vmu_rt1176 pre-flight parameters and fixes

Working notes for the Zephyr HAL port on the NXP MR-VMU-RT1176, recorded
2026-09-26. Written for whoever prepares this aircraft to fly, including a
future session picking the work up cold.

Each entry says how it was established. "Verified" means a measurement on this
board; "observed" means read back over MAVLink; "inferred" means reasoned from
source and not yet confirmed on hardware. Treat inferred entries as claims to
check, not facts.

---

## 1. Must be set before flight

| Parameter | Value | Why |
|---|---|---|
| `AHRS_ORIENT` | **0** | Was 8 (ROLL_180). The per-IMU rotations are now set in the hwdef from PX4's reference, so a global board rotation double-applies. Leaving it at 8 composes `PITCH_180` with `ROLL_180` = `YAW_180`: the airframe flies believing forward is backward. |

This one is not optional and is not cosmetic. `AHRS_ORIENT=8` corrects the Z
axis, so a level board reads a convincing `accel z = -1 g` while the horizontal
frame is yawed 180 degrees. It looks right on the bench and diverges on takeoff.

**Verify physically after setting it** — do not take the parameter on trust:

- nose down, HUD pitches down
- roll right, HUD rolls right
- yaw right, heading increases

If pitch or roll is inverted or swapped, the socket-to-rotation mapping below is
wrong and the aircraft must not fly.

---

## 2. Relaxed for bench work — UNSAFE to fly with

These were loosened deliberately during bring-up. Every one removes a protection.

| Parameter | Bench value | Restore to | Consequence if flown as-is |
|---|---|---|---|
| `ARMING_CHECK` | 0 | 1 (all) | No pre-arm checks at all. Observed via the board's own `Warning: Arming Checks Disabled`, not read directly. |
| `BATT_MONITOR` | 0 | board's monitor type | No voltage/current sensing, so **no low-battery failsafe**. The type for this board was never determined. |
| `FS_GCS_ENABLE` | 0 | per your preference | No GCS-loss failsafe. |
| `ARMING_CRSDP_IGN` | 1 | 0 | Pre-arm ignores a genuine crash dump. Set to get past a stale dump; it hides real ones. |
| `DISARM_DELAY` | 0 | 10 (default) | No auto-disarm after landing. |

`FS_*` generally was turned off for bench work. Decide each one deliberately
rather than restoring blindly.

---

## 3. Chosen configuration — leave alone

| Parameter | Value | Note |
|---|---|---|
| `FSTRATE_ENABLE` | 1 | Maintainer's chosen configuration. Runs the fast rate loop. |
| `FSTRATE_DIV` | 1 | Full gyro rate. Its governor hunts 202..1012 Hz continuously — see known issues. |
| `SCHED_LOOP_RATE` | 200 | Was 600 against a measured ~195-308 Hz capability. A target the loop cannot reach means main never idles. **Applies live, no reboot needed** — the loop moved to 193 Hz immediately. |
| `MOT_SPIN_ARM` | 0.20 | Correct for these ESCs, which start at roughly 1200 us. 0.1 was too low to spin them. |
| `SERIAL7_PROTOCOL` | -1 | Only one port may hold RCIN. With this unset, RC never links. |
| `RC_OPTIONS` | bit 8 set (256) | `CRSF_CUSTOM_TELEMETRY`, needed for Yaapu telemetry on the handset. |

---

## 4. Calibrations outstanding

None of these have been done on this board, and the first two block things that
look like bugs:

- **Accelerometer** — this is what holds `MAV_SYS_STATUS_AHRS` unhealthy.
  `accel_calibrated_ok_all()` requires a non-zero offset *and* scale per accel;
  defaults are zero. Nothing in code can substitute for the six-orientation
  calibration. Verified by reading the health condition in `GCS.cpp:489-492`.
- **Compass** — measured field magnitude was 183 mGauss against roughly 500
  expected in Australia, which is the signature of absent hard-iron offsets.
  Cannot be done while the compass is not producing samples (see known issues).
- **RC** — never calibrated.

Calibrations write to storage via `AP_Param::save_queue`, which is drained by
the io thread. That thread is starved (known issue below). Flash writes were
observed completing, so persistence probably works — but if a calibration
succeeds and reads back zero after a reboot, that is the cause rather than a
failed calibration.

---

## 5. Firmware changes in the build (not parameters)

Committed:

- `AP_HAL_Zephyr: keep XIP usable across a ROM flash erase or program` — the
  missing FlexSPI controller reset after every ROM erase/program.
- `AP_HAL_Zephyr: place the fault path in ITCM so a fault can be recorded` — the
  whole fault chain was in XIP flash, so a fault caused by the flash controller
  could not fetch its own handler. Verified: converted resets that recorded
  nothing into resets recording reason 35.
- `AP_HAL_Zephyr: carry the fault record across the reset and report it`.

Uncommitted, on the board, pending hardware verification:

- `wait_busy` after every ROM erase/program. Freeze interval went from a
  15-300 s median to 631 s and later a 40 minute clean run. Not proven.
- USB device stack into ITCM.
- D-cache invalidation after flash writes. `_flash_read_data()` reads the NOR
  through the memory-mapped window with `memcpy`, so a read after a write could
  be served a stale cache line.
- `AP_SCHEDULER_LOOP_YIELD_US` 50 to 100 us. Fixed `AP_Logger: stuck thread ()`
  for a while, but it has recurred — **not** a settled fix.
- UART buffer sizing to ChibiOS parity (512 base, doubled for USB, doubled again
  for `HAL_MEM_CLASS >= 500`). Ours was a flat 1024 on the USB MAVLink port.
- CDC ACM FIFOs 1024 to 4096 tx / 2048 rx on this board's DTS.
- All four `AP_GPS` health gates made board-overridable, upstream defaults
  untouched, this board relaxed by >=50%: average 215->330 ms, **per-frame
  245->400 ms**, delayed frames 2->3, lagged samples 5->8. The per-frame gate was
  the binding one; relaxing only the average provably changed nothing.
- Per-IMU rotations from PX4 (section 6).
- Diagnostics that should NOT ship: `FTPDIAG` printks in `GCS_FTP.cpp`, the
  per-thread CPU report and thread-state strings in `Scheduler.cpp`.

---

## 6. IMU rotations, and where they came from

PX4 is the only other autopilot supporting this hardware. FMUv6X-RT is the same
design as MR-VMU-RT1176, so `boards/px4/fmu-v6xrt/init/rc.board_sensors` is the
reference. The rotation belongs to the **socket**, not the part: PX4 gives every
candidate on bus 1 `-R 12` and every candidate on bus 2 `-R 6`, across
`icm42688p` and `icm45686` alike. PX4's `Rotation` enum is numerically identical
to `AP_Math/rotations.h` (checked: NONE 0, YAW_180 4, YAW_270 6, ROLL_180 8,
PITCH_180 12), so values transfer directly.

| Socket | PX4 | ArduPilot hwdef |
|---|---|---|
| `imu_sensor1` (LPSPI1) | `-R 12` | `ROTATION_PITCH_180` |
| `imu_sensor2` (LPSPI2) | `-R 6` | `ROTATION_YAW_270` |
| BMI088 on SPI3 | `-R 4` | `ROTATION_YAW_180` when declared |
| `imu_sensor3` (ISM330DHCX) | not started by PX4 | `ROTATION_NONE` is a **placeholder, not a checked value** |
| BMM150 compass | no `-R`, declared internal | `ROTATION_NONE` — matches PX4 |

Note PX4 uses **no global board rotation at all**. Neither should this board,
hence `AHRS_ORIENT=0`.

One divergence left unresolved: PX4 declares the BMM150 **internal** (`-I`); our
hwdef declares it **external** (`true`). In ArduPilot this does not change the
rotation — `AHRS_ORIENT` is never applied to compasses, as `rotate_field()` uses
only `MAG_BOARD_ORIENTATION` (ROTATION_NONE here) and the per-compass rotation,
and `get_board_orientation()` is defined but never called from any compass source
file. It does affect calibration handling and pre-arm behaviour.

---

## 7. Known unresolved issues

**Uncommanded reboots.** Not fixed, cause not identified. The SoC freezes
completely for over 2 s and the hardware watchdog resets it. Established: the
freeze stops main, the timer thread *and* the monitor thread, and the monitor is
what feeds the watchdog — so this is not the deliberate "main loop stuck" starve
path. Anything that only runs "when it goes wrong" cannot report it, which is why
a 500 ms recorder and a watchdog pre-reset interrupt both produced nothing.
The WDG statustext now carries continuously-sampled state, with sticky flash
counters written by the flash path itself:

- `FL` low byte = flash ops started minus completed; nonzero means it died
  **inside** a flash operation. High byte: 1 erase, 2 program.
- `FICSR` = flash offset of that operation.
- `FA` = main's PC, `FLR` = what main was blocked on (0 = runnable).

Best interval so far is roughly 40 minutes clean. A reset in flight stops the
motors.

**Starved PREEMPT(10-11) band.** The io thread, the MAVFTP worker, `log_io` and
storage all read `queued` — ready to run, never scheduled. Consequences:
`AP_Logger: stuck thread`, MAVFTP parameter fetch failing, and parameters
possibly not persisting. Measured per-thread CPU: `main` 41-54%, `rcin` 17%,
`tmr` 8-16%, `rate` 6-9%, and roughly 20% in interrupt handlers, which
`k_thread_runtime_stats` does not attribute to any thread.

`rcin` at 17% for decoding CRSF frames is the anomaly and the lever. RX is
per-frame, not per-byte (`rx=15206919` over `rxev=927377` is ~16 bytes per
event), so the byte rate does not justify it. **MAVFTP works on ChibiOS**, so
this is our HAL spending cycles ChibiOS does not, rather than the board being
short of them.

**Sensors dropping out for seconds.** `GPS 1: probing for u-blox` appearing
mid-run means AP saw no parseable GPS message for 4 s (`GPS_TIMEOUT_MS 4000`) and
restarted detection, which is what produces `EKF variance: position lost`. The
compass health bit is simply "a sample within the last 500 ms" and it is false.
Both are device paths below main, in the same starved band. Main was at 54% CPU
in the same sample, so main is not stalled, and a 2 s main stall would have
rebooted the board instead.

Next measurement: whether `UARTSTAT s3 rx=` keeps incrementing through a GPS
dropout. If it does, bytes arrive and AP is not consuming them; if it stops, the
LPUART3 receive path itself stops.

**Rate governor hunting.** Cycles 202/253/337/506/1012 Hz continuously without
settling, reconfiguring notch filters, the DShot rate and motor dt each time.
The GCS message is now throttled to one per 10 s, which silences the reporting
and not the hunting. Lowering `SCHED_LOOP_RATE` to 200 freed headroom the
governor reads as spare and may have made the oscillation worse.

**TELEM1 (SERIAL1)** has a physical break — `rx=0` always, `RXEDGIF=0` at the
register while GPS1 shows all flags set. Not a firmware problem.

---

## 8. Measurements taken 2026-09-26/27

Appended. Sections 1 to 7 above are the text as originally written and are
deliberately unchanged; nothing here overrides them.

### Loop rate: 190 Hz to 466-483 Hz

All with EKF3 running, logging on and the fast-rate thread on, at
`SCHED_LOOP_RATE` 600. Steady figure from a 7-minute soak: 41 samples, min 445,
median 460, max 481 Hz, `leaks=0`, no reboots. A later run measured median 482.

Read `boost=` on the `LOOPRATE` line, not just `loop_hz`. `boost=0` means main
never got ahead of its schedule, so the number is a ceiling. Every 190 Hz
reading had `boost=0`; the 460+ readings have `boost` rising ~1300 per 10 s.

| Change | Measured effect |
|---|---|
| `HAL_GYROFFT_ENABLED 0` / `HAL_WITH_DSP 0` | ITCM 95.32% -> 88.01%, ~36 KB freed |
| Kernel heap pool 65536 -> 16384, SCurve+AP_Param out of `.ramfunc` into ITCM | RAM 76.79% -> 51.79% |
| Async UART RX restart backoff | 376-385 -> 450-463 Hz; timer thread 14% -> 4% |

Rate loop measured by an unconditional iteration count: 10121 iterations per
10 s, so **1012 Hz** at decimation 1, matching the raw gyro rate. The governor
sits at `dec=1`/`tgt_dec=1` and no longer hunts.

### The UART finding

`UART_RX_DISABLED` set a restart flag and `_rx_timer_tick()` runs at 1 kHz, so
every tick did a full `uart_rx_disable()` + `uart_rx_enable()`, tearing down and
re-arming an eDMA transfer. A port with nothing on its RX pin does that forever.
PC sampling put `mcux_lpuart_rx_enable` at 7.0% and `mcux_lpuart_rx_disable` at
3.8% of all thread time.

### "Intermittent IMU detection" was heap exhaustion

`INS needs at least 1 gyro and 1 accel` is allocation failure, not flaky SPI -
the same boot also prints `Unable to allocate scheduler TaskInfo`. It tracks
static RAM exactly: every build at 76.79% failed to find the IMU, every build at
51.79% found it. Free heap went from **3104 bytes** to **231 KB**, and
`EK3_ENABLE` now holds 1 across a reboot where `NavEKF3::InitialiseFilter()`
used to set it back to 0 on allocation failure.

### The `.ramfunc` budget

Keep its content at or under **65,536 bytes**. The region start is aligned to a
power of two of its own size, so 110,292 bytes of content cost **262,144 bytes**
of OCRAM - and OCRAM is where the malloc arena lives. Exceeding it is what broke
boot. Note also that with the board disarmed, no RC link and no GPS fix, the PC
profiler reported `ocram=0%` of thread samples: relocated flight-path code was
not executing, so nothing could be won there on a bench.

### Orientation: an OPEN observation, not a conclusion

Section 1 stands. What was measured, so it is on the record:

- The parameter's full name is `AHRS_ORIENTATION`; the short `AHRS_ORIENT` used
  above and in upstream's own `COMPASS_ORIENT` help text does not exist as a
  parameter and a `PARAM_REQUEST_READ` for it never answers.
- With the PX4 per-IMU rotations in the firmware and `AHRS_ORIENTATION` 0, the
  HUD is inverted: ground where sky should be, `ATTITUDE` roll +179.8 deg,
  `RAW_IMU` zacc +988 where a level board reads about -988.
- `YAW_270` cannot change Z, and the rotation does reach the driver (the
  generated `hwdef.h` passes `ROTATION_YAW_270` to the probe), so a 180 degree
  flip is missing somewhere else.
- `INS_ACC_ID` is set with no `ACC2`/`ACC3`, so exactly one IMU registers as
  instance 0 - socket 2, since `check_whoami()` accepts no ICM-42686 for lpspi1.
- PX4's real `boards/px4/fmu-v6xrt/init/rc.board_sensors` starts the BMM150 with
  `-I`, **internal**. This hwdef declares it **external**, and
  `AP_Compass_Backend::rotate_field()` applies the board orientation only to an
  internal compass. That is the one divergence from PX4's sensor config and the
  leading candidate for the missing flip. UNTESTED.

### Measurement tooling

- `libraries/AP_HAL_Zephyr/zephyr/src/ap_pcprofile.c` - PC profiler sampled from
  the system-clock interrupt, no SWD probe needed. Diagnostic, must not ship.
- **Save the ELF with every capture.** waf writes one `zephyr.elf` per board, so
  a later build replaces it and old addresses resolve to different, plausible and
  wrong functions.
- Resolve PC buckets with `objdump -d`. On this binary `nm` and `addr2line -f`
  agreed with each other and were both wrong.
- An mcumgr `os` reset over the `-if-smp` port reboots the board when ArduPilot
  is wedged and MAVLink is dead. `uploader.py` cannot.
