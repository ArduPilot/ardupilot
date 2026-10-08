# mr_vmu_rt1176 — day 2: why the HUD was upside down

Companion to `pre-flight-fixes.md`. That file is the pre-flight parameter record
and its sections 1-7 are not to be edited to match a later opinion. This file is
the reasoning and the sources behind one specific bug found on 2026-09-27.

Convention as in day 1: **verified** = measured on this board or read from source
that was actually opened; **inferred** = reasoned and not yet confirmed. Nothing
in the conclusion below has been confirmed on hardware yet.

---

## 1. The bug

With the committed hwdef (per-socket rotations copied from PX4) and
`AHRS_ORIENTATION 0`:

- `ATTITUDE` roll **+179.8 deg**, `RAW_IMU` zacc **+988**, where a level
  right-way-up board reads about **-988**. HUD upside down, ground where sky
  should be. **Verified** (bench, 2026-09-27).

The board is **not** physically inverted (maintainer, definitive).

`ROTATION_YAW_270` maps (x,y,z) -> (y,-x,z) and so cannot change Z, and the
board rotation was 0, so nothing in that configuration applied the 180 degree
flip the board needs. The rotation does reach the driver - the generated
`build/mr_vmu_rt1176/hwdef.h` contains
`HAL_INS_PROBE3 ... get_device("imu_sensor2"), ROTATION_YAW_270` - so this was
never a plumbing fault. **Verified.**

---

## 2. Root cause: PX4 rotates in the driver, ArduPilot does not

**PX4** negates two axes inside the driver, before any `-R` rotation is applied.
`src/drivers/imu/invensense/icm42688p/ICM42688P.cpp`:

```c
739:  accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MAX : -accel.y[i];
740:  accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MAX : -accel.z[i];
...
788:  gyro.x[i] = gyro.x[i];
789:  gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MAX : -gyro.y[i];
790:  gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MAX : -gyro.z[i];
```

Accel and gyro are treated identically, so there is no accel/gyro frame
mismatch to worry about. **Verified** (read via `gh api`).

**ArduPilot** takes the axes as read.
`libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp:533`:

```c
Vector3f accel{float(d.accel[0]), float(d.accel[1]), float(d.accel[2])};
Vector3f gyro{float(d.gyro[0]), float(d.gyro[1]), float(d.gyro[2])};
```

and hands them straight to `_rotate_and_correct_accel()`. **Verified.**

Negating y and z is exactly `ROTATION_ROLL_180`: (x,y,z) -> (x,-y,-z).

### The translation rule

    ArduPilot hwdef rotation  =  PX4's -R  composed with  ROLL_180 applied FIRST

i.e. `R_ap = R_px4 ∘ ROLL_180`. **Copying a PX4 `-R` value straight into an
ArduPilot hwdef is wrong by 180 degrees of roll, every time.** That is what was
done here, and it is the whole bug.

---

## 3. The rule validated on hardware neither of us configured

Holybro Pixhawk 6X REV6 exists in both autopilots, so both values are known
independently. ArduPilot from
`libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X/hwdef.dat` (`BOARD_MATCH(FMUV6_BOARD_HOLYBRO_6X_REV6)`),
PX4 from `boards/px4/fmu-v6x/init/rc.board_sensors`:

| Part | PX4 `-R` | `-R` composed with ROLL_180 | ArduPilot hwdef says | |
|---|---|---|---|---|
| `iim42652` | `-R 6` = `YAW_270` | `ROLL_180_YAW_270` | `ROTATION_ROLL_180_YAW_270` | match |
| `icm45686` | `-R 10` = `ROLL_180_YAW_90` | `YAW_90` | `ROTATION_YAW_90` | match |

Two independent matches. Computed numerically against the actual case bodies in
`libraries/AP_Math/vector3.cpp`, not from the enum names. **Verified.**

(`adis16470` is a different driver with its own convention and is not evidence
either way for the Invensense rule.)

---

## 4. What that makes the correct value for this board

PX4's facts for FMUv6X-RT, verified via `gh api` against
`boards/px4/fmu-v6xrt/`:

`src/spi.cpp`, for hardware types V6XRT000 and V6XRT001:

```
LPSPI1: DRV_IMU_DEVTYPE_ICM42686P
LPSPI2: DRV_IMU_DEVTYPE_ICM42688P
LPSPI3: DRV_GYR_DEVTYPE_BMI088 + DRV_ACC_DEVTYPE_BMI088
```

`init/rc.board_sensors`:

```
icm42688p -6 -R 12 -b 1 -s start      # bus 1
bmi088 -A -R 4 -s start ; bmi088 -G -R 4 -s start
icm42688p    -R  6 -b 2 -s start      # bus 2
bmm150 -I start                        # INTERNAL, no -R
bmp388 -I -b 3 -a 0x77 ; bmp388 -I -b 2
```

PX4 uses **no global board rotation at all**. Our board is a revision PX4
supports, and the `0x44` WHOAMI on lpspi1 is the ICM-42686P that PX4 expects
there - an earlier doubt about the board revision was unfounded.

**Only socket 2 is live for us.** `check_whoami()` in
`AP_InertialSensor_Invensensev3.cpp` accepts ICM40609, ICM42688 (P and V),
ICM42605, ICM40605, IIM42652, IIM42653, ICM42670, ICM45686 and ICM56686 - and
**no ICM-42686** - so lpspi1 never initialises. `INS_ACC_ID` is set with no
`INS_ACC2_ID`/`ACC3_ID`, confirming one IMU as instance 0. **Verified.**

Applying the rule:

| Socket | PX4 | AP equivalent |
|---|---|---|
| `imu_sensor2` (LPSPI2, the live one) | `-R 6` `YAW_270` | **`ROTATION_ROLL_180_YAW_270`** |
| `imu_sensor1` (LPSPI1) | `-R 12` `PITCH_180` | no single-enum match; inert anyway, no AP driver |
| BMI088 (LPSPI3) | `-R 4` `YAW_180` | no single-enum match; currently undeclared in our hwdef |

**Proposed, NOT yet verified on hardware:**

```
IMU Invensensev3 SPI:imu_sensor2 ROTATION_ROLL_180_YAW_270
AHRS_ORIENTATION = 0
```

---

## 5. Why the correction must be per-IMU and not AHRS_ORIENTATION

`AP_Compass_Backend::rotate_field()`:

```c
if (!state.external) { mag.rotate(_compass._board_orientation); }  // AHRS_ORIENTATION
else                 { mag.rotate(state.orientation.get()); }       // COMPASS_ORIENT
```

and `libraries/AP_AHRS/AP_AHRS_Backend.cpp:86-87` feeds `AHRS_ORIENTATION` into
**both** the INS and the compass - but `rotate_field()` only uses it when that
compass is **internal**. Our hwdef declares the BMM150 **external**
(`COMPASS BMM150 I2C:2:0x10 true ROTATION_NONE`), where PX4 starts it `-I`
internal. So a non-zero `AHRS_ORIENTATION` rotates the IMU and leaves the
compass behind, putting the two frames 180 degrees apart - the constant compass
error and the `AngErr=178`. A per-IMU rotation is applied inside the backend,
before offsets, so the compass never sees the discrepancy. **Verified** from
source.

---

## 6. The day-1 warning was correct, and this is the proof

`pre-flight-fixes.md` section 1 says:

> `AHRS_ORIENT=8` corrects the Z axis, so a level board reads a convincing
> `accel z = -1 g` while the horizontal frame is yawed 180 degrees. It looks
> right on the bench and diverges on takeoff.

Computed against `vector3.cpp`:

| Configuration | Net rotation | Single-enum equivalent |
|---|---|---|
| Bench-level: `YAW_270` in hwdef + `AHRS_ORIENTATION 8` | `ROLL_180` after `YAW_270` | `ROTATION_ROLL_180_YAW_90` |
| Correct PX4 equivalent | `YAW_270` after `ROLL_180` | `ROTATION_ROLL_180_YAW_270` |

Those differ by **`YAW_180`** - forward is backward. The bench-level
configuration is exactly the trap described above: correct Z, 180 degrees of
yaw error, and a stationary board cannot reveal it because gravity has no
heading. **Verified by arithmetic.**

Two consequences worth stating plainly. A level roll/pitch reading is **not**
evidence that an orientation is right. And `ROTATION_ROLL_180_YAW_90`, which was
proposed earlier in the session from that bench reading, is the wrong-by-180
value; it was composed in the wrong order (board-rotation-after instead of
driver-flip-before) and would have flown backwards.

---

## 7. Still open

- **Nothing in section 4 is confirmed on hardware.** Needs a build, a flash, and
  then the physical check: nose down pitches down, roll right rolls right, yaw
  right increases heading.
- `imu_sensor1` and the BMI088 composed rotations have no single-enum match, so
  if either is ever driven they need separate handling.
- The BMI088 on LPSPI3 is wired and started by PX4 with `-R 4` but is not
  declared in our hwdef.
- The compass external/internal divergence from PX4 remains. It is not needed to
  fix this bug once the rotation is per-IMU, but it is a real difference.
- Accelerometer, compass and RC calibration have still never been done on this
  board, and all of them must be redone after any rotation change.

---

## 8. Compass cal requires the ENTIRE VEHICLE powered

**The compass and the barometer are on the main battery rail. USB power alone is
not enough.** With the battery off they are detected but never produce samples,
and compass calibration cannot start.

Symptoms seen with USB power only, battery off:

```
COMMAND_ACK: DO_START_MAG_CAL: FAILED
AP: Compass calibration failed to start
COMMAND_ACK: FIXED_MAG_CAL_YAW: FAILED
AP: Mag[0]: unhealthy
```

and measured over MAVLink:

```
mag triple CHANGED 1 time in 30 s  ->  ~0.03 Hz
3D_MAG        present=1 health=0
ABS_PRESSURE  present=1 health=0      <- the baro too, same rail
3D_GYRO       present=1 health=1      <- SPI, on USB power
3D_ACCEL      present=1 health=1
```

With the battery connected, the same measurement:

```
mag changed 64 times in 28 s  ->  ~2.3 Hz
3D_MAG health=1   ABS_PRESSURE health=1   AHRS health=1
```

Why it presents as a calibration fault rather than a power fault:
`Compass::_start_calibration()` in `AP_Compass_Calibration.cpp:74` tests
`!healthy(i)` **first** and returns false with **no message**. Only the later
guards - priority change, allocation, GPS lock, thread creation - explain
themselves. So an unpowered compass produces a bare "failed to start" with no
reason given. `Compass::healthy()` is simply "a sample arrived within the last
500 ms".

**Do not read an unhealthy compass or baro as a driver, orientation or
calibration problem until the vehicle is fully powered.** That mistake was made
here: the 183 mGauss reading recorded on day 1, which looked like missing
hard-iron offsets, was a stale sample from an unpowered sensor. The field
magnitude with power on is about 512 mGauss, which is correct for Australia.

### Still marginal once powered

At ~2.3 Hz the compass is slower than the ~10 Hz AP normally reads, and the
gaps flap across the health threshold:

```
gaps between samples: min 50 ms, median 254 ms, max 3681 ms
gaps over 500 ms: 10 of 63
```

So `healthy()` goes false intermittently even with the battery on, and a
calibration command can still be rejected if it lands in a gap - **retry before
concluding anything**. A 3.7 second stall is a separate, real problem: the
backend registers a periodic callback at `MEASURE_TIME_USEC`
(`AP_Compass_BMM150.cpp:230`), so a gap that long means the callback is not
being serviced on time, which is the starved-scheduling signature recorded in
day-1 section 7 and is NOT explained by power.

---

## 9. MAVFTP: Scheduler::delay() sleeps far too long

**MAVFTP is not broken. It is ~400x too slow to answer, and every GCS times out
before it does.** Established 2026-09-27 with counters added to GCS_FTP.

### MAVFTP works when driven slowly

Driving `OpenFileRO @SYS/threads.txt` by hand, six attempts at 8 second
intervals, over the USB CDC link: **six requests, six FILE_TRANSFER_PROTOCOL
replies.** Verified.

```
12:25:22  seq=0  push=1 qspace=4  spins=0  pops=0  replies=0   enter=0
12:25:30  seq=1  push=1 qspace=3  spins=0  pops=0  replies=0   enter=0
12:25:38  seq=2  push=1 qspace=3  spins=0  pops=1  replies=0   enter=0
12:25:46  seq=3  push=1 qspace=2  spins=0  pops=1  replies=0   enter=0
12:25:47-49                        <- replies #1..#4 all arrive at once
12:25:54  seq=4  push=1 qspace=4  spins=5  pops=4  replies=4   enter=4 lock=4 ok=4
12:26:02  seq=5  push=1 qspace=4  spins=14 pops=5  replies=5   enter=5 lock=5 ok=5
```

### What is exonerated

`send_reply()` was bracketed with counters at each early-return:

- `txbuf_fail=0` - the radio flow-control gate
  (`GCS_MAVLINK::last_txbuf_is_greater(33)`) never rejects. It cannot on USB:
  with no radio, `last_radio_status.received_ms` stays 0, so the
  `millis() - received_ms > 5000` stale-report branch returns true.
- `nospace=0` - `HAVE_PAYLOAD_SPACE` never fails.
- `enter == lock == ok` - no blocking on `comm_chan_lock(chan)`, and every
  attempted send succeeded immediately.

So: no deadlock, no channel-space shortage, no radio gate, and no UART write
problem. Three earlier theories of mine are dead, and so is one more:
**console/MAVLink contention on the CDC stream is NOT the cause** - FTP
succeeded with the full console diagnostics printing throughout.

### The actual defect

The worker's idle loop is `hal.scheduler->delay(2)`
(`GCS_FTP.cpp`, `while (!requests.pop(request))`), which implies roughly 500
iterations per second. Measured: `spins` went 5 -> 14 in 8 seconds, so **9
iterations in 8 s, about 1.1 Hz** - around 400x slow. The first reply took
**25 seconds** to appear after the first request.

`Scheduler::delay(uint16_t ms)` in `AP_HAL_Zephyr/Scheduler.cpp` is a deadline
loop that sleeps in **1 ms steps**:

```c
const uint64_t start_us = AP_HAL::micros64();
const uint64_t target_us = (uint64_t)ms * 1000U;
while (AP_HAL::micros64() - start_us < target_us) {
    k_msleep(1);
    ...delay callback, main thread only...
}
```

So a `delay(ms)` issues **ms separate sleeps and ms separate wakes**, and each
wake has to win the CPU again before it can re-test the deadline. On main at
PREEMPT(3) that is cheap. At PRIORITY_IO, below main, tmr, rcin, rate, rcout and
the bus threads, it is not - which is why this presents as "the FTP thread is
never scheduled" and why it worsened as the board got busier. INFERRED: the
per-priority split is the proposed mechanism and is being measured (see below).

### Why it looks like a queue or a scheduling bug

At ~1 Hz the worker handles about one request per second. A GCS retries far
faster, the request queue is `AP_MAVLINK_FTP_MAX_SESSIONS` = **5** deep, so it
saturates within seconds and everything after is dropped on arrival with
`push=0 qspace=0`. MAVProxy's FTP timeout is far shorter than the 25 s the
worker needs, so it always gives up. That also explains the day-1 observation
that FTP "worked once immediately after boot": an empty queue and few retries is
the only condition a 1 Hz worker can service.

Note `ResetSessions` calls `send_reply(reply)` once and **discards the return
value**, so a dropped ACK is invisible to the GCS, which then retries the reset -
filling the queue faster.

### Before changing delay()

- **The deadline loop is itself a deliberate earlier fix.** Its comment records
  that counting a fixed number of iterations ADDED the delay callback's cost to
  the wait, so `delay(100)` took 300 ms with a 2 ms callback. Any fix must keep
  the deadline behaviour and stop issuing one wake per millisecond - e.g. sleep
  the whole remaining interval when there is no callback to run, and step 1 ms
  only when `_min_delay_cb_ms <= ms` on the main thread.
- **There is a standing warning not to re-fix `delay()` without measuring it on
  hardware first**, because it has been "fixed" wrongly before. A DELAYPROF
  probe (mean requested vs actual for `delay(<=4ms)`, split main vs non-main) was
  added to the 10 s report for exactly this. **Its result is not in this document
  yet.**
- The separate unbounded `while (!send_reply(reply))` retry in the non-Reset
  path has no timeout. It is not the current fault, but on a link that stops
  accepting it would park the worker permanently.

---

## 10. RETRACTION of section 9's cause: delay() is NOT the problem

Section 9 above is left as written, but **its conclusion is wrong and is
retracted here.** Measured 2026-09-27, minutes after writing it.

### delay() is accurate

A DELAYPROF probe was added to `Scheduler::delay()` recording mean requested vs
mean actual for `delay(<=4ms)`:

```
DELAYPROF main n=54 req=1851us act=2021us x1.09 | other n=0 req=0us act=0us x0.00
```

**x1.09 - a 9% overshoot, not 400x.** Section 9's "~400x too slow" figure was
derived from the rate at which the FTP worker's `spins` counter advanced. That
inference was invalid: **`spins` only increments when `requests.pop()` FAILS**,
i.e. when the queue is empty. With requests arriving and being serviced, a low
`spins` rate says nothing whatever about how long `delay()` sleeps. A timing
claim was made from a counter that does not measure time.

`other n=0` also shows the worker never called `delay(2)` in that window, so it
never reached the idle loop at all - which is itself the clue section 9 missed.

### What the four runs actually show

| Run | Path | Result |
|---|---|---|
| 12:25 | `@SYS/threads.txt` | 6 requests, **6 replies** |
| 12:29 | `@SYS/threads.txt` | **0 replies**, parked at `pops=1 enter=0` |
| 12:31 | `@PARAM/param.pck` | void - queue still saturated from the 12:29 park |
| 12:33 | `@PARAM/param.pck` | 6 requests, **3 replies**, then parked at `pops=4 enter=3` |

The pattern is the same each time it fails: the worker pops a request, and
`dbg_send_enter` never increments for it. **It parks inside the OpenFileRO
handling, before `send_reply` is ever called**, after a variable number of
successful transactions, and never recovers. The queue then saturates and every
later request is dropped with `push=0 qspace=0` - which is the state originally
reported as "MAVFTP is broken".

It happens on both `@SYS` and `@PARAM` paths, so it is not specific to the
thread-walking that generates `@SYS/threads.txt`.

### Everything now excluded, with the evidence

- **delay() / timing** - measured x1.09. Retracted above.
- **Scheduling, priority, CPU sharing** - the worker demonstrably runs and
  completes transactions; `spins`, `pops`, `enter`, `ok` all advance.
- **Deadlock on `comm_chan_lock`** - `enter == lock` on every attempted send.
- **Channel buffer space** - `nospace=0`.
- **Radio flow control** - `txbuf_fail=0`; on USB the stale-report branch of
  `last_txbuf_is_greater()` returns true anyway.
- **Console/MAVLink contention on the CDC stream** - FTP returned 6/6 replies
  with the full console diagnostics printing throughout. Both belong on the CDC
  by design; if interleaving ever did break framing, that would be a parser
  defect, not a reason to remove console output.
- **Queue saturation as a root cause** - it is a CONSEQUENCE of the park, not a
  precondition: the 12:33 run began with a fresh queue after a reboot.

### Where to look next

The hang is inside the AP_Filesystem open path reached from `OpenFileRO`,
non-deterministic, and permanent once entered. Bracketing counters between the
`pops++` and the `send_reply()` call - around `setup_reply()` and around the
filesystem open itself - would localise it to a statement the way the
`send_reply` bracket did.

Worth noting alongside: the standing note that `@SYS` MAVFTP fetches "often
fail, retry 2-4x" describes the same area, except this parks permanently rather
than failing and retrying.

---

## 11. The below-main threads' CPU scales with the ACHIEVED loop rate

`AP_SCHEDULER_LOOP_YIELD_US` (HAL_Zephyr_Class.cpp:68, applied at :293) is the
only CPU that any thread below main ever receives on this board. Measured, not
assumed: with the machine 100% busy and 0% idle, main released **4.48%** of the
CPU with its boost dropped, against `100 us x 466 loops/s = 4.66%` predicted.
That agreement is the proof - everything the prio>=7 threads get arrives through
that single call.

The consequence is that their income is a PRODUCT:

    income = AP_SCHEDULER_LOOP_YIELD_US x achieved_loops_per_second

so it falls when *either* term falls. At 443 Hz with a 400 us yield the band
receives ~17.7% of the machine; at 290 Hz the same yield gives only ~11.6%. A
third of the band's CPU disappears without anything else changing.

Why this is worth writing down: it inverts the intuition that lowering the loop
rate "frees up CPU". On this HAL a lower loop rate hands the IO band LESS, because
there are fewer yields per second. Anything living below main - storage, log_io,
AP_io, compasscal, the MAVFTP worker, and the I2C sensor buses at prio 7 - gets
squeezed, not relieved.

Measured effects of the band being squeezed, same build, disarmed:
- I2C transfers per 10 s window: 427-498 -> 56-92
- mean I2C transfer WALL time: 5-7 ms -> 63-98 ms (the transfer is ~200 us of
  work; the rest is preemption)
- worst single transfer: 45-86 ms -> 509 ms, i.e. back over the 500 ms
  `Compass::healthy()` window
- `AP_Logger::io_timer()` entries per 10 s: ~10000 expected, 23-58 observed

### What is NOT established - a theory that failed its own test

It is tempting to conclude "lowering SCHED_LOOP_RATE caused the regression", and
two data points appeared to support it: 600 -> 423-443 Hz (ratio 0.74) and
400 -> 277-310 Hz (ratio 0.73), suggesting the loop simply achieves ~73% of
whatever it is commanded.

**That was tested by setting it back to 600, and it is refuted.** The third point
does not fit:

    SCHED_LOOP_RATE 600 (earlier)  -> 423-443 Hz   ratio 0.74
    SCHED_LOOP_RATE 400            -> 277-310 Hz   ratio 0.73
    SCHED_LOOP_RATE 600 (restored) -> 307-336 Hz   ratio 0.52   <-- does not fit

Restoring 600 did not restore the loop rate, so SCHED_LOOP_RATE is not the cause
of the drop from ~440 Hz to ~310 Hz. Something else changed in between and is
still unidentified; the boost count also fell from b=1054-1360 to b=276-400 per
10 s, which is a further clue (fewer `wait_for_sample()` boosts means fewer
loops, not merely slower ones). Per-thread CPU during the degraded state shows
`main3=53-55% SPI2=10-11% AP_timer2=11% AP_rcin6=10-12%` - i.e. **no new
consumer**, main is unchanged, so the missing time is not a thread that started
eating CPU.

The mechanism in the first half of this section stands on its own measurement.
The causal claim about SCHED_LOOP_RATE does not, and is recorded here only so it
is not re-proposed.
