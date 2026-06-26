# Laurel / RP2350 Known Bugs and Flight-Blocking Issues

Last updated: 2026-06-26 (XIP stall safety fixes + SD card / logging fully fixed). Branch: `rp2350-v5-etc-dual-core`.

---

## OPEN — TOP PRIORITY (fix before next flight)

### PERF-001 / FLY-002 — EKF CPU overload → auto-decimates to 8× (123 Hz) → all GPS modes blocked
**Observed:** 2026-06-24. Tlog: `laurel_2026-06-24_20-36-19.tlog`.
**⚠️ Re-measurement needed (2026-06-26):** These numbers were taken before the XIP stall safety fixes (ARCH-003 below) and before SD card logging was working. ChibiOS scheduler and DMA code now run from SRAM instead of XIP flash, and the SD card no longer crashes the board at boot — both may shift the EKF duty cycle and Core1 load. Re-run tasks.txt profiling with current firmware before acting on the numbers below.
**Symptom (boot):** Every boot, EKF auto-decimation cascade fires within the first 40s:
`EKF CPU 99% (>50), decim->4` → `decim->5` → `decim->6` → `decim->7` → `decim->8`
Settles at: `C1: rate=988Hz ekf=123Hz ekf_duty=94% decim=8`. Core1 at 99%. Does not
improve once airborne.
**Symptom (flight):** With EKF barely converging at 123 Hz and saturated CPU, the EKF
solution is unreliable. Even with valid accel cal and excellent GPS (HDOP 0.58, 19 sats),
all GPS/altitude modes are blocked:
- Loiter, Auto, PosHold → `requires position`
- AltHold → `need alt estimate`
**Context:** EKF runs on Core1 (375 MHz Cortex-M33). Each EKF tick costs 1400–31000 µs
(min = prediction-only, max = GPS+baro measurement update). The slow ticks (~15 ms avg)
exceed the 8 ms slot at decim=8 → sustained 93–96% duty → CPU saturated.
Single EKF core (`EK3_IMU_MASK=1`). 24×24 float state matrix.
**Investigation path:**
  1. **Check FPU compiler flags** — Cortex-M33 has hardware FPU but it must be enabled:
     `-mfpu=fpv5-sp-d16 -mfloat-abi=hard`. If soft-float is used, all matrix ops are
     emulated in SW → 10–20× slower. Check `build/Laurel/compile_commands.json`.
  2. **Check EKF memory placement** — if state matrices land in slow SRAM or uncached XIP
     region, cache misses dominate. Check linker map for `NavEKF3` / `NavEKF3_core` symbols.
  3. **Try lower GPS measurement rate** — ublox at 5 Hz instead of 10 Hz halves expensive
     measurement-update ticks. Set `GPS_DRV_OPTIONS` bit to limit output rate.
  4. **Try `EK3_LOOP_RATE`** — pre-set decim to avoid the boot cascade of warnings.
**Not yet investigated.**

---

### BUG-004 — EKF completely dead; all GPS-assisted modes blocked
**Observed:** 2026-06-24 outdoor flight session. Tlog: `laurel_2026-06-24_20-36-19.tlog`.
**Symptom:** EKF_STATUS_REPORT flags = 0x0 throughout every armed interval (298 samples
across 4 arm events). No attitude solution, no velocity, no position, no altitude from
EKF. Vehicle stuck in STABILIZE. All mode-change attempts during arm #3 (5-minute armed
session) were rejected:
- `Mode change to Auto failed: requires position` (+578s)
- `Mode change to Loiter failed: requires position` (+591s, +602s, +604s, +605s)
- `Mode change to Position Hold failed: requires position` (+632s, +637s, +639s)
- `Mode change to Altitude Hold failed: need alt estimate` (+656s)
GPS was excellent throughout (HDOP avg 0.58, 17–19 sats, 100% 3D fix) — GPS is not the
problem.
**Root cause confirmed:** Accel calibration values not persisting (CAL-001). The
6-position calibration was performed multiple times (2026-06-23 and twice on 2026-06-24)
but `INS_ACCOFFS_*` are reverting to zero on each reboot or reflash. Without valid offsets
the accelerometer has a persistent ~0.4 m/s² lateral bias → DCM attitude error ~2–4° →
EKF3 detects DCM/EKF divergence (`AHRS: DCM Roll/Pitch inconsistent`, 10–25° reported)
and refuses to initialise → EKF flags stay 0x0 → all GPS/altitude modes blocked.
**`AHRS_ORIENTATION 8` (ROLL_180) — confirmed CORRECT** (2026-06-24): hand-hold test
showed corrections in the right direction. Board is mounted upside-down. Not the issue.
**Resolution:** Fix CAL-001 (parameter storage not persisting). Everything else unblocks.

---

## OPEN — Pre-flight Calibration (not code bugs, but block flight)

### CAL-001 — 6-position accel calibration not persisting across reboot (BUG)
**Symptom:** `PreArm: 3D Accel calibration needed` returns after every reboot or reflash
despite the full 6-position calibration having been completed. User performed the
6-position "Calibrate Accel" procedure at least twice on 2026-06-24 and once on
2026-06-23; the error reappeared each subsequent session. Correct behaviour is: do it
once, it persists in flash, never needed again until hardware changes.
**Root cause — two mechanisms, likely both contributing:**
  1. **Firmware reflash wipes parameter storage.** A firmware flash via OpenOCD
     (`program arducopter.bin ... 0x10010000`) erases all flash sectors the binary
     occupies. If the StorageFlash parameter area is within or adjacent to the flashed
     region, calibration values stored the previous day are wiped on reflash. This
     explains yesterday's cal being gone today.
  2. **INS_ACCOFFS_* may not be persisting even without reflash.** The two cals done
     on the evening of 2026-06-24 (before the flight in the tlog) also did not take.
     No firmware was reflashed between those cal attempts and the flight. This points to
     a separate StorageFlash write/read fault on RP2350: params are written but either
     not flushed to flash or the wrong flash area is being used.
**⚠️ Status update (2026-06-26):** The XIP stall safety fixes (ARCH-003) may have resolved mechanism 2 above. AP_Param flash writes were triggering DMA BusFaults and ChibiOS UNDEFINSTR crashes during the write window. If the write aborted mid-erase due to a crash, the parameter page would be left erased (0xFF) rather than written — appearing as params reverting to defaults after reboot. With the XIP stall fixes in place, flash writes should now complete reliably. **Re-test required before closing this bug:** perform a 6-position accel cal, reboot without reflashing, verify `INS_ACCOFFS_X/Y/Z` persist.
**Investigation needed (if still failing after re-test):**
  - After 6-position cal completes in Mission Planner, immediately fetch full param list
    and verify `INS_ACCOFFS_X/Y/Z` are non-zero and `INS_ACC_CALTEMP` is non-zero.
  - Reboot (without reflashing) and fetch params again — if INS_ACCOFFS_* revert to 0,
    the StorageFlash backend is still broken independently of XIP stall.
  - Check `HAL_STORAGE_SIZE` and StorageFlash page layout in hwdef.dat vs the actual
    binary size to confirm the parameter area is not being overwritten by OpenOCD.
  - Check `AP_Param::save_io_worker()` / `StorageFlash::write()` paths on RP2350 for
    any early-return or failed erase condition.
**Workaround (none available):** Until confirmed fixed, calibration must be redone after every
reflash. The flight blocks remain in place until this is resolved. See BUG-004.

### CAL-002 — RC not calibrated / not found
**Symptom:** `PreArm: RC not found`. Mode changes via RC switches impossible. Prevents
arming.
**Action:** Connect RC receiver to SERIAL3 (PIOUART0, GPIO20/21) with SERIAL3_PROTOCOL=23
(RCIN/CRSF). Calibrate RC in Mission Planner → Initial Setup → Mandatory Hardware →
Radio Calibration. For PPM: GPIO21 edge-capture via SoftSigReaderRP2350.

### CAL-003 — Battery voltage/current not calibrated
**Symptom:** `PreArm: Battery 1 low voltage failsafe`. Prevents arming.
**Action:** Measure actual battery voltage, set `BATT_VOLT_MULT` and `BATT_AMP_PERVLT`
for the Laurel hardware voltage divider and current sense resistor values. Set
`BATT_LOW_VOLT` and `BATT_CRT_VOLT` to appropriate values for the battery chemistry.

---

## OPEN — Performance / EKF

### PERF-002 — Main loop 400 Hz target not met; best achieved is 360–367 Hz (Config E)
**Observed:** bench testing, all bugs fixed. Branch: `rp2350-v5-etc-dual-core`.
**Symptom:** With the best known config (Config E: SPI0 on Core1, DCM backup at 1/8 rate, `ekf_decim_min=2`), the settled main loop rate is 360–367 Hz against a target of 400 Hz. Gap is ~35 Hz. Core0 is at 100% CPU. Core1 is at 97% (SPI + rate + EKF all on Core1). EKF adaptive decimation lands at decim=8 (EKF running 61 Hz) to keep Core1 duty cycle manageable.
**Remaining headroom:** Two concrete optimisations have been designed (see PERF-003 and PERF-004 below) that together are expected to close the gap: lock-free EKF publish saves ~40 Hz, cross-core IMU latency fix saves ~18 Hz — total projected 418 Hz.
**Config E regressions vs Config A that must be accepted or mitigated:**
- `InertialSensor::update*`: 156 µs → 405 µs (Core0 waiting on Core1 SPI DMA). See PERF-004.
- `GCS::update_receive`: 1269 µs → 1773 µs (Core1 at 97% → more SMP spinlock contention under every `chSysLock` Core0 issues).

---

### PERF-003 — `read_AHRS` semaphore stall: lock-free EKF result sharing not yet implemented
**Observed:** tasks.txt in Config E. `read_AHRS*` = 1070 µs AVG, MAX = 5547 µs.
**Root cause:** `AP_AHRS::update()` calls `WITH_SEMAPHORE(_rsem)` to read the EKF output. `_rsem` is also held by Core1 during EKF publish. When Core1 publishes (at 61 Hz) the Core0 call into `read_AHRS` blocks until Core1 releases the semaphore. The MIN=243 µs (fast path, no contention) vs MAX=5547 µs (contention path) variance drives scheduler irregularity.
**Fix designed, not yet implemented:** Replace `_rsem` with an atomic triple/double buffer. Core1 writes EKF results to a write slot then `std::atomic::store(release)` the published index. Core0 reads from `published.load(acquire)` — never blocks. Eliminates the semaphore contention entirely. Expected `read_AHRS` drop: 1070 µs → ~400–500 µs. At 181 Hz fast-task rate: 620 µs × 181 = 112 ms/s freed = ~+11% Core0 = **~+40 Hz**. Implementation risk: medium — needs correct `memory_order_acquire/release` across Cortex-M33 cores; only the publish/consume path changes, not EKF internals.

---

### PERF-004 — `InertialSensor::update` cross-core latency: 405 µs in Config E vs 156 µs in Config A
**Observed:** tasks.txt Config E. 405 µs AVG in fast task (target 156 µs).
**Root cause:** With SPI0 on Core1, Core0's `InertialSensor::update` must signal the Core1 SPI thread, then wait for the DMA completion to be posted back. Core1 is at 97% load — the SPI thread may queue behind the rate thread and EKF before being scheduled. The 250 µs extra round-trip is Core0 sleeping in a semaphore wait while Core1 catches up.
**Option A (raise SPI thread priority):** Set the SPI0 bus thread priority higher than EKF on Core1 so Core0-initiated requests are served immediately. Risk: more EKF preemptions → higher `ekf_dur` jitter → adaptive algorithm may decimate further.
**Option B (push model for IMU reads):** SPI thread posts completed IMU frames to a Core0-readable ring buffer without waiting for Core0 to ask. Core0 reads the latest frame without blocking. In theory this is already how it should work via the IMU FIFO path — investigate why `InertialSensor::update` still blocks in Config E before choosing an option.
**Expected saving:** ~250 µs × 181 Hz = 45 ms/s = ~+4.5% Core0 = **~+18 Hz**.

---

### DIAG-001 — `AP_InternalError` bitmask not preserved across reset; invisible after reboot
**Symptom:** When `INTERNAL_ERROR()` fires, the scheduler enters a fast-spinning empty loop (visible as an anomalously high Perf Hz reading — 500–700+ Hz is the diagnostic signature). After the watchdog fires and the board resets, the error bitmask is lost. The only way to confirm an InternalError occurred is to observe the Perf Hz anomaly live or to halt under GDB during the event. Post-reset diagnosis is impossible.
**Fix designed, not yet implemented:** Add a write to a WATCHDOG SCRATCH register inside `AP_InternalError::error()` (or its platform hook) saving the bitmask. SCRATCH registers survive PSM resets. The existing watchdog detection path in `rp2350_watchdog_init()` already reads SCRATCH[6]/[7] — add SCRATCH[8] (or repurpose a free slot) for the InternalError bitmask. After a WD reset, `HAL_ChibiOS_Class.cpp` can report it via GCS just as it does for the watchdog reset event. Filed as design intent; not yet implemented.

---

### ARCH-001 — Symmetric XIP lockout (Core1 → Core0) not implemented
**Symptom:** No current crash — Core1 does not write to flash today. If any future code path causes Core1 to call `AP_Param::save()` or trigger a flash erase (e.g. parameter storage relocated, or `AP_Logger_Flash` enabled on Core1), Core1 will call `rpEflBeforeXipOff()` which currently assumes it is always called from Core0. Core0 would not be parked. If Core0 is executing code from XIP flash while Core1 disables XIP, Core0 gets IBUSERR and crashes.
**Fix designed, not yet implemented:** `rpEflBeforeXipOff()` reads `SIO->CPUID` to determine which core is writing flash, then rings the OTHER core's doorbell. For Core1 → Core0 lockout, Core0 needs a matching SRAM-resident handler installed in `rp2350_vectors[42]` (Core0's RAM vector table in `board_rp2350.c`). The Core1→Core0 path is symmetric with the existing Core0→Core1 protocol. Deferred until a concrete need arises.

---

### ARCH-002 — Rate thread is not XIP-lockout-immune; misses ticks during flash erase
**Symptom:** During flash erase (parameter storage writes at first boot and after GCS param saves), Core1 is parked by the XIP lockout protocol. TIMER0_ALARM1 (Core1 tick) does not fire during the park window (~30–50 ms per erase sector). The rate thread misses ~30–50 tick periods. The tick re-arms immediately when Core1 resumes.
**Current acceptability:** Flash writes happen only at first boot and on user-initiated param saves — never during flight. The 30–50 ms gap at boot is acceptable.
**Future option — not yet pursued:** A rate loop whose entire call chain executes from SRAM (no XIP fetches) would not need to be parked and could maintain exact 1 kHz through flash erases. Requires moving the rate loop body, `AP_InertialSensor` fast path, and motor output path entirely into SRAM — very large footprint, significant refactoring. Not feasible without dedicated work.

---

## OPEN — Flight / Tuning

### FLY-001 — Stabilize mode: vehicle tips severely; cannot self-hold level
**Observed:** 2026-06-24 outdoor flight session. Tlog: `laurel_2026-06-24_20-36-19.tlog`.
**Symptom:** Vehicle was airborne in Stabilize across 4 arm events (57s, 10s, 290s, 90s).
During the 5-minute arm (#3), raw IMU data recorded three severe tilt spikes:
- +538s: `ax=8.50 m/s²` → ~60° tilt
- +718s: `ax=7.06 m/s²` → ~46° tilt
- +728s: `ax=8.14 m/s²` → ~56° tilt
The vehicle was tipping to near-horizontal and being caught. AHRS attitude estimate
(DCM) reported Roll ±11°, Pitch ±15° (average across all armed samples) — these are
the corrected attitude values; the raw IMU spikes show the vehicle was physically much
further over at those moments. Yawrate reached ±100 °/s, confirming dynamic instability.
**Root causes (two, independent):**
  1. **CAL-001 (accel calibration missing)** — EKF has no solution (flags 0x0). Stabilize
     falls back to DCM only, which itself has a biased attitude from the uncalibrated accel.
     The rate controller is working from a wrong reference frame.
  2. **Default PIDs not matched to Laurel** — default ArduCopter PIDs are sized for a
     generic ~450mm quad. Laurel frame geometry, motor KV, prop size, and all-up weight
     are not accounted for. Rate P/I/D gains need tuning.
**Action items (in order):**
  1. CAL-001: 6-position accel calibration — eliminates DCM attitude bias and allows EKF
     to initialise, giving the rate controller a correct attitude reference.
  2. Verify motor/ESC calibration — unequal thrust is indistinguishable from wrong PIDs.
  3. First flight attempt post-cal: fly conservatively in Stabilize at low altitude with
     spotter. If oscillating: reduce `ATC_RAT_RLL_P` / `ATC_RAT_PIT_P` by 20%.
     If sluggish/sagging: increase by 20%.
  4. Once able to hover, use AutoTune (`AUTOTUNE_AXES 3` for roll+pitch first).

### FLY-002 — Loiter / position / altitude modes unavailable
**Observed:** 2026-06-24 outdoor session. Tlog: `laurel_2026-06-24_20-36-19.tlog`.
**Symptom:** During arm #3 (5-minute armed session), while GPS was excellent (HDOP 0.58,
17–19 sats, 100% 3D fix), every GPS-assisted and altitude-assisted mode was rejected:
- Loiter, Auto, PosHold → `requires position`
- AltHold → `need alt estimate`
**Root cause:** EKF flags 0x0 throughout — no position, no altitude, no velocity estimate
from EKF (see BUG-004). GPS quality is not the problem; the EKF is simply not running.
**Resolution:** Fix CAL-001. Once accel calibration is done, EKF initialises, and GPS
lock (which was already present and healthy) provides the position solution. All these
modes should become available within ~30s of arming outdoors after cal is done.

