# RP2350 XIP Cache Profiler — Notes & Run Log

## Background

The RP2350B0 has a single **shared 16KB XIP cache** serving both cores,
physically constructed as two 8KB banks with interleaved odd/even cache lines
at 8-byte granularity — both Core0 and Core1 compete for the same capacity. All application code executes from flash
via XIP. Cache misses stall the CPU for ~50–100 ns while the QMI/QSPI
controller fetches a 16-byte cache line. At 375 MHz this is 19–37 cycles per
miss — catastrophic for tight loops.

Because the cache is **shared**, any thread running on either core can evict
lines that another thread (on either core) was using. Promoting hot functions
to SRAM reduces cache pressure for all threads regardless of which core they
run on.

`AP_XIP_PROFILER_ENABLED` (defined in `hwdef.dat`) activates the profiler:
- `CH_CFG_CONTEXT_SWITCH_HOOK` in `chconf.h` calls `ap_xip_cs_hook()` on
  every context switch.
- The hook snapshots and resets the RP2350 hardware counters
  `XIP_CTRL->CTR_HIT` and `XIP_CTRL->CTR_ACC` (21-bit saturating), then
  accumulates them per-thread in a SRAM table.
- Results are emitted in `@SYS/threads.txt` under the `XIPCacheV1` header.

Output format: `<thread>   HIT=<n> ACC=<n> (<pct>%) W=<windows>`
- `HIT` — cumulative cache hits
- `ACC` — cumulative cache accesses (hits + misses)
- `pct` — hit rate; 100% means all code was in cache, low % means misses
- `W` — number of context-switch-out events (sample windows)

**Cost of the profiler itself:** ~4 MMIO reads/writes + linear search up to 24
entries on every context switch. Estimate ~2–5% CPU overhead at high thread
switch rates. Disable for production runs (`#` out the `define` in hwdef.dat).

## Reading the data

A thread with:
- **99%** — essentially all code resident in cache, no action needed
- **97%** — minor pressure, low priority
- **92–94%** — significant miss pressure, candidate for ramfunc2 promotion
- **< 90%** — high priority, must be in SRAM

The `rate` thread (400 Hz attitude controller) and `SPI0` (IMU DMA) are the
most latency-sensitive: a miss in the rate loop directly widens the control
timestep. The `ArduCopter` main thread runs at ~400 Hz but is less jitter-
sensitive than `rate`.

## ramfunc2 promotion

Functions in `rp2350_ramfunc2_registry.txt` are placed in the `.ramfunc2`
SRAM section at link time. The build script (`PICO2.py` via the waf generator)
reads the registry, grabs the compiled `.o` symbol via `objcopy`, and injects
it with `--add-section`.  

RAM budget: the `.ramfunc2` section shares SRAM with the rest of the app.
Promoting too many large functions will overflow SRAM or starve the heap.
Check `BUILD SUMMARY` BSS+Data vs available SRAM (512 KB on RP2350B0).

---

## Run log

### Run 1 — 2026-06-04 — deliberately degraded (ArduCopter entries commented out)

> **NOTE: This is NOT a true baseline.** The registry was intentionally
> knobbled for this run — all 8 `ArduCopter/` entries were commented out to
> create a known-bad state for comparison. The other sections
> (`AC_AttitudeControl`, `AP_AHRS`, `AP_InertialSensor`, `AP_NavEKF3`, etc.)
> were still active. Results show the penalty of removing just the ArduCopter
> hot-path functions from SRAM.

**Registry state:** Only `rate_thread`, `AC_AttitudeControl`, `AP_AHRS`,
`hrt`, `xip_profiler`, fault handlers, `AP_InertialSensor`, `AP_NavEKF3`
in SRAM. All 8 `ArduCopter/` entries commented out.

**Perf:** `main=99Hz rate=98Hz load=100%`

| Thread       | Hit%  | Misses     | Windows | Miss/win |
|--------------|-------|------------|---------|----------|
| rate         | 92%   | ~12.3M     | 43,207  | **285**  |
| SPI0         | ~94%  | ~155M      | 669,907 | **231**  |
| ArduCopter   | 97%   | ~130M      | 766,598 | 170      |
| timer        | 97%   | ~36M       | 227,640 | 158      |
| I2C0         | 97%   | ~766K      | 9,810   | 78       |
| rcin         | 98%   | ~3.4M      | 43,439  | 78       |
| idle (Core1) | 97%   | ~12.7M     | 43,478  | 292      |
| All others   | 99%   | —          | —       | —        |

**Observations:**
- `rate` at 92% is the worst *rate-sensitive* thread — 285 misses per window
  directly delays the 400 Hz control loop.
- `SPI0` at 94% has the highest absolute miss count (155M) and highest
  miss/window (231). SPI0 = ICM-42688 DMA handler; misses here stall
  IMU sample delivery to the rate thread.
- `idle (Core1)` miss/win=292 is high but idle threads don't matter for
  latency.
- `load=100%` despite only 99 Hz main loop indicates CPU saturation from
  XIP stalls + profiler overhead.

---

### Run 2 — 2026-06-04 — half ArduCopter entries restored

**Registry state:** Restored 4 of 8 commented entries:
- `ArduCopter/Attitude.cpp|Copter::run_rate_controller_main`
- `ArduCopter/Copter.cpp|Copter::read_AHRS`
- `ArduCopter/motors.cpp|Copter::motors_output`
- `ArduCopter/motors.cpp|Copter::motors_output_main`

Still commented: `ekf_check`, `land_detector`, `update_flight_mode`, `update_precland`

**Perf:** `main=~100Hz rate=~100Hz` (load not captured; SPI0/ArduCopter improved noticeably)

| Thread       | Hit%  | Misses (disp) | Windows   | Miss/win |
|--------------|-------|---------------|-----------|----------|
| rate         | 92%   | 16,486,466    | 88,693    | **186**  |
| SPI0         | 97%   | †             | 837,589   | **~207†**|
| ArduCopter   | 98%   | †             | 977,164   | ~58†     |
| timer        | 97%   | †             | 282,498   | ~155†    |
| I2C0         | 97%   | †             | 8,538     | ~85†     |
| idle (Core1) | 97%   | †             | 89,002    | ~75†     |
| All others   | 99%   | —             | —         | —        |

† HIT/ACC counters are 21-bit saturating hardware registers accumulated into
  uint64 SRAM tallies; the displayed 32-bit values may be low-32 truncations
  of the real uint64 total, so miss counts marked † are estimates from %.

**Observations:**
- `SPI0` jumped from ~94% → 97% — the 4 restored ArduCopter functions
  (`run_rate_controller_main`, `read_AHRS`, `motors_output`,
  `motors_output_main`) significantly reduced SPI0 cache eviction pressure.
- `ArduCopter` improved marginally 97% → 98%.
- `rate` is unchanged at 92% — those 4 functions are apparently not the
  source of the `rate` thread's miss pressure.
- SPI0 miss/win dropped 231 → ~207 (10% improvement).
- rate miss/win dropped 285 → 186 (35% improvement!) despite no hit% change —
  indicates the absolute ACC count per window shrank, meaning rate windows
  are shorter (better scheduling) even if in-window hit% is the same.

---

### Run 3 — 2026-06-04 — all 8 ArduCopter entries active

**Registry state:** All 8 `ArduCopter/` entries active, plus all previous
(`rate_thread`, `AC_AttitudeControl`, `AP_AHRS`, `hrt`, `xip_profiler`,
fault handlers, `AP_InertialSensor`, `AP_NavEKF3`).

Added vs Run 2:
- `ArduCopter/ekf_check.cpp|Copter::check_ekf_reset`
- `ArduCopter/land_detector.cpp|Copter::update_land_and_crash_detectors`
- `ArduCopter/mode.cpp|Copter::update_flight_mode`
- `ArduCopter/precision_landing.cpp|Copter::update_precland`

**Perf:** `main=115Hz rate=109Hz load=100%`

| Thread       | Hit%  | Misses     | Windows | Miss/win |
|--------------|-------|------------|---------|----------|
| rate         | 91%   | 749,424    | 3,587   | **209**  |
| SPI0         | 99%   | 9,578,279  | 46,038  | **208**  |
| ArduCopter   | 99%   | 10,496,325 | 57,421  | 183      |
| timer        | 99%   | 2,194,922  | 20,794  | 106      |
| All others   | 99%   | —          | —       | —        |

**Observations:**
- `SPI0` went from 94% (Run 1) → 97% (Run 2) → **99%** (Run 3). The
  additional 4 ArduCopter entries (`ekf_check`, `land_detector`,
  `update_flight_mode`, `update_precland`) eliminated the remaining SPI0
  eviction pressure entirely.
- `ArduCopter` reached **99%** — fully resident.
- `rate` is stuck at 91–92% across **all three runs**. The rate thread is
  pinned to Core1 (`thread_create_pinned_to_core(..., rate_core=1)`) but the
  XIP cache is **shared** — so Core0 promotions do relieve shared cache
  pressure. However the rate thread's own hot callees (`AC_PID::update_all()`,
  `AC_AttitudeControl_Multi::rate_controller_run()`,
  `AP_MotorsMulticopter::output()`) are still in flash and consume enough
  cache lines that overall rate hit% remains at 91% despite the freed
  capacity.
- SPI0 miss/win dropped from 231 (Run 1) → ~207 (Run 2) → **208** (Run 3)
  — effectively flat vs Run 2. The big SPI0 improvement was Run 1→2 when
  the most-evicting ArduCopter paths moved to SRAM.
- `load=100%` persists. The rate thread's direct callees are the next
  priority for SRAM promotion.

---

### Run 4 — 2026-06-05 — profiler OFF, rate callees in SRAM, 93.75 MHz flash

**Registry state:** All Run 3 entries plus all rate-thread callees added in
between sessions:
- `AC_AttitudeControl_Multi::rate_controller_run`, `rate_controller_run_dt`,
  `update_throttle_gain_boost`, `update_throttle_rpy_mix`
- `AC_PID::update_all`
- `AP_MotorsMulticopter::output`, `output_logic`
- `AP_MotorsMatrix::output_to_motors`, `output_armed_stabilizing`
- `AP_InertialSensor::wait_for_sample`, `get_next_gyro_sample`
- `FastRateBuffer::get_next_gyro_sample`
- `ChibiOS::SPIDevice::do_transfer`, `acquire_bus`, `transfer`
- `ChibiOS::SPIBus::stop_peripheral`, `start_peripheral`, `spi_lld_*`
- `ChibiOS::RCOutput::push_local`, `push`
- `NavEKF3_core::runYawEstimatorPrediction`, `correctDeltaAngle`,
  `correctDeltaVelocity`
- USB TX/RX poll helpers, UART DMA helpers
- `Copter::read_inertia`

**hwdef.dat:** `# define AP_XIP_PROFILER_ENABLED 1` (profiler disabled)
**QMI:** `RP_QMI_CLKDIV 4` / `RP_QMI_RXDELAY 2` → SCK **93.75 MHz** (up from
62.5 MHz in Run 3). Correct formula: `sample = (CLKDIV+RXDELAY)/(2×sysclk)`.
At 375 MHz: sample = 8 ns (2 ns past tCLQV=6 ns), hold = 2.67 ns. ✓

**Perf (stable range):**

| Reading             | main Hz | rate Hz | load |
|---------------------|---------|---------|------|
| First sample        | 190     | 164     | 66%  |
| Peak rate           | 151     | 329     | 72%  |
| Typical             | ~165    | ~200    | ~72% |

No `XIPCacheV1` section in threads.txt (profiler disabled).

**threads.txt stack snapshot:**
```
rate          PRI=182  STACK=0/4320      ← OVERFLOW (0 bytes free)
ArduCopter    PRI=180  STACK=5432/7168
main          PRI=128  STACK=16264/105008
ISR           PRI=255  STACK=24208/24576
```

**Observations:**
- Load dropped from **100% → 66–79%** with profiler off and rate callees in SRAM.
  This is the combined effect: ~2–5% from removing profiler overhead, the rest
  from eliminating flash stalls in the 329 Hz rate loop.
- Rate thread achieves **329 Hz** at peak vs 109 Hz in Run 3 — the adaptive rate
  controller is no longer CPU-saturated so it can run faster.
- **`rate` stack completely exhausted (0/4320 free)** → triggers
  `PreArm: Internal errors 0x800000 l:182 stack_ovrfl`. The rate thread stack
  needs to be increased; this is unrelated to QMI timing but must be fixed.
- CLKDIV=4 RXDELAY=2 is the **maximum safe SCK speed** on this board. All
  attempts above this (CLKDIV=4 with RXDELAY<2, CLKDIV=3) crashed due to
  violation of setup/hold timing. See hwdef.dat for full history.

---

### RXDELAY timing formula (corrected)

$$\text{sample} = \frac{\text{CLKDIV} + \text{RXDELAY}}{2 \times f_\text{sysclk}}$$

Constraints at 375 MHz, W25Q64JVXGIM ($t_\text{CLQV} \le 6$ ns):
- **Setup**: $\text{CLKDIV} + \text{RXDELAY} \ge 4.5$ → sum ≥ 5
- **Hold**: $\text{RXDELAY} \le \text{CLKDIV}$ (RXDELAY=CLKDIV means sample at
  period boundary, valid since $t_\text{CHQX} > 0$)
- Must also be valid at **150 MHz** (bootloader speed when dummy flash read
  runs in `rp_clocks.c` before PLL switch)

---

### Cross-run comparison

| Metric           | Run 1 (degraded) | Run 2 (4 entries) | Run 3 (8 entries) | Run 4 (rate callees, no profiler, 93.75 MHz) |
|------------------|------------------|-------------------|-------------------|----------------------------------------------|
| rate hit%        | 92%              | 92%               | 91%               | N/A (profiler off)                           |
| SPI0 hit%        | 94%              | 97%               | **99%**           | N/A                                          |
| ArduCopter hit%  | 97%              | 98%               | **99%**           | N/A                                          |
| timer hit%       | 97%              | 97%               | **99%**           | N/A                                          |
| main Hz          | 99               | ~100              | 115               | ~165                                         |
| rate Hz          | 98               | ~100              | 109               | **~200–329**                                 |
| load             | 100%             | 100%              | 100%              | **66–79%**                                   |
| flash SCK        | 62.5 MHz         | 62.5 MHz          | 62.5 MHz          | **93.75 MHz**                                |

### Next candidates for Run 4 — rate thread callees

Direct callees of `run_rate_controller_main` that remain in flash and are
the likely source of the rate thread's persistent 91% hit rate:

- `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp|AC_AttitudeControl_Multi::rate_controller_run`
- `libraries/AC_PID/AC_PID.cpp|AC_PID::update_all` (called 3× per rate tick)
- `libraries/AP_Motors/AP_MotorsMulticopter.cpp|AP_MotorsMulticopter::output`
- `libraries/AP_Motors/AP_Motors_Class.cpp|AP_Motors::output`
- `libraries/AP_InertialSensor/AP_InertialSensor.cpp|AP_InertialSensor::get_delta_angle`

These execute at 400 Hz and repeatedly fetch the same flash cache lines.
Promoting them to SRAM frees those lines for other threads too (shared cache).

---

## TODO / next steps

- [ ] Get Run 2 threads.txt and fill in the table above
- [ ] If `rate` improves to ≥97%, try commenting back `SPI0`-related entries
      to see if that's where the SPI0 miss pressure comes from
- [ ] If load drops below 100%, try disabling the profiler to measure true
      baseline performance
- [ ] Candidates for future promotion: `AP_Motors` (called from motors_output),
      `AC_PID` (called from rate controller)
- [ ] Consider promoting `AP_InertialSensor_Invensensev3::read_fifo` higher
      priority — it feeds SPI0 and currently sits at 94%



In the RP2350 Datasheet, RXDELAY is a crucial QSPI Memory Interface (QMI) parameter. It delays the sampling clock of incoming QSPI/SPI data to compensate for round-trip propagation delays, measured in units of half a system clock cycle.How it WorksAt high frequencies or with longer routing traces, data launched by the external Flash/PSRAM may arrive late at the RP2350's GPIO pads. If data setup and hold times are violated, you will get corrupted bytes or bit shifts. Adjusting RXDELAY shifts the RP2350's internal read capture window forward in time, aligning it perfectly with the incoming data stream.Where to Configure RXDELAYYou will find RXDELAY within the QMI registers (specifically M0_TIMING and M1_TIMING):Direct Mode / Boot (e.g., Flashing): Governed by DIRECT_CSR.RXDELAY.Sub-Bank/Window Settings: Configured via M0_TIMING.RXDELAY and M1_TIMING.RXDELAY.Common ConfigurationsTypical values for RXDELAY depend on your system clock and QMI clock divisor (CLKDIV):Standard / Default (e.g., 133MHz Core, 1:2 or 1:3 Divider): Often set to 0x2 or 0x3 (2 to 3 half-clock cycles).High-Speed / Overclocked (e.g., \(150\text{ MHz} +\)): Many developers scale RXDELAY to exactly match the divisor. For example, when running with CLKDIV at \(4\), RXDELAY is often set to \(4\).


Good — RXDELAY is in units of half a sysclk cycle, not a full cycle. That changes the math significantly.



At 375 MHz: 1 half-cycle = 1.333 ns

Flash (W25Q64JVXGIM) drives data on SCK falling edge, valid ≤ 6 ns later (tCLQV). RP2350 default samples on rising edge, RXDELAY shifts that sample point forward by N × 1.333 ns.

CLKDIV	SCK	Rising edge	RXDELAY needed (≥ tCLQV)	Capture point	Hold margin
3	125 MHz, period 8 ns	4.0 ns	N×1.333 ≥ 2.0 → N≥2	4.0 + 2×1.333 = 6.67 ns ✓	1.33 ns
4	93.75 MHz, period 10.67 ns	5.33 ns	N×1.333 ≥ 0.67 → N≥1	5.33 + 2×1.333 = 8.0 ns ✓	2.67 ns
Current CLKDIV=4 RXDELAY=4: capture at 5.33 + 5.33 = 10.67 ns — exactly at the next falling edge. That's a bug regardless of speed.

CLKDIV=3, RXDELAY=2 gives 125 MHz SCK (within W25Q64JVXGIM's 133 MHz fast-read spec) with a 1.33 ns hold margin — tight but correct per datasheet. That's the aggressive option.

Reviewed and updated hwdef.dat+9-6
Done. SCK 62.5 → 125 MHz (+100%