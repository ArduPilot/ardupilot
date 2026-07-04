# Laurel RP2350 de-overclocked baseline

The stable target configuration for Laurel, reached by moving the core1 rate
path into SRAM so the core clock and voltage could be walked back down from the
original 375 MHz / 1.30 V overclock. Confirmed on hardware.

## Configuration

| Item              | Value                          | Where                     |
|-------------------|--------------------------------|---------------------------|
| Core clock        | 225 MHz (1.5x the 150 MHz spec)| `hwdef.dat`               |
| PLL_SYS           | 900 MHz VCO / POSTDIV 4x1      | `RP_PLL_SYS_VCO_FREQ`      |
| Core voltage      | 1.15 V (VSEL 0x0c, ~312 MHz cap)| `RP_VREG_VSEL`           |
| QMI flash timing  | CLKDIV 3 / RXDELAY 2 (75 MHz SCK)| `RP_QMI_CLKDIV/RXDELAY`  |
| Main loop         | 200 Hz                         | `SCHED_LOOP_RATE`         |
| Gyro backend      | 4 kHz (INS_GYRO_RATE 2)        | `defaults.parm`           |
| Rate thread       | 2 kHz (FSTRATE_DIV 2, ENABLE 2)| `defaults.parm`           |

Measured on the bench at this config: core0 ~65%, core1 ~37%, core1 flash
share ~0.7%, rate-loop gyro-to-output latency ~190 us average.

A 1500 MHz VCO cannot divide to 225 MHz (1500 / 225 is not integer), so the VCO
itself drops to 900 MHz (FBDIV 75) and POSTDIV 4x1 yields 225 MHz.
`clk_peri = clk_sys`, so the SPI clock scales down with the core clock - this
only makes the IMU SPI safer. `HAL_EXPECTED_SYSCLOCK` is cosmetic on RP2350 (the
compile-time check is skipped for dynamic-clock parts).

## How the clock came down

The overclock existed to hide XIP flash latency: both cores fetch instructions
through a shared 16 KB XIP cache, and core0's EKF plus core1's rate loop were
both thrashing it. The fix was to make core1's hot path SRAM-resident so it
stops competing for the cache, leaving it to core0:

1. Relocate the core1 rate/IMU path (rate controller, IMU read, motor mixing,
   filters, scheduler glue) into `.ramtext` via the RAMFUNC2 registry.
2. Remove the flash-resident 64-bit divide from the hot path (see below).
3. Relocate newlib memcpy/memset into `.ramtext` (see below).
4. With core1 off flash and core0 owning the cache, drop the clock 375 -> 225
   and the voltage 1.30 -> 1.15 V.

The EKF runs inline in the 200 Hz main loop on core0. It is deliberately not a
thread: the earlier threaded-EKF scaffolding was removed as it only added a
one-tick lag and shared-state locking for no benefit at these loop rates.

## memcpy/memset relocation - the boot-order gotcha

memcpy/memset are the top flash-resident functions on the core1 rate/IMU path
(IMU FIFO reads and servo/motor output copies). Relocating them into `.ramtext`
is worth ~4% of core1 flash time and a large cut in XIP-cache eviction pressure
on core0 - but it double-faults at boot unless one subtlety is handled first.

`rp_clock_init()` (ChibiOS `rp_clocks.c`) copies `.ramtext` into SRAM early,
before the PLL switch, using a word loop. GCC recognises that loop as a memcpy
idiom and lowers it to a `memcpy()` call. That is harmless while memcpy lives in
flash, but once memcpy is relocated into `.ramtext`, the copy loop calls the
very memcpy it has not finished copying into SRAM yet - executing uninitialised
memory. The loop that copies memcpy was calling memcpy.

Fix, in two parts:

- `rp_clocks.c`: mark the copy-loop pointers `volatile` so GCC keeps the
  explicit word loop and emits no `memcpy` call. (On the
  `andyp1per/ChibiOS rp2350_baseline` submodule branch.)
- `common_rp2350_smp.ld`: `EXCLUDE_FILE` the newlib mem* objects from the flash
  `.text` sweep and pull them into `.ramtext`. Note the linked memcpy is the
  *stub* variant `lib_a-memcpy-stub.o`, not `lib_a-memcpy.o` - excluding only
  the latter silently leaves memcpy in flash.

After this, memcpy/memset live in SRAM, boot is clean, and core1 flash drops to
~0.7% (the residual is two small C++ template thunks, not worth chasing).

## TIME_US2I 64-bit divide bypass

`TIME_US2I()` / `chTimeUS2I()` convert microseconds to systick intervals with a
64-bit multiply and divide (`__udivmoddi4`), a 748-byte flash routine. At
Laurel's 1 MHz systick the conversion is the identity, yet the compiler still
emits the divide. It appears on the hot path via the per-transfer SPI timeout
(`SPIDevice::do_transfer`) and the rate-thread poll sleep
(`Scheduler::delay_microseconds`). Both bypass it when
`CH_CFG_ST_FREQUENCY == 1000000`, keeping `__udivmoddi4` off the rate path.

## Profiling

The relocation decisions were driven by an on-chip PC sampler; see
`xip-cache-and-pgo.md` and the `PROFc1` STATUSTEXT lines
(`AP_RP2350_PC_SAMPLER_ENABLED`, investigation-only). Core0 has its own readout at
`@SYS/pcprof0.txt` (core1 is `pcprof.txt`); `Tools/debug/rp2350_pc_profiler.py
--histogram` attributes either and labels the report by core.

## Core0 is compute-bound

Unlike core1, core0 does not benefit from relocation, and this was measured, not
assumed. Core0 runs the 200 Hz main loop plus the inline EKF from XIP flash at a
high cache hit rate (87-99%, varying with bench USB traffic), ~65% load / ~35%
idle, spread over a broad flat instruction footprint - there is no hot kernel to
move, and the sampler drops ~42% of samples to hash saturation precisely because
the footprint is so wide. At these clocks a cache-served flash fetch costs about
the same as an SRAM fetch, so pulling code out of flash only saves the miss
fraction.

Three relocations were tried and reverted:

- Hot libm `sin`/`cos`/`fmod` chains into Scratch X (SRAM8, core0's zero-contention
  I-code bus): they left the flash hot list but core0 load did not change. Not
  worth 1.1 KB of a 4 KB core0-critical bank.
- `constrain_value_line<float>` back to flash: it looked core1-cold by sample
  count but is on the rate path - it became a three-cache-line flash hotspot,
  taking core1 from 0 to 3% flash time and `glat` from 190 to 204 us. A function
  that is fast in SRAM shows few samples, so sample count understates how much the
  rate thread depends on it staying fast; flash execution reveals the real call
  frequency through miss latency. See the note in `rp2350_ramfunc2_registry.txt`.
- `AP::ahrs` back to flash: genuinely core1-cold, but no measurable core0 gain.

The remaining core0 levers are production-only (dropping
`HAL_ENABLE_THREAD_STATISTICS` saves ~1.5%/core but also disables the
`coreNload%` telemetry, which reads the idle thread's `.stats.cumulative`) or
bench-only (USB `Vector78`/`sduSOFHookI`, absent in flight).

## Core1 Scratch Y (SRAM9)

Where core0 gains nothing from relocation, core1 does - because Scratch Y
(SRAM9) is core1's dedicated I-code bus and attacks fetch contention, not cache
misses. It had been left empty, and a note claimed a rate-PID test showed "no
improvement". That test was invalid: `c1_main.c` writes `c1_vtable` to the SRAM9
base (0x20081000) at runtime, but it was never reserved in the linker, so any
code placed there had its first 256 B corrupted by the vtable copy. The linker
now reserves that block and starts relocated code at 0x20081100.

With that fixed, the rate-thread fast half was moved from contended striped
`.ramtext` into Scratch Y: `rate_controller_run_dt` and `AC_PID::update_all`
(plus `update_i`; `get_ff` inlines). These are core1-exclusive - the scalar
`AC_PID` is used only by the rate controllers, while position control uses
`AC_PID_2D`/`AC_P_2D`/`AC_PID_Basic`/`AC_P_1D`. Result: rate-controller compute
(`rtc`) dropped 17 -> 14 us, repeatably. The win is bounded because Scratch Y
relieves only instruction fetch; the PID data stays in striped SRAM.

The rule for Scratch Y is core1-EXCLUSIVE only: if core0 also runs a function
parked there it reaches SRAM9 over the fabric and contends with core1's
dedicated fetch. The IMU read+filter runs on core1 (`HAL_CORE_SPI0=1`), so the
gyro-notch apply chain is also core1-only (the earlier "dual-core" note was
wrong). That chain - `apply_gyro_filters` and `NotchFilter<Vector3>`/
`HarmonicNotchFilter<Vector3>` apply - was moved to Scratch Y next. It is
verified core1-exclusive and boot-safe, but not yet perf-measured: the notch
only runs when `INS_HNTCH_ENABLE=1` is set at boot (the filters allocate at
init), so a disarmed bench with the notch off never exercises it. The coeff
`update` (slow half) stays in RAMFUNC2 - it is reachable from the core0
`AP_Vehicle` path. `NotchFilter<float>::apply` (the PID notch) also stays until
its core0 use is ruled out. See `rp2350_scratchy_registry.txt`.

Scratch Y only helps fetch-bound code, and that is a narrow set. The motor
mixing (`AP_MotorsMatrix::output_armed_stabilizing` and friends) is the biggest
core1 consumer (~2.1% armed) and is core1-exclusive on this config, so it looked
like the best target - but moving it to Scratch Y showed no improvement (~2.1%
either way). It is a large, data-heavy function (thrust factors, per-motor
loops, saturation math): its time is compute and data access, and the data stays
in striped SRAM regardless, so contention-free instruction fetch buys nothing.
The lesson: Scratch Y pays off for tight, fetch-bound loops like the rate PID and
not for big data-heavy functions, even when they dominate core1. Reverted; it
stays in RAMFUNC2.

## Open items

- Flight validation: the numbers above are bench-only (disarmed).
- `glat` average ~190 us is higher than the ~19 us seen at an earlier 375 MHz /
  1 kHz-rate config. That gap is the clock plus the 4 kHz/2 kHz rates, not flash or
  core0 code placement (both were ruled out above); a live-param sweep of
  `FSTRATE_DIV` / `INS_GYRO_RATE` would isolate it.

- Flight validation: the numbers above are bench-only (disarmed).
- `glat` average ~190 us is higher than the ~19 us seen at an earlier 375 MHz /
  1 kHz-rate config. That gap is now the clock plus the 4 kHz/2 kHz rates, not
  flash; a live-param sweep of `FSTRATE_DIV` / `INS_GYRO_RATE` would isolate it.
