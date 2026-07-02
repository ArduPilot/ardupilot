# Laurel XIP Cache Locality / PGO Investigation

Tracking the effort to reduce RP2350 XIP cache thrashing on the EKF/AHRS hot
path. The RP2350 runs application code from external QSPI flash through a 16 KB
XIP cache; the EKF/AHRS/Copter hot path is ~130 KB, so it cannot all stay
cached. Today ~45 KB of the hottest code is hand-placed in SRAM via the
`__RAMFUNC2__` registry (`rp2350_ramfunc2_registry.txt`). This investigation
puts measurement behind that selection before deciding whether full
profile-guided optimisation (PGO) is worth the build complexity.

Plan: Step 0 measure, Step 1 data-driven SRAM relocation, Step 2 (only if
warranted) full GCC `-fprofile-use` with linker hot/cold clustering.

Status: Steps 0-1 tooling landed; the Step 1c on-chip timer PC sampler is
implemented and working on hardware (2026-07-02) and gives the first EKF-active
core1 profile. Next: a full-histogram MAVLink-FTP readout to rank the spread
EKF math for relocation (Step 2 gate).

## Step 0 - XIP cache hit-rate counter in perf_report

The RP2350 XIP controller exposes free-running cache counters (datasheet
12.3.8): `CTR_HIT` (`XIP_CTRL+0x0C`) and `CTR_ACC` (`XIP_CTRL+0x10`). Writing
any value clears a counter.

- `rp2350_xip_cache_stats(hit, acc)` in `board_rp2350.c` reads both counters and
  clears them, so each call returns the counts since the previous call.
- `Copter::perf_report()` (runs every ~10 s) calls it under `#if defined(RP2350)`
  and appends ` xip=NN%` (hit/acc over the interval) to the existing
  `Perf: main=.. rate=.. c0=.. c1=..` line, on both the console and the MAVLink
  STATUSTEXT.

Reading this: `xip=NN%` is the cache hit rate over the last reporting window.
Low hit rate = the in-flash working set is thrashing the 16 KB cache and is a
candidate for SRAM relocation; high hit rate = relocation will not help much and
Step 2 reordering is unlikely to be worth it.

Caveats:
- The counters are read a few cycles apart, so the ratio is approximate (fine
  over millions of accesses).
- If everything (`c0`, `c1`, `xip`) is 3 digits the STATUSTEXT can exceed the
  50-char MAVLink limit and the tail truncates; the console line is unaffected.
  Real release-build loads are 2-digit so this is an edge case.
- Counters saturate rather than wrap; not reachable within a 10 s window at any
  realistic XIP access rate.

Verification: builds and links clean for Laurel (`./waf copter`, 34 s);
`rp2350_xip_cache_stats` present in the ELF at `0x101863b0`.

## Step 0b - gyro-to-attitude latency (core0 -> core1 handoff)

Current Laurel defaults run `FSTRATE_ENABLE 1`: the rate thread is active and
pinned to core1 (this supersedes the older 1-khz note that recorded
FSTRATE_ENABLE 0). So the high-rate attitude/PID loop is `rate_thread.cpp` on
core1, consuming IMU samples that the SPI0 read thread produces on core0. That
cross-core handoff - not flash contention - is the coupling on the control path.
Note SPI0 (PL022, IMU) and the QMI/XIP flash are separate controllers on
separate pins; there is no direct SPI-vs-XIP bus contention.

To decide whether moving the IMU read thread to core1 (and routing its SPI DMA
IRQ there) is worth it, `perf_report` now emits a second line:

```
RTlat: glat=NN/MMus rtc=KKus
```

- `glat` avg/max = gyro-to-attitude latency: at the moment the rate controller
  finishes, the age of the freshest IMU sample (timestamped on core0 at SPI
  read via `AP_InertialSensor::get_gyro_last_sample_us`). avg is mean latency,
  max is worst-case (jitter). Captures producer jitter + handoff + compute.
- `rtc` avg = rate controller compute time on core1 (the `rate_controller_run_dt`
  call).

Implementation: `rate_thread.cpp` measures both per cycle and calls
`copter_rate_timing_record()`; `Copter::perf_report` averages and resets each
window. RP2350-guarded; the shared fast-rate buffer is untouched (only a
read-only INS getter was added).

Interpreting it for the core1-IMU-relocation decision:

- High `glat` jitter (max >> avg) with low `rtc` => core1 is waiting on the
  sample; producer/handoff jitter is on the critical path. Moving the IMU read
  to core1 and/or pipelining (1-cycle lag, double-buffer) should help.
- High `rtc` (and `glat` tracking `rtc`) => core1 is compute/XIP bound. SRAM
  relocation (Step 1) is the win; moving the IMU thread will not help.
- `glat - rtc` approximates the pre-compute latency (sample staleness +
  scheduling), i.e. the part a handoff change could actually remove.

## Step 1 - SWD statistical PC profiler

`Tools/debug/rp2350_pc_profiler.py` - standalone, stdlib-only (plus
`arm-none-eabi-nm`/`-c++filt`). Deliberately not a Claude skill so anyone can
run it.

It samples the Cortex-M33 DWT PCSR (`0xE000101C`) over OpenOCD's TCL RPC while
the target runs at full speed. PCSR reads are non-intrusive (no halt), so the
profile reflects real timing. Each PC is attributed to a function via the ELF
symbol table and bucketed by region (XIP flash vs SRAM vs ROM vs invalid).

Output:
1. Region breakdown - how much time is spent in XIP-cached flash vs already in
   SRAM vs idle (PCSR invalid, e.g. core parked in WFE).
2. Top-N functions by exclusive sample count, tagged by region.
3. Registry suggestions - the hottest XIP-resident functions above a threshold
   that are not already in `rp2350_ramfunc2_registry.txt`, printed as
   ready-to-paste `path|symbol` lines.

Usage (OpenOCD must already be running with a `tcl_port`):

```bash
# core0 (ChibiOS main loop), with intra-function hot-kernel analysis
Tools/debug/rp2350_pc_profiler.py --elf build/Laurel/bin/arducopter \
    --tcl-port 50001 --samples 30000 --by-line 20 --out /tmp/prof_core0.md

# core1 (EKF / PID / attitude dispatch) - must select that core first
# (Raspberry Pi OpenOCD rp2350.cfg names the cores rp2350.cm0/rp2350.cm1)
Tools/debug/rp2350_pc_profiler.py --target-select rp2350.cm1 \
    --samples 30000 --out /tmp/prof_core1.md
```

### Hot-kernel analysis (`--by-line N`)

Function-level ranking decides *what* to relocate, but relocating a whole
function to SRAM bypasses the cache for all of it - including any cold body
(prearm/error/init blocks) bundled in the same function. That wastes SRAM. The
DWT samples are full-resolution PCs, so `--by-line N` re-bins each of the top N
XIP/SRAM functions at 8-byte cache-line granularity and reports:

- the hot kernel size = distinct 8-byte lines covering `--kernel-pct` (default
  90%) of the function's samples, as bytes and as a fraction of function size;
- a verdict: SPLIT CANDIDATE (large function, small kernel - relocate hot blocks
  only / hot-cold split) vs relocate whole (uniformly hot);
- the top source lines (via addr2line) so the hot kernel is identifiable.

This is what tells us whether function-granularity selection is over-relocating.
A function that is uniformly hot relocates whole with no waste; a large function
with a tiny kernel should be split (or left in XIP and PGO-reordered) rather than
relocated whole.

Note on soft-float: the M33 has no double-precision FPU, so libgcc routines
(`__aeabi_dadd`, `__subdf3`, ...) tend to rank hot. They have no source path, so
the `path|symbol` registry cannot place them - they would need a symbol-based
linker section pick if relocation proves worthwhile. Prefer eliminating doubles
on the hot path (use float literals/math) where possible.

Workflow to iterate the SRAM working set:
1. Fly/replay a representative workload on the bench (idle bench under-counts the
   EKF path - it only runs hot when armed/fusing).
2. Run the profiler on core0 and core1.
3. Paste the suggested `path|symbol` lines into `rp2350_ramfunc2_registry.txt`,
   rebuild, and watch `xip=NN%` in `perf_report` plus `read_AHRS` timing.
4. Stop when SRAM headroom runs low or the hit rate stops improving.

Verified in software: symbol parse (13775 functions), registry diff, region
attribution (`rp2350_xip_cache_stats`->xip at `0x101863b0`, `Copter::read_AHRS`
->sram at `0x20005800`), source-path relativisation, demangling, clean failure
when OpenOCD is absent; flake8 clean.

The hardware sampling loop itself is the one path not yet exercised on a live
target. The OpenOCD `profile` command is a fallback if PCSR reads misbehave.

## Step 1b - on-chip self-hosted PCSR profiler (SWD-free)

The external SWD profiler cannot be made non-intrusive on the merged dual-core
SMP firmware. Confirmed on hardware (2026-06-26):

- OpenOCD's attach `examine` halts core0 (the SMP primary). Once halted it will
  not `resume` (stays halted, or resumes into `fault_capture`); only `reset run`
  clears it, which reboots and loses steady state.
- There is a single SW-DP shared by both cores' AHB-APs (datasheet Figure 10),
  so external reads of core1's DWT PCSR contend with core0 over one link - ~50%
  of samples come back invalid when both cores run.
- Datasheet section 3.5.9 rules out a secure-debug lockdown: a blank-OTP dev
  chip is "fully open", and we can both halt core0 and observe Secure PCs, so
  SPIDEN/SPNIDEN are 1. The halt is OpenOCD examine behaviour, not a chip gate.

The fix is to stop using SWD and sample on-chip. RP2350 exposes every AP
(except the RP-AP) in the system memory map via the self-hosted debug window at
`CORESIGHT_PERIPH_BASE = 0x40140000` (datasheet 3.5.6):

- core0 AHB-AP at `0x40142000`, core1 AHB-AP at `0x40144000`.
- A core may not drive its OWN AP through the window (immediate bus fault), but
  core0 driving core1's AP is the intended use.
- The Mem-AP reaches core1's PPB - including its DWT - which is exactly the
  block holding PCSR. The ADIv6 Mem-AP register frame is at `+0xD00` CSW,
  `+0xD04` TAR, `+0xD0C` DRW within the AP.

So core0 reads core1's PCSR with no debugger: set core1-AP CSW for 32-bit
no-increment access, TAR = `0xE000101C` (DWT_PCSR), then read DRW in a loop.
Each DRW read re-issues the access and returns a fresh PC sample. A core0
sampler accumulates a top-N histogram and dumps it over MAVLink/console;
attribution to functions happens offline with the existing profiler's symbol
table. This is fully non-intrusive (no halt), needs no OpenOCD, and works while
the vehicle flies.

Deadlock caveat (datasheet 3.5.6): driving the other core's AP to reach an APB
peripheral (address `0x4xxxxxxx`) can deadlock until a 65536-cycle timeout.
Sampling core1's DWT (PPB, `0xE000xxxx`) avoids that, but the sampler must not
point TAR at APB addresses.

### Prototype attempt - 2026-06-26 (not yet working)

First firmware prototype: core0 reads core1's AHB-AP IDR / DWT PCSR via
`0x40144000`, accumulates a top-N PC histogram, dumps it from `perf_report`.
It reboot-loops the flight controller. Findings:

- OpenOCD cannot pre-validate the path: `mdw 0x40144000` returns a bus fault
  because OpenOCD reaches it through core0's AHB-AP, and the datasheet forbids
  an AHB-AP accessing an AHB-AP. The ROM table at `0x40140000` reads fine, so the
  window itself is live; only a core's own CPU load/store can reach the other
  core's AP. That means it can only be exercised from firmware.
- The firmware crash is a BusFault `IBUSERR` (CFSR=0x100) escalated to HardFault,
  with a stacked return PC of `0x20beeffe` (poison) reached from
  `AP_Vehicle::loop` - i.e. control flow jumped to garbage, then
  `_unhandled_exception` reset the chip (hence the reboot loop and silent USB).
- Reducing to a read-only IDR probe still crashed, and the `rp2350_c1pc_stage`
  breadcrumb read back as 0 - so the access path may need a debug-domain
  power-up/enable first (datasheet 3.5.4: CDBGPWRUPREQ in the SW-DP CTRL/STAT,
  normally asserted by an external debugger), or adding the code/BSS perturbed
  a latent issue. Root cause not yet isolated.

Likely cause: the self-hosted CoreSight APs need the debug power domain up
(datasheet 3.5.4 / CDBGPWRUPREQ), which an external debugger asserts on attach
but standalone firmware does not - and CDBGPWRUPREQ lives in the SW-DP, which is
not memory-mapped, so firmware cannot set it the easy way. Whether firmware can
power the debug domain another way (POWMAN / a system-bus path) is unresolved.

### Recommended next direction - timer-driven stacked-PC sampling

Abandon the debug hardware entirely. The standard statistical profiler needs
none of it: a periodic per-core interrupt reads the *stacked* (interrupted) PC
and histograms it. core1 samples core1 directly - no AP, no SWD, no debug power,
no attach-halt - and it works in flight.

Mechanism (validated as far as building):

- Use a dedicated periodic source, NOT the system tick. ChibiOS here is tickless
  (`CH_CFG_ST_TIMEDELTA 10`, `CH_CFG_ST_FREQUENCY 1000000`), so
  `CH_CFG_SYSTEM_TICK_HOOK` fires on virtual-timer expiries at irregular
  intervals - biased for statistical sampling. Instead drive sampling from an
  RP2350 hardware TIMER alarm IRQ at a fixed, non-harmonic rate (e.g. 7 kHz):
  TIMER0/TIMER1 each have 4 alarms, one of which the tickless scheduler already
  uses, so a free alarm can own the sampler. The alarm IRQ must be enabled on
  each core's NVIC to sample that core - core1 setup has to run on core1 (e.g.
  from the rate thread's first pass).
- In the hook, read the interrupted PC from the exception frame: it is the 7th
  word of the frame on the process stack, i.e. `*(uint32_t *)(__get_PSP() +
  0x18)`. That offset is fixed even with FPU lazy stacking (FP regs follow xPSR).
  Guard PSP to a valid SRAM range before dereferencing so a bad frame skips
  instead of faulting.
- Keep the hook O(1): increment a per-core histogram bucket, do not scan. Two
  linear bucket arrays (XIP `0x10000000..`, SRAM `0x20000000..`, e.g. 256-byte
  buckets, uint16 saturating) keep it to a few adds per tick. Profiling core1
  only (skip core0 via `port_get_core_id()`) halves the memory.
- `perf_report` (not the ISR) sorts the histogram and dumps the top-N buckets as
  `addr count`; attribute the bucket addresses to functions offline with the
  existing profiler's ELF symbol table.

This was implemented and is working on hardware (2026-07-02). See Step 1c.

## Step 1c - on-chip timer PC sampler (implemented, working)

`libraries/AP_HAL_ChibiOS/rp2350_pc_sampler.cpp`, enabled by
`AP_RP2350_PC_SAMPLER_ENABLED` in the Laurel hwdef (investigation-only, strip
before upstreaming). The SMP tickless scheduler binds TIMER0 ALARM0/ALARM1 to
the two cores' per-core ticks and leaves ALARM2/ALARM3 free, so each core
samples itself from a free alarm at ~5 kHz (197 us, prime). Reads out over
MAVLink from `perf_report` as `PROFc1` STATUSTEXT; attributed offline with
`Tools/debug/rp2350_pc_profiler.py --histogram` (paste the MP text). Token
tags: F=flash from 0x10010000, S=sram from 0x20000000, C=scratch from
0x20080000. No SWD, so it profiles the armed EKF path in flight.

Four design points, each learned the hard way - do not undo them:

- Priority 0 (zero-latency), NOT the tick priority. `PORT_FAST_PRIORITIES` is 0
  so `CORTEX_MAX_KERNEL_PRIORITY` is 1; the kernel raises BASEPRI to mask every
  IRQ at priority >= 1 (including the tick at 2) inside critical sections. A
  masked sampler defers its ticks and they all fire the instant BASEPRI drops -
  i.e. at `chSysUnlock` (chsys.h:385) - manufacturing a ~15% false hotspot.
  Priority 0 is unmaskable and samples uniformly. It requires a bare naked
  handler (OSAL_IRQ_PROLOGUE/EPILOGUE are illegal above kernel priority): the
  stub reads EXC_RETURN bit 2 to pick MSP vs PSP, loads the stacked PC at
  frame+0x18, and tail-calls the C sink with LR still holding EXC_RETURN so the
  sink's return is the exception return.
- Own the vectors via strong `Vector48`/`Vector4C` symbols with the ST-LLD
  handlers suppressed (`ST_TIMER_ALARM2/3_SUPPRESS_ISR` in the hwdef). Core0 and
  core1 use SEPARATE vector tables (core0 `rp2350_vectors`; core1 at
  0x20081000), both copied from the flash table, so a strong symbol there covers
  both cores; runtime-patching one table would not.
- Full-resolution PC hash (4096 slots/core), NOT a bucketed array. The LTO build
  packs functions into 24-200 B sections; 512 B buckets merged 5-16 functions
  and bucket starts landed in padding, un-attributable. Exact PCs map 1:1 to
  functions via nm.
- Sampler entry (Vector48/4C + sink) lives in `.ramtext` (SRAM) so the ~5 kHz
  path never XIP-stalls and runs during XIP lockout. Read-only, per-core arrays,
  no locks - safe at priority 0 because it makes no OS calls.

Gotchas hit (recorded so we do not repeat them):

- AP `gcs().send_text` / vsnprintf has no `%.*s` (variable precision); it
  silently prints nothing. NUL-terminate each line and use plain `%s`.
- The RAMFUNC2 generator `rp2350_ramfunc2_sections.sh` aborted under
  `set -o pipefail` when `nm` hit a GIMPLE-only LTO object; the nm scan is now
  tolerant of unreadable objects.

## Step 1d - full-histogram readout over MAVLink FTP

`perf_report` emits only the top-16 hottest PCs, which on the EKF-active bench
are ~8% of all samples - too few, because the load is spread over many
functions. So the sampler also serves the top ~512 PCs as `@SYS/pcprof.txt`
(`AP_Filesystem_Sys`), fetched with `ftp get @SYS/pcprof.txt` and attributed by
`rp2350_pc_profiler.py --histogram`. Constraints learned:

- Runtime free heap is only ~20 KB after EKF init (NOT the ~196 KB static heap
  region); a 48 KB dump fails with ENOMEM and returns an empty file. The dump is
  a bounded top-N (~512 PCs, ~7 KB) into a static selection buffer, sized like
  the working `tasks.txt` fetch.
- The sampler hash was cut to 2048 slots/core (24 KB, was 48 KB) to give that
  scarce heap back - it was itself part of the squeeze.

See Run 3 for the first full attribution.

## Baseline numbers

| Metric | Value | Notes |
|--------|-------|-------|
| `xip=` hit rate, EKF3 active bench | 90% | disarmed but EKF3 active + core1 saturated (Run 2) |
| `xip=` hit rate, idle bench | 99% | disarmed, IMU + rate loop, EKF not yet active (Run 1/2) |
| core1 timer-sampler F/S/o split | 89% / 4% / 6% | EKF3 active, priority-0 unbiased (Run 2) |
| core0 XIP / SRAM / invalid sample split | 94.1% / 5.3% / 0% | disarmed bench, 30k SWD samples (Run 1) |
| core1 XIP / SRAM / invalid sample split | 98.4% / 1.5% / 0% | disarmed; 81.4% is `__idle_thread` busy-spin (Run 1, EKF idle) |
| `read_AHRS` AVG us | TBD | current ~4220 us baseline (FEATURE_GAP) |
| `glat` avg/max us | 205 / 412-4466 | EKF3 active; high jitter, max >> avg (Run 2) |
| `rtc` avg us | 85 | EKF3 active; steady, well below glat (Run 2) |
| free SRAM | ~196 KB heap | after +48 KB sampler BSS; from map (Run 2) |

### Run 1 - 2026-06-26 (disarmed bench, first hardware capture)

First live PCSR profile. Setup: official Raspberry Pi Debug Probe
(CMSIS-DAP, firmware 2.3.1) on the J12 SWD header; OpenOCD running on the
Windows host; `rp2350_pc_profiler.py` run from WSL over the TCP tcl_port
(WSL2 mirrored networking, so `--host localhost` reaches it). 30k samples
per core.

Conditions: vehicle disarmed, no EKF fusion, so this captures the rate/IO
path, NOT the 130 KB EKF/AHRS hot set the investigation targets. The
`xip=` hit rate and the armed rows above are still pending.

- core0 never parks (invalid 0%). Hottest XIP-resident:
  `PIORXDriver::_service_rx_fifo` (3.3%, CRSF/ELRS PIO-UART RX), then the
  IMU backend `_notify_new_accel/gyro_raw_sample` paths.
- core1 is 81.4% `__idle_thread` (busy-spins from flash, hence invalid 0%
  is misleading). Active work is `rate_controller_thread` plus the motor
  output chain (`SRV_Channels::set_output_pwm`,
  `AP_MotorsMatrix::check_for_failed_motor`, `RCOutput::write`).

Do not act on this run's RAMFUNC2 suggestions: disarmed, the EKF callees
that dominate when fusing are absent. Rerun armed before relocating.

`perf_report` over the same session (steady across cycles):

```
Perf: main=~250Hz rate=~335Hz core1load:~55% core2load:~18% xip=99%
RTlat: glat=186/644-1832us rtc=94us
```

Two reads from this:

- `xip=99%`. The cache is not thrashing. With the existing ~45 KB
  RAMFUNC2 set, only ~1% of XIP accesses miss even with the rate loop
  running. This is the Step 2 decision-gate signal: if the armed number
  holds near this, full PGO / hot-cold clustering has almost nothing left
  to win and is not worth the build-system cost. Confirm armed (EKF
  fusion adds working set), but the margin is large.
- `rtc` is steady at 94 us while `glat` averages 186 us and spikes to
  1.8 ms. Compute is not the bottleneck and (given xip=99%) neither is
  flash. The control-path coupling is the cross-core sample handoff:
  `glat - rtc` ~92 us of pre-compute latency plus large jitter. Per Step
  0b that points at the IMU-read-to-core1 / pipelining change, not more
  SRAM relocation. The 1.8 ms max spikes may be disarmed-bench artifacts
  (USB/MAVLink bursts); recheck armed before drawing the jitter
  conclusion.

Note core load is ~55% peak, not the 100% saturation of the older
`XIP.notes.md` runs - the current RAMFUNC2 set plus 93.75 MHz flash
already removed the CPU-bound regime.

### Run 2 - 2026-07-02 (on-chip timer sampler, EKF3 active)

First capture from the Step 1c on-chip sampler. Two vehicle states seen on the
same boot: (a) EKF "waiting for GPS config data" - only the rate loop and
scheduler run, core1 ~90%; (b) EKF3 initialised on the IMU (no GPS needed to go
active) and `AHRS: EKF3 active` - core1 saturates: `ekf_duty` 93-99%,
`ekf_dur` ~8 ms, the adaptive decimation forced 3 -> 7, `core1load` 100%, and
`xip` fell 99% -> 90%. State (b) is the representative heavy EKF workload.

With the priority-0 sampler the `chSysUnlock` bias is gone. Hottest core1
functions (EKF3 active, n=254k):

```
3.3% SRV_Channels::set_output_pwm   (xip, motor output every rate cycle)
1.4% memset                         (xip, EKF matrix zeroing)
1.9% __stats_*_measure_crit_thd + chTMStartMeasurementX  (ChibiOS TM/stats)
1.0% calc_pwm + RCOutput::write + check_for_failed_motor  (motor chain, xip)
```

Two takeaways:

- The motor-output chain (`set_output_pwm` + `calc_pwm` + `write` +
  `check_for_failed_motor`, ~4.3%) is a clean, concentrated relocation cluster.
- `CH_DBG_STATISTICS` / `CH_CFG_USE_TM` per-critical-section timing costs ~2% on
  core1 - a free win to disable if `@SYS/threads.txt` stats are not needed.

Open item: the emitted top-16 PCs are only ~8% of samples; resolved by the
full-histogram readout (Step 1d) - see Run 3.

### Run 3 - 2026-07-02 (full histogram via @SYS/pcprof.txt, EKF3 active)

First full per-function attribution (n=480k, top-512 PCs = 54% of samples;
drop=18% - the 2048-slot table overflows on the full working set, so the cold
tail is partial but the top ranking is solid). Key result: core1 is dominated
by the 1 kHz rate loop and ChibiOS overhead, NOT the NavEKF3 fusion math (the
EKF thread was decimated to 167 Hz/7, so its covariance work is a minor spread
contributor). Rough clusters, as fraction of all core1 samples:

```
~12% motor output   SRV_Channels::set_output_pwm 3.6, RCOutput::write 2.4,
                    calc_pwm 1.4, check_for_failed_motor 1.4, output_ch 1.2, ...
~5%  filters + PID  calc_lowpass_alpha_dt 1.6, SlewLimiter::modifier 1.2,
                    LowPassFilter2p::apply, DigitalLPF::_apply, AC_PID::update_i
~5%  ChibiOS TM/stats  chTM*MeasurementX + __stats_*_measure_crit_*
~6%  ChibiOS locking   chMtxLock/Unlock, chSysLock/Unlock, WithSemaphore
6%   idle           rp2350_idle_c1
```

Two takeaways:

- The motor-output chain (~12%, concentrated in a few SRV_Channels/RCOutput
  functions, run every rate cycle) is the top data-driven RAMFUNC2 cluster.
- CH_CFG_USE_TM + CH_DBG_STATISTICS cost ~5% of core1 (measure-every-critical-
  section). Disabling them reclaims that outright if per-thread @SYS stats are
  not needed - bigger than most single relocations. Verify core1load reporting
  first (it may read thread cumulative time).

Limitations: many hot AP functions have no source path from nm (LTO-merged), so
the path|symbol registry form does not apply - they need symbol-based .ld picks.
And a GPS-fused flight would weight the EKF math more than this bench.

## Decision gate for Step 2 (full PGO)

Pursue `-fprofile-use` + linker `.text.hot`/`.text.unlikely` clustering only if,
after relocating the measured hot set to SRAM, the armed `xip=` hit rate is
still low AND a meaningful fraction of samples remain in XIP. If relocation
alone drives the hit rate high, the size mismatch (130 KB path vs 16 KB cache)
means in-flash reordering has little left to win and Step 2 is not worth the
build-system cost.

### Step 2 method: instrumentation PGO, not AutoFDO; stay on GCC 10

Decision (verified against the pinned `arm-none-eabi-gcc` 10.2.1,
`10-2020-q4-major`):

- Use instrumentation `-fprofile-generate`/`-fprofile-use`. It compiles and
  links for cortex-m33 today; libgcov is in the toolchain and the counters
  (`__gcov0.*`) plus dump runtime (`__gcov_dump_one`, `__gcov_exit`) land in BSS
  at named symbols. The counters are therefore snapshot-able over the existing
  OpenOCD/SWD link and the `.gcda` reconstructed offline - no filesystem or
  semihosting needed, and it reuses the Step 1 profiler's infrastructure.
- Rejected AutoFDO (`-fauto-profile`). The flag is accepted by GCC 10, but the
  profile-generation side is blocked: `create_gcov` consumes Linux perf.data
  with LBR branch records, which Cortex-M33 does not have. The PCSR sampler only
  yields flat IP samples (no branch history), so AutoFDO would mean hand-rolling
  a PCSR->AFDO converter for a line-frequency-only profile - weaker than the
  exact edge counts instrumentation gives, which is what the hot/cold split
  decision (see `--by-line`) actually needs.
- Do not upgrade GCC for this. Instrumentation PGO is fully supported on 10.2.1.
  AutoFDO's weakness is the missing branch trace on Cortex-M, which no GCC
  version fixes. Upgrading diverges from the toolchain ArduPilot pins and
  validates the whole port against, for no profile-side gain.
- Open risk to de-risk before committing Step 2: the only unproven link is
  reconstructing a valid `.gcda` from a BSS counter snapshot. Prototype that
  round trip on a Laurel build first.
- Reconsider AutoFDO only as a fast-follow if the instrumented build's timing
  overhead proves to misrepresent the real armed/flight workload; even then,
  prove `create_gcov`/AFDO feasibility before any GCC bump.
