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

Status: Step 0 and Step 1 tooling landed and verified in software. Hardware
baseline numbers still to be collected.

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

Status when parked (2026-06-26): a per-core counter hung off
`CH_CFG_SYSTEM_TICK_HOOK` was wired up to confirm the hook fires on core1, but it
was not flashed (the Debug Probe dropped off USB) and is moot anyway once the
tickless caveat above is accounted for - the tick hook is the wrong source. To
resume, skip it and go straight to the hardware TIMER alarm: program a free
alarm for a fixed interval, enable its IRQ on core1's NVIC from core1, and in the
ISR read `*(uint32_t *)(__get_PSP() + 0x18)` into the per-core bucket histogram.
Verify on the bench that the histogram's top buckets attribute to the expected
core1 hot functions (rate controller, EKF, IMU read_fifo) before trusting it.
All prototype/verification code was reverted; not committed. The PGO go/no-go
this profiler informs is already answered (xip=99%), so this is lower priority.

## Baseline numbers (to fill in on hardware)

| Metric | Value | Notes |
|--------|-------|-------|
| `xip=` hit rate, armed, release build | TBD | from perf_report |
| `xip=` hit rate, idle bench | 99% | disarmed, IMU + 335 Hz rate loop running (Run 1) |
| core0 XIP / SRAM / invalid sample split | 94.1% / 5.3% / 0% | disarmed bench, 30k samples (Run 1) |
| core1 XIP / SRAM / invalid sample split | 98.4% / 1.5% / 0% | disarmed; 81.4% is `__idle_thread` busy-spin, active core1 ~18% (Run 1) |
| `read_AHRS` AVG us | TBD | current ~4220 us baseline (FEATURE_GAP) |
| `glat` avg/max us | 186 / 644-1832 | disarmed; high jitter, max >> avg (Run 1) |
| `rtc` avg us | 94 | disarmed; steady, well below glat (Run 1) |
| free SRAM | TBD | headroom for more relocation |

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
