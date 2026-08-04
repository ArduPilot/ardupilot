# RPI_UAVFC profiling

How to measure where CPU time goes on this board and how to act on the result.
`DEVELOPMENT.md` covers bring-up state and the architecture these tools serve.

Everything here is investigation-only and should be stripped before any of this
is proposed upstream.

## Turning it on

`hwdef.dat` ships with all of it off, because it is not free:

```
define HAL_ENABLE_THREAD_STATISTICS TRUE
define AP_RP2350_PC_SAMPLER_ENABLED 1
define AP_RP2350_DEBUG_REPORT_ENABLED 1
```

then reconfigure - `./waf copter` alone will not regenerate `hwdef.h`. Statistics
cost 13.6% of core1 and 10.2% of core0 non-idle time, so a profile taken with
them on is measuring a slower machine than the one you fly. The PC sampler adds
a ~5.1 kHz ISR per core and 24 KB of BSS, and cannot see itself: it samples the
interrupted PC, so its own cost never appears in its own output.

Without `HAL_ENABLE_THREAD_STATISTICS` the `Perf` line loses `core1load`
entirely - `Scheduler::get_core1_load_pct()` reads
`ch1.idlethread.stats.cumulative`, which only exists with `CH_DBG_STATISTICS`.
Core0 load comes from `AP_Scheduler` and survives either way.

## What is compiled in

`AP_RP2350_PC_SAMPLER_ENABLED` builds the on-chip statistical PC sampler in
`libraries/AP_HAL_ChibiOS/rp2350_pc_sampler.cpp`.

The RP2350 SMP tickless scheduler binds TIMER0 ALARM0 to core0 and ALARM1 to
core1 for its own ticks, leaving ALARM2 and ALARM3 free. The sampler takes
ALARM2 for core0 and ALARM3 for core1, each firing on its own core's NVIC at a
fixed non-harmonic rate. The alarm ISR reads the interrupted PC from the
exception frame at PSP+0x18 and increments a per-core address histogram.
`ST_TIMER_ALARM2_SUPPRESS_ISR` and `ST_TIMER_ALARM3_SUPPRESS_ISR` keep the
ChibiOS ST handlers off those two alarms.

Core0's sampler is armed on the first `perf_report`; core1's is armed from the
rate thread.

That makes the two flags coupled in one direction: `rp2350_pc_sampler_init_
core0()` is called only from inside `Copter::perf_report`, which is compiled
out with `AP_RP2350_DEBUG_REPORT_ENABLED` 0. Enabling the sampler on its own
therefore leaves `@SYS/pcprof0.txt` empty, because core0's alarm is never
armed - core1 still works, since the rate thread arms it independently. Set
both, or move the init call out of the gated function.

`AP_RP2350_PC_SAMPLER_ENABLED` defaults to 0 in `AP_HAL/AP_HAL_Boards.h`.
Before that default existed only RPI_UAVFC defined it, so every other RP2350
board failed to build on `-Werror=undef` once the guards became value tests.

Attribution to function names happens offline against the ELF symbol table, so
a `--debug` build gives better file and line resolution.

## Reading it out

There are three routes, in increasing order of detail.

### perf_report, automatic

`Copter::perf_report` is on the scheduler at 0.1 Hz, so it emits every ten
seconds with no action needed. Connect a GCS and watch the messages:

- loop rate, rate-thread Hz, scheduler CPU load
- `RTlat: glat=<avg>/<max>us rtc=<avg>us` - gyro-to-attitude latency, which
  captures the core0 to core1 handoff, plus rate-controller compute time
- `XIPpark: n=<count> max=<us>` - core1 park count and worst case
- the core1 PC-sampler top 16, as region-tagged `offset:count` tokens

Region tags in those tokens are `F` for flash from 0x10020000, `S` for SRAM
from 0x20000000, and `C` for scratch from 0x20080000.

Do not trust the `F=`/`S=`/`o=` percentages on the summary line. `cnt` is a
saturating `uint16_t` but `osum` is computed as `total - fsum - ssum`, so once
the idle PCs peg at 65535 the shortfall lands entirely in `o` and both real
shares shrink. On a two-minute run `o` climbed 29% to 58% with nothing actually
migrating. Take the region split from the full FTP histogram instead, and
discount the saturated entries.

### Baselines

RPI_UAVFC, armed with CRSF RC and motors running, at 225 MHz / 4 kHz / 2 kHz /
200 Hz, **with statistics and the sampler compiled in** - so core loads here are
several points higher than the same firmware built to fly:

 - core0 load 45-47%, core1 load 36-40%
 - XIP cache hit rate 87-89%
 - core1 non-idle sample split: 1.8% flash, 97.2% SRAM, 1.0% scratch
 - core0 non-idle sample split: 62.3% flash, 30.7% SRAM, 7.0% scratch
 - of core0's flash time, 23.3% is linker veneers
 - `RTlat` glat about 197 us average, 700-1400 us maximum, `rtc` flat at 13 us

Two runs a few hours apart agree to within a point on every split, so the
numbers are stable. The v1 baseline, for comparison: core0 around 65%, core1
around 37%, core1 flash share around 0.7%, gyro-to-output latency around 190 us.

A flight build (statistics off, sampler off, debug reports off) measured
`xip=95%` and `rtc=12us` on the same hardware, against 88% and 13 us here.
Core0 load is not comparable between the two runs - the flight build also had a
working GPS and an EKF doing real fusion, which the profiled runs did not.

That was measured with the outputs on PWM. It stopped being true the moment
bidirectional DShot ran, because the whole send/decode chain was flash-resident
and none of it had ever been profiled.

### Bidirectional DShot, before and after relocation

Same hardware and rates, sampler on, **statistics off** - see the warning below.
Core1 non-idle split:

| | PWM baseline | bdshot, unrelocated | bdshot, relocated |
|--------|--------------|---------------------|-------------------|
| flash  | 1.8%         | 65.6%               | 17.7%             |
| SRAM   | 97.2%        | 34.1%               | 73.3%             |
| scratch| 1.0%         | 0.2%                | 9.0%              |

Core1 idle went from 21.9% of emitted samples to 51.4%, so the core is doing
roughly half the work it was. `RCOutput_pico::read_telemetry` alone fell from
19.2% of emitted samples to 3.5%.

That 5.5x drop settles a question worth remembering: a high sample count does
not by itself prove code is fetch-bound, and `AP_MotorsMatrix` is the local
counter-example that gained nothing from Scratch Y. `read_telemetry` was
genuinely fetch-bound, and the only way to tell the two apart was to move it and
re-measure.

Treat both improvements as lower bounds. The idle PC saturates its `uint16_t`
(`S175C` on core1, `S1758` on core0 both pegged at 65535), so real idle is
higher than the histogram can express and the non-idle shares are overstated.

What is left on core1 is small and diffuse. The largest single item is
`__udivmoddi4` at 1.2% of emitted - a 64-bit division on the per-frame path,
which wants removing rather than relocating. `bdshot_decode_gcr::decode` is
still in flash and always will be under this mechanism: the registries relocate
`.text` only, and it is a lookup table read per nibble, so it is XIP *data*
traffic the PC sampler cannot see.

### Current baseline

Bidirectional DShot600 at 2 kHz (`SERVO_DSHOT_RATE` 1, `FSTRATE_DIV` 2),
`SERVO_DSHOT_ESC` 0, sampler on, statistics off:

| | core1 | core0 |
|---------|-------|-------|
| idle    | 58.3% | 53.9% |
| flash   | 13.2% | 72.2% |
| SRAM    | 77.1% | 19.4% |
| scratch |  9.7% |  8.3% |

Splits are of non-idle samples. Every one of core1's top 24 functions is
SRAM-resident; nothing in flash appears until 0.4%, and what is left is two
16-byte `chSysLock`/`chSysUnlock` veneers and `RCOutput::timer_tick`. That is
the tail, and the veneers are exactly the tiny hot leaves the relocation
warnings above say to leave alone.

`__udivmoddi4` and the `AP_HAL::micros64` veneer are both gone from core1, which
is the confirmation that the RCOUT_US2I and 32-bit rcout timer changes landed.
The remaining `hrt_micros64` at 0.9% is the underlying HAL timer read in SRAM,
not on the rcout path.

Treat the improvement from the previous run as directional only. Three things
changed together - those two commits, `SERVO_DSHOT_ESC` coming out, and the
DShot rate becoming a confirmed 2 kHz - so the idle gain cannot be attributed to
any one of them. What is unambiguous is structural: a symbol either appears in
the histogram or it does not.

### The idle PC moves between builds

`rp2350_idle_c0` and `rp2350_idle_c1` are a few bytes apart in SRAM and their
addresses shift with almost any code change. Any script that hard-codes the idle
token to separate idle from non-idle will silently report 0% idle and a
meaningless split after the next build. Read the addresses out of the ELF each
time:

```
arm-none-eabi-nm -S build/RPI_UAVFC/bin/arducopter | grep rp2350_idle_c
```

### The sampler cannot see blocking

The PC sampler records the interrupted PC, so a thread blocked on a mutex, an
event or a parked core shows up as idle time, not as contention. It is the wrong
instrument for stalls, lock contention or anything that manifests as absence.

What worked instead was a handful of `volatile uint32_t` counters compiled in
temporarily and read over SWD while the vehicle ran: one per stage of the path
under suspicion, plus a max-interval counter to latch a transient that would
otherwise have to be caught live. That is how the 17 ms flash-park stall was
found, and how `SERVO_DSHOT_ESC` was caught injecting sends nobody asked for -
the send, wake and signal counters simply would not add up.

### Statistics breaks CRSF on this board

Do not profile RC-critical behaviour with `--enable-stats`. `CH_IRQ_PROLOGUE`
and `CH_IRQ_EPILOGUE` call `__stats_start_measure_crit_isr` /
`__stats_stop_measure_crit_isr`, both SRAM-relocated, so a flash-resident
handler reaches them through XIP veneers. PIOUART RX uses that prologue and
epilogue, and at CRSF's 420 kbaud - a byte every 24 us into a shallow FIFO with
no DMA - the added latency drops bytes and the link sits in continuous failsafe.

It was survivable on PWM, which is why the baseline above exists at all. It
stopped being survivable once core1's bdshot fetches started competing for the
same XIP cache. The cost is losing `core1load`; the sampler's region split,
which is what the relocation work actually needs, does not depend on statistics.

### Full histograms over MAVLink FTP

The STATUSTEXT dump only covers concentrated hotspots. Spread-out code such as
the EKF needs the whole table:

- `@SYS/pcprof.txt` - core1
- `@SYS/pcprof0.txt` - core0

Pull both with an FTP get from your GCS, then attribute them:

```
Tools/debug/rp2350_pc_profiler.py --histogram <file> --elf build/RPI_UAVFC/bin/arducopter
```

Pass `-` instead of a filename to read from stdin. Which core the dump came
from is taken from the `PROFc<n>` token in the text, so there is nothing to
specify.

### Live sampling over SWD

The same tool can sample the Cortex-M33 DWT PCSR (0xE000101C) through a running
OpenOCD TCL RPC connection. Reading PCSR does not halt the core, so this is
non-intrusive and reflects real timing, unlike halt/read/resume profilers.

```
Tools/debug/rp2350_pc_profiler.py --elf build/RPI_UAVFC/bin/arducopter \
    --tcl-port 50001 --samples 30000
```

To sample core1, OpenOCD has to select it first: pass
`--target-select rp2350.cm1`. Run `targets` in the OpenOCD telnet console to
confirm the core names for your build. Needs `arm-none-eabi-nm` and
`arm-none-eabi-c++filt` on PATH. See `FLASHING.md` for the OpenOCD invocation;
`--host localhost` works because WSL2 mirrored networking exposes the Windows
binary's TCL port on the Linux side.

Besides the ranked table, this mode prints ready-to-paste registry lines for
the hottest XIP-resident functions that are not yet relocated.

## Acting on the result: the three registries

Hot code is moved out of XIP flash into SRAM by listing it in one of three
registries under `hwdef/common/`. Format is `path|symbol`, one per line.

| Registry | Region | Size | Rule |
|--------------------------------|-----------------|--------|------|
| `rp2350_ramfunc2_registry.txt` | `.ramtext`, main SRAM | large | general hot code, either core |
| `rp2350_scratchx_registry.txt` | SRAM8 @ 0x20080000 | 4096 B | core0-only hot paths |
| `rp2350_scratchy_registry.txt` | SRAM9 @ 0x20081000 | 3840 B | core1-EXCLUSIVE hot paths |

Scratch X and Y are dedicated I-CODE buses giving single-cycle fetch with zero
bus-fabric contention, X for core0 and Y for core1. The first 256 B of SRAM9 is
reserved for `c1_vtable`, hence the 3840 B budget rather than 4096.

Two rules matter more than the rest:

**A symbol must appear in exactly one registry.** The linker claims `.text`
sections first-come-first-served, so a duplicate is silently dropped from one
of them and you get no warning. If a relocation appears not to take effect,
check for a duplicate first.

**Scratch Y is for core1-exclusive code only.** If core0 also executes a
function parked in SRAM9, core0 reaches it over the bus fabric and contends
with core1's dedicated fetch, which is worse than leaving it in flash. The
canonical example is `AC_PID`, used only by the rate controllers, versus the
`AC_PID_2D` / `AC_P_2D` / `AC_PID_Basic` / `AC_P_1D` family used by position
control on core0.

**Relocation is not free for the cores left behind.** A flash-resident caller
reaching an SRAM-resident callee is out of branch range, so the linker leaves a
16-byte veneer in flash and core0 fetches it through the XIP cache. On the
current build 35 distinct veneers account for 23% of all core0 flash-resident
samples - `AP_HAL::millis`, the `WithSemaphore` constructor and destructor,
`constrain_value_line`, `__stats_stop_measure_crit_isr`, `memcpy`, `memset`.
Relocating a small hot leaf that core0 calls from flash can therefore cost core0
more than it saves. Prefer relocating whole call trees, and when the profiler
suggests a tiny leaf, check who calls it first.

Entries carry a size comment (`# 164 B (sram)`) so the budgets can be tracked
by eye. Keep adding them.

Watch the ISR stack when relocating: `MAIN_STACK` in `hwdef.dat` has been
raised repeatedly as RAMFUNC2 grew, and it has previously run at 98% occupancy.

## Interpreting what you see

A high core1 flash share means the rate path is still fetching through the XIP
cache and competing with core0. That is what the relocation work targets, and
on the current build it is done.

`XIPpark` maxima explain the worst `RTlat` outliers but not the baseline
jitter - a 4097 us park produced a 4318 us glat maximum, while park-free windows
still ran 700-1400 us. Attribute a single large excursion to the park, but not
the routine spread. See the XIP-off section in `DEVELOPMENT.md`.

Check the drop count in the histogram header before believing a low share. The
hash table is 2048 slots per core, and core0 dropped 42% of its samples against
core1's 9% because core0's code is far more spread out. Anything diffuse on
core0 - the EKF above all - is undersampled, so its share is a floor rather
than a measurement.

Samples taken while a higher-priority ISR was running are attributed to the
last thread frame. On core1 almost all time is thread-mode compute so the bias
is small; on core0 it is more noticeable.

Before assuming a timing problem at all, rule out data correctness. A sensor
returning wrong values looks like a performance fault from a distance and is
not one. Check that the sensor actually probed before profiling anything.
