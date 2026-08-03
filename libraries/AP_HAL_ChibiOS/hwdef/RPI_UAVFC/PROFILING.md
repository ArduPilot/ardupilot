# RPI_UAVFC profiling

How to measure where CPU time goes on this board and how to act on the result.
`DEVELOPMENT.md` covers bring-up state and the architecture these tools serve.

Everything here is investigation-only and should be stripped before any of this
is proposed upstream.

## What is compiled in

`hwdef.dat` sets `AP_RP2350_PC_SAMPLER_ENABLED 1`, which builds the on-chip
statistical PC sampler in `libraries/AP_HAL_ChibiOS/rp2350_pc_sampler.cpp`.

The RP2350 SMP tickless scheduler binds TIMER0 ALARM0 to core0 and ALARM1 to
core1 for its own ticks, leaving ALARM2 and ALARM3 free. The sampler takes
ALARM2 for core0 and ALARM3 for core1, each firing on its own core's NVIC at a
fixed non-harmonic rate. The alarm ISR reads the interrupted PC from the
exception frame at PSP+0x18 and increments a per-core address histogram.
`ST_TIMER_ALARM2_SUPPRESS_ISR` and `ST_TIMER_ALARM3_SUPPRESS_ISR` keep the
ChibiOS ST handlers off those two alarms.

Core0's sampler is armed on the first `perf_report`; core1's is armed from the
rate thread.

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

Region tags in those tokens are `F` for flash from 0x10010000, `S` for SRAM
from 0x20000000, and `C` for scratch from 0x20080000.

Do not trust the `F=`/`S=`/`o=` percentages on the summary line. `cnt` is a
saturating `uint16_t` but `osum` is computed as `total - fsum - ssum`, so once
the idle PCs peg at 65535 the shortfall lands entirely in `o` and both real
shares shrink. On a two-minute run `o` climbed 29% to 58% with nothing actually
migrating. Take the region split from the full FTP histogram instead, and
discount the saturated entries.

### Baselines

RPI_UAVFC, measured armed with CRSF RC and motors running, at 225 MHz / 4 kHz /
2 kHz / 200 Hz:

 - core0 load 44-47%, core1 load 37-42%
 - XIP cache hit rate 88-89%
 - core1 non-idle sample split: 1.6% flash, 97.6% SRAM, 0.9% scratch
 - core0 non-idle sample split: 62% flash, 31% SRAM, 7% scratch
 - `RTlat` glat 34-210 us average (bimodal, see `DEVELOPMENT.md`), 700-1100 us
   maximum, `rtc` a flat 13 us

The v1 baseline, for comparison: core0 around 65%, core1 around 37%, core1
flash share around 0.7%, gyro-to-output latency around 190 us average.

Core1 is effectively finished - every function in its top 45 is SRAM-resident
and the only remaining flash candidates are a few bytes each. Core0 is now the
flash-bound core and is where the remaining wins are.

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
`arm-none-eabi-c++filt` on PATH. Use the `flash-debug-hardware` skill to bring
OpenOCD up rather than hand-rolling the invocation.

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
current build 35 distinct veneers account for 19% of all core0 flash-resident
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

`XIPpark` maxima have been checked against `RTlat` glat maxima on hardware and
do not track: see the XIP-off section in `DEVELOPMENT.md`. Do not read a large
park maximum as an explanation for rate-loop jitter without re-running that
comparison.

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
