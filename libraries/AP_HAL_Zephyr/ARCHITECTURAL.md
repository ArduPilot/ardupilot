# Architecture and porting notes

This is for someone writing or extending an ArduPilot HAL, not for someone
using one. Most of it generalises past Zephyr: the classes of bug below turn up
on any new port, and several of them cost real time here before they were
understood.

Every rule carries the measurement it rests on. An uncited rule cannot be
re-checked and will eventually be wrongly "corrected".

Three companion documents, and this one does not repeat them:
[README.md](README.md) for build and layout,
[COMPARED_TO_CHIBIOS.md](COMPARED_TO_CHIBIOS.md) for feature parity and the
differences that catch a ChibiOS developer out, and [DEBUGGING.md](DEBUGGING.md)
for tooling and instrumentation.

## Before you write any code

**Read the git history first.** `git log --grep` across fix and root-cause
commits is the fastest index into what a port has already paid for. An hour in
the log beats a day of rediscovery when you inherit something mid-flight.

**Download the SoC reference manual, and keep it in reach.** Convert it to text
for grepping if you can. A large share of the incidents here ended with a
register-level read that the manual settled in minutes, and at least one
conclusion of "hardware fault" turned out to be a wrong register address.

**Read the vendor's own Zephyr board files.** For any SoC, the vendor's
reference board under `modules/zephyr/boards/<vendor>/` shows how they expect
their own silicon to be configured. It is the practical companion to the
manual.

## Know when you are done

**The main loop needs to exceed 400 Hz on capable hardware, or 200 Hz on
anaemic hardware, with at least one IMU sampling at 1 kHz or better.** Below
that you still have work to do, whatever else is working.

**A dedicated rate thread lifts EKF pressure off the main loop**, on
configurations that support it. It does not excuse slow sampling: the sensor
reads themselves still need to be above 1 kHz, with 2 or 4 kHz the normal
tiers.

**When the CPU genuinely cannot afford the maths, offload it.** On an
FPU-less part, every placement and compiler lever can measure null because the
loop is already compute-bound in soft-float arithmetic. The architectural
answer there is `AP_ExternalAHRS` against a fusion IMU that runs its own
filter, not more micro-optimisation. Budget that escape hatch early for weak
targets.

**A per-unit cost means nothing until multiplied by the rate.** "Saves 0.20 us
of a 162 us transfer, 0.1%" is arithmetically true and useless as a decision
input. At 3100 transfers a second the same saving is most of a core.

## Clocks and time

**Cycle-counter `micros()` wraps are a class of bug, not an incident.** Three
instances here: a 1 GHz cycle counter wrapping every 4.29 s, Xtensa CCOUNT at
240 MHz wrapping every 17.9 s, and the general lesson that a deterministic
death at the same number of seconds after every boot *is* this bug. On any new
architecture, compute the wrap period of whatever backs `micros()` on day one
and extend to 64-bit under a spinlock if it is under an hour.

**The kernel tick is a quantisation unit and both extremes bite.** Too fine: a
1 MHz tick on a 16 MHz systimer gives `CYC_PER_TICK` 16, the driver cannot
program an alarm 16 cycles ahead, every alarm lands in the past, and the board
hangs in a timer-interrupt storm before the console exists. Too coarse: a
100 us tick rounds every sub-tick sleep up by 2x, which reads as "sleeping is
24x worse than busy-waiting".

Check what the tick maths *costs*, too. A non-power-of-two `CYC_PER_TICK` puts
a soft 64-bit divide in every `k_uptime_get()` on cores without hardware
divide, measured at 1 to 1.75 us per time call on RV32.

**`delay(ms)` needs a bounded loop, not a clock comparison.** Both the
compare-uptime form (returns in 16 us, millisecond granularity) and the
ChibiOS-identical micros64 comparison (hangs outright) failed on hardware here.
The surviving form counts exactly N iterations of `k_msleep(1)`. Do not re-fix
it without running CPUInfo's `delay(1)` row on hardware.

**Kconfig tick symbols inline into application translation units.** Change the
tick rate without wiping AP objects and you get a mixed-clock binary, with
`millis()` running 10x slow in stale units and correctly in fresh ones.

## Floating point

**Verify the FPU is actually being used.** `objdump -d | grep -c 'v..\.f32'`
on ARM, or the equivalent. A double-precision-FPU part here shipped soft-float
for weeks, with zero VFP instructions, because of a toolchain flag gap. Every
timing conclusion drawn in that window was invalid.

**`HAL_HAVE_HARDWARE_DOUBLE` is per-architecture, not per-port.** Asserting it
true for every board compiled the double-ftype EKF3 into 8,243 soft-float
libgcc calls on a single-precision-FPU Xtensa part: 43 ms per loop iteration
and permanent scheduler overrun. Key it on `__XTENSA__`, `__riscv_flen < 64`
and so on; `AP_HAL/board/zephyr.h` shows the pattern.

**Know where your soft-float lives.** Espressif parts bind `__addsf3`-class
routines and `__udivdi3` to mask ROM through the `*.rom.libgcc.ld` scripts,
which is faster than a flash-resident copy because it costs no XIP cache
pressure, and it means IRAM placement of maths code buys nothing. Verify with
`nm` and by disassembling a hot EKF function to see where its calls land.

**libm can hide a factor of 4.** On Xtensa, aliasing `sqrtf` to the inner
`__ieee754_sqrtf` and skipping the errno wrapper was worth 4x. GCC never
inlines FP divide or sqrt on soft-float targets, even with `-ffast-math`.

## Memory and code placement

Where code sits can matter more than what it does, on any part that executes
from external flash. [COMPARED_TO_CHIBIOS.md](COMPARED_TO_CHIBIOS.md) covers
why; this is how to go about moving it.

**Find out what is paying the tax with a PC-sampling histogram, never by
intuition.** A uniform 10 to 20x task inflation on the RT1176 turned out to be
the kernel's context-switch path running XIP from external NOR; moving it to
ITCM took the loop from 80 to 282 Hz. The same method on an ESP32-C6 measured
null, because that platform already places the kernel, arch core, ISR entry and
timer driver in IRAM. There was nothing left for placement to win, and knowing
that saved doing the work.

**Wildcard placement fails silently and greedily.** waf object names come from
source files, so `Scheduler.cpp.1.o` exists in several libraries and a
`*Filter*.o`-style wildcard both over- and under-matches. Verify the link map
shows exactly what you intended and check the region deltas. A guard shared
between two boards' fragments once registered one board's 350 KB ITCM roster on
another and overflowed its entire SRAM by 627 KB.

**A lever that fills up is only half-applied.** When ITCM filled to 87% here,
607 KB of hot code stayed in XIP while 795 KB of other executable RAM sat idle.
When a placement destination fills, ask immediately where the remainder goes
next.

**On single-address-space parts, code placement eats your heap.** Boot-time
SRAM leftover feeds the libc heap ArduPilot allocates from, so every KB moved
into IRAM is a KB of heap gone. Check free RAM after every placement change.

## Zero-latency interrupts, or what ChibiOS calls fast interrupts

Both RTOSes offer the same mechanism under different names: a class of
interrupt the kernel's own critical sections can never mask, for hard-latency
work such as timestamping signal edges. Both charge the same price: handlers in
this class must not call any kernel API.

On ARMv7-M both rest on the same silicon feature. `BASEPRI` masks interrupts
only at or below a threshold, unlike `PRIMASK` which masks everything. Put the
kernel's critical-section mask at a threshold, keep a priority level or two
above it, and interrupts there stay live through every kernel lock.

**The defaults differ, and that is the trap.**

|                        | ChibiOS                                              | Zephyr                                                                |
| ---------------------- | ---------------------------------------------------- | --------------------------------------------------------------------- |
| Name                   | fast interrupts                                      | zero-latency interrupts (ZLI)                                         |
| Default                | **on**, BASEPRI kernel lock is the default port mode | **off**, `irq_lock()` uses PRIMASK until `CONFIG_ZERO_LATENCY_IRQS=y` |
| Registration           | ordinary vector at a reserved priority               | `IRQ_DIRECT_CONNECT(..., IRQ_ZERO_LATENCY)`, direct ISR only          |
| Kernel APIs in handler | forbidden                                            | forbidden, "undefined behavior"                                       |
| Levels reserved        | 2 by default, priorities 0 and 1                     | 1, `CONFIG_ZERO_LATENCY_LEVELS`                                       |

Because ChibiOS's kernel lock never masks the top levels by default, a
latency-critical ISR on a ChibiOS board is protected without anyone having
thought about it. Port the same logic to Zephyr and that property is silently
lost: every `irq_lock()` anywhere in the system, in drivers, kernel or
application, now delays your edge ISR.

Measured on `mr_vmu_rt1176` against a live CPPM receiver with a clean signal:
with default Zephyr locking, GPIO edge-ISR entry was delayed up to about 300 us
often enough to corrupt roughly 30% of measured pulse widths, so a fixed 390 us
pulse measured anywhere from 51 to 610 us. RC input decoded, and was unflyable.
Enabling ZLI and re-registering the ISR as a zero-latency direct ISR collapsed
the spread to plus or minus 1 to 2 us. Timestamping has to come from the raw
DWT cycle counter, because `k_cycle_get_64()` takes a kernel spinlock and is
therefore illegal in a ZLI handler.

**Hardware input capture beats both.** ArduPilot's ChibiOS RC input latches the
edge timestamp in the timer peripheral itself, so ISR latency affects only when
the value is read, never the value. ZLI minimises software timestamp latency;
capture hardware eliminates it. On i.MX RT the equivalent path is pad, then
XBAR, then QTMR capture register, and that is what this port now uses.

## Console output blocks

ChibiOS console writes land in a buffered stream drained by an I/O thread, so a
saturated console never stalls a flight thread. Raw Zephyr `printk` is a
synchronous per-character `uart_poll_out()` busy-wait, and each of the tree's
~108 serial drivers implements that wait its own way.

Measured on ESP32-C6: a priority-3 thread printing one line every 10 s through
the USB Serial/JTAG console starved the WiFi thread until the softAP stopped
beaconing. That driver re-armed a 50 ms per-character wait on every trickle of
successful characters.

The policy is to discard characters that cannot be printed rather than block on
a UART that is not ready. Three ways to get it, cheapest first.

**Kconfig only.** `CONFIG_LOG_MODE_DEFERRED` with `CONFIG_LOG_BLOCK_IN_THREAD=n`
buffers into a ring drained by a low-priority thread and drops on overflow
instead of blocking the caller. `CONFIG_LOG_PRINTK=y` routes raw `printk`
through the same ring, covering every thread and ISR on every UART with no
custom code. `log_panic()` still flushes synchronously, so crash output
survives.

Cost: output is delayed and reordered relative to execution, and deferred mode
can hide the last line before a hang. Drops are silent beyond a dropped-count
marker.

**One patch at the choke point.** All UART-console `printk` traffic funnels
through `console_out()` in `drivers/console/uart_console.c`. Measure how long
`uart_poll_out()` took, enter drop mode above a few milliseconds, probe one
character per 100 ms until it flows. That gives drop-on-stall for every serial
driver, using the driver's own blocking as the sensor, with no per-driver
knowledge. Kconfig-gate it.

**Per-driver surgery.** Only where a driver's own timeout logic is broken. One
case so far: the ESP32 USB serial driver re-armed its 50 ms window on every
stray success, patched to a 2 ms per-character budget while healthy and instant
discard while stalled.

Trial the Kconfig tier first and judge deferred-mode truthfulness against what
bring-up needs. Hold the choke-point patch for boards that must keep immediate
mode.

## Power and electrical

Software cannot fix volts, but it triggers them.

**RF transmit surges brown out small boards, and a brown-out core reset can
wipe peripherals underneath a still-running kernel.** A first beacon at the
default 20 dBm dipped a 3.3 V rail here; the partial reset returned the
systimer to defaults, so uptime froze and every sleep became permanent, and it
cleared interrupt-matrix routes with *zero CPU writes* for any watchpoint to
catch. Cap TX power at radio start, before the first beacon, because a later
cap is a per-boot coin flip. And treat "registers changed but no code wrote
them" as a hardware event: read the reset cause first.

**Concurrent current draws stack.** A 10 dBm cap that survived alone browned
out again once a storage thread was unstarved and its flash program current
landed during radio start. Think in rail budget, not per subsystem.

**Low-power idle and MPU defaults are bring-up hazards.** A boot hang here was
fixed by pinning peripheral-domain clocks that WFI idle otherwise gated, and a
WFI clock-domain freeze can take SWD down with it, which looks like a dead
board. Default `ARM_MPU` settings produced MemManage faults against ordinary AP
memory usage. Enable both deliberately, late, one at a time.

**Expect power-gated rails on small boards and plumb them first.** One board
here gates the LDO powering both its I2C sensors and the bus pull-ups, default
off. Every sensor probe then fails on a dead, pull-up-less bus, with no error
hinting at power. Read the schematic for enable pins before writing any driver
and drive them with devicetree GPIO hogs so they are up before code runs.

## What ArduPilot assumes of the C++ runtime

**ArduPilot requires zeroed statics and never writes `= 0` initialisers.**
Statics live in `.bss` and the port must guarantee zero-init actually happens
on every RAM section AP objects land in. Boot crashes here traced to exactly
that guarantee being violated. If you add noinit or custom RAM sections, audit
what falls into them.

**A vtable ODR violation can break boot, not just correctness.** Watch for
classes compiled with different feature-define sets across the AP archive and
the RTOS side. `-fno-rtti` hides nothing from the linker's vtable placement.

**`--wrap,malloc`-style link interposition must target the final link**, not
the app archive. Applied at the wrong layer it silently binds nothing.

**Any allocator you write needs locking from day one.** The region allocator
here shipped an unlocked path and corrupted the heap under ordinary thread
concurrency.

**Treat "Free RAM: 0" as a tooling symptom.** Heap introspection is per-libc
and per-port. Verify the number against the linker map before making any
RAM-budget decision from it.

## Peripheral drivers

**Diff against ChibiOS first.** Nearly every port bug here was a stub or a TODO
where `AP_HAL_ChibiOS` had working code. Both forms of `transfer_fullduplex()`
must be overridden, for instance; with only the 3-arg form the 2-arg calls fail
silently and boot becomes glacial. "Can we make this behave like ChibiOS" is
almost always yes, and it beats inventing a mechanism that exists on one HAL.

**Check whether the RTOS already has the thing before building it.** Zephyr
ships several variants per peripheral, CPU and RTIO and DMA, selected by
Kconfig plus devicetree. SPI DMA on the RT1176 was `default y` in the driver
and inactive only because the board devicetree declared no `dmas`. The
corollary: a devicetree edit is trivial, and avoiding one is never justified.

**Watch for interrupt-per-byte drivers.** LPSPI with `FCR = 0`, or LPI2C at
400 kHz, are structurally the same failure as the DMA case. Fine per unit,
fatal at rate.

**Bus index mapping is configuration, not convention.** ArduPilot bus 0 is
whatever `I2C_ORDER` and `SPIDEV` say it is, and the mapping to devicetree
labels has to be explicit per board. A wrong mapping here made an onboard baro
and compass simply vanish.

**Leftover diagnostics are load-bearing in both directions.** A probe-scan
diagnostic left enabled ate 80% of the CPU and stretched boot past 170 s.
Conversely, a diagnostic's side effect was silently what made SPI work, and
removing it broke the board. Audit what your bring-up scaffolding actually does
before either deleting or shipping it.

**Label workarounds loudly, in code, every time, until the real fix lands.**
Unlabelled workarounds get inherited as facts.

## Sensors and identity

**Probe-identify every onboard chip and do not trust the board documentation.**
A part documented here as an ICM-20602 answers WHOAMI `0x44`: it is an
ICM-42686-P, unsupported by the driver of the day, and the mismatch presented
as SPI DMA data corruption for days.

**A fusion IMU is an ExternalAHRS backend, not an InertialSensor backend.** Its
value is the onboard fusion processor, which is exactly what a weak soft-float
CPU needs. Note its bus may also be power-gated.

## Adding a board

1. `hwdef/<BoardName>/hwdef.dat`, copied from the nearest existing board.
2. `zephyr/boards/<arch>/<board_name>/` with the devicetree, defconfig,
   `board.cmake` and `board.yml`. Skip this if Zephyr already has the board
   upstream; `native_sim` and the Espressif parts do.
3. A Kconfig fragment, either `zephyr/prj.<BoardName>.conf` or
   `zephyr/boards/<board_name>.conf`.
4. A `class <BoardName>(zephyr_board)` in `Tools/ardupilotwaf/boards.py`,
   copied from `CubeOrangeZephyr` or `mr_vmu_rt1176`.
5. `./waf configure --board <BoardName> && ./waf copter -j12`

`hwdef.dat` here is a simplified subset of the ChibiOS format, parsed by
`Tools/ardupilotwaf/zephyr_hwdef.py`, which is authoritative. It covers what to
probe and in what order, not pin mux:

```text
BOARD_NAME <name>
MCU <type>                                  informational
APJ_BOARD_ID <n>
SERIAL_ORDER <port> ...
I2C_ORDER <bus> ...
CAN_ORDER <bus> ...
SPIDEV <name> ...                           generates the SPI DT spec decls
define <NAME> [<value>]
IMU <driver> SPI:<device> [<rotation>]
IMU <driver> SPI:<accel> SPI:<gyro>         two-device parts, e.g. BMI088
BARO <driver> I2C:<bus>:<addr> | SPI:<device>
COMPASS <driver>[:<probe_method>] I2C:<bus>:<addr> [<external>] [<rotation>]
```

Pin mux, peripheral enables and clocks live in devicetree, not here.

Offering one chip-select to more than one driver is normal: detection decides.
Probing is not always side-effect free, so check the driver before adding a
speculative probe to a live bus.

## Dependencies without west

Zephyr and its dependencies are git submodules and pinned checkouts here. West
is not required for day-to-day work, and this is a deliberate architectural
choice rather than an omission.

The reasoning: ArduPilot already has a dependency model, git submodules with
pinned SHAs, and a build system that owns the tree. West wants to own the
workspace layout and to resolve a manifest at fetch time. Running both means
two tools with different ideas about what is checked out where, and a
contributor needing to learn the second one before they can build.

The shape that results:

- `modules/zephyr` is an ordinary submodule.
- Zephyr's own dependency repositories cannot be submodules, because git will
  not track paths inside another submodule's gitlink. Their source of truth is
  `Tools/scripts/zephyr_manifest_v4_4_0_map.tsv`: one row per dependency with
  name, URL, commit and path.
- `Tools/scripts/zephyr_get_prerequisites.sh` reads that map and materialises
  every checkout.
- West may still be used *outside* the repository as a one-time metadata
  resolver, to discover the commit SHAs for a given Zephyr manifest revision.
  Its output is then translated into the TSV and committed.

The cost is that bumping the Zephyr version is a deliberate, scripted step
rather than a `west update`. That is the intended trade: the pins are visible,
reviewable and diffable in the repository rather than resolved at fetch time.
