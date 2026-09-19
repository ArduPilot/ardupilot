# Architecture and porting notes

This is for someone writing or extending an ArduPilot HAL, not for someone
using one. Most of it generalises past Zephyr: the classes of bug below turn up
on any new port, and several of them cost real time here before they were
understood.

Every rule carries the measurement it rests on. An uncited rule cannot be
re-checked and will eventually be wrongly "corrected".

Six companion documents, and this one does not repeat them:
[README.md](README.md) for build and layout,
[COMPARED_TO_CHIBIOS.md](COMPARED_TO_CHIBIOS.md) for feature parity and the
differences that catch a ChibiOS developer out,
[DEBUGGING.md](DEBUGGING.md) for tooling and instrumentation,
[PARITY_DETAIL.md](PARITY_DETAIL.md) for the per-subsystem parity audit,
[PROCESS.md](PROCESS.md) for how a change gets verified before it is committed,
and [BOOTLOADER_SECURITY.md](BOOTLOADER_SECURITY.md) for the bootloader and the
update path.

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

**A dedicated rate thread moves the rate controller and the motor output off
the main loop**, on configurations that support it. Read
`ArduCopter/rate_thread.cpp` before quoting it as an EKF answer: the thread
wakes on each filtered gyro sample, runs the attitude rate controller and
pushes PWM. The EKF is not moved: the file's own design notes say it is
unaffected because it uses delta angles calculated from the raw gyro values,
not from the published filtered ones. What the thread buys is control latency
and filter sample rate, not EKF headroom, and it does not excuse slow sampling:
the sensor reads themselves still need to be above 1 kHz, with 2 or 4 kHz the
normal tiers.

**When the CPU genuinely cannot afford the maths, offload it.** On an
FPU-less part, every placement and compiler lever can measure null because the
loop is already compute-bound in soft-float arithmetic. The architectural
answer there is `AP_ExternalAHRS` against a fusion IMU that runs its own
filter, not more micro-optimisation. Budget that escape hatch early for weak
targets.

**A per-unit cost means nothing until multiplied by the rate**, so do the
multiplication before you argue about it. A saving quoted as a percentage of
one transfer and the transfer itself rank completely differently once the rate
is carried through, and it is usually the second that turns out to be worth
attacking. [DEBUGGING.md](DEBUGGING.md) works the arithmetic through with the
rest of the measurement discipline.

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

### Boot time is a correctness property

A board that takes too long to reach its first IMU sample does not merely boot
slowly. It changes what the EKF is told about time, for the whole flight, and
it presents as a navigation or sensor fault rather than as anything to do with
boot. Budget boot time like a requirement, and measure it.

The mechanism is in `AP_InertialSensor`, so it applies to every HAL.
`AP_InertialSensor_Backend::_notify_new_gyro_raw_sample()` splits sensors in
two, and its own comment says why: a non-FIFO driver passes a `sample_us` and
`dt` comes from consecutive timestamps, while a FIFO driver passes nothing and
`dt = 1.0f / _imu._gyro_raw_sample_rates[instance]`. That rate is an
*estimate*, refitted once a second from the publish count in
`_update_sensor_rate()`. The whole Invensense family - v1, v2 and v3, the
fast-sampling drivers a flight board actually uses, including the ICM-42688-P
on this port's own RT1176 - calls with the default `sample_us = 0`, so all of
them are on the estimate.

How far the estimate may move per refit is gated by `sensors_converging()`,
which is `AP_HAL::millis64() < HAL_INS_CONVERGANCE_MS && !armed`, with
`HAL_INS_CONVERGANCE_MS` 30,000 in `AP_InertialSensor.h`:

| window | filter constant | observation clamped to | fastest movement |
| --- | --- | --- | --- |
| first 30 s of uptime, disarmed | 0.8 | rate x [0.5, 2.0] | about 20%/s |
| after that | 0.98 | rate x [0.95, 1.05] | 0.1%/s |

An instance whose first sample arrives after 30 s of board uptime never gets
the fast window. Its estimate starts at the driver's compiled-in nominal and
crawls at 0.1%/s, so closing a factor of about 1.9 takes ln(1.9)/ln(1.001),
around 640 s, and a flight is 60 to 90 s. The same predicate re-tunes the gyro
low-pass and the harmonic notches in `update_gyro_filters()`, so a late board
also flies with filters designed for a sample rate the sensor is not running
at.

**The case this came from.** CubeOrangeZephyr, flying the standard mission
under Renode, diverged: velocity, position and magnetometer innovations all
blew up together, an EKF consistency gate failed, a vibration failsafe and a
GPS glitch followed, and EKF and GPS positions ended 3.4 m apart against 0.1 m
on the ChibiOS build of the same board flying the same mission. The chain
behind it, read link by link in the code:

* `libraries/AP_HAL_Zephyr/zephyr/src/main.cpp` calls
  `usbd_init_with_timeout(&cube_usbd, K_MSEC(30000))`. `usbd_init()` runs on a
  worker thread; main waits up to 30 s, aborts it, logs "usbd_init() timed out"
  and only then starts ArduPilot.
* `usbd_init()` reaches `stm32_usb_pwr_enable()` in
  `modules/zephyr/drivers/usb/common/stm32/stm32_usb_pwr.c`, which spins
  `while (!LL_PWR_IsActiveFlag_USB()) { k_msleep(100); }` with no bound.
* That flag is `PWR->CR3` bit 26, `USB33RDY`, "USB 3.3 V supply ready". Real
  silicon sets it within microseconds of `USB33DEN`. The emulator's PWR model
  stored CR3 and nothing ever set the bit.
* The AP time base was already running before `main()`, so the whole 30 s was
  charged to uptime. Measured: first RAW_IMU at ~44.5 s of
  `SYSTEM_TIME.time_boot_ms`, against ~14.5 s for ChibiOS on the same board.

So all three instances began from their drivers' nominal rates, 8000 Hz on
lane 0 and 9000 Hz on lanes 1 and 2, and never got near the truth. The primary
lane's believed rate was about half what the sensor was actually delivering, so
it integrated roughly 1.9 seconds of inertial motion per real second while GPS,
barometer and compass arrived on the real clock. Prediction on one clock and
measurement on another is why every innovation went at once, which no
single-sensor theory could account for.

The one-line emulator fix - `case CR3: return cr3 | USB33RDY;` in
`Tools/renode/peripherals/stm32/AP_STM32H7_PWR.cs` - moved first RAW_IMU to
~14.5 s and the re-flown mission passed, landing 0.2 m from its start point.
Note what was not fixed: the 30 s wait in `main.cpp` is still there, so any
board where that bit does not come up still has 30 s of uptime ahead of its
first sample.

**The dose-response check is what makes this a cause and not a correlation.**
The ChibiOS reference flight has the same fault in a milder form on its own
IMU1 and IMU2, which caught only part of the fast window, and their innovations
are 30 to 45 times IMU0's in that same flight - one HAL, one sensor set, one
mission, innovation error tracking dt error.

**What it looks like in a log.** A converged rate estimate wanders as the
sensor drifts. A wrong one moves at exactly the 0.1%/s rail, in whichever
direction its nominal was wrong, which means `IMU.GHz` looking perfectly steady
is the failure rather than reassurance. Fit the slope and extrapolate
backwards: if it lands on a round number the driver compiles in, at a time
before the log starts, that lane never converged. Dataflash logging starts at
arming, so the boot window itself is invisible in a `.BIN`; measure it from the
stream instead. `Tools/renode/zephyr_boot_timeline.py` timestamps every
STATUSTEXT, the first HEARTBEAT and the first IMU message against the board's
own `time_boot_ms`, and `Tools/zephyr/zephyr_ins_rate_probe.py` runs the rail
and back-extrapolation checks over a pair of logs.

**What not to do about it.** `HAL_INS_CONVERGANCE_MS` is an `#ifndef`, so a
board header can raise it. That would make the flight work and would also hide
the boot defect, so it is a workaround and the comment has to say so. The real
answers are upstream of the EKF: boot inside the window, and - as a change to
every vehicle on every board, so an upstream proposal rather than a HAL edit -
measure the window from each instance's first sample rather than from board
uptime, which costs a normally-booting board nothing.

The dataflash logs and the rest of that evidence set live in `binaries/` in the
working tree. `.gitignore` excludes it, so it is local to whoever ran the
flights and cannot be reopened from a fresh clone.

### A clock the devicetree asked for and never got

A devicetree node names a clock source. The SoC clock driver refuses that
source and writes no register. The peripheral driver then sizes its timing
against the source it was told about. Nothing logs anything, and the two halves
of one driver disagree for the life of the board. Two instances of this on one
board cost days here.

`enabled_clock()` in
`modules/zephyr/drivers/clock_control/clock_stm32_ll_h7.c` accepts
`STM32_SRC_PCLK1` and `STM32_SRC_PCLK2` only under
`CONFIG_SOC_SERIES_STM32H7RSX`. On a plain H743 it returns `-ENOTSUP` and
`stm32_clock_control_configure()` gives up before touching the mux:

* `&i2c1` and `&i2c2` named PCLK1. The mux stayed where the bootloader left
  it, PLL3_R at 240 MHz - itself above what the H743 accepts as an I2C kernel
  clock - while `i2c_stm32_runtime_configure()` sized TIMINGR against the
  declared PCLK1 at 120 MHz. Both buses therefore overran their configured
  bitrate: 400 kHz asked for, about 1.05 MHz measured on the board, past the
  1 MHz Fast-mode-Plus ceiling. It failed silently.
* `&spi4` named PCLK2. That one failed loudly, with the device never becoming
  ready: the same defect with a visible symptom.

Both nodes now name the source the hardware is actually on, and the reasoning
sits inline in
`libraries/AP_HAL_Zephyr/zephyr/boards/arm/cube_orange_zephyr/cube_orange_zephyr.dts`.
Name the source the mux is on, not the one you wish it were on, and check that
the driver's computed rate and the hardware's rate are the same number.

**Peripherals share PLL outputs, so tuning one retunes another.** On the same
board SPI123 needed its kernel clock halved from 240 MHz - at 240 MHz SPI1
reads came back as garbage - which is `div-q = 8` on PLL1. SDMMC1's kernel mux
on the H743 is a single bit, PLL1_Q or PLL2_R and nothing else, so that fix
also pinned SDMMC to 120 MHz. `sdmmc_stm32.c` requires exactly 48 MHz
under `CONFIG_SDMMC_STM32_CLOCK_CHECK` and returns `-ENOTSUP` otherwise, so
`disk_access_init()` failed on every attempt, `f_mount()` was never reached,
and the only thing visible was `AP_Logger` reporting "Failed to create log
directory /APM/logs : ENOSPC" - `FR_NOT_ENABLED` mapped to `ENOSPC` in
`AP_Filesystem_FATFS.cpp` - four layers downstream of a clock nobody had meant
to touch. The fix was to bring up the otherwise unused PLL2 at 48 MHz for
SDMMC alone. Before changing a PLL output, list every peripheral fed from it.

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
intuition.** A uniform task inflation on the RT1176, an order of magnitude
across unrelated threads, turned out to be the kernel's context-switch path
running XIP from external NOR: the kernel archive was already in ITCM but the
ARCH archive never was, so the switch path itself still ran from flash. Moving
it was the largest single main-loop gain on that board. What is placed there
now, how, and what each successive sampling round was worth in loop rate are
all in `libraries/AP_HAL_Zephyr/zephyr/itcm_hot_code.ld`, applied from the
neighbouring `CMakeLists.txt`. The board's most recent recorded loop rate is in
`libraries/AP_HAL_Zephyr/hwdef/mr_vmu_rt1176/README.md`; read it as the best
figure written down rather than a settled one, because a later profiling run on
the same board read substantially lower and nothing in the tree resolves the
two. The same method on an ESP32-C6 measured null, because that platform
already places the kernel, arch core, ISR entry and timer driver in IRAM. There
was nothing left for placement to win, and knowing that saved doing the work.

**Wildcard placement fails silently and greedily.** waf object names come from
source files, so `Scheduler.cpp.1.o` exists in several libraries and a
`*Filter*.o`-style wildcard both over- and under-matches. Verify the link map
shows exactly what you intended and check the region deltas. A guard shared
between two boards' fragments once registered one board's 350 KB ITCM roster on
another and overflowed its entire SRAM by 627 KB.

**A lever that fills up is only half-applied.** The RT1176's ITCM is now at
99.4% of 491,520 B with hot code still left in XIP and a megabyte of OCRAM not
used for code at all (`hwdef/mr_vmu_rt1176/README.md`). When a placement
destination fills, ask immediately where the remainder goes next, instead of
recording the lever as applied.

**On single-address-space parts, code placement eats your heap.** Boot-time
SRAM leftover feeds the libc heap ArduPilot allocates from, so every KB moved
into IRAM is a KB of heap gone. Check free RAM after every placement change.

**Declare every RAM bank before you argue about placement.** A bank that no
`chosen` property selects and no driver claims is not spare memory, it is
absent, and the difference does not show up as an error. CubeOrangeZephyr
selects one bank: `zephyr,sram = &sram0`, the 512 KB AXI SRAM, in
`libraries/AP_HAL_Zephyr/zephyr/boards/arm/cube_orange_zephyr/cube_orange_zephyr.dts`.
The H743's ITCM (64 KB) and DTCM (128 KB) are declared by the SoC devicetree in
`modules/zephyr/dts/arm/st/h7/stm32h7.dtsi` and hold nothing. SRAM1 and SRAM2
(128 KB each), SRAM3 (32 KB) and SRAM4 (64 KB) are declared too, across
`stm32h742.dtsi` and `stm32h743.dtsi` in the same tree, and none of them is in
anyone's heap - a bank being present in the devicetree buys nothing on its own.
ChibiOS on the same silicon spans six banks - `RAM_MAP` in
`libraries/AP_HAL_ChibiOS/hwdef/scripts/STM32H743xx.py`. That, and not waste,
is why the two builds' free-memory reports differ by most of a megabyte: 68 KB
free on Zephyr against 589 KB on ChibiOS, measured as `PM.Mem` minima in the
two Renode flights, whose logs sit in the gitignored `binaries/` tree described
above and so cannot be reopened from a fresh clone. Do the declaration work
first; otherwise every later RAM-budget decision is made against a number that
means something else.

**`__RAMFUNC__` is inert unless the board's generated header makes it real.**
`AP_HAL/AP_HAL_Boards.h` defines it empty by default. ChibiOS turns it into a
section attribute only for boards running XIP from external flash -
`chibios_hwdef.py` emits it when `EXT_FLASH_SIZE_MB` is set and
`INT_FLASH_PRIMARY` is not - and CubeOrange sets neither, so every
`__RAMFUNC__` annotation in shared AP code compiles to nothing on the board
this port is compared against. No Zephyr hwdef emits it either, so it is empty
on all four boards here, and the RT1176 gets its hot code into ITCM through a
linker fragment instead. Check what the board's own generated header defines
before concluding that another HAL does or does not relocate something.

## Threads and priorities

**Zephyr's `k_mutex_unlock()` restores the priority the owner held when it
locked.** `k_mutex_lock()` snapshots the owner's priority into
`owner_orig_prio` and the final unlock writes it back, in
`modules/zephyr/kernel/mutex.c`, under the `CONFIG_PRIORITY_CEILING` guard that
holds at the default. That is priority-inheritance bookkeeping and
it is kernel behaviour, not an ArduPilot one, but the effect is that any
priority change made while a mutex is held is silently undone at unlock, by a
function that reads as though it only releases a lock.

ArduPilot raises the main loop's priority once per loop and drops it in
`boost_end()`, and two HAL semaphores are held across that boost, so the kernel
was putting main back at the boosted level for the rest of each loop - above
the timer and SPI threads that produce the sample main is waiting for. ChibiOS
does not have the problem: `chMtxUnlock()` recomputes the owner's priority from
every mutex it still owns, and ArduPilot keeps its intended level in the thread
itself. On Zephyr the HAL has to keep that level: `Semaphore::give()` calls
`Scheduler::reassert_main_priority()` after every unlock, and the counters that
say how often it fired are in `Scheduler.cpp`.

If you carry a priority scheme over from another RTOS, find out what your
kernel does to a thread's priority behind your back, and count the corrections
rather than assuming there are none.

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
| Default                | **on**, BASEPRI kernel lock is the default port mode | **off**, `CONFIG_ZERO_LATENCY_IRQS=n`, so no level sits above the lock  |
| Registration           | ordinary vector at a reserved priority               | `IRQ_DIRECT_CONNECT(..., IRQ_ZERO_LATENCY)`, direct ISR only          |
| Kernel APIs in handler | forbidden                                            | forbidden, "undefined behavior"                                       |
| Levels reserved        | 2 by default, priorities 0 and 1                     | 1, `CONFIG_ZERO_LATENCY_LEVELS`                                       |

The mechanism is the same either way on ARMv7-M: `arch_irq_lock()` writes
`BASEPRI` to `_EXC_IRQ_DEFAULT_PRIO`
(`modules/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h`), and PRIMASK is the
ARMv6-M baseline path, not this one. What `CONFIG_ZERO_LATENCY_IRQS=y` changes
is the threshold: it adds `CONFIG_ZERO_LATENCY_LEVELS` to `_IRQ_PRIO_OFFSET`
(`.../arch/arm/cortex_m/exception.h`), reserving that many top priority levels
that the kernel lock never reaches. With it off the offset is zero and there is
nothing above the lock to register a handler at.

Because ChibiOS's kernel lock never masks the top levels by default, a
latency-critical ISR on a ChibiOS board is protected without anyone having
thought about it. Port the same logic to Zephyr and that property is silently
lost: every `irq_lock()` anywhere in the system, in drivers, kernel or
application, now delays your edge ISR.

Measured on `mr_vmu_rt1176` against a live CPPM receiver with a clean signal:
with default Zephyr locking, every `irq_lock()` window in the system added to
the timestamp, up to about 300 us, corrupting roughly 30% of decoded pulse
widths. RC input decoded, and was unflyable. Enabling ZLI and re-registering
the ISR as a zero-latency direct ISR brought the decode back. Those figures and
what the change costs elsewhere are recorded with the switch itself, under
`AP_RCIN_GPIO2_DIRECT_ISR` in `libraries/AP_HAL_Zephyr/zephyr/Kconfig`.
Timestamping has to come from the raw DWT cycle counter, because
`k_cycle_get_64()` takes a kernel spinlock and is therefore illegal in a ZLI
handler.

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

**One patch at the choke point. Not built - this is a design, not code to
copy.** Our Zephyr fork's copy of `modules/zephyr/drivers/console/uart_console.c` is stock
here: `console_out()` calls `uart_poll_out()` and waits. All UART-console
`printk` traffic does funnel through that one function, so the shape of the
patch would be: measure how long `uart_poll_out()` took, enter drop mode above
a few milliseconds, probe one character per 100 ms until it flows. That would
give drop-on-stall for every serial driver, using the driver's own blocking as
the sensor, with no per-driver knowledge. Kconfig-gate it.

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

**Low-power idle is a bring-up hazard, and this port does not use it.** WFI
can gate a whole clock domain: on the RT1176 it stops SysTick and DWT CYCCNT
together, and a WFI clock-domain freeze can take SWD down with it, which looks
like a dead board. Pinning the peripheral-domain clocks that idle gates is not
the answer this port arrived at. `CONFIG_AP_NO_WFI_IDLE` is `default y` for
every ARM board and `zephyr/CMakeLists.txt` refuses to build an ARM target
without it, because ChibiOS-ArduPilot - the firmware that actually flies these
vehicles - never executes WFI either, and its own realtime counter is DWT
CYCCNT, which stops in WFI.

There is a separate lesson in the board-specific half, because it is not what
anyone assumed. WFI does not stop TIM5 on an STM32H7: the H7 has two
independent gates per peripheral, `RCC_APB1LENR` for Run mode and
`RCC_APB1LLPENR` for Sleep, and the Sleep gate is not implied by the Run one.
With TIM5's Sleep gate clear the counter freezes for the duration of every idle
period, so every interval measured across a sleep comes out short by the idle
fraction - which is why the error was never a fixed factor. The bit comes out
of reset set, something earlier in the boot chain had cleared it, and
`hrt_init()` in `libraries/AP_HAL_Zephyr/system.cpp` now sets it explicitly
rather than trusting the reset value. The whole finding is written up in the
`AP_NO_WFI_IDLE` help text in `libraries/AP_HAL_Zephyr/zephyr/Kconfig`.

**MPU defaults are a bring-up hazard too, and turning the MPU off is not the
fix.** Default `ARM_MPU` settings produced MemManage faults against ordinary AP
memory usage on the RT1176. Disabling it cleared that one fault and broke
`NOCACHE_MEMORY` for every driver that needs it: the CAAM entropy driver's DMA
descriptor buffer stayed cacheable and its wait loop spun forever on a stale
cached flag, parking the CPU before ArduCopter init. It is `=y` today, and
`libraries/AP_HAL_Zephyr/zephyr/prj.nxprt1176.conf` carries the reasoning next
to the symbol. The real gap is that generic `arm_mpu_regions.c` has no explicit
region for ITCM, DTCM or the nocache area.

**Expect power-gated rails on small boards and plumb them first.** One board
here gates the LDO powering both its I2C sensors and the bus pull-ups, default
off. Every sensor probe then fails on a dead, pull-up-less bus, with no error
hinting at power. Read the schematic for enable pins before writing any driver
and drive them with devicetree GPIO hogs so they are up before code runs.

## What ArduPilot assumes of the C++ runtime

**ArduPilot relies on statics being zeroed, and mostly does not write `= 0` to
say so.** A few translation units do write it - `ArduCopter/land_detector.cpp`
is one - but the overwhelming majority of AP state is a bare static or a class
member left to `.bss`, and nothing in the source marks the assumption. The port
must guarantee zero-init actually happens on every RAM section AP objects land
in. Boot crashes here traced to exactly that guarantee being violated. If you
add noinit or custom RAM sections, audit what falls into them.

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

**Watch for interrupt-per-byte drivers, and check the driver you have rather
than the one a document describes.** Zephyr's LPSPI driver used to leave
`FCR = 0` and take an interrupt per received word; our fork's copy now sets
an RX watermark so a transfer raises roughly one interrupt per batch, in
`lpspi_master_set_rx_watermark()` in
`modules/zephyr/drivers/spi/spi_nxp_lpspi/spi_nxp_lpspi.c`. LPI2C at 400 kHz is
the same shape, and so is the DMA case above. Fine per unit, fatal at rate.

**Bus index mapping is configuration, not convention.** ArduPilot bus 0 is
whatever `I2C_ORDER` and `SPIDEV` say it is, and the mapping to devicetree
labels has to be explicit per board. A wrong mapping here made an onboard baro
and compass simply vanish.

**Leftover diagnostics change behaviour in both directions.** An I2C probe-scan
diagnostic left enabled ate most of the CPU and stretched boot past 170 s
against 4 s with it off - recorded against the switch in
`libraries/AP_HAL_Zephyr/hwdef/mr_vmu_rt1176/README.md`, and the reason every
profile captured before it was found measured the scan. Conversely, a
diagnostic's side effect was silently what made SPI work, and removing it broke
the board. Audit what your bring-up scaffolding actually does before either
deleting or shipping it.

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
   `board.cmake` and `board.yml`. Skip this only if Zephyr already has your
   exact board upstream. Of the four boards here only `native_sim` does:
   `ESP32S3Zephyr` carries its own board directory under
   `zephyr/boards/xtensa/` even though Espressif's parts are upstream.
3. Kconfig fragments. `_discover_zephyr_conf_fragments()` in
   `Tools/ardupilotwaf/zephyr.py` merges up to four layers, each on top of the
   last: `zephyr/prj.conf` for every board, then `prj.<mfr>.conf`, then
   `prj.<soc>.conf`, then the board layer - `zephyr/prj.<BoardName>.conf`, or
   `zephyr/boards/<board_name>.conf`, or a board-variant file matching
   `<board_name>_*.conf`, which is how `native_sim_native_64.conf` is found. A
   bootloader build appends `prj-bl.conf` and `prj.<BoardName>-bl.conf`, and
   `--enable-stats` appends its fragment last so an explicit request wins.
   A new SoC needs the middle two layers created, not just the board one.
4. A `class <BoardName>(zephyr_board)` in `Tools/ardupilotwaf/boards.py`,
   copied from `CubeOrangeZephyr` or `mr_vmu_rt1176`.
5. `./waf configure --board <BoardName> && ./waf copter -j12`

`hwdef.dat` here is a simplified subset of the ChibiOS format. The parser is
`_dispatch()` in `Tools/ardupilotwaf/zephyr_hwdef.py`; read it for the current
directive set, because an unrecognised line is ignored without a message:

```text
include <path>                              parsed inline, relative to this file
BOARD_NAME <name>
MCU <type>                                  informational
SERIAL_ORDER <port> ...                     EMPTY or NONE reserves an index
I2C_ORDER <bus> ...
CAN_ORDER <n> ...                           consumed by the class generator
IOMCU_UART <port>                           appended after SERIAL_ORDER, so
                                            AP_SerialManager never offers it
ROMFS <name-in-romfs> <path-in-tree>
SPIDEV <name> <bus> <devid> <cs> <mode> <low_speed> <high_speed>
define <NAME> [<value>]                     a later definition replaces an
                                            earlier one, as in ChibiOS
IMU <driver> SPI:<device> [<rotation>]
IMU <driver> SPI:<accel> SPI:<gyro>         two-device parts, e.g. BMI088
BARO <driver> I2C:<bus>:<addr> | SPI:<device>
COMPASS <driver> I2C:<bus>:<addr> [<external>] [<rotation>]
COMPASS <driver>:<probe_method> <instance> [<rotation>]
PIN <PAD> <FUNC> [<property> ...]           class generator
DMA <PERIPH> <rx_ch> <tx_ch> | <ch>         class generator, NXP only
PERIPH <node> <prop>                        class generator
```

Two things about that grammar bite in silence. All seven `SPIDEV` fields are
required and a short line is dropped with no message, so a device that never
appears is worth counting tokens over. And the two `COMPASS` forms take
different second arguments: the plain form takes a bus connection, the
`<driver>:<probe_method>` form takes an instance index, not a bus.
`APJ_BOARD_ID` is read by `Tools/ardupilotwaf/zephyr.py` for the upload step,
not by this parser, so its absence here is not a bug.

Pin mux is no longer devicetree-only. `PIN`, `DMA` and `PERIPH` are collected
verbatim and handed to `Tools/ardupilotwaf/zephyr_class_generator.py`, which
emits a per-board devicetree overlay plus a Kconfig fragment at configure time;
the overlay is appended to `EXTRA_DTC_OVERLAY_FILE` alongside the hand-written
board DTS, which the generator never modifies. It is inert on a board with no
such lines, and all three hardware boards carry them today: 75 `PIN` lines on
`mr_vmu_rt1176`, 42 on `CubeOrangeZephyr`, 7 on `ESP32S3Zephyr`. A peripheral
is emitted only if it has at least one `PIN` line, and two things the generator
cannot express keep a peripheral in the hand-written DTS - a node that must stay
disabled, and a child node needing properties other than pinctrl. Clocks and
peripheral enables are still devicetree only.

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

* `modules/zephyr` is an ordinary submodule.
* Zephyr's own dependency repositories cannot be submodules, because git will
  not track paths inside another submodule's gitlink. Their source of truth is
  `Tools/zephyr/zephyr_manifest_v4_4_0_map.tsv`: one row per dependency with
  name, URL, commit and path.
* `Tools/zephyr/zephyr_get_prerequisites.sh` reads that map and materialises
  every checkout.
* West may still be used *outside* the repository as a one-time metadata
  resolver, to discover the commit SHAs for a given Zephyr manifest revision.
  Its output is then translated into the TSV and committed.

The cost is that bumping the Zephyr version is a deliberate, scripted step
rather than a `west update`. That is the intended trade: the pins are visible,
reviewable and diffable in the repository rather than resolved at fetch time.
