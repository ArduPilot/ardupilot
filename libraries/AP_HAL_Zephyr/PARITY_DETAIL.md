# Parity detail: what ChibiOS does, and what we did instead

Design rationale for the parity work. What `AP_HAL_ChibiOS` does internally,
what this port does in its place, why, and how the answer differs per board.

[COMPARED_TO_CHIBIOS.md](COMPARED_TO_CHIBIOS.md) has current state, one row per
feature. This file does not restate it.

## Crash dump

### What ChibiOS does

Three layers, all present and wired together, and all three live in this
repository. There is no `modules/CrashDebug` submodule any more: it is absent
from disk, from `.gitmodules` and from `git ls-files`, and the old
`hwdef/common/crashdump.c` went with it.

1. **`AP_HAL_ChibiOS/CrashCatcher.cpp`** and `.h` are the capture engine, an
   in-tree adaptation of Adam Green's CrashCatcher carrying its original
   Apache-2.0 header. It runs from the Cortex-M hard-fault entry point, walks a
   list of memory regions, and feeds the bytes through four callbacks the
   target implements, declared in `CrashCatcher.h`:
   `CrashCatcher_GetMemoryRegions()`, `CrashCatcher_DumpStart()`,
   `CrashCatcher_DumpMemory()`, `CrashCatcher_DumpEnd()`.

2. **`AP_HAL_ChibiOS/CrashDump.cpp`** implements those four callbacks and hands
   the bytes to one of two backends.

   `CrashDump_flash.cpp` writes the dump into a dedicated internal MCU flash
   region, sized as whatever is left at the end of the `flash` linker region
   after the firmware image (`hwdef/common/common.ld`, `.crash_log (NOLOAD)`,
   `__crash_log_base__` / `__crash_log_end__`). It builds the region list
   itself: ChibiOS's own registry walk
   (`chRegFirstThread()` / `chRegNextThread()`) for thread stacks, then BSS and
   heap, each truncated to the space the region has left. Writes go through
   `stm32_flash_write()` in 32-byte chunks, the alignment an H7 flash write
   needs, with `stm32_watchdog_pat()` after each chunk so the dump cannot
   trigger a watchdog reset mid-write.

   `CrashDump_SD.cpp` is the other backend, writing the same dump to a FatFs SD
   card. It replaced the raw-UART hex-dump path: `HAL_CRASH_SERIAL_PORT` no
   longer appears anywhere in the tree.

   `CrashCatcher_DumpStart()` also calls `save_fault_watchdog()`
   (`system.cpp:257`), the same persistent-data mechanism the monitor thread
   uses, so one fault populates both the dump and the RTC-backup forensic
   summary.

3. **`AP_Filesystem_Sys.cpp`**, shared HAL-generic code, exposes the
   flash-resident dump as `@SYS/crash_dump.bin`, backed by
   `hal.util->last_crash_dump_ptr()` / `last_crash_dump_size()`. Retrievable
   over MAVFTP on the next boot after a crash, no debugger required. Note which
   symbol gates it: the directory entry, the read and the `stat()` case are all
   inside `#if AP_CRASHDUMP_FLASH_ENABLED`, not `AP_CRASHDUMP_ENABLED`, so the
   SD backend publishes no `@SYS` file and neither does a board that defines
   only the umbrella symbol.

Gating is generated rather than written by hand. `chibios_hwdef.py`'s
`write_crashdump_config()` emits
`AP_CRASHDUMP_ENABLED (AP_CRASHDUMP_FATFS_ENABLED || AP_CRASHDUMP_FLASH_ENABLED)`
into `hwdef.h`. Both halves default on at `flash_size >= 2048` KB and off for a
bootloader or AP_Periph build. A board with a FatFs-capable SD card on a
supported controller takes the FatFs backend by default; every other board
takes the flash one.

### Board scope: crash dump

The absence was HAL-wide when this was written: no board overrode
`AP_CRASHDUMP_ENABLED` and no board had a `Util::last_crash_dump_*()`
override. That part has changed for one board. `hwdef/mr_vmu_rt1176/hwdef.dat`
sets `AP_CRASHDUMP_ENABLED 1`, `AP_HAL_Zephyr/Util.cpp` implements both
accessors by reading the XIP-mapped `coredump_partition` directly, and
`zephyr/src/rt1176_coredump_backend.c` is the Zephyr `coredump` backend that
fills it. Retrieval is still not reachable from a GCS, for the
`AP_CRASHDUMP_FLASH_ENABLED` reason given above;
[DEBUGGING.md](DEBUGGING.md) has the reading-it-back side. The rest of the fix
is still not one implementation for all boards.

* **`mr_vmu_rt1176`**: external XIP NOR, no internal flash, needs the custom
  ROM-API backend. Firmware runs XIP from 64 MB of external NOR over FlexSPI
  (`reg = <0x30000000 DT_SIZE_M(64)>`), and param storage occupies 128 KB at
  `0x620000`, so region budget is not a constraint. The open question was
  mechanism: CrashCatcher writes synchronously from a fault handler, with no
  interrupts and no scheduler, which is not what the ROM-API flash write path
  assumes. The answer was `zephyr/src/rt1176_coredump_backend.c`, a
  `CONFIG_DEBUG_COREDUMP_BACKEND_OTHER` backend hanging off Zephyr's own
  `z_fatal_error()` path rather than off the vector table, writing through the
  same ROM-API primitives `Storage.cpp` uses. Read that file's header before
  changing its erase or write footprint: `z_fatal_error()` holds
  `arch_irq_lock()` across the whole dump, so nothing feeds the watchdog while
  it runs.
* **`CubeOrangeZephyr`**: STM32H743 with 2 MB of real internal flash, the exact
  model ChibiOS's flash backend was written against. Architecturally the
  closest of the set to a drop-in port: a dedicated `.crash_log` flash region
  and `stm32_flash_write()`-style chunked writes both apply more directly here
  than on rt1176. `CONFIG_FLASH` and `CONFIG_FLASH_MAP` both come out `y` in
  its merged configuration, so Zephyr's standard
  `coredump_backend_flash_partition.c` could plausibly be used as-is without
  rt1176's custom-backend detour. Not tried.
* **`ESP32S3Zephyr`**: CrashCatcher cannot be ported here at all. Its entry
  point is `AP_HAL_ChibiOS/hwdef/common/CrashCatcher_armv7m_asm.S`,
  hand-written Cortex-M assembly that builds a
  `CrashCatcherExceptionRegisters` frame for
  `CrashCatcher_Entry()`, and the C++ engine above it reads ARMv7-M
  exception-frame bits (`EXC_RETURN`, `PSP`/`MSP`, the FPU and stack-align
  flags) and the Cortex-M fault status registers at `0xE000ED28` directly.
  Xtensa LX7 is a different instruction set, register file and exception-frame
  layout. Any crash dump here goes through Zephyr's `coredump` subsystem
  exclusively. Storage is the easiest of the real-silicon targets:
  `CONFIG_FLASH=y`, `CONFIG_FLASH_MAP=y` and `CONFIG_ZMS=y` are all
  genuinely enabled in `zephyr/boards/esp32s3_zephyr.conf`, on real internal
  flash with no XIP-trap constraint. The practical blocker is that this board's
  hwdef disables `AP_LOGGER_ENABLED`, so there is no `LOG_WDOG` flight-log line
  to report a dump exists. `@SYS/crash_dump.bin` does not cover for that on any
  board here, for the reason given above.
* **`native_sim`**: a crash is a host SIGSEGV, not a Cortex-M or Xtensa hard
  fault. Not applicable.

## Persistent crash and watchdog forensics across reset

ChibiOS saves `hal.util->persistent_data` (the shared
`AP_HAL::Util::PersistentData` struct: attitude, home position, fault
addr/ICSR/LR, internal-error state, last MAVLink msgid and cmd,
`semaphore_line`, SPI/I2C counters, `scheduler_task`, armed and safety state)
into STM32 RTC backup registers every 100 ms from the monitor thread, and reads
it back on the next boot via `was_watchdog_armed()`, reporting it as a
`LOG_WDOG` dataflash message.

`CrashCatcher_DumpStart()` calls the same `save_fault_watchdog()` the monitor
thread calls, so on ChibiOS a fault handler populates the crash dump and the
forensic summary in one shot. A Zephyr fault handler should therefore populate
the same persistent-storage target the crash dump uses, not a separate
mechanism.

Two things this port used to lack are in it now, in a reduced form: the
persistent record itself, and the runtime check that decides whether to arm the
watchdog at all. `Scheduler::save_persistent_data()` copies the struct into a
`__noinit` backup behind a magic word and `restore_persistent_data()` reads it
back on a boot that follows a watchdog reset; `ap_persistent_save_fault()` is
the bridge the C fatal handler calls to fill in the fault fields.
`AP_BoardConfig::watchdog_enabled()`, the `BRD_OPTIONS` parameter bit, is
checked twice in `Scheduler.cpp` — before the hardware watchdog timeout is
installed, and again before the monitor thread resets the board — so the
watchdog is opt-in here exactly as it is on ChibiOS, rather than armed
unconditionally from Kconfig.

Still short of ChibiOS: the monitor thread escalates in two stages, not three.
It warns and dumps thread state at `MONITOR_WARN_MS` (500) and resets at
`MONITOR_RESET_MS` (1800), against a 2000 ms hardware watchdog. There is no
self-healing forced mutex release — `Scheduler::try_force_mutex()` only prints,
and says in its own comment that Zephyr has nothing to force — and no
self-triggered informative hard fault just before the hardware watchdog fires.

### Board scope: persistent crash and watchdog forensics across reset

The watchdog pat itself (the main loop calling
`schedulerInstance.watchdog_pat()` in `HAL_Zephyr_Class.cpp`) is shared code
and applies to every board that arms a watchdog device. Checked per board
against each board's own Kconfig:

* **`CubeOrangeZephyr`**: STM32 **IWDG**, a different physical peripheral from
  rt1176's `WDOG1`, wired through Zephyr's generic `watchdog` API the same way.
  `zephyr/boards/cube_orange_zephyr.conf` sets `CONFIG_WATCHDOG=y` and
  `CONFIG_WDT_DISABLE_AT_BOOT=n` with a comment citing the monitor thread, so
  this board's watchdog Kconfig was set up aware of the shared pat mechanism.
  Correct by inspection, unverified on hardware.
* **`ESP32S3Zephyr`**: had the bug rt1176 had. Its DTS enabled the node
  (`&wdt0 { status = "okay"; }`) with no `CONFIG_WATCHDOG=y` anywhere, and
  `device_is_ready(wdt)` is silently false without the Kconfig symbol, so the
  node was dead. Closed: `zephyr/prj.ESP32S3Zephyr.conf` now sets
  `CONFIG_WATCHDOG=y` and `CONFIG_WDT_DISABLE_AT_BOOT=n`, matching the other
  boards. Correct by inspection, unverified on hardware.
* **`native_sim`**: no watchdog concept for a host process.

## RC output

### The init gate

`HAL_Zephyr_Class.cpp` called `rcin->init()` but deliberately not
`rcout->init()`. `RCOutput::_map_ready` defaults false and is only set true
inside `init()`, and `_apply_channel()` bails immediately when `!_map_ready`,
so every `write()`, `set_freq()`, `enable_ch()` and `push()` call was pure
bookkeeping with no `pwm_set()` reaching the FlexPWM peripheral. That is what
made it safe to enable the fast rate thread before motors were verified.

The gate lived in `HAL_Zephyr_Class.cpp`, compiled identically for every board,
so it correctly gated output everywhere rather than on one board. It was a
bring-up measure and it is gone: that file calls both `rcin->init()` and
`rcout->init()` now. `ESP32S3Zephyr` has an independent reason RCOutput cannot
run there: `CONFIG_PWM=n` in `zephyr/boards/esp32s3_zephyr.conf` ("No eFlexPWM
on ESP32-S3", true generically since eFlexPWM is an NXP peripheral). A real
ESP32-S3 PWM driver would need Zephyr's LEDC-based PWM binding.
`AP_RCIN_ENABLED 0` and no PWM pins in its hwdef mean the whole RC and motor
path is off by board design there.

### DShot came from an existing in-tree driver

The driver (CogniPilot, Copyright NXP) is a complete FlexIO-based DShot and
bidirectional-DShot implementation for this SoC's FlexIO peripheral,
including rt1176 clock setup and GCR/RLL telemetry decode. Functionally a
from-scratch equivalent of ChibiOS's bdshot machinery, built on FlexIO instead
of DMA plus timer capture. It was instantiated only on Cerebri's own board
overlays.

Wiring it up meant carrying a copy of the driver in-tree as `zephyr/src/nxp_flexio_dshot.c`
plus header and binding, byte-identical to the original apart from the
provenance note at the top of that file, instantiated it in the board DTS on
FMU_CH1-8 (whose pads matched cerebri's `vmu_rt1170` overlay pin for pin, same
VMU hardware family), and drove it from `set_output_mode()` in `RCOutput.cpp`. The
work was devicetree wiring, `set_output_mode()` integration, and three Kconfig
symbols. The driver source needed zero changes. Runtime PWM to DShot pad
arbitration goes through a driverless `ap_rcout_mux` pinctrl-state node, with
PWM as the deterministic boot default.

FlexIO is NXP-specific silicon. The same peripheral family exists on some STM32
parts under a different name, but not on STM32H743, and Xtensa has no FlexIO
equivalent. DShot on the other real-silicon boards needs independent driver
work: STM32 timer plus DMA bdshot for `CubeOrangeZephyr`, matching ChibiOS's
own approach, and an RMT- or LEDC-based driver for `ESP32S3Zephyr`.

## DMA channel arbitration

ChibiOS has `Shared_DMA`, a runtime broker that lends and evicts channels
between peripherals and reports contention through `@SYS/dma.txt`. No board
here has an equivalent. Every channel is a fixed devicetree binding owned by
one peripheral at compile time. That absence is architectural and applies
everywhere.

The specific starvation mechanism below is `mr_vmu_rt1176`-only.
`GRP0PRI` / `GRP1PRI` are fields of the NXP **eDMA** controller's `CR`
register, a silicon IP block used on i.MX RT parts.

* **`CubeOrangeZephyr` (STM32H743)** uses ST's DMA controller plus **DMAMUX**,
  a separate request-routing crossbar that is not part of the DMA controller,
  with a per-stream priority field (Low/Medium/High/Very High) set per DMA
  request in the stream's own config register. There is no "fixed group of 16
  channels always beats the other 16" concept. The closest STM32 analogue to a
  starvation bug is two streams at the same priority level racing on hardware
  round-robin, a different failure mode needing its own investigation. This
  board is not at risk of the eDMA bug, which says nothing about whether its
  own DMA priority assignments have ever been reasoned about.
* **`ESP32S3Zephyr`** uses Espressif's **GDMA**: per-channel priority
  registers, no group concept. Moot: its DTS and Kconfig wire DMA for none of
  the peripherals below.
* **`native_sim`**: no DMA hardware.

### The eDMA channel and group table

Group 0 is channels 0-15, Group 1 is channels 16-31.

| Peripheral | Role                             | Channels   | Group       |
| ---------- | -------------------------------- | ---------- | ----------- |
| LPSPI1     | IMU SENSOR1                      | 0, 1       | **Group 0** |
| LPSPI2     | IMU SENSOR2 (primary flight IMU) | 2, 3       | **Group 0** |
| LPSPI3     | IMU SENSOR3 (BMI088)             | 4, 5       | **Group 0** |
| LPUART1    | Debug console                    | 6, 7       | **Group 0** |
| LPUART4    | TELEM1                           | 10, 11     | **Group 0** |
| LPUART5    | GPS2                             | 12, 13     | **Group 0** |
| LPUART6    | RC-IN (SBUS/CRSF)                | 14, 15     | **Group 0** |
| LPUART8    | TELEM2                           | 16, 17     | Group 1     |
| LPUART10   | TELEM3                           | 18, 19     | Group 1     |
| LPUART3    | GPS1, relocated from Group 0     | 20, 21     | Group 1     |
| LPI2C1/2/3 | I2C buses (mag/baro/external)    | 22, 23, 24 | Group 1     |

### Fixed-priority group arbitration, and the ERGA fix

The eDMA `CR` register's group-priority bits sit at their reset values,
because nothing writes them: `dma_mcux_edma.c`'s only two references to `->CR`
are debug reads, and the SDK `EDMA_Init()` it calls read-modify-writes `CR`
without touching `GRP0PRI` or `GRP1PRI`. Under fixed-priority arbitration that
leaves one whole group winning against the other regardless of any individual
channel's own priority field — a global controller property, not something
scoped to whichever peripheral is under investigation.

Which group loses is written down two different ways here, and neither is a
reading off hardware. This section and the table above take the reset default
as `GRP1PRI=1 > GRP0PRI=0`, which makes Group 0 the starved half;
`zephyr/src/rt1176_edma_arbitration_fixup.c`'s own header comment says the
reverse. Settle it from the reference manual, or from one `CR` read on the
board, before relying on either. The fix below does not depend on the answer.

One peripheral already sits outside Group 0: the `dmas` property on `&lpuart3`
in the board DTS asks for channels 20 and 21, on the far side of the group
boundary from the rest of the UARTs. Its own comment gives the reason as a
collision with another peripheral's fixed assignment during the GPS1 work, not
group priority. Read against the table, and taking the direction above, seven
peripherals sat on the losing side:

* **All three IMU SPI buses (LPSPI1/2/3)**, the highest-rate,
  most latency-sensitive DMA consumers on the board. The DTS bring-up notes
  record a separate DMA RX-overrun bug on these same buses (mid-frame channel
  re-arm racing a level-sensed eDMA request), fixed independently.
  Group-priority disadvantage was an additional structural risk on top.
* **LPUART6 (RC-IN)**, safety-critical control input, the same class of
  peripheral as GPS1 (async RX with idle-line DMA completion) and the same
  silicon behaviour.
* **LPUART5 (GPS2)**, same driver, same traffic pattern, very likely the same
  u-blox part as GPS1. Structurally identical to the port that did misbehave,
  though nothing in the tree ties that misbehaviour to group priority.
* LPUART4 (TELEM1) and LPUART1 (console), lower real-time criticality, same
  structural disadvantage.

Meanwhile TELEM2, TELEM3 and all three I2C buses, comparatively low-rate
traffic, held the advantaged Group 1 slot purely as an accident of
channel-number sequencing.

`zephyr/src/rt1176_edma_arbitration_fixup.c` sets `CR.ERGA` (Enable Round Robin
Group Arbitration, bit 3) at `POST_KERNEL` init, so all 32 channels arbitrate
fairly regardless of group. It is a read-modify-write of one bit and it is
invisible from the console, so it keeps the before and after values in the
globals `g_edma0_cr_before` and `g_edma0_cr_after` for exactly that reason:
read those two over SWD to confirm the bit took. No such capture is recorded
anywhere in this repository, so treat the fix as correct by inspection until
one is.

One precision point. `ERCA` is Enable Round Robin **Channel** Arbitration and
only affects arbitration *within* a group. `ERGA` is the bit that governs
Group 0 against Group 1. `GRP0PRI`, `GRP1PRI` and `ERCA` are all left at reset
defaults; this is a single-bit read-modify-write. `lpuart3`'s channel
relocation is left where it is: it is harmless, and nothing depends on which
group a channel lands in any more.

Still open: a hardware test showing the seven previously-disadvantaged
peripherals arbitrate fairly under real simultaneous DMA load, and that nothing
regressed for the six that used to hold Group 1 unconditionally.

### eDMA behaviour to know before porting another i.MX RT

These are properties of the silicon and of Zephyr's `dma_mcux_edma.c`, not of
this board, and they are worth writing down because none of them fails as "DMA
is slow". They fail as a boot hang, as a second transfer failing where the
first worked, as all 32 channels stopping at once, or as RX that works under
load and stalls when the link goes quiet.

* **A UART RX channel has to do scatter/gather, cyclically.**
  `uart_mcux_lpuart.c` sets `dest_scatter_en = true` on the RX block config and
  `.cyclic = 1` in the DMA config it hands over, which sends
  `dma_mcux_edma.c` down `dma_mcux_edma_configure_sg_loop()`. A controller, or
  a model of one, that handles only a single linear block will not carry this
  driver's RX at all.

* **`DREQ` belongs to the TCD that is completing, not the one being loaded.**
  `configure_sg_loop()` sets DREQ on the descriptor it installs as the hardware
  TCD, while `EDMA_TcdSetTransferConfig()` in the NXP SDK's `fsl_edma.c` clears
  it on the pool entries. Read the other way round, `ERQ` is disarmed one
  transfer early. RX survives that anyway only because `dma_mcux_edma.c` calls
  `EDMA_EnableAutoStopRequest(..., false)` on the live reload path before the
  descriptor completes.

* **`CSR[ACTIVE]` is busy-waited with no timeout, in thread context.**
  `dma_mcux_edma.c`'s reload path spins on `CSR & ACTIVE`, and
  `uart_rx_enable()` reaches it from inside `UARTDriver::begin()`. Anything
  that leaves `ACTIVE` set hangs the board during boot inside a UART `begin()`,
  which reads as a UART fault rather than a DMA one.

* **`CITER` must reload from `BITER` at major completion.** Where it does not,
  `EDMA_SubmitTransfer()` returns `kStatus_EDMA_Busy`, `dma_config()` turns
  that into `-EFAULT`, and the *second* `uart_tx()` fails while the first
  worked. A first-transfer-only symptom points straight here.

* **The DMAMUX is two read-modify-writes of a value-retaining register.**
  `dma_mcux_edma.c` calls `DMAMUX_SetSource()` then `DMAMUX_EnableChannel()`,
  not one combined write and not `EDMA_SetChannelMux()` — this part has no
  `FSL_FEATURE_EDMA_HAS_CHANNEL_MUX`. A `CHCFG` that does not read back what
  was written loses either the source or the enable.

* **`nxp,a-on` changes how memory-to-memory transfers start.** Zephyr's RT11xx
  SoC devicetree sets that property on the eDMA node, so `dma_mcux_edma.c`
  starts mem2mem from the DMAMUX always-on bit and leaves `SOURCE` at 0 on
  those channels. Routing that treats `SOURCE == 0` as a real request source
  will mis-route them.

* **One error stops all 32 channels.** `EDMA_GetDefaultConfig()` sets
  `enableHaltOnError`, so `CR[HOE]` is 1 and the first error latches
  `CR[HALT]`. On top of that, because this part has a separate error interrupt
  (`FSL_FEATURE_EDMA_HAS_ERROR_IRQ` is 1 in `MIMXRT1176_cm7_features.h`, so the
  devicetree does not set `no-error-irq`), the compiled-in path is
  `dma_mcux_edma_error_irq_handler()` — and that aborts every channel its own
  bookkeeping thinks is busy, without checking which channel actually errored.

* **Where the TCD pool lives is a Kconfig choice.**
  `CONFIG_DMA_MCUX_USE_DTCM_FOR_DMA_DESCRIPTORS=n` in
  `zephyr/prj.mr_vmu_rt1176.conf`, so the scatter/gather pool sits in ordinary
  SRAM where the controller's own descriptor fetch reaches it. Turning it on
  moves the pool into this board's 32 KB DTCM; if the fetch cannot reach there
  it reads zeros and RX dies after one buffer.

* **The RX ring is two descriptors deep.** `CONFIG_DMA_TCD_QUEUE_SIZE` defaults
  to 2. When bytes arrive faster than the driver swaps buffers, `dma_reload()`
  returns `-ENOBUFS` rather than quietly dropping data, so the symptom to look
  for is an error return, not missing bytes.

* **Partial RX delivery has exactly one trigger, the LPUART idle line.**
  `uart_mcux_lpuart.c` starts the async RX timeout from a single place, inside
  the `kLPUART_IdleLineFlag` branch of its ISR. Without `STAT[IDLE]` and
  `CTRL[ILIE]` working, a buffer is handed up only when it fills, so traffic
  that never fills a 128-byte buffer is never delivered: RX looks fine under
  load and stalls when the link goes quiet.

## Open defect: the kernel tick is finer than the hardware will honour

`prj.conf` sets `CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000000` for every Zephyr
board. On mr_vmu_rt1176 the counter runs at 1 GHz, so `CYC_PER_TICK` is 1000
cycles - a 1 us quantum.

The board's devicetree then sets `zephyr,min-timeout-cycles = <10000>` in
`zephyr/boards/arm/mr_vmu_rt1176/mr_vmu_rt1176_mimxrt1176_cm7.dts`, which
floors every kernel timeout at 10 us. That is ten times the tick quantum, so
the last factor of ten of tick resolution buys nothing the hardware will
honour, while the tick ISR keeps paying for it: a DWT PC-sampling profile on
silicon put `sys_clock_isr` at about 7 % of all CPU, the largest single entry
in that profile.

The two settings need reconciling - either the tick is coarser or the floor is
lower - and neither has been tried. ArduPilot takes its own microsecond time
base from GPT2, not from the kernel tick, so the vehicle code does not depend
on the finer quantum.

## Open defect: the UART RX re-arm retriggers itself at 1 kHz

Still in the shipping tree, found while modelling the eDMA and never fixed. It
is an ArduPilot bug, not an emulator one, and it runs on hardware.

`_rx_need_restart` (`UARTDriver.h:115`) exists so that an RX teardown seen from
the async callback gets rebuilt in thread context, because `uart_rx_enable()`
from an ISR is not safe on every Zephyr driver. `_async_cb()` sets it on every
`UART_RX_DISABLED` event (`UARTDriver.cpp:547`), with no way to tell an
unwanted teardown from a deliberate one.

Both deliberate teardowns therefore set the flag they just cleared.
`_rx_timer_tick()` clears `_rx_need_restart`, calls `uart_rx_disable()`, then
`uart_rx_enable()` (`UARTDriver.cpp:759-766`), and `mcux_lpuart_rx_disable()`
ends by invoking the user callback with `UART_RX_DISABLED` unconditionally and
synchronously on the calling thread, under `irq_lock` (its last act, in
`modules/zephyr/drivers/serial/uart_mcux_lpuart.c`). So the flag is true again
before the tick returns, and the next tick tears RX down and rebuilds it once
more. `begin()` has the same shape (`UARTDriver.cpp:234-240`).

The rate is the timer thread's: `_uart_timer_tick()` is registered with
`register_timer_process()` (`UARTDriver.cpp:290`) and that thread runs at
1000 Hz (`Scheduler.cpp`, `k_timer_start(&s_hal_timer, K_USEC(1000),
K_USEC(1000))`). So every async UART on the board tears its RX DMA down and
rebuilds it a thousand times a second. Buffered bytes are not lost —
`mcux_lpuart_rx_disable()` flushes and releases the buffer first — but it turns
the receiver off (`LPUART_EnableRx(lpuart, false)`) before `uart_rx_enable()`
turns it back on, so anything arriving in that window is gone at the hardware
level, and the churn is paid on every tick either way.

Scope is one board today. The async path is only taken when
`uart_callback_set()` succeeds, which needs `CONFIG_UART_ASYNC_API=y`, and only
`zephyr/prj.mr_vmu_rt1176.conf` sets it. Everywhere else `_use_async` stays
false and the interrupt path runs instead.

The proposed fix was never written and never tested: a `_rx_self_disable` flag
raised around the two deliberate `uart_rx_disable()` calls and tested in the
`UART_RX_DISABLED` case, which is enough on its own because that callback runs
synchronously on the calling thread. There is no `_rx_self_disable` anywhere in
the tree — grep before assuming someone landed it.

Treat it as a hardware change, not a cleanup. It stops a teardown and rebuild
that has been running at 1 kHz on silicon for the whole life of this port, so
GPS1 and the TELEM ports may behave differently afterwards — most likely
better, but differently. Build it, flash it and check the board before
committing it.

## Wide/Octal SPI (`WSPIDevice`)

Not a real gap. Architecturally different, correctly and deliberately unused.

ChibiOS's `WSPIDevice` wraps its QUADSPI/OCTOSPI peripheral for boards with a
*separate* external NOR chip from their internal program flash, primarily so
the bootloader can program that external chip (`AP_FlashIface_JEDEC` into
`Tools/AP_Bootloader/bl_protocol.cpp`, gated on `EXT_FLASH_SIZE_MB`).

rt1176 has no internal program flash. The external FlexSPI NOR *is* the program
flash, executed XIP and managed by the BootROM's own flash API, both already
implemented here in `Storage.cpp`'s ROM-API backend.
`Tools/AP_Bootloader/hwdef_zephyr.h` documents `EXT_FLASH_SIZE_MB` as
deliberately left undefined for that reason. `Zephyr::WSPIDevice` and
`WSPIDeviceManager` alias `AP_HAL_Empty`'s no-op stub, and nothing repo-wide
routes through `hal.wspi` outside `AP_FlashIface` itself, which this
bootloader does not use.

The other boards reach the same N/A conclusion for structurally different
reasons, so rt1176's justification should not be read as the general one:

* **`CubeOrangeZephyr` (STM32H743)**: 2 MB of internal flash is the program
  store, same as every ChibiOS STM32 board, and no external wide or octal SPI
  NOR is wired in its DTS or hwdef. N/A because there is no external flash chip
  to wrap, not because something else owns the role.
* **`ESP32S3Zephyr`**: the attached SPI NOR holding the Zephyr image is managed
  by ESP-IDF's own flash and MMU subsystem below Zephyr, outside any
  `WSPIDevice`-shaped abstraction.
* **`native_sim`**: no flash hardware.

## Software PPM/RC signal capture (`SoftSigReader`)

ChibiOS's `SoftSigReader` / `SoftSigReaderInt` decode PPM-SUM via hardware
timer input-capture plus DMA, using the same `Shared_DMA` broker as above.

`AP_HAL_Zephyr/RCInput.cpp` implements PPM-SUM by a simpler mechanism: plain
GPIO edge interrupt plus `AP_HAL::micros()` timestamping, gated on a board
declaring an `rcin-gpios` devicetree property. That path is generic HAL-wide
code, and `CubeOrangeZephyr` is its existing proof, with
`rcin-gpios = <&gpiod 14 GPIO_ACTIVE_HIGH>;` in its DTS.

`mr_vmu_rt1176` declared no such property while that was being written, because
the RC-IN connector is wired to a single net (`UART6_TX_TO_IO__RC_INPUT`, RX
pad not connected) already occupied by the UART-based SBUS/CRSF path, with no
second pin to dedicate to GPIO-edge capture. Its DTS now carries a
`zephyr,user` node with both `rcin-gpios` (`&gpio2 8`) and a `pwms` phandle to
`qtmr1` channel 0 — the same pad, reached two ways — because the QTMR capture
path described below needs the second entry.

The rt1176 implementation went further than SoftSigReader parity. PPM-SUM is
decoded by QTMR hardware input capture: RC pad, XBARA1 crossbar, GPR12 timer
input select, QTMR1 ch0, through Zephyr's in-tree `pwm_mcux_qtmr`
`CONFIG_PWM_CAPTURE` driver. The period is latched by timer silicon at each
edge, the same `tim->CCR` architecture ChibiOS's `SoftSigReaderInt` reads,
rather than the software-timestamp approximation. Bench: 8 channels, 0.1 to
2.2 us per-channel stdev with USB telemetry and SD logging running, stick
tracking and TX-loss failsafe user-confirmed. The software GPIO edge path still
exists behind `!CONFIG_AP_RCIN_PWM_CAPTURE` for boards without a
capture-capable timer route. `pwm_stm32.c`'s capture support is the same
portable API for `cube_orange_zephyr` when wanted. The zero-latency-interrupt
material that came out of the software path is in
[ARCHITECTURAL.md](ARCHITECTURAL.md).

`ESP32S3Zephyr` declares no `rcin-gpios` and sets `AP_RCIN_ENABLED 0` in its
hwdef, so RC input of any kind is off by board design. `native_sim` has no RC
input hardware.

## CAN / CAN-FD / DroneCAN

### What exists and what is missing

`AP_HAL_Zephyr/CANIface.cpp` and `.h` is a genuine driver against Zephyr's
native CAN subsystem: send, receive, select, RX callback, error counting. Not
scaffolding. It is classic CAN 2.0B only. `send()` hard-rejects anything over 8
bytes, frame conversion always sets `canfd = false`, and there is no
`init(bitrate, fdbitrate)` overload to request an FD data-phase rate.

ChibiOS's CAN-FD backend (`CANFDIface.cpp`, STM32H7 and G4 only) adds FD
framing and bit-rate switching, independent nominal and data-phase timing,
64-byte payloads, and message-RAM-based filtering. This port has a single
wildcard pass-all filter and no per-message-ID filtering at all, a functional
regression against either ChibiOS backend.

One correction to the record: ChibiOS's CAN-FD driver does not use the FDCAN
peripheral's hardware timestamp counter per frame. Both backends timestamp via
`AP_HAL::micros64()` in software, so that is not a differentiator despite
sometimes being described as one.

### How it was disabled, precisely

CAN is wired on rt1176: both FlexCAN1 and FlexCAN2 have schematic-confirmed
pinmux to physical JST-GH connectors (`mr_vmu_rt1176-pinctrl.dtsi`), and
`hwdef.dat` documents `CAN_ORDER 1 2`, 2 of the board's 3 physical FlexCAN
buses. `HAL_NUM_CAN_IFACES 0` in hwdef.dat plus `status = "disabled"` on both
DTS nodes gated everything off, which compiled `AP_DroneCAN` out entirely since
`HAL_ENABLE_DRONECAN_DRIVERS` chains from `HAL_NUM_CAN_IFACES`.

hwdef.dat's comment read "CAN support in Zephyr HAL... currently not
implemented", but `CANIface.cpp` was added *earlier* than that comment. The DTS nodes were disabled as part of a global,
precautionary disable of nearly every non-essential peripheral after an
unrelated SWD lockup scare, suspected pin/pad contention. Every other
peripheral disabled in that sweep (SPI, I2C, ADC, PWM, SD card) was
progressively re-enabled.

### Getting it live

Beyond the DTS re-enable, three never-exercised bugs had to go: an ISR-illegal
`k_mutex` dropping every RX frame at the first hop, a missing
`get_system_id_unformatted()` override silently aborting `AP_DroneCAN::init()`,
and no `StorageCANDNA` area at `HAL_STORAGE_SIZE=8192` (now 16384). Plus
`board/zephyr.h`'s unconditional `HAL_MAX_CAN_PROTOCOL_DRIVERS 0`, CANIface's
stale filter-flag API, and a new `CANIface::get_stats()` backing
`@SYS/canN_stats.txt`, which was the instrument that cracked it.

Bench evidence, read back with `Tools/scripts/zephyr_can_nodes.py` over
`@SYS/can0_stats.txt` and `can1_stats.txt`: Matek F405 GPS on CAN1 as node 113,
mRo M10025 on CAN2 as node 125, both discovered by name, unique ID and software
version; 423k and 73k frames received respectively, zero overflows, zero bus
errors, `error-active`, TEC and REC at 0. The autopilot is on the network as
node 10 carrying its OCOTP unique ID. The SWD-lockup concern did not
reproduce. PiccoloCAN and FETtec are out of scope; only AP_DroneCAN matters
here. A
`Duplicate Node .../125` prearm was seen and closed as not repeatable on the
bench, with no code change: a decision, not a root-caused fix.

Adding real CAN-FD means writing a Zephyr counterpart to ChibiOS's
`CANFDIface.cpp`, several times the size of the classic `CANIface.cpp` it would
sit beside.

### Board scope: CAN / CAN-FD / DroneCAN

Each board lands somewhere different, and none shares rt1176's story.

* **`CubeOrangeZephyr` (STM32H743)** has the most capable CAN silicon of the
  set: real hardware FDCAN, genuine CAN-FD rather than software-emulated. Its
  DTS nodes are `status = "okay"` and its base defconfig sets
  `CONFIG_CAN_STM32H7_FDCAN=y`, but `prj.CubeOrangeZephyr.conf` overrides that
  back off with `CONFIG_CAN=n` and a documented hazard: *"STM32H7 FDCAN needs
  PLL2Q as kernel clock (FDCANSEL); without PLL2 configured the driver may
  hardfault during message-RAM init."* That is a clock-tree configuration bug,
  not a precautionary sweep. Re-enabling CAN here needs PLL2 wired up first.
* **`ESP32S3Zephyr`**: `CONFIG_CAN=n` in its board conf and `HAL_CAN_ENABLED 0`
  in its hwdef.dat, under the same "reduce BSS" theme as its other disables.
  The chip does have CAN-capable silicon, Espressif's **TWAI** peripheral,
  classic CAN 2.0 only with no hardware CAN-FD on this part, but nothing in the
  board's DTS instantiates or wires it. A from-scratch bring-up, not a
  re-enable.
* **`native_sim`** is the one board where CAN works without hardware. Its
  hwdef.dat sets `CAN_ORDER 1` and `HAL_NUM_CAN_IFACES 1`, its board conf sets
  `CONFIG_CAN=y` and `CONFIG_CAN_LOOPBACK=y`, and `boards.py`'s `native_sim`
  class sets `self.with_can = True` and wires the full CANARD/DroneCAN define
  set. Loopback only, one node talking to itself, so it does not substitute for
  real hardware, but it means the DroneCAN software stack (CANIface,
  AP_DroneCAN, Canard integration) can be regression-tested in CI.

## DSP, gyro FFT and the dynamic harmonic notch

### What ChibiOS's DSP class does

`AP_HAL_ChibiOS/DSP.cpp` and `.h` is a hand-optimised real-time FFT engine on
ARM CMSIS-DSP: Hanning-windowed gyro samples through a complex radix-8 FFT,
bit-reversal, complex to real conversion, magnitude-squared, and peak-bin
detection with Candan's-estimator sub-bin interpolation for a precise
vibration-frequency estimate, profiled and tuned per MCU. `AP_GyroFFT` uses it
to track the dominant motor and prop noise frequency continuously and feed the
dynamic harmonic notch, which re-tunes its centre frequency in real time as RPM
changes with throttle.

### What was on Zephyr, and what is there now

There was no `DSP.cpp` anywhere in `AP_HAL_Zephyr`. Unlike almost every other
feature-disable in `board/zephyr.h`, this one had no `#ifndef` guard:
`HAL_WITH_DSP 0` and `HAL_GYROFFT_ENABLED 0` were hard overrides applying to
every Zephyr board, unturnable-on by any board's hwdef without editing
`zephyr.h`. rt1176's own flash budget (`HAL_PROGRAM_SIZE_LIMIT_KB 2048`) would
automatically qualify it under ArduPilot's generic default
(`HAL_GYROFFT_ENABLED = flash > 1024KB`).

Both halves of that are closed. `AP_HAL_Zephyr/DSP.cpp` and `.h` exist, both
symbols in `board/zephyr.h` are `#ifndef` defaults a hwdef can override, and
`hwdef/mr_vmu_rt1176/hwdef.dat` overrides both to 1. No other Zephyr board opts
in, so the per-board notes below still say where each of the others stands.

It degraded gracefully rather than silently: `AP_GyroFFT` compiles out entirely
behind `#if HAL_GYROFFT_ENABLED`, so there was no dead code and no
inert-but-present parameter surface. The rest of the harmonic-notch framework
is unaffected, since `AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED` depends on
`AP_INERTIALSENSOR_ENABLED`, not on `HAL_WITH_DSP`. Only the FFT-tracked mode
was gone.

### Board scope: the CMSIS-DSP path is ARM-only

* **`mr_vmu_rt1176`** and **`CubeOrangeZephyr`** are both Cortex-M7 with a
  hardware FPU. `CubeOrangeZephyr`'s `boards.py` class sets the identical
  `-mfpu=fpv5-d16 -mfloat-abi=hard` flags plus `CONFIG_FPU=y` and
  `CONFIG_FPU_SHARING=y`. CMSIS-DSP is ARM-only by construction, ARM's own
  library hand-tuned per Cortex-M core, so the porting path applies to both
  identically. The rt1176 work should be a near-copy for `CubeOrangeZephyr`,
  modulo per-board Kconfig wiring.
* **`ESP32S3Zephyr`**: CMSIS-DSP cannot be used at all. Xtensa LX7 is not an
  ARM core, and CMSIS-DSP's intrinsics and build assume Cortex-M throughout.
  Espressif ships **ESP-DSP** with a different API surface and its own FFT
  tuned for Xtensa, so this would be a rewrite rather than a reuse. Moot today
  regardless: no IMU on the board (`HAL_INS_DEFAULT HAL_INS_NONE`) to feed an
  FFT.
* **`native_sim`**: x86-64, same CMSIS-DSP inapplicability, and no real IMU
  data. A portable reference FFT could give CI coverage of the surrounding
  `AP_GyroFFT` logic, which is a different goal from vibration tracking.

### Why it matters

The dynamic harmonic notch is ArduPilot's mainstream tool for suppressing
motor and propeller vibration noise from the control loop as it changes with
throttle and RPM. Without it, either the noise leaks into the rate controller
(motor and ESC heating, audible growl or oscillation at certain throttle
settings) or the tuner falls back to more conservative low-pass filtering,
trading away control-loop responsiveness. rt1176 is configured as a 4-motor
quad (`FRAME_CLASS=1`), where losing dynamic tracking costs more per motor than
on a design with more motors to average across. RPM-based tracking via ESC
telemetry is a usable substitute if bidirectional DShot lands, so this is a
quality-of-tuning gap rather than a safety one, with no path at all for ESCs
that do not expose RPM telemetry.

## Two silent bugs found while reading Storage and AnalogIn

These were not capability gaps. The code ran, appeared to succeed, and returned
wrong or meaninglessly optimistic results with no diagnostic trail. Both were
found by reading `Storage.cpp` and `AnalogIn.cpp` line by line while auditing
for ChibiOS parity, not because either was suspected.

### `Storage::healthy()` returned true with zero working backend

`_healthy = true;` was set unconditionally regardless of the `persistent` flag,
in `Storage::init()`'s shared sequencing logic downstream of all four backend
probes (ROM-API flash, ZMS, FRAM, SD/FAT). Not board-conditional code, so every
board was affected, and each reaches that line with a different set of backends
available:

* **`mr_vmu_rt1176`**: `CONFIG_AP_RT1176_ROMAPI_FLASH`. Latent here
  specifically because the one enabled backend works.
* **`ESP32S3Zephyr`**: `CONFIG_ZMS=y`, `CONFIG_FLASH=y` and
  `CONFIG_FLASH_MAP=y` all genuinely enabled in
  `zephyr/boards/esp32s3_zephyr.conf`. Internal-flash ZMS, architecturally the
  simplest of the four backends since
  the part has real internal flash with no XIP-trap constraint. Unverified on
  hardware.
* **`CubeOrangeZephyr`**: was an open question, and the answer turned out to be
  FRAM, with SD/FAT behind it, rather than nothing. `cube_orange_zephyr.dts`
  declares the `fram0` node `Storage.cpp`'s
  `DT_NODE_HAS_STATUS(DT_NODELABEL(fram0), okay)` check is looking for: an
  `infineon,fm25xxx` child of `&spi2` on CS0, the same part, bus and
  chip-select as ChibiOS's `SPIDEV ramtron SPI2` line. `CONFIG_ZMS` reaches
  this board from the shared `zephyr/prj.conf` but has nothing to mount on:
  the board declares no `storage_partition`, so that backend compiles down to
  its "not found in DTS" branch. SD/FAT is enabled too (`CONFIG_DISK_ACCESS=y`,
  `CONFIG_SDMMC_STM32=y`, `CONFIG_FAT_FILESYSTEM_ELM=y` in
  `zephyr/prj.CubeOrangeZephyr.conf`), but it is the fallback, not the store:
  `Storage::init()` reaches `_try_file_mount()` only when neither ZMS nor FRAM
  answered, so on a board whose FRAM comes up the parameter path never touches
  the card.
* **`native_sim`**: `CONFIG_FAT_FILESYSTEM_ELM=y` and `CONFIG_DISK_ACCESS=y` in
  `zephyr/boards/native_sim_native_64.conf`, over a RAM disk
  (`CONFIG_DISK_DRIVER_RAM=y`), so SD/FAT is the path there.

Fix: `_healthy = persistent;`, at the end of `Storage::init()` where the four
backend probes have each set their own flag.

### `AnalogIn` returned a hardcoded 0.0 V on every pin

`AnalogIn::init()` required both LPADC1 and LPADC2 ready:

```cpp
_adc_ready = (_adc1 != nullptr && _adc2 != nullptr &&
              device_is_ready(_adc1) && device_is_ready(_adc2));
```

The board DTS disables `lpadc2` (`status = "disabled";`, its pinout separately
flagged in the DTS as not schematic-confirmed). Because that status is not
`"okay"`, the `#if DT_NODE_HAS_STATUS(...)` guard around `_adc2`'s assignment
compiles out entirely, leaving `_adc2` at its default-member-initialiser
`nullptr`. The `&&` therefore made `_adc_ready` unconditionally false,
regardless of LPADC1, which *is* enabled and schematic-confirmed with 4 working
rail-monitor channels.

`read_pin_raw()` starts with `if (!_adc_ready || ...) { return 0.0f; }`, so
every analog pin read returned 0.0 V. `AP_HAL::AnalogIn` has no `healthy()`
concept in either HAL, so nothing surfaced it: no printk, no GCS message, no
prearm check. On rt1176 the primary battery monitor comes from an I2C SMBus
smart battery, independent of `AnalogIn`, so the visible cost was analog-pin
RSSI, the POWER2/LPADC2 connector, and a constant plausible-looking 0.0 V for
any user-configured analog sensor (airspeed, rangefinder, spare `BRD_ADC` pin).

Fix on rt1176, in `AnalogIn::init()`:
`_adc_ready = (_adc1 != nullptr && device_is_ready(_adc1));`

That fix does nothing for `CubeOrangeZephyr`, and the underlying cause is
bigger than the dual-LPADC check. `AnalogIn::init()` only ever looks for two
hardcoded NXP devicetree labels, `DT_NODELABEL(lpadc1)` and
`DT_NODELABEL(lpadc2)`. `cube_orange_zephyr.dts` has no `adc`-labelled node at
all, despite `CONFIG_ADC=y` in that board's defconfig. Zephyr's
`DT_NODE_HAS_STATUS(DT_NODELABEL(x), okay)` is safe on a label that does not
exist, resolving to 0 rather than erroring, so `AnalogIn::init()` compiles
cleanly there, both guards compile out, `_adc1` and `_adc2` stay `nullptr`, and
`_adc_ready` is false regardless. That board needs a separate code path
recognising STM32's ADC devicetree node label, most likely `adc1` per Zephyr's
STM32 ADC binding convention, and the DTS does not declare that node yet: a
two-part gap.

`ESP32S3Zephyr` sets `AP_AIN_ENABLED 0` in its hwdef ("Disable analog inputs to
reduce ADC driver BSS"), so this is moot there by board design.
`native_sim` uses `CONFIG_ADC_EMUL=y`, also not `lpadc`-labelled, so the same
NXP-label limitation would apply if `AP_AIN_ENABLED` were ever turned on.

## Lua scripting

`AP_SCRIPTING_ENABLED` was hard-disabled at the board level
(`libraries/AP_HAL/board/zephyr.h`) with no comment explaining why,
unlike the deliberate feature-scoping disables nearby (mount, ADSB, camera)
which read as clear smallest-component-set choices. Nothing about rt1176's
resources looked like a hard blocker: 1 MB RAM, a working filesystem, and
abundant external flash.

The disable was also set a second time, identically, in the `configure_env()`
of all four Zephyr board classes in `Tools/ardupilotwaf/boards.py` —
`mr_vmu_rt1176`, `native_sim`, `CubeOrangeZephyr` and `ESP32S3Zephyr`. The same
line, copy-pasted, none commented. That uniformity across boards with very
different resource budgets reads as boilerplate inherited when each later board
class was created from an earlier one, not four independent decisions.

One partial exception: `ESP32S3Zephyr/hwdef.dat` sets
`define AP_SCRIPTING_ENABLED 0` a third time at the hwdef layer, alongside a
cluster of other disables (`AP_LOGGER_ENABLED 0`, `AP_NOTIFY_ENABLED 0`,
`AP_AIRSPEED_ENABLED 0`) under one stated theme, "reduce BSS" and "DRAM is very
tight on ESP32-S3". That board has an explicit board-specific rationale on
record, redundant with the `boards.py` override above it. `mr_vmu_rt1176` and
`CubeOrangeZephyr` had none anywhere.

There was no rationale to preserve. The `boards.py` lines are removed;
`board/zephyr.h`'s `#ifndef` default keeps every board off unless its hwdef
opts in, and `mr_vmu_rt1176` now does: scripts on SD, Lua heap via MultiHeap's
generic malloc backend from the OCRAM arena, runtime-gated by `SCR_ENABLE`.

## `AP_HAL::Util` per-board notes

**`get_hw_rtc()` / `set_hw_rtc()`.** ChibiOS backs these with STM32
backup-domain RTC registers that survive power-off. Zephyr's `_rtc_usec` was a
plain in-RAM value, zeroed every boot and only restored by GPS time sync.
Rarely bites in normal flight since GPS syncs it quickly, but log timestamps
before GPS lock, and anything relying on wall-clock continuity across a reboot,
show boot-relative or 1970 time.

The gap was HAL-wide; the fix differs materially per board.
**`CubeOrangeZephyr` has the exact STM32 RTC backup-domain peripheral ChibiOS
itself uses**, a closer drop-in target than rt1176, whose NXP SNVS/RTC block is
a different peripheral family. rt1176 is done via a direct-register backend
(`zephyr/src/rt1176_snvs_rtc.c`, since Zephyr's `counter_mcux_snvs` driver
exposes no absolute-set API), surviving warm and watchdog resets and, with VBAT
fitted, power-off. `ESP32S3Zephyr` has its own RTC timer domain
(`RTC_SLOW_MEM` and the RTC counter) that survives some reset types, notably
deep-sleep which this port does not use, but not others. Xtensa reset semantics
differ enough from Cortex-M that "does it survive *this* board's actual reset
paths" needs its own investigation.

**`get_system_id()` / `get_system_id_unformatted()`.** ChibiOS reads the STM32
UDID. With no override, MAVLink's `AUTOPILOT_VERSION.uid2` is always zero, the
GCS connect-time board/serial STATUSTEXT is skipped, and `AP_Logger`'s per-log
board-serial line is skipped. Diagnostic and fleet-management capability, not
flight-critical, but relevant to incident forensics.

This is the most uniform fix in the whole audit. All three real-silicon boards
expose a factory unique ID through the same portable Zephyr API,
`hwinfo_get_device_id()`, backed by rt1176's OCOTP register, STM32H743's 96-bit
UDID, or ESP32-S3's factory-programmed MAC/eFuse ID depending on the build. One
`Util::get_system_id()` override calling that API works unmodified across all
three. `native_sim` has no factory ID and needs a synthetic stand-in or
continues returning zero.

**`persistent_data.spi_count` and `i2c_count`, the `PM.SPIC` and `PM.I2CC` log
fields.** ChibiOS increments these in `AP_HAL_ChibiOS/SPIDevice.cpp` and
`I2CDevice.cpp`. This port did not, so `PM.SPIC` and `PM.I2CC` read 0 on every
Zephyr row and the bus-activity column dropped out of every cross-HAL `PM`
comparison. Both are counted now, in the same places ChibiOS counts them:
`AP_HAL_Zephyr/SPIDevice.cpp` increments in both `transfer()` forms immediately
before `spi_transceive()`, and `AP_HAL_Zephyr/I2CDevice.cpp` increments once
per attempt, matching ChibiOS's per-`i2cStart` count. That is HAL-wide code, so
every board gets it.

Read the two fields as totals, not as rates. Nothing in the tree ever resets
`spi_count` or `i2c_count`, and `AP_Scheduler::Log_Write_Performance()` copies
the raw values straight into the `PM` packet, so each row carries the count
since boot. Dividing one row by the logging interval produces a number that
means nothing, and it is an easy mistake to make twice.

One measurement here is not explained. On the emulated CubeOrangeZephyr
comparison against CubeOrange on ChibiOS, mid-flight `PM` rows showed 18
cumulative I2C transfers on the Zephyr side against 19,234 on the ChibiOS side,
same emulated board, same mission. The declared device list does not account
for a gap that size: both hwdefs declare the same internal compass as
`COMPASS AK09916:probe_ICM20948`, reached through an SPI-attached ICM20948, not
over I2C. Either the Zephyr side is barely touching the I2C bus or the count is
landing somewhere it should not. Not chased down.

**`safety_switch_state()`, confirmed not a gap on rt1176.**
`HAL_HAVE_SAFETY_SWITCH 0`, and the board DTS documents removing guessed,
non-schematic safety-switch and LED GPIO nodes after they coincided with a
lost-SWD-access incident during bring-up. `SAFETY_NONE` is the correct answer
for this hardware. Revisit if a real safety-switch pin is schematic-confirmed.

**`get_true_random_vals()`, `set_imu_temp()`, `trap()`, and the DFU and
persistent-param methods.** Each either has zero callers repo-wide, is not
overridden by *either* HAL, or is tied to a boot and flash model
ChibiOS-specific enough that there is no natural 1:1 port target. Noted for
completeness, not actionable.
