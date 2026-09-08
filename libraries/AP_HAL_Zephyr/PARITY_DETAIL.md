# Parity detail: what ChibiOS does, and what we did instead

Design rationale for the parity work. What `AP_HAL_ChibiOS` does internally,
what this port does in its place, why, and how the answer differs per board.

[COMPARED_TO_CHIBIOS.md](COMPARED_TO_CHIBIOS.md) has current state, one row per
feature. This file does not restate it.

## Crash dump

### What ChibiOS does

Three layers, all present and wired together.

1. **`modules/CrashDebug/CrashCatcher`**, a vendored third-party submodule, is
   the capture engine. It hooks the Cortex-M hard-fault entry point, walks a
   list of memory regions (thread stacks discovered by walking ChibiOS's own
   `chRegFirstThread()` / `chRegNextThread()` list, BSS, heap, and a bounded
   window around the faulting stack pointer), and feeds the bytes through four
   callbacks the target implements: `CrashCatcher_GetMemoryRegions()`,
   `CrashCatcher_DumpStart()`, `CrashCatcher_DumpMemory()`,
   `CrashCatcher_DumpEnd()`.

2. **`AP_HAL_ChibiOS/hwdef/common/crashdump.c`** implements those callbacks. It
   writes the dump into a dedicated internal MCU flash region, sized as
   whatever is left at the end of the `flash` linker region after the firmware
   image (`hwdef/common/common.ld:197-204`, `.crash_log (NOLOAD)`,
   `__crash_log_base__` / `__crash_log_end__`). Writes go through
   `stm32_flash_write()` in 32-byte chunks with `stm32_watchdog_pat()` between
   chunks, so the dump cannot trigger a watchdog reset mid-write. A secondary
   path (`HAL_CRASH_SERIAL_PORT`) hex-dumps the same data over a raw UART for
   boards without spare flash, gated interactively: it waits for the string
   `dump_crash_log` typed at the serial port, so it never fires unless someone
   is at the bench. `CrashCatcher_DumpStart()` also calls
   `save_fault_watchdog()` (`system.cpp:257`), the same persistent-data
   mechanism the monitor thread uses, so one fault populates both the flash
   dump and the RTC-backup forensic summary.

3. **`AP_Filesystem_Sys.cpp`**, shared HAL-generic code, exposes the
   flash-resident dump as `@SYS/crash_dump.bin`, backed by
   `hal.util->last_crash_dump_ptr()` / `last_crash_dump_size()`
   (`AP_Filesystem_Sys.cpp:144-147, 290-292`). Retrievable over MAVFTP on the
   next boot after a crash, no debugger required.

Gating: `crashdump_enabled` defaults to `flash_size >= 2048` KB and not a
bootloader build (`chibios_hwdef.py:1071`).

### Board scope: crash dump

The absence was HAL-wide: no board overrode `AP_CRASHDUMP_ENABLED`, no board
had a `Util::last_crash_dump_*()` override. The fix is not one implementation
for all boards.

- **`mr_vmu_rt1176`**: external XIP NOR, no internal flash, needs the custom
  ROM-API backend. Firmware runs XIP from 64 MB of external NOR over FlexSPI
  (`reg = <0x30000000 DT_SIZE_M(64)>`), and param storage occupies 128 KB at
  `0x620000`, so region budget is not a constraint. The open question was
  mechanism: CrashCatcher writes synchronously from a fault handler, with no
  interrupts and no scheduler, which is not what the ROM-API flash write path
  assumes.
- **`CubeOrangeZephyr`**: STM32H743 with 2 MB of real internal flash, the exact
  model ChibiOS's `crashdump.c` was written against. Architecturally the
  closest of the set to a drop-in port: a dedicated `.crash_log` flash region
  and `stm32_flash_write()`-style chunked writes both apply more directly here
  than on rt1176. `CONFIG_FLASH` / `CONFIG_FLASH_MAP` are available in
  principle, since nothing in its Kconfig fragments disables them, unlike
  rt1176's explicit `=n`, so Zephyr's standard
  `coredump_backend_flash_partition.c` could plausibly be used as-is without
  rt1176's custom-backend detour. Not tried.
- **`ESP32S3Zephyr`**: CrashCatcher cannot be ported here at all.
  `modules/CrashDebug/CrashCatcher/Core/src/CrashCatcher_armv7m.S` is
  hand-written Cortex-M assembly that walks the ARMv7-M exception frame format
  and register layout. Xtensa LX7 is a different instruction set, register file
  and exception-frame layout. Any crash dump here goes through Zephyr's
  `coredump` subsystem exclusively. Storage is the easiest of the real-silicon
  targets: `CONFIG_FLASH=y`, `CONFIG_FLASH_MAP=y` and `CONFIG_ZMS=y` are all
  genuinely enabled in `boards/esp32s3_zephyr.conf`, so rt1176's
  `CONFIG_FLASH_MAP=n` blocker does not exist. The practical blocker is that
  this board's hwdef disables `AP_LOGGER_ENABLED`, so there is no `LOG_WDOG`
  flight-log line to report a dump exists. `@SYS/crash_dump.bin` retrieval over
  MAVFTP would still work independently of the logger.
- **`native_sim`**: a crash is a host SIGSEGV, not a Cortex-M or Xtensa hard
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

Also missing against ChibiOS: the escalating monitor-thread diagnostics (200 ms
log, 500 ms self-healing mutex release, 1800 ms self-triggered informative hard
fault about 250 ms before the hardware watchdog fires), and
`AP_BoardConfig::watchdog_enabled()`, the `BRD_OPTIONS` parameter bit. That
check is HAL-generic and already available; this port arms the watchdog
unconditionally from compile-time Kconfig instead of checking it at runtime.

### Board scope: persistent crash and watchdog forensics across reset

The watchdog pat itself (the main loop calling
`schedulerInstance.watchdog_pat()` in `HAL_Zephyr_Class.cpp`) is shared code
and applies to every board that arms a watchdog device. Checked per board
against each board's own Kconfig:

- **`CubeOrangeZephyr`**: STM32 **IWDG**, a different physical peripheral from
  rt1176's `WDOG1`, wired through Zephyr's generic `watchdog` API the same way.
  `boards/cube_orange_zephyr.conf` sets `CONFIG_WATCHDOG=y` and
  `CONFIG_WDT_DISABLE_AT_BOOT=n` with a comment citing the monitor thread, so
  this board's watchdog Kconfig was set up aware of the shared pat mechanism.
  Correct by inspection, unverified on hardware.
- **`ESP32S3Zephyr`**: has the bug rt1176 had. Its DTS enables the node
  (`&wdt0 { status = "okay"; }`), but no `CONFIG_WATCHDOG=y` appears anywhere
  in `prj.ESP32S3Zephyr.conf` or `boards/esp32s3_zephyr.conf`.
  `device_is_ready(wdt)` is silently false without the Kconfig symbol, so this
  is very likely a dead DTS node. Not confirmed on hardware. The fix is one
  line in the board conf.
- **`native_sim`**: no watchdog concept for a host process.

## RC output

### The init gate

`HAL_Zephyr_Class.cpp` called `rcin->init()` but deliberately not
`rcout->init()`. `RCOutput::_map_ready` defaults false and is only set true
inside `init()`, and `_apply_channel()` bails immediately when `!_map_ready`,
so every `write()`, `set_freq()`, `enable_ch()` and `push()` call was pure
bookkeeping with no `pwm_set()` reaching the FlexPWM peripheral. That is what
made it safe to enable the fast rate thread before motors were verified.

The gate lived in `HAL_Zephyr_Class.cpp`, compiled identically for every board,
so it correctly gated output everywhere rather than on one board.
`ESP32S3Zephyr` has an independent reason RCOutput cannot run there:
`CONFIG_PWM=n` in `boards/esp32s3_zephyr.conf` ("No eFlexPWM on ESP32-S3", true
generically since eFlexPWM is an NXP peripheral). A real ESP32-S3 PWM driver
would need Zephyr's LEDC-based PWM binding. `AP_RCIN_ENABLED 0` and no PWM pins
in its hwdef mean the whole RC and motor path is off by board design there.

### DShot came from an existing in-tree driver

The driver (659 lines, CogniPilot, Copyright NXP) is a complete FlexIO-based
DShot and bidirectional-DShot implementation for this SoC's FlexIO peripheral,
including rt1176 clock setup and GCR/RLL telemetry decode. Functionally a
from-scratch equivalent of ChibiOS's bdshot machinery, built on FlexIO instead
of DMA plus timer capture. It was instantiated only on Cerebri's own board
overlays.

Wiring it up (`0dad5bcf2d`) vendored it in-tree as
`zephyr/src/nxp_flexio_dshot.c` plus header and binding, byte-identical to the
original plus provenance notes, instantiated it in the board DTS on FMU_CH1-8
(whose pads matched cerebri's `vmu_rt1170` overlay pin for pin, same VMU
hardware family), and drove it from `set_output_mode()` in `RCOutput.cpp`. The
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

- **`CubeOrangeZephyr` (STM32H743)** uses ST's DMA controller plus **DMAMUX**,
  a separate request-routing crossbar that is not part of the DMA controller,
  with a per-stream priority field (Low/Medium/High/Very High) set per DMA
  request in the stream's own config register. There is no "fixed group of 16
  channels always beats the other 16" concept. The closest STM32 analogue to a
  starvation bug is two streams at the same priority level racing on hardware
  round-robin, a different failure mode needing its own investigation. This
  board is not at risk of the eDMA bug, which says nothing about whether its
  own DMA priority assignments have ever been reasoned about.
- **`ESP32S3Zephyr`** uses Espressif's **GDMA**: per-channel priority
  registers, no group concept. Moot: its DTS and Kconfig wire DMA for none of
  the peripherals below.
- **`native_sim`**: no DMA hardware.

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
| LPUART3    | GPS1, moved here by `d7f0c285a0` | 20, 21     | Group 1     |
| LPI2C1/2/3 | I2C buses (mag/baro/external)    | 22, 23, 24 | Group 1     |

### Fixed-priority group arbitration, and the ERGA fix

The eDMA `CR` register's group-priority bits sit at silicon reset default,
`GRP1PRI=1 > GRP0PRI=0`, because nothing in this tree or in Zephyr's
`dma_mcux_edma.c` ever writes `CR`. The driver's only two references to `->CR`
are debug reads. With fixed-priority arbitration active, Group 1 channels
unconditionally win against Group 0 channels regardless of any individual
channel's own priority field. That is a global controller property, not
something scoped to whichever peripheral is under investigation.

GPS1's fix (`d7f0c285a0`) relocated one peripheral out of Group 0. Read against
the table, seven peripherals remained disadvantaged:

- **All three IMU SPI buses (LPSPI1/2/3)**, the highest-rate,
  most latency-sensitive DMA consumers on the board. The DTS bring-up notes
  record a separate DMA RX-overrun bug on these same buses (mid-frame channel
  re-arm racing a level-sensed eDMA request), fixed independently.
  Group-priority disadvantage was an additional structural risk on top.
- **LPUART6 (RC-IN)**, safety-critical control input, the same class of
  peripheral as GPS1 (async RX with idle-line DMA completion) and the same
  silicon behaviour.
- **LPUART5 (GPS2)**, same driver, same traffic pattern, very likely the same
  u-blox part as GPS1 pre-fix. Structurally identical to a bug already proven
  live, just never exercised.
- LPUART4 (TELEM1) and LPUART1 (console), lower real-time criticality, same
  structural disadvantage.

Meanwhile TELEM2, TELEM3 and all three I2C buses, comparatively low-rate
traffic, held the advantaged Group 1 slot purely as an accident of
channel-number sequencing.

`zephyr/src/rt1176_edma_arbitration_fixup.c` sets `CR.ERGA` (Enable Round Robin
Group Arbitration, bit 3) at `POST_KERNEL` init, so all 32 channels arbitrate
fairly regardless of group. Register-verified on hardware:
`g_edma0_cr_before=0x490` to `after=0x498`, exactly bit 3, with every
peripheral including both live CAN buses running under it.

One precision point. `ERCA` is Enable Round Robin **Channel** Arbitration and
only affects arbitration *within* a group. `ERGA` is the bit that governs
Group 0 against Group 1. `GRP0PRI`, `GRP1PRI` and `ERCA` are all left at reset
defaults; this is a single-bit read-modify-write. GPS1's channel relocation is
left in place, harmless and no longer load-bearing.

Still open: a hardware test showing the seven previously-disadvantaged
peripherals arbitrate fairly under real simultaneous DMA load, and that nothing
regressed for the six that used to hold Group 1 unconditionally.

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

- **`CubeOrangeZephyr` (STM32H743)**: 2 MB of internal flash is the program
  store, same as every ChibiOS STM32 board, and no external wide or octal SPI
  NOR is wired in its DTS or hwdef. N/A because there is no external flash chip
  to wrap, not because something else owns the role.
- **`ESP32S3Zephyr`**: the attached SPI NOR holding the Zephyr image is managed
  by ESP-IDF's own flash and MMU subsystem below Zephyr, outside any
  `WSPIDevice`-shaped abstraction.
- **`native_sim`**: no flash hardware.

## Software PPM/RC signal capture (`SoftSigReader`)

ChibiOS's `SoftSigReader` / `SoftSigReaderInt` decode PPM-SUM via hardware
timer input-capture plus DMA, using the same `Shared_DMA` broker as above.

`AP_HAL_Zephyr/RCInput.cpp` implements PPM-SUM by a simpler mechanism: plain
GPIO edge interrupt plus `AP_HAL::micros()` timestamping, gated on a board
declaring an `rcin-gpios` devicetree property. That path is generic HAL-wide
code, and `CubeOrangeZephyr` is its existing proof, with
`rcin-gpios = <&gpiod 14 GPIO_ACTIVE_HIGH>;` in its DTS.

`mr_vmu_rt1176` declares no such property. Zero hits for `rcin-gpios` or
`zephyr,user` in its DTS, because the RC-IN connector is wired to a single net
(`UART6_TX_TO_IO__RC_INPUT`, RX pad not connected) already occupied by the
UART-based SBUS/CRSF path. There is no second pin to dedicate to GPIO-edge
capture without a same-pin dynamic UART/GPIO mux.

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

Bench evidence (`6e87b1599c`, tool `4d2b7d0b2d`): Matek F405 GPS on CAN1 as
node 113, mRo M10025 on CAN2 as node 125, both discovered by name, unique ID
and software version through `Tools/scripts/zephyr_can_nodes.py`; 423k and 73k
frames received respectively, zero overflows, zero bus errors, `error-active`,
TEC and REC at 0. The autopilot is on the network as node 10 carrying its
OCOTP unique ID. The SWD-lockup concern did not reproduce. PiccoloCAN and
FETtec are out of scope; only AP_DroneCAN matters here. A
`Duplicate Node .../125` prearm was seen and closed as not repeatable on the
bench, with no code change: a decision, not a root-caused fix.

Adding real CAN-FD would mirror ChibiOS's 1106-line `CANFDIface.cpp` against
the existing 308-line classic implementation.

### Board scope: CAN / CAN-FD / DroneCAN

Each board lands somewhere different, and none shares rt1176's story.

- **`CubeOrangeZephyr` (STM32H743)** has the most capable CAN silicon of the
  set: real hardware FDCAN, genuine CAN-FD rather than software-emulated. Its
  DTS nodes are `status = "okay"` and its base defconfig sets
  `CONFIG_CAN_STM32H7_FDCAN=y`, but `prj.CubeOrangeZephyr.conf` overrides that
  back off with `CONFIG_CAN=n` and a documented hazard: *"STM32H7 FDCAN needs
  PLL2Q as kernel clock (FDCANSEL); without PLL2 configured the driver may
  hardfault during message-RAM init."* That is a clock-tree configuration bug,
  not a precautionary sweep. Re-enabling CAN here needs PLL2 wired up first.
- **`ESP32S3Zephyr`**: `CONFIG_CAN=n` in its board conf and `HAL_CAN_ENABLED 0`
  in its hwdef.dat, under the same "reduce BSS" theme as its other disables.
  The chip does have CAN-capable silicon, Espressif's **TWAI** peripheral,
  classic CAN 2.0 only with no hardware CAN-FD on this part, but nothing in the
  board's DTS instantiates or wires it. A from-scratch bring-up, not a
  re-enable.
- **`native_sim`** is the one board where CAN works without hardware. Its
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

### What was on Zephyr

No `DSP.cpp` anywhere in `AP_HAL_Zephyr`. Unlike almost every other
feature-disable in `board/zephyr.h`, this one had no `#ifndef` guard:
`HAL_WITH_DSP 0` and `HAL_GYROFFT_ENABLED 0` were hard overrides applying to
every Zephyr board, unturnable-on by any board's hwdef without editing
`zephyr.h`. rt1176's own flash budget (`HAL_PROGRAM_SIZE_LIMIT_KB 2048`) would
automatically qualify it under ArduPilot's generic default
(`HAL_GYROFFT_ENABLED = flash > 1024KB`).

It degraded gracefully rather than silently: `AP_GyroFFT` compiles out entirely
behind `#if HAL_GYROFFT_ENABLED`, so there was no dead code and no
inert-but-present parameter surface. The rest of the harmonic-notch framework
is unaffected, since `AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED` depends on
`AP_INERTIALSENSOR_ENABLED`, not on `HAL_WITH_DSP`. Only the FFT-tracked mode
was gone.

### Board scope: the CMSIS-DSP path is ARM-only

- **`mr_vmu_rt1176`** and **`CubeOrangeZephyr`** are both Cortex-M7 with a
  hardware FPU. `CubeOrangeZephyr`'s `boards.py` class sets the identical
  `-mfpu=fpv5-d16 -mfloat-abi=hard` flags plus `CONFIG_FPU=y` and
  `CONFIG_FPU_SHARING=y`. CMSIS-DSP is ARM-only by construction, ARM's own
  library hand-tuned per Cortex-M core, so the porting path applies to both
  identically. The rt1176 work should be a near-copy for `CubeOrangeZephyr`,
  modulo per-board Kconfig wiring.
- **`ESP32S3Zephyr`**: CMSIS-DSP cannot be used at all. Xtensa LX7 is not an
  ARM core, and CMSIS-DSP's intrinsics and build assume Cortex-M throughout.
  Espressif ships **ESP-DSP** with a different API surface and its own FFT
  tuned for Xtensa, so this would be a rewrite rather than a reuse. Moot today
  regardless: no IMU on the board (`HAL_INS_DEFAULT HAL_INS_NONE`) to feed an
  FFT.
- **`native_sim`**: x86-64, same CMSIS-DSP inapplicability, and no real IMU
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

- **`mr_vmu_rt1176`**: `CONFIG_AP_RT1176_ROMAPI_FLASH`. Latent here
  specifically because the one enabled backend works.
- **`ESP32S3Zephyr`**: `CONFIG_ZMS=y`, `CONFIG_FLASH=y` and
  `CONFIG_FLASH_MAP=y` all genuinely enabled in `boards/esp32s3_zephyr.conf`.
  Internal-flash ZMS, architecturally the simplest of the four backends since
  the part has real internal flash with no XIP-trap constraint. Unverified on
  hardware.
- **`CubeOrangeZephyr`**: an open question. Its hwdef.inc sets
  `HAL_WITH_RAMTRON 1` and defines a `FRAM_CS`-equivalent pin, implying FRAM is
  the intended backend, but its board conf disables SD/FAT (SDMMC driver hang)
  and never sets `CONFIG_ZMS`, and nothing confirmed a `fram0` devicetree node
  with `status="okay"` exists in `cube_orange_zephyr.dts` to satisfy
  `Storage.cpp`'s `DT_NODE_HAS_STATUS(DT_NODELABEL(fram0), okay)` check. If
  that node is absent, this board may have zero working storage backends, which
  makes a health check that can never report failure considerably more
  consequential there. Worth a direct check.
- **`native_sim`**: `CONFIG_FAT_FILESYSTEM_ELM=y` and `CONFIG_DISK_ACCESS=y`,
  so SD/FAT is the likely path.

Fix: `_healthy = persistent;` (`Storage.cpp:254`).

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

Fix on rt1176 (`AnalogIn.cpp:100`):
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
(`libraries/AP_HAL/board/zephyr.h:295-297`) with no comment explaining why,
unlike the deliberate feature-scoping disables nearby (mount, ADSB, camera)
which read as clear smallest-component-set choices. Nothing about rt1176's
resources looked like a hard blocker: 1 MB RAM, a working filesystem, and
abundant external flash.

The disable was also set a second time, identically, in every board class's
`configure_env()` in `Tools/ardupilotwaf/boards.py`: `mr_vmu_rt1176`:1886,
`native_sim`:1948, `CubeOrangeZephyr`:2006, `ESP32S3Zephyr`:2055. The same
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
