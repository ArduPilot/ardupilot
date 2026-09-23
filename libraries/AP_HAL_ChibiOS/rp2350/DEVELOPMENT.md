# RP2350 port: developer notes

Notes for developers working on the ArduPilot port to the Raspberry Pi
RP2350. User-facing board documentation is in each board's `README.md`;
this file covers how the port works, how to build, flash, debug and
profile it, and what is still open.

## Boards

| Board | Package | Purpose |
|-------|---------|---------|
| `Pico2` | RP2350A (QFN-60) | Bare Pico 2 module running Simulation on Hardware; the bench target for scheduler, core1 and timing work |
| `Laurel` | RP2350B (QFN-80) | First RP2350B flight controller, used for the port bring-up |
| `RPI_UAVFC` | RP2350B (QFN-80) | Revision of the Laurel board with a different pinout; the board that flies |

All three run the same clock (225 MHz), flash timing and two-core SMP
configuration. Each board directory holds `hwdef.dat`, `hwdef-bl.dat`,
`defaults.parm`, a `README.md`, and, for Laurel and RPI_UAVFC, a
`HARDWARE.md` with the GPIO and connector reference.

## Where the code lives

- `libraries/AP_HAL_ChibiOS/rp2350/`: core1 start-up (`c1_main.c`), the
  thread core table (`rp2350_core_affinity.h`), the SRAM placement
  registries, `rp2350_memfunctions.S`, the watchdog, and the PIO programs
  with their generated headers in `pio/`.
- `libraries/AP_HAL_ChibiOS/`: the RP2350 drivers - `PIOUART.cpp`,
  `UARTDriver_rp2350.cpp`, `RCOutput_pico.cpp`, `RP2350_pio1.cpp` (the PIO1
  broker), `SoftSigReaderRP2350.cpp` - and the profiling aids
  (`rp2350_perf_report.cpp`, `rp2350_pc_sampler.cpp`, `xip_profiler.cpp`).
  RP2350 branches in shared files such as `Scheduler.cpp`, `SPIDevice.cpp`
  and `sdcard.cpp` are guarded by `RP2350`.
- `libraries/AP_HAL_ChibiOS/hwdef/common/`: `board_rp2350.c`,
  `chibios_board_rp2350.mk` and the SMP linker script
  `common_rp2350_smp.ld`.
- `libraries/AP_HAL_ChibiOS/hwdef/scripts/PICO2.py`: the MCU definition
  used by all three boards (`MCU PICO2 PICO2`).
- `libraries/AP_OSD/AP_OSD_pico.cpp`: the PIO analog OSD.
- `modules/ChibiOS`: the RP2350 ChibiOS port, from ArduPilot/ChibiOS#113.
- `Tools/scripts/rp2350_*.py`, `Tools/debug/`: build and profiling tools,
  described below.

## Building, flashing and debugging

All three boards share one build path: `env.RP_MCU`, set at configure from
`MCU PICO2` in the hwdef, selects the RP2350 linker scripts, the pre-link
SRAM placement step and the UF2 outputs. A new board needs no waf change.

### Host tools

- The ArduPilot ARM toolchain (gcc 10.2.1 or later), from
  `Tools/environment_install/install-prereqs-ubuntu.sh`.
- pioasm and picotool, needed only to regenerate PIO headers or to
  `--upload` a bootloader. The installer offers them (`DO_AP_RP2350_ENV=1`),
  or `Tools/scripts/rp2350_pioasm.py --install` puts the pinned 2.2.0
  release in `~/.local/opt/pico-sdk-tools` and prints the `PATH` line.
- OpenOCD with RP2350 support, from
  <https://github.com/raspberrypi/pico-sdk-tools/releases>. The 0.12.0
  release that distributions package predates the RP2350. On Linux it also
  needs `libhidapi-hidraw0`.
- `gdb-multiarch`. The toolchain's `arm-none-eabi-gdb` needs
  `libncurses.so.5`, which Ubuntu 24.04 does not ship.

### Building

```bash
./waf configure --board RPI_UAVFC      # or Laurel, Pico2
./waf copter
```

| Output in `build/<board>/bin/` | Contents |
|---|---|
| `arducopter` | ELF |
| `arducopter.bin` | app image, written at the app base |
| `arducopter.apj` | app, for `uploader.py` and ground stations |
| `arducopter.uf2` | app image at the app base, for the BOOTSEL drive |
| `arducopter_with_bl.hex` | bootloader, `0xff` padding and app, from `0x10000000` |

`hwdef.dat` and `hwdef-bl.dat` are read only at configure time. After an
edit, re-run `./waf configure`; otherwise the build succeeds with the old
`build/<board>/hwdef.h`. The SRAM placement registries are different: their
section lists are regenerated before every link.

A plain build is `-Os`, and its ELF has no line information for ArduPilot
sources (only the ChibiOS library is built with `-ggdb`). For GDB, configure
with `--debug-symbols`, which adds `-g` and nothing else. `--debug` builds at
`-Og` with `HAL_DEBUG_BUILD`, which changes code size, timing and what fits
in SRAM, so it is not the firmware that flies.

The PIO headers in `rp2350/pio/` are committed. After editing a `.pio`, run
`Tools/scripts/rp2350_pioasm.py`; `--check` reports stale headers, and no CI
job runs it. Details are in Peripherals.

### Bootloaders

```bash
./waf configure --board RPI_UAVFC --bootloader && ./waf bootloader
Tools/scripts/build_bootloaders.py RPI_UAVFC   # refreshes Tools/bootloaders/
```

`build_bootloaders.py` writes `<board>_bl.bin`, `.hex` and `.uf2`, which are
committed, plus an `.elf`, which is not. The app hwdefs set
`AP_BOOTLOADER_FLASHING_ENABLED 0`, so only BOOTSEL or SWD can change the
bootloader.

The bootloader runs on core0 only (`RP_CORE1_START FALSE`); the app starts
core1. On a cold power-up it waits `HAL_BOOTLOADER_TIMEOUT` for an uploader
before starting the app: 3 s on Laurel and RPI_UAVFC, 15 s on Pico2. After a
software reboot or a watchdog reset it starts the app at once. The app's
request to stay in the bootloader is a flag in WATCHDOG `SCRATCH[0]`, which
survives every reset except power-on.

### Flash layout

Each board has 4 MB of QSPI flash at `0x10000000`, written in 4 KB pages.
Parameter storage is two 16 KB sectors (`AP_FLASH_STORAGE_QUAD_PAGE`), 32 KB.
On RPI_UAVFC each region has its own 64 KB erase block; on Pico2 and Laurel
the bootloader and the parameters share block 0, so a block erase aimed at
the bootloader also takes the parameters.

| | Pico2, Laurel | RPI_UAVFC |
|---|---|---|
| Bootloader | `0x10000000`, 32 KB | `0x10000000`, 32 KB |
| Parameters | `0x10008000-0x1000FFFF` | `0x10010000-0x10017FFF` |
| App base | `0x10010000` | `0x10020000` |
| `FLASH_RESERVE_START_KB` / `STORAGE_FLASH_PAGE` | 64 / 8 | 128 / 16 |
| `APP_START_OFFSET_KB` in `hwdef-bl.dat` | 32 | 96 |
| `APJ_BOARD_ID` | 7181 (Pico2), 7182 (Laurel) | 7180 |

The app and the bootloader each compute the app base, and nothing checks that
they agree. Keep `FLASH_RESERVE_START_KB` and `STORAGE_FLASH_PAGE` in step
with `FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB` and `APP_START_ADDRESS`
in `hwdef-bl.dat`. Set `APP_START_ADDRESS` explicitly, because
`chibios_hwdef.py` derives `FLASH_LOAD_ADDRESS` from the STM32 base. When
storage sits above the bootloader's 32 KB, as on RPI_UAVFC, `hwdef-bl.dat`
also needs `FLASH_RESERVE_END_KB 0`. Otherwise the generator assumes storage
is at the top of flash and reserves everything above it, leaving no app area.

### Flashing

Use the ROM bootloader (BOOTSEL) for the first bootloader install and for
recovery. To enter it, hold the board's BOOT button (BOOTSEL on Pico2)
while plugging in USB; on Laurel, holding BOOT (SW1) and pressing RESET
(SW2) also works. A part with no valid image in flash also falls through
to the ROM. Then copy
`Tools/bootloaders/<board>_bl.uf2` to the drive that appears, or, with
picotool on `PATH`, run `./waf bootloader --upload` from a `--bootloader`
configure. That runs `picotool load -v -x <uf2> -f` on a board that is
already in BOOTSEL. `arducopter.uf2` loads the same way and writes only the
app. Any other firmware loaded through BOOTSEL (Betaflight, a Pico SDK
program) is written from `0x10000000` and replaces the ArduPilot bootloader.

Once the bootloader is in place, `./waf copter --upload` (or a GCS with the
`.apj`) reboots the firmware into the bootloader. It finds the board as
`/dev/serial/by-id/usb-ArduPilot_<board>-BL_<serial>-if00`; the app is the
same name without `-BL`. If the bootloader does not appear, `uploader.py`
waits indefinitely for a re-plug. For unattended work, or a board mounted in
a vehicle, use SWD.

For SWD, use any CMSIS-DAP probe. A Pico or Pico 2 running Raspberry Pi's
debugprobe firmware works
(<https://github.com/raspberrypi/debugprobe/releases>); it enumerates as
`2e8a:000c`.

| Probe GPIO (header pin) | Target |
|---|---|
| GP2 (4) / GP3 (5) | SWCLK / SWDIO |
| GND (3) | GND, mandatory |
| GP4 (6) probe TX / GP5 (7) probe RX | optional: a SERIALn RX / TX pair |

Each board has a 3-pin SWD header: SWCLK, GND, SWDIO. On Laurel this is J12,
with pin 1 (SWCLK) nearest the board edge. The probe powers only itself, so
`cannot read IDR` means the target has no power. The Pico SDK puts a console
on GPIO0/GPIO1, but these hwdefs do not. Those pins are PWM on Pico2, the IMU
chip-select on Laurel and LEDs on RPI_UAVFC. Wire the probe UART to a real
serial port, such as Pico2 SERIAL1 (GPIO12 TX, GPIO13 RX), or leave it off.

```bash
# app only, at the board's app base
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" \
  -c "program build/RPI_UAVFC/bin/arducopter.bin verify reset exit 0x10020000"
# bootloader and app
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" \
  -c "init; halt" \
  -c "program Tools/bootloaders/RPI_UAVFC_bl.bin 0x10000000 verify" \
  -c "program build/RPI_UAVFC/bin/arducopter.bin 0x10020000 verify" \
  -c "reset run" -c shutdown
```

A wrong app base does not boot: RPI_UAVFC's address on a Laurel puts the app
64 KB too high. `arducopter_with_bl.hex` writes both images, but it fills the
gap between them with `0xff`, and parameter storage is in that gap on all
three boards. Every parameter returns to its default, so flash the two
`.bin` files to keep a tune. A verify of the app says nothing about the
bootloader. Check each image with `dump_image` and `cmp`.

With OpenOCD already running, GDB can load the ELF:
`gdb-multiarch --nx --batch -ex "target extended-remote :50000"
-ex "mon reset halt" -ex load -ex "mon reset run" <elf>`.

### Recovering a board

- If a stored parameter breaks the board, erase only the parameter area. For
  example, `ESC_CALIBRATION` 2 or 3 makes RPI_UAVFC reboot repeatedly. Run
  `flash erase_address 0x10010000 0x10000` on RPI_UAVFC, or
  `0x10008000 0x8000` on Pico2 and Laurel, after `init; halt`. The
  bootloader and the app are untouched.
- If the app verifies but the board does not boot, the bootloader is
  probably gone, often because a BOOTSEL image replaced it. Write
  `<board>_bl.bin` at `0x10000000` over SWD, or load its UF2 through BOOTSEL.
- A reset taken with the debugger attached does not reliably start core1:
  the app comes up with core1 in the boot ROM, or every reset stops in the
  ROM. Shut OpenOCD down and power-cycle. If it recurs, drop `reset` from the
  `program` line and power-cycle after OpenOCD exits.

### Debugging over SWD

```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" \
  -c "gdb port 50000" -c "tcl port 50001" -c "telnet port 50002"
```

A good attach reports `SWD DPIDR 0x4c013477` and a Cortex-M33 on both
`rp2350.cm0` and `rp2350.cm1`. Older builds spell the port commands
`gdb_port` and so on. `rp2350.cfg` makes the two cores one SMP target with
`-rtos hwthread`. There is one GDB port, `info threads` lists the cores (not
ChibiOS threads), and `thread 2` is core1.

OpenOCD alone leaves both cores running. Connecting GDB halts them, and a
flight build does not reliably resume afterwards, so treat a GDB halt as
post-mortem. For live state, read memory through the telnet or TCL port.
This does not halt the board, and it is what the PC profiler uses.

```text
$ telnet localhost 50002
> mdw <addr> 4          # target keeps running
> halt                  # both cores, no reset
> targets               # state of rp2350.cm0 / rp2350.cm1
```

Most diagnostic state is in plain globals, such as `rp2350_xip_park_count`,
`spi_start_fail_count[]` and `fault_info`. Find addresses with
`arm-none-eabi-nm -C <elf>`, and find field offsets without a target using
`gdb-multiarch --nx -batch -ex "ptype /o SPIDriver" <elf>`. Two traps return
plausible numbers rather than errors. First, every address moves on a
rebuild, so before trusting a read, `dump_image` 256 bytes from the app base
and `cmp -n 256` them against the `.bin`. Second, anything reached through a
pointer, such as `hal.util->persistent_data`, is stale until static
initialisation has run.

For GDB, run `gdb-multiarch --nx <elf>`, then `target extended-remote :50000`
and `mon halt`. `--nx` stops any `.gdbinit` from running on connect.
`Tools/debug/gdb-openocd-rp2350.init` connects with `mon reset halt`, which
is right for debugging from reset but destroys the state you came to
inspect. For USB, `p SDU1.state` should be `SDU_READY` and
`p SDU1.config->usbp->state` should be `USB_ACTIVE`.

### Fault breadcrumbs

HardFault, BusFault, UsageFault and MemManage all go to `fault_capture()` in
`system.cpp`, which runs from SRAM. It saves IPSR, CFSR, HFSR, BFAR, MMFAR,
EXC_RETURN and the stacked frame in `fault_info`, sets `magic` to
`0xDEADFA17`, and spins until the watchdog fires. `fault_info` is in
`.noinit`, so you can read it after the reset (check the magic). With a
debugger attached, the handler executes `bkpt #0` first, so a fault halts the
core. That halt comes from the handler, not from OpenOCD.

A vector with no handler resets through `_unhandled_exception()`. Afterwards
WATCHDOG `SCRATCH[7]` reads `0x55484E44` ("UHND") and `SCRATCH[2]` holds the
exception number. The bootloader overwrites the other scratch registers: it
leaves its timeout in `SCRATCH[4]`, and in `SCRATCH[5]` a `0xB001xxxx` code
naming the branch it took (see `Tools/AP_Bootloader/AP_Bootloader.cpp`). To
read all eight registers, run `mdw 0x400d800c 8`.

### Console and MAVLink

SERIAL0 is USB CDC. It carries MAVLink and console text from
`hal.console->printf()` and `DEV_PRINTF()`. The two are the same, and both
print in release builds unless `HAL_CONSOLE_DISABLED` is set. A raw serial
reader shows the text between the binary frames. Open the port by its
`/dev/serial/by-id/` name, which survives a reboot where the `ttyACM` number
does not. Nothing appears before USB enumerates, so for an earlier crash use
the breadcrumbs above. `AP_HAL::panic()` repeats its message on the console.

To drive a UART from the host, use MAVLink SERIAL_CONTROL with device
`100 + n` for SERIALn and flags `EXCLUSIVE | RESPOND` (6). If bit 0 (`REPLY`)
is set, the packet is dropped silently and the port is never opened.

Debug and profiling switches and the `@SYS` files are in Performance.

### Build gotchas

- waf builds the ChibiOS library in one task that runs `make`. That task
  reruns only when `hwdef.h`, the linker scripts, `hwdef/common/*`,
  `rp2350/*.[chS]`, `modules/ChibiOS/os/hal/**` or the board makefile
  changes. After editing anything else under `modules/ChibiOS` (`os/rt`,
  `os/common/ports`, startup), delete `build/<board>/modules/ChibiOS`.
- `make`'s dependency files in `build/<board>/.dep` still name a source's old
  path after it moves. When `c1_main.c` and `rp2350_memfunctions.S` moved
  into `rp2350/`, the result was
  `No rule to make target '../hwdef/<board>/c1_main.c'`. Fix it with
  `rm -rf build/<board>/.dep build/<board>/modules/ChibiOS`.
- Each board's `chibios_board.mk` includes
  `hwdef/common/chibios_board_rp2350.mk`. A board without one silently falls
  back to the ARMv7-M makefile and fails with "unknown ARMv7-M variant".
- `waf configure` runs `git submodule update`, which can move
  `modules/ChibiOS` back to the recorded commit. When comparing submodule
  states, pass `--no-submodule-update` and check
  `git -C modules/ChibiOS log --oneline -1`.
- `modules/ChibiOS` is ArduPilot's stable_21.11.x fork at RT 7.0.6. Do not
  take upstream's RT 8 tree for its RP2350 support: that moves every board
  to a new kernel. The RT 7 shims sit in port-owned files (`PORT_WORKING_AREA`
  in the ARMv8-M-ML-ALT `chcore.h`, the `PORT_CORE0/1_BSS_SECTION` aliases,
  `mpu_v8m.h`); the shared files touched are `chinstances.c` and `hal_usb.h`.
  The aliases put `ch0`, `ch1` and the idle stacks in the port's
  `.ram4_clear.core0` and `.ram5_clear.core1` sections, which
  `common_rp2350_smp.ld` collects into `.bss`: the scratch banks have no
  room for them. `arm-none-eabi-nm -n <elf>` should show `ch0` and `ch1`
  between `__bss_base__` and `__bss_end__`.
- RP2350 code in a shared file must sit wholly inside `#if defined(RP2350)`,
  and only an STM32 build shows when it does not. Before pushing shared
  changes, build one STM32 board and one STM32 bootloader.

### Tools

| Tool | Purpose |
|---|---|
| `Tools/scripts/rp2350_pioasm.py` | regenerate or `--check` PIO headers; `--install` pioasm and picotool |
| `Tools/scripts/rp2350_ramfunc2_sizes.py` | annotate the RAMFUNC2 registry with symbol sizes and placement |
| `Tools/scripts/rp2350_heap_budget_report.py` | estimate heap headroom from `Linker.map` and allocation sites |
| `Tools/scripts/bin2uf2.py` | wrap a `.bin` as an RP2350 UF2; used by waf and `build_bootloaders.py` |
| `Tools/debug/rp2350_pc_profiler.py` | sample DWT PCSR over OpenOCD TCL; see Performance |
| `Tools/debug/gdb-openocd-rp2350.init` | GDB script: connect to port 50000, then reset and halt |

## Dual-core architecture

All three boards run ChibiOS RT in SMP mode (`CH_CFG_SMP_MODE TRUE`,
`RP_CORE1_START TRUE` in the hwdef). Core0 runs the vehicle main loop and
nearly everything else. Core1 runs the latency-critical chain from the IMU bus
through the rate controller to the motor outputs.

### ChibiOS SMP on the RP2350

- There are two OS instances, `ch0` and `ch1`. Each has its own ready list,
  idle thread, virtual timers and tick alarm: the kernel is tickless at 1 MHz
  on TIMER0, with ALARM0 for core0 and ALARM1 for core1.
- A thread belongs to the instance it was created on (`tp->owner`) for life.
  Threads never migrate, so placement is decided once, at creation.
- Waking a thread on the other core sends a reschedule message through the
  SIO FIFO, which raises `SIO_IRQ_FIFO` (IRQ25) there at minimum priority. A
  virtual timer callback runs on the core that armed it.
- The registry is global. `@SYS/threads.txt` lists the threads of both cores
  and tags each one `C0` or `C1`, which is the quickest way to check placement.

### Core1 start-up

1. Before `halInit()`, `rp2350_board_pre_hal_init()` in
   `hwdef/common/board_rp2350.c` copies the flash vector table to
   `rp2350_vectors` (base of Scratch X, SRAM8) for core0 and to
   `rp2350_c1_vectors` (base of Scratch Y, SRAM9) for core1. Vector fetches
   never go through XIP, and each core reads its own bank.
2. `start_core1()` in the ChibiOS RP2350 `hal_lld.c`, called from `halInit()`,
   power-cycles core1 through the PSM so that a core1 left running by a soft
   reset goes back to the boot ROM. It then runs the ROM launch handshake
   with the entry point `_crt0_c1_entry`.
3. The crt0 entry (`CRT0_EXTRA_CORES_NUMBER=1` in `chibios_board_rp2350.mk`)
   calls `__c1_cpu_init()` in `rp2350/c1_main.c`. That clears core1's NVIC
   enables and pending bits (the ROM leaves `SPARE_IRQ_1` enabled) and points
   VTOR at `rp2350_c1_vectors`.
4. `c1_main()` waits for `c1_os_start`, which core0 sets in
   `rp2350_board_post_sys_init()` once `chSysInit()` has returned. It takes
   the kernel lock with `port_lock()`, and `chInstanceObjectInit(&ch1,
   &ch_core1_cfg)` creates the instance, its idle thread, tick alarm and FIFO
   interrupt. `c1_main()` enables the flash-lockout doorbell (below), sets
   `c1_xip_lock_ready`, releases the lock and sleeps for ever. Core0 waits up
   to 100 ms for `c1_xip_lock_ready` before it carries on.

Core1 holds the kernel lock while it initialises because its main and idle
threads join the registry both cores share, and it waits for core0 to leave
`chSysInit()` because that ends by releasing the kernel lock without having
taken it. The ChibiOS port does neither: started from
`chSysWaitSystemState()`, core1 initialised without the lock and its closing
`chSysUnlock()` could free core0's lock in the middle of a critical section.

`chSysWaitSystemState()` also used to read `ch_system.state` with a plain load
that the compiler hoisted out of the loop, so a core1 that arrived early spun
for ever; the fix is in the ChibiOS submodule. Boot-test on hardware any
change to core1's start-up timing.

Core1's exception stack (`.c1_mstack`) is the same size as core0's. Its 1 KB
process stack carries only `c1_main()`. In a `stack_overflow` internal error,
line `0xFFFE` means core1's exception stack and `0xFFFF` means core0's.

### Thread placement

`thread_create_alloc()` in `hwdef/common/malloc.c` creates every heap thread
with `THD_DESCRIPTOR_AFFINITY(..., thread_get_preferred_core(name))`. That
function looks the name up in `AP_THREAD_CORE_TABLE` (exact match). It
returns NULL, meaning the calling core, when the name is not listed or the
core has not started. Threads from `hal.scheduler->thread_create()`, the
`SPIn`/`I2Cn` bus threads and the UART threads all go through it. `rcout` is
static: `Scheduler::init()` builds its descriptor by hand and looks up
`"rcout"` itself. The monitor, timer, RCIN, IO and storage threads are plain
`chThdCreateStatic()` calls on core0, so a table entry for them does nothing.

The default table in `rp2350/rp2350_core_affinity.h` is equivalent to:

```c
#define AP_THREAD_CORE_TABLE \
    { "rate",   1 }, { "rcout", 1 }, { "SPI0", 1 }, { "OSD_c1", 1 }, \
    { "SPI1",   0 }, { "I2C0",  0 }, { "I2C1", 0 }
```

A board replaces the whole table from its hwdef, e.g.
`define AP_THREAD_CORE_TABLE { "rate", 1 }, { "rcout", 1 }, { "SPI0", 0 }`.
An unlisted thread starts on its creator's core. `DeviceBus` creates a bus
thread when the first device registers a callback, so `SPI1`, `I2C0` and
`I2C1` are pinned to core0 in case their first user runs on core1. If core1
has not come up within core0's 100 ms wait after `chSysInit()`, threads
listed for core1 start on core0.

### The default split

| Thread | Core | Role |
|---|---|---|
| `rate` | 1 | Copter fast-rate thread, only if `FSTRATE_ENABLE` is non-zero |
| `rcout` | 1 | motor output, woken by the rate thread every iteration |
| `SPI0` | 1 | IMU bus, feeds the rate thread |
| `OSD_c1` | 1 | PIO analog OSD scan-out |
| main thread | 0 | main loop with AHRS/EKF inline, navigation, GCS, logging |
| timer, monitor, rcin, io, storage | 0 | static, never looked up |
| `SPI1`, `I2C0`, `I2C1` | 0 | microSD and I2C buses, pinned |
| UART, `log_io`, scripting, others | creator | core0 in practice |

A gyro sample never leaves core1. The SPI0 DMA interrupt wakes the `SPI0`
thread, which reads and filters the IMU FIFO, and
`AP_InertialSensor::push_next_gyro_sample()` signals the `FastRateBuffer`
semaphore. The `rate` thread (priority 182) preempts the bus thread (181),
runs the controller, and `motors_output()` wakes `rcout` with an event. The
chain needs no inter-core wake-up, and an overrunning main loop cannot delay
it. Its hot code is SRAM-resident (see Performance), which
keeps core1 out of the shared XIP cache and leaves the cache to core0. Core1
deliberately runs few threads, because every kernel operation on either core
takes the same spinlock.

The EKF runs inline in the core0 main loop. An earlier threaded EKF on core1
was removed because it added a one-loop lag and cross-core locking of shared
state for no gain at these loop rates. All three hwdefs set
`AP_GCS_UPDATE_RATE_HZ 25` to leave core0 time for the estimators.

Laurel and RPI_UAVFC ship `INS_GYRO_RATE 2`, `FSTRATE_ENABLE 2`,
`FSTRATE_DIV 2` and `SCHED_LOOP_RATE 200`, which gives a 4 kHz gyro backend,
a 2 kHz rate thread on core1 and a 200 Hz main loop on core0. Laurel measured
about 65% busy on core0 and 37% on core1 in that configuration. Both boards
measured about 190 us on average from gyro sample to motor output, of which
the rate controller is 12-13 us. That instrumentation is not in the tree;
`RATE_LOOP_TIMING_DEBUG` in `rate_thread.cpp` times only the stages, so
re-measure before relying on the figure. With `FSTRATE_ENABLE 0`, the
parameter default, the controller runs in the core0 main loop while `SPI0`
and `rcout` stay on core1, so every sample and every output crosses cores.

### Interrupt routing

A ChibiOS driver enables its interrupt in the NVIC of the core that calls its
start function (`spiStart()`, `i2cStart()`, `sdStart()`, ...). DMA follows
the same rule: `dmaChannelAllocI()` routes a channel taken on core0 to
`DMA_IRQ_0` in core0's NVIC and one taken on core1 to `DMA_IRQ_1` in core1's.
Each core's DMA vector runs at the most urgent priority any of its channels
asked for. An interrupt therefore lands on the core that last started the
peripheral, which need not be its bus thread's core.

`SPIBus::apply_config()` restarts SPI from whichever thread acquires the bus
with a new configuration. The IMU probe runs on the main thread, so SPI0 first
starts on core0; the first high-speed FIFO read on the `SPI0` thread moves its
interrupts to core1, as does every later speed switch. `sdcard_init()` stops
SPI1 so that it restarts on core0. UARTs start in `begin()`, in practice on
core0. `nvicDisableVector()` masks a vector only on the calling core.

### AP_RATE_THREAD_SEPARATE_CORE

Defaults to 0 in `AP_HAL/AP_HAL_Boards.h`; `AP_HAL/board/chibios.h` sets it to
1 for RP2350 SMP builds. Copter creates the rate thread with a plain
`hal.scheduler->thread_create(..., "rate", ...)` and leaves placement to the
table. In its dynamic-rate logic (`FSTRATE_ENABLE 1`, or `2` while disarmed),
`rate_thread.cpp` then counts every iteration towards `rate_loop_count` during
main loop overrun (`get_extra_loop_us() > 0`) and stops treating overrun as a
reason to raise the decimation; only `running_slow`, a log download or the
target divisor do. On one core, overrun means the rate thread is taking too
much time; on its own core it says nothing. Nothing checks the define against
the table: a table putting `"rate"` on core0 also needs it set to 0.

### Cross-core locking

- The kernel lock raises BASEPRI to `CORTEX_BASEPRI_KERNEL` and takes SIO
  spinlock 31 (`PORT_SPINLOCK_NUMBER` in the SMP port's `chcoresmp.h`). One
  spinlock covers both cores, so `chSysLock()` and
  `hal.scheduler->disable_interrupts_save()` exclude the other core. A
  section guarded only by `__disable_irq()` or a BASEPRI write does not.
- Kernel traffic on one core therefore costs the other. RPI_UAVFC measured
  about 0.6% of each core waiting for the spinlock, worst single wait 10-12 us,
  mostly behind short scheduler critical sections. Those counters have since
  been removed from the port. Keep long work out of locked regions: for
  example, `spiStart()` reallocates DMA channels under the lock, so
  `apply_config()` only cycles the peripheral on a real change.
- Mutexes, semaphores and events work across cores, at the cost of an
  inter-core notification and the other core's interrupt latency.
- A lock shared between an interrupt and threads on either core, without the
  kernel lock, needs its own spinlock. Its thread side must raise BASEPRI to
  `CORTEX_BASEPRI_KERNEL` first, because masking only the object's vector is
  not enough. A kernel-level ISR that exits through `chSysUnlockFromISR()`
  sets BASEPRI to 0, and the object's interrupt can then fire on the core
  holding the lock and spin for ever. `PIORXDriver::_lock_thread()` in
  `PIOUART.cpp` shows the pattern.

### Flash writes park core1

A boot-flash erase or program turns XIP off for both cores, and any flash
fetch in that window faults. The EFL driver brackets every operation with
hooks, which `hwdef/common/board_rp2350.c` implements as a lockout that parks
core1 in SRAM:

```text
core0  rpEflBeforeXipOff()
         c1_xip_lock = 1; SIO->DOORBELL_OUT_SET = 1     (IRQ26 on core1)
         wait up to 10 ms for c1_xip_lock == 2
         on timeout: CAS 1 -> 0, fail with FLASH_ERROR_HW_FAILURE
       each page (or one erase), under the kernel lock, in .ramtext:
         exit XIP, program or erase, re-enter XIP
       rpEflAfterXipOn()
         c1_xip_lock = 0

core1  VectorA8 -> c1_xip_lockout_handler()            (.ramtext)
         clear the bell, save and clear all NVIC enables
         CAS 1 -> 2 (if core0 has already withdrawn, return)
         spin while c1_xip_lock != 0
         restore the NVIC enables, return
```

The doorbell runs at `CORTEX_MINIMUM_PRIORITY`, so BASEPRI masks it while
core1 holds the kernel lock, and core1 never parks holding the spinlock that
core0 needs for each page. At priority 0, as in bring-up, it deadlocked both
cores. The handler takes no OSAL prologue, epilogue or kernel lock. `VectorA8`
itself is in flash, which is safe: XIP is on when core1 enters it and back on
before core1 is released. With compare-and-swap on both sides, a timeout can
neither strand core1 parked nor let a write run while core1 fetches from
flash. Until `c1_xip_lock_ready` is set the hook does not park, which relies
on nothing writing flash that early.

Storage must stay on core0, because only core0 can park the other core. A
write from core1 would ring core0's doorbell, which core0 never enables, so it
would fail after 10 ms. The storage thread is a `chThdCreateStatic()` in
`Scheduler::init()`, so it is on core0 by construction and no table entry can
move it. Anything else that writes boot flash, such as a bootloader update,
must also run on core0.

Core1 stays frozen for a whole `efl_lld_program()` call. That covers every
page, plus any time core0 spends on other threads between pages, since the
kernel lock is released between pages but the park is not. A sector erase also
holds core0's kernel lock throughout. While core1 is parked there are no IMU
reads, no rate loop and no DShot frames. On RPI_UAVFC the interval between
`dshot_send_groups()` calls reached 17.6 ms against a 500 us period, all
during boot and around arming.

`AP_STORAGE_NO_WRITE_WHILE_ARMED` in `Storage.cpp` keeps parks out of flight.
On arming it flushes the lines already queued, then writes nothing until
disarm; stock code only defers the sector erase. Logging goes to microSD, not
boot flash. Across three bdshot flights on RPI_UAVFC, `RTDT.dtMax` stayed at
or below 1.3 ms while armed. All three boards set the define.

`rp2350_xip_park_count` and `rp2350_xip_park_max_us` (timed from core1's
acknowledgement) can be read over SWD, or from the `XIPpark:` line printed
with `AP_RP2350_DEBUG_REPORT_ENABLED`. Removing the park would need everything
core1 can reach in the window in SRAM: code, const data, literal pools, every
ISR that can fire on core1 and the context-switch path.

### Measuring per-core load

- `BRD_IDLE_STATS 1` makes `sysperf.c` measure each core from its idle-thread
  hooks, starting the first time `threads.txt` is read: the start in
  `main_loop()` checks the parameter before parameters are loaded, so it never
  runs. Until that first read, `Util::get_system_load()` reports 0% load.
  ArduPilot/ardupilot#33366 moves the start after parameter load.
  `threads.txt` then starts with
  `ThreadsV3 C0 LOAD=.. PEAK=.. C1 LOAD=.. PEAK=..`, and
  `Util::get_system_load()` reports the busier core. Both cores time against
  the shared 1 MHz TIMER0, and LOAD is a decaying average, halved each time
  a second of run or idle time builds up.
- `HAL_ENABLE_THREAD_STATISTICS TRUE` (`CH_DBG_STATISTICS`) adds a per-thread
  LOAD, as a share of the thread's own core, and an `ISR` line per core. It
  also enables `ChibiOS::Scheduler::core1_load_pct()`, which is 100 minus the
  idle share of core1's idle thread since the last call. RPI_UAVFC measured
  statistics at 13.6% of core1's and 10.2% of core0's non-idle time. They ship
  on for Laurel and off for Pico2 and RPI_UAVFC.
- The `Perf:` line from `rp2350_perf_report.cpp` shows `core1load` only with
  statistics on. Its `core0load` is `AP_Scheduler::load_average()`, the main
  loop's use of its time budget, not core0 CPU.

Nothing watches core1 directly. The hardware watchdog is fed from core0, so a
stuck core1 shows up only indirectly, when IMU data and motor output stop.

## Performance: XIP flash, SRAM placement and profiling

### Why XIP flash is the limit

Over a megabyte of code and constants is read from QSPI flash through one
16 KB XIP cache shared by both cores. A hit costs about what an SRAM fetch
does; a miss stalls the core for a QSPI transaction at a 75 MHz flash clock,
tens of cycles at 225 MHz. Moving code to SRAM therefore saves only its miss
fraction. It pays for code with no loop locality (AP_DAL's per-frame
snapshot), code larger than the cache (`NavEKF3_core::CovariancePrediction`,
about 19 KB) and code that runs often enough to evict everything else.

Because the cache is shared, core1's fetches evict core0's working set. The
design keeps core1's whole rate path (rate thread, IMU read and filters,
mixer, rcout, DShot) in SRAM, leaving the cache to core0's main loop and EKF.
Code that must run while XIP is off for a flash write also has to be in
SRAM (see Dual-core architecture); that, not speed, is why the ChibiOS
scheduler is in the RAMFUNC2 registry.

The build uses the ChibiOS default `-Os`, as flash bandwidth rather than core
speed is the bottleneck. The M33 FPU is single precision, so a `double` on a
hot path is a soft-float library call. Full PGO is not implemented.

### Clock and flash timing

All three boards set the same clock in `hwdef.dat`: a 900 MHz VCO (1500 MHz
cannot divide to 225) with post-dividers 4 and 1, `MCU_CLOCKRATE_MHZ 225`,
`RP_VREG_VSEL 12` (1.15 V), and `RP_QMI_CLKDIV 3` / `RP_QMI_RXDELAY 2`. That
is 1.5x the datasheet's 150 MHz clk_sys/clk_peri limit, validated by testing
on the bring-up boards only, not across process, voltage and temperature.
The port started at 375 MHz / 1.30 V, an overclock that existed only to hide
XIP misses. Once core1's hot path was in SRAM the clock came down to 225 MHz
and the voltage to 1.15 V. That is where the walk-down stopped, not a
measured minimum; re-measure the budget before going lower.

- `MCU_CLOCKRATE_MHZ` must match the PLL defines: `system.cpp`
  static-asserts it against `RP_PLL_SYS_CLK`.
- clk_peri is clk_sys, so every SPI, UART and PIO divider follows it.
- The QMI samples `(CLKDIV + RXDELAY) / (2 x sysclk)` after the SCK falling
  edge, which must exceed tCLQV (6 ns on the 133 MHz Winbond part) with
  RXDELAY < CLKDIV: 3/2 gives a 75 MHz SCK and an 11.1 ns sample point.
  `rp_clock_init()` (ChibiOS `rp_clocks.c`) applies it and the regulator
  setting before the PLL switch. Laurel's `hwdef.dat` keeps the derivation.

### Where code runs from

| Region | Address | Size | Holds |
|--------|---------|------|-------|
| flash | `__vectors_base__`: 0x10010000 (Laurel, Pico2), 0x10020000 (RPI_UAVFC) | 4 MB | everything not placed elsewhere |
| main SRAM | 0x20000000 | 512 KB, striped | `.data` with `.ramtext` inside, `.bss`, stacks, heap |
| Scratch X (SRAM8) | 0x20080000 | 4 KB | core0 vectors (288 B), `.ram4_init` |
| Scratch Y (SRAM9) | 0x20081000 | 4 KB | core1 vectors (288 B), `.ram5_init` |

Code reaches SRAM through `__FASTRAMFUNC__` (`.ramtext`, from `PICO2.py`), an
explicit `.ramtext` section, or, for nearly everything, the registries below;
`__RAMFUNC__` is empty on RP2350. `rp_clock_init()` copies `.ramtext` before
the PLL switch so the SRAM fault handlers exist if the clock change faults.

### The placement registries

In `libraries/AP_HAL_ChibiOS/rp2350/`, one `path|symbol` entry per line:

| Registry | Lands in | Budget | For |
|----------|----------|--------|-----|
| `rp2350_ramfunc2_registry.txt` | main SRAM, in `.data` | the heap | hot code either core runs |
| `rp2350_scratchx_registry.txt` | Scratch X | 3808 B | core0-only hot paths |
| `rp2350_scratchy_registry.txt` | Scratch Y | 3808 B | core1-exclusive hot paths |

Before linking, the `rp2350_ramfunc2_gen` task in
`Tools/ardupilotwaf/chibios.py` runs `rp2350_ramfunc2_sections.sh`, which
`nm`s the build and writes three fragments of `*(.text.<mangled>)` picks that
`common_rp2350_smp.ld` INCLUDEs into `.data`, `.ram4_init` and `.ram5_init`.
There is one such task per build, run after every program's objects and
before every link, since all programs share the fragments.
Unpicked function sections fall through to `.text_rest` in flash.

- Matching is on the demangled name without arguments. The path is
  documentation only, so every overload moves and a wrong path goes unseen.
- When the build includes `arducopter` on hardware the script runs with
  `--strict`: an entry that matches nothing fails the build ("no symbol
  match"). Other vehicles, the bootloader and simulation builds skip the
  check.
- `[needs DEFINE]` excuses a miss when the define is off in the generated
  `hwdef.h`, `[unless DEFINE]` when `hwdef.h` sets it to 0 or FALSE, and
  `[inlinable]` a header function the compiler may inline away.
- `(archive)|lib_a-<name>.o` moves a whole library object; that is how
  newlib's single-precision trig and `powf`/`logf` cores are placed.
- A symbol must be in one registry only: the first matching linker statement
  wins and `.data` comes first, so a scratch entry also listed in RAMFUNC2
  silently stays in main SRAM.
- Commented-out entries record A/B runs that took code back out; read the
  comment before re-adding one.

Scratch X and Y sit outside the striped SRAM on their own bus ports. By
convention X is core0's and Y core1's, so code there never queues behind the
other core or DMA. A core0 caller of a function in Y contends with core1 and
does worse than in main SRAM, hence core1-exclusive code only: the scalar
`AC_PID` runs only in the rate controllers and lives in Y, while core0's
position control uses `AC_PID_2D`, `AC_P_2D`, `AC_PID_Basic` and `AC_P_1D`.
A bank speeds up instruction fetch only: the rate PID gained, the data-heavy
motor mixing did not. Both banks are nearly full and overflow fails the link
(`arm-none-eabi-objdump -h` shows `.ram4_init` and `.ram5_init`).

`rp2350_optimize_registry.txt` takes `path|O2` entries, appended to that
file's compile line to override `-Os`: the EKF3 sources,
`AC_AttitudeControl.cpp` and `RCOutput_bdshot.cpp`. A path that never
compiles only warns at the end of the build. `-O2` code is larger, which
costs SRAM where those functions are relocated.

### memcpy, memset and 64-bit division

`rp2350/rp2350_memfunctions.S` replaces newlib-nano's byte-at-a-time
`memcpy`/`memset`, top consumers on both cores, with the RP2040 bootrom's
16-bytes-per-iteration versions in `.ramtext.memfunctions`. It is assembled
in divided syntax on purpose (under unified syntax the Thumb-1 forms stop
setting flags and the loops overrun), and has no `#if defined(RP2350)` guard
because an assembler source never sees `hwdef.h`. `rp_clock_init()` copies
`.ramtext` through `volatile` pointers: without them GCC turns the loop into
a `memcpy` call into SRAM not yet filled, and boot double-faults. Keep them,
and add no registry entries for `memcpy` or `memset`. The same hazard keeps
`__udivmoddi4` and `__aeabi_uldivmod` in flash, so take 64-bit division off
hot paths instead: all three hwdefs set `AP_RCOUT_USE_32BIT_TIME 1`, and with
`CH_CFG_ST_FREQUENCY` at 1000000 ChibiOS's microsecond conversions are casts.

### RAM budget

`.ramtext` is inside `.data`, so every byte moved to main SRAM comes out of
the heap: about 92 KB of code in the current RPI_UAVFC copter build
(`__ramfunc_end__ - __ramfunc_start__`), leaving about 332 KB of static heap.
`__rp2350_min_heap__` (128 KB, `common_rp2350_smp.ld`) is a link-time floor
("RP2350: SRAM over limit after RAMFUNC placement"). What is free in flight
is far less: thread stacks, the EKF, the log buffer (`LOG_FILE_BUFSIZE`,
80 KB by default here), filters and the analog OSD come out of the heap, some
only when enabled at boot. Measure with `@SYS/memory.txt` (FREE, largest
block LRG) or `PM.Mem` in a log, armed and with the features you fly.

`Tools/scripts/rp2350_heap_budget_report.py --map build/<board>/Linker.map`
breaks SRAM down and suggests a `__rp2350_min_heap__`; with `--threads-file`
and `--tasks-file` captures it adds stack use and working-area sizes, which
`--apply` writes into the source. Pass the map for your board, as it
otherwise picks Laurel's or Pico2's, and link the vehicle you are sizing
last: every vehicle overwrites the board's `Linker.map`. It also tries to
fetch `@SYS/memory.txt` with mavproxy from the first ArduPilot USB port.
`Tools/scripts/rp2350_ramfunc2_sizes.py <elf> --update` rewrites the size
comments in the RAMFUNC2 registry; its default ELF is Laurel's.

Core0 relocation is limited by heap, not benefit. Moving about 2.3 KB of
AP_DAL cut its share of core0 8-10x on RPI_UAVFC, but the rest of core0's
flash time is spread over large libraries that would each take much of the
free heap. Rank candidates by samples per KB and check free heap in flight.

### Standing check: when you add to a hot path, look at where it landed

Code is in flash unless a registry or attribute says otherwise, and no diff
shows it. After changing anything that runs at kHz rates:

```bash
arm-none-eabi-nm -C build/<board>/bin/arducopter | grep -i <function>
```

0x10xxxxxx is flash, 0x2000xxxx-0x2007xxxx main SRAM, 0x20080xxx Scratch X,
0x20081xxx Scratch Y. Bidirectional DShot, the PIO OSD interrupt and the PIO
UART RX handler each shipped partly in flash; the DShot evictions put CRSF
into continuous failsafe.

Relocation is a property of the whole call path. Flash and SRAM are out of
branch range of each other, so each crossing goes through a 16-byte
`*_veneer`: in flash when flash calls SRAM, in SRAM when SRAM calls flash
(whose body still comes through XIP). The RPI_UAVFC copter build has about
700. Relocate whole call trees, check what SRAM code calls, and leave alone a
tiny leaf called from flash, whose veneer costs what the move saves.

### Profiling switches

| Switch | Default | Cost | Output |
|--------|---------|------|--------|
| `AP_RP2350_DEBUG_REPORT_ENABLED` | 0 | crowds the GCS pane | perf report |
| `AP_RP2350_PC_SAMPLER_ENABLED` | 0 | ~5 kHz ISR per core, ~27 KB BSS | PC histograms |
| `HAL_ENABLE_THREAD_STATISTICS` (or `--enable-stats`) | off; Laurel sets TRUE | timing on every ISR, lock and switch | thread LOAD, `core1load` |
| `AP_XIP_PROFILER_ENABLED` | set by no board | context-switch hook | per-thread hit rate |
| `AP_RP2350_SPI_CYCLE_STATS_ENABLED` | 0 | negligible | `spi_stopstart_count[]`/`_us[]`, read over SWD |

Reconfigure after changing a define. Profile with thread statistics off:
their cost has measured from a few percent to over 10% of a core's non-idle
time, and a stats build once put CRSF on RPI_UAVFC into continuous failsafe.
RPI_UAVFC's defaults set the cheap parameters `BRD_IDLE_STATS 1` (per-core
LOAD/PEAK in `threads.txt`) and `SCHED_OPTIONS 1` (`tasks.txt` from boot).

### The perf report

`rp2350_perf_report()` runs every 10 s from the HAL io thread after boot:

```text
Perf: main=<Hz> rate=<Hz>
Perf: core0load:<%> core1load:<%> xip=<%>
XIPpark: n=<count> max=<us>
PIO<n> rx=<bytes> drop=<n> ovr=<n> fe=<n>
PROFc1 n=<samples> F=<%> S=<%> o=<%>
```

- `rate` is the gyro rate over the rate thread's decimation. `core0load` is
  `AP_Scheduler`'s load average, not core0's CPU load. `core1load` is 100%
  minus core1's idle share and needs thread statistics; a steady 100% can
  mean the lookup of core1's idle thread failed.
- `xip` is the hit rate from the global `CTR_HIT`/`CTR_ACC` counters, both
  cores together, cleared on each read (as the XIP profiler also does).
- `XIPpark` counts core1 flash-write parks and the longest: it explains a
  single large rate-loop excursion, not routine jitter. The PIO OSD adds an
  `OSD:` line.

### PC sampling

`rp2350_pc_sampler.cpp` takes TIMER0 ALARM2 for core0 and ALARM3 for core1
(the SMP ticks use ALARM0/1; all three hwdefs suppress the ChibiOS handlers
for 2 and 3). Each fires every 197 us (prime, about 5.08 kHz) at priority 0,
which kernel BASEPRI cannot mask; a maskable sampler piles its deferred
samples onto `chSysUnlock`. A naked stub in `.ramtext` reads the stacked PC
into a 2048-slot per-core hash with saturating 16-bit counts. Core1's alarm
is armed by the first `thread_create()` thread on core1, core0's only by the
first perf report, so core0 stays empty without the debug report.

`PROFc1` STATUSTEXT carries the core1 top 16 as `<tag><hex offset>:<count>`
(`F` from `__vectors_base__`, `S` from 0x20000000, `C` from 0x20080000, `o`
absolute). Diffuse code such as the EKF needs the full readout:
`@SYS/pcprof.txt` (core1) and `@SYS/pcprof0.txt` (core0) over MAVLink FTP,
the top 512 PCs under `PROF c<n> n=<total> drop=<n> top=<n>`. Attribute
either against the exact ELF that is running (`-` reads stdin); the report
ranks functions, marks those in RAMFUNC2 and prints candidate registry lines:

```bash
Tools/debug/rp2350_pc_profiler.py --histogram pcprof0.txt \
    --elf build/<board>/bin/arducopter
```

- Counts accumulate from boot and are never reset; reboot before a capture.
- Counts saturate at 65535 and the idle loops (`rp2350_idle_c0`/`_c1`, whose
  addresses move between builds) peg within about a minute, overstating
  non-idle shares. The summary `o` is what F and S leave, so saturation,
  drops and scratch all land there; take region splits from the FTP
  histogram.
- Check `drop=`: core0's spread-out code overflows the hash far more than
  core1's, so diffuse code like the EKF is undersampled.
- It cannot see blocking: a thread waiting on a lock or a parked core counts
  as idle. For stalls, read temporary `volatile` stage counters over SWD.

Without `--histogram` the tool reads the DWT PCSR (0xE000101C) through
OpenOCD's TCL port without halting the core, so it profiles the flight build
with nothing extra compiled in, given an OpenOCD attach that leaves the
firmware running (see Building, flashing and debugging). `rp2350.cm1`
selects core1; `--by-line N` re-bins the top N functions into 8-byte cache
lines and flags large functions with a small hot kernel as SPLIT CANDIDATE:

```bash
Tools/debug/rp2350_pc_profiler.py --elf build/<board>/bin/arducopter \
    --tcl-port 50001 --samples 30000 [--target-select rp2350.cm1]
```

`@SYS/threads.txt` gives each thread's core and `STACK=<free>/<total>`: bytes
free, not used (misreading it once grew `MAIN_STACK`, now back at the 0x600
default). `AP_XIP_PROFILER_ENABLED` adds an `XIPCacheV1` table of per-thread
hit rates; uncomment `ap_xip_cs_hook` in the RAMFUNC2 registry with it, or the
hook runs from flash and disturbs what it measures. `@SYS/tasks.txt` gives
each scheduler task's MIN/MAX/AVG time, overruns and share of the loop.

### Acting on a profile

- Do not compare absolute shares between boots: EKF and GPS state move
  core0's whole workload (one build measured 46% idle, then 67% after a
  reboot). Use a ratio against code on the same clock that the change does
  not touch, such as DAL samples over NavEKF3 samples for AP_DAL.
- A high count does not prove code is fetch-bound: bdshot `read_telemetry`
  dropped 5.5x when moved, the motor mixing not at all. Move and re-measure.
- A low count does not prove code is cold: `constrain_value_line<float>`
  looked cold in SRAM and became a rate-loop hotspot back in flash.

### Loop rates that follow from the budget

| Setting | Value | Set in | Effect |
|---------|-------|--------|--------|
| `SCHED_LOOP_RATE` | 200 | `defaults.parm`, all three | core0 main loop |
| `INS_GYRO_RATE`, `INS_FAST_SAMPLE` | 2, 1 | Laurel, RPI_UAVFC | 4 kHz IMU backend |
| `FSTRATE_ENABLE` | 2 | Laurel, RPI_UAVFC | divisor fixed once armed |
| `FSTRATE_DIV` | 2 | Laurel, RPI_UAVFC | 2 kHz rate thread on core1 |
| `AP_GCS_UPDATE_RATE_HZ` | 25 | `hwdef.dat`, all three | GCS receive and send tasks |

Core0 runs everything but the rate path, mostly from XIP, so its loop rate is
the main cost lever. Attitude runs in the SRAM-resident rate thread on core1,
so the main loop (nav, EKF inline, GCS, logging) needs only 200 Hz. Disarmed,
`FSTRATE_ENABLE 2` lets the divisor rise to avoid gyro buffer overruns
(`rate=` shows the real rate). `AP_GCS_UPDATE_RATE_HZ` defaults to 400, every
loop at 200 Hz; at 25, MAVLink receive and streaming run at most 25 times a
second, leaving core0 time for the estimators.

## Peripherals

How the PIO, serial, RC output, SPI, microSD, I2C, ADC and GPIO drivers
work on RP2350, and what bites when changing them. Thread placement,
cross-core locking and the flash-write lockout are in Dual-core
architecture; the OSD program has its own section.

### PIO blocks and who owns them

Each PIO block has four state machines (SMs), 32 instruction slots and a
GPIOBASE of 0 (reaches GPIO0-31) or 16 (GPIO16-47). Pin fields are 5 bits
relative to GPIOBASE, so the pin window belongs to the block, not the SM.

| Block | Owner | SMs | Slots | GPIOBASE |
|---|---|---|---|---|
| PIO0 | PIOUART0 (SM0 TX, SM1 RX), PIOUART1 (SM2, SM3) | 4 | 29 | 16 |
| PIO1 | one of: OSD, NeoPixel, PIOUART2/3 | 1-4 | 31, 4 or 29 | 0; 16 for UARTs |
| PIO2 | DShot, SM n drives output n+1 | 4 | 13 or 29 | 0 |

| | Pico2 | Laurel | RPI_UAVFC |
|---|---|---|---|
| UART0, SERIAL1 | GPIO12/13 | GPIO12/13 | GPIO44/45 |
| UART1, SERIAL2 | GPIO10/11 (FUNCSEL 11) | GPIO8/9 | GPIO36/37 |
| PIOUART0, SERIAL3 | GPIO20/21 | GPIO20/21 | GPIO42/43 |
| PIOUART1, SERIAL4 | GPIO16/17 | GPIO34/35 | GPIO16/17 |
| PIO1 | unused | unused | OSD (`OSD_TYPE` 6) or NeoPixel on GPIO2 |
| PIO2 | unused, DShot off | DShot GPIO28-31 | DShot GPIO9, 8, 7, 6 |

Loading a program is a blind write to INSTR_MEM, which reads back as zero,
so nothing detects two users of a block. `RP2350_pio1.cpp` arbitrates PIO1:
`pio1_claim()` succeeds only for the owner `pio1_intended_owner()` picks -
the PIO UARTs if the board builds more than two, else the OSD if `OSD_TYPE`
or `OSD_TYPE2` is 6, else NeoPixel. It returns NONE until
`AP_Param::initialised()`, and a driver that loses must not touch the block.

Rules for adding a PIO user:

- Choose the block by pin window. GPIOBASE is marked read-only in
  `rp2350.h`, so it is written through a raw pointer at offset 0x168.
- FUNCSEL is 6, 7, 8 for PIO0, PIO1, PIO2. FUNCSEL 11 is UART_AUX: the SMs
  run and nothing reaches the pin.
- A new PIO1 user needs a `PIO1Owner` value and a rule in
  `pio1_intended_owner()`. PIOUART defines the PIO1 vectors only when it has
  a third or fourth instance, which leaves them to the OSD.
- `palSetPadMode()` writes all of GPIOn_CTRL, so INOVER/OUTOVER go after it.
  Ask for `PAL_RP_PAD_IE` even on an output the program waits on, and for
  SCHMITT and a drive strength, or they come out zero. GPIO32-47 are pads
  0-15 of IOPORT2; IOPORT1 silently ignores pad 32 and up.
- SHIFTCTRL direction bits mean shift right when set; PUSH_THRESH is 24:20,
  PULL_THRESH 29:25, 0 means 32. The register constants are in `PIOUART.h`;
  check any it does not itself use against `rp_pio.h`. Both thresholds were
  once wrong unnoticed: NeoPixel showed LED1 right and every later LED a
  byte out, which is frame alignment, not timing or byte order.
- SM_RESTART clears neither X nor Y and leaves the PC at 0; set up pins, Y
  and the entry `jmp` through SMx_INSTR while the SM is stopped. Over SWD,
  SMx_ADDR is the PC.
- `RCOutput_pico.cpp` and `AP_OSD_pico.cpp` take those constants from
  `PIOUART.h`, which defines them for every RP2350 build; only the UART
  driver itself is gated on `HAL_HAVE_PIO_UARTS > 0`.

### PIO programs and generated headers

| `rp2350/pio/` | Programs (slots) | From |
|---|---|---|
| `uart.pio` | `uart_tx` (10), `uart_rx` (9), `uart_rx_sbus` (10) | Betaflight, pico-examples |
| `dshot.pio` | `dshot_600` (13), `dshot_600_bidir` (29) | Betaflight |
| `ws2812.pio` | `ws2812` (4) | pico-examples via Betaflight |
| `osd_tx.pio` | `osd_tx_pal`, `osd_tx_ntsc` (31) | Betaflight |

The `.pio.h` headers are committed pioasm output, so a build needs no
pioasm. Edit the `.pio` and run `Tools/scripts/rp2350_pioasm.py`; `--check`
fails on a stale header and `--install` fetches checksummed pioasm and
picotool 2.2.0, pinned because pioasm writes its version into each header.
`rp2350_pio.h` sets `PICO_NO_HARDWARE`: only the arrays and the wrap, label
and `.define public` macros are used. Load with `rp2350_pio_load()`, which
adds the offset to JMP targets, as pioasm emits them program-relative even
under `.origin`.

### Hardware UARTs

The two PL011s run under the ChibiOS SIO driver (`UARTDriver_rp2350.cpp`
and the `RP2350` blocks of `UARTDriver.cpp`).

- Some pads carry the UART at FUNCSEL 11, not 2; the generator passes the
  TX pin's slot. RX is pulled up before `sioStart()`: a floating RX reads as
  a permanent BREAK whose interrupts starve USB.
- Inversion is INOVER/OUTOVER on the pad. Parity and stop bits on a running
  port go through `sio_apply_framing()`, so SBUS out works here.
  `OPTION_SWAP` is refused and RS-485 DE falls back to none.
  `OPTION_HDPLEX` is accepted and ignored: half duplex needs a PIO UART.
- DMA channel numbers in `hwdef.h` are advisory. SPI and ADC take
  `RP_DMA_CHANNEL_ID_ANY` and start first, so the UART falls back to any
  channel (TREQ selects the peripheral) and to non-DMA if none is free;
  RXDMAE set with no channel behind it overruns the FIFO silently.
- With no RX idle interrupt, `_rx_timer_tick()` flushes partial receives by
  aborting the RX channel. An aborted channel reads `TRANS_COUNT` as 0, so
  the count comes from `WRITE_ADDR`; the TX timeout path uses `READ_ADDR`.

`_begin()` calls `thread_init()` unconditionally, as on every other HAL, and
failing to create the thread panics. An earlier RP2350-only gate deferred it
for ports opened before system init, which SERIAL1 (MSP DisplayPort) is:
frames queued where nothing drained them, `msp_serial_send_frame()` then
saw too little `txspace()` and stopped writing, and the port never sent a
byte while receive worked. A port with baud 0 keeps `_tx_initialised` false,
which is what keeps the thread off an unstarted backend.

### PIO UARTs

`PIOUART.cpp` (`PIORXDriver`) is a UART on two SMs: 8N1 or 8N2, 8 PIO
cycles a bit, divider from the runtime `clk_sys`, 512-byte rings, no DMA.

- Both receive programs check the stop bit before pushing. A bad one raises
  PIO flag 4+sm (polled; only 0-3 reach INTE) and waits for an idle line, so
  a line held low gives framing errors, not a stream of 0x00. RXNEMPTY has
  no watermark, so there is an interrupt per byte, as in Betaflight; FJOIN
  gives each direction an eight-entry FIFO against latency.
- `_write()` queues, primes the FIFO and arms TXNFULL; the interrupt drains
  the ring and disarms TXNFULL when it is empty (armed, it fires for ever).
  Nothing blocks. `tx_pending()` covers ring, FIFO and SM back at `pull`.
- One vector per port serves errors, RX and TX, from Scratch X on core0,
  sharing a per-port spinlock with callers on either core.
- `uart_rx_sbus` (8E2, parity skipped) is used for `OPTION_RXINV` at 100000
  baud; the ISR then passes on only 25-byte frames that start with 0x0F
  after a 2 ms gap and have a clear top flags nibble. FPort and other
  inverted 8N1 use the normal program.
- No parity on transmit (`_write()` sends nothing and warns once, so SBUS
  out needs a hardware UART), receive parity unchecked, no flow control.
- hwdef: `PIOUARTn_TX`/`_RX` as `INPUT FLOATING`, GPIO16-47 only. The
  instance is the port's position among PIOUARTs in `SERIAL_ORDER` but its
  pins come from the label number: number them from 0 and list in order.
- `@SYS/uarts.txt` gives `RXDRP=`, `FE=`, `OE=` (RXSTALL) and
  `IRQ=<us/s>,max=<us>`; the ports also appear in the `UART` log message.

Serial RC goes on a PIO UART (RPI_UAVFC SERIAL3, `SERIAL3_OPTIONS` 1 for
SBUS). Laurel and Pico2 also read PPM on GPIO21 with `SoftSigReaderRP2350`
(PAL edge callbacks); PIOUART0 takes the pin when `SERIAL3_PROTOCOL` is 23.

### Half duplex on a PIO UART

`uart_tx` owns the turnaround, as in Betaflight. `.mov_status txfifo < 1`
lets `mov x, status` see an empty FIFO without stalling, so the line is
released one instruction after the last stop bit, only if nothing more is
queued. Full duplex runs the same program with SET_COUNT 0, making the `set
pindirs` no-ops while side-set still drives the pin. Y holds the extra stop
bits, loaded at `_begin()`; the program never writes Y.

The RX SM then listens on the TX pin. The pad pulls down if either
`OPTION_PULLDOWN_TX` or `_RX` is set, else up; SmartAudio asks for pull-down
and needs it. Either inversion option implies both, as on STM32. The
receiver hears our own frame, and as on the ST path the HAL drops it:
`_write()` sets `_hd_echo_active`, the ISR discards while it is set, and
`_hd_echo_check()` (from `_flush()` and `_available()`) clears it once
`tx_pending()` is false, purging the RX FIFO under the lock first or the
last echoed byte slips through. `AP_SmartAudio` needs this: replies open
with the same 0xAA 0x55 as requests. `tx_pending()` goes false as the stop
bit starts; wait a bit time before switching anything on that edge.

SmartAudio v2 works end to end on the bench, SETs included. Keep
`VTX_OPTIONS` bit 4 (`VTX_PULLDOWN`): its leading 0x00 gives the first start
bit a falling edge on a line resting low. The shared pin is a free
loopback: log popped bytes ahead of the echo drop to see what went out.

### RC output: PWM slices, groups and the buzzer

A group is one PWM slice of two channels (`HAL_PWM_GROUP_CHANNELS` 2),
declared `PWMn_A`/`PWMn_B` with type `TIMn`, slices 0-11, FUNCSEL 4; both
share the slice's divider and TOP. The divider's integer part is 8 bits, so
a 1 MHz counter needs `clk_sys` under 256 MHz (`static_assert`). OneShot is
refused (no EGR), and `setup_group_DMA()` always fails: with no timer DMAR,
BLHeli passthrough is off and DShot and LEDs use the PIO.

The buzzer is an `ALARM` pin with no `GPIO(n)` or `HAL_BUZZER_PIN`, either
of which lets `AP_Notify::Buzzer::init()` switch it to SIO. The transducers
need a driven square wave, so `HAL_BUZZER_NOMINAL_FREQ_HZ` 4000 replaces
every note's frequency. On RPI_UAVFC the buzzer (slice 1 B) shares slice 1
with LED output 5 (1 A), and whichever of RCOutput and the tone alarm last
calls `pwmStart()` on `PWMD1` sets the divider. `set_default_rate()` skips
LED-mode groups so it cannot overwrite the alarm's period, and
`set_group_mode()` leaves the slice running for an LED mode, so a NeoPixel
strip on output 5 keeps the alarm's divider (unchecked on hardware); do not
use output 5 for PWM there.

### DShot

`RCOutput_pico.cpp` drives DShot from PIO2; `set_group_mode()` and
`dshot_send()` branch to it.

- DShot600 only (`MOT_PWM_TYPE` 6); other rates raise a config error. SM n
  drives output n+1, so DShot is limited to `SERVO1`-`SERVO4`.
- One program fits at a time, so `SERVO_BLH_BDMASK` must cover every DShot
  output or none (else a config error); a direction change reloads the
  block. `HAL_WITH_BIDIR_DSHOT` is emitted for every RP2350 board with
  DShot, and the hwdef `BIDIR` tag means nothing here.
- PIO clock 24 MHz plain (fractional divider fine), 75 MHz bidirectional,
  whose decode assumes exactly 75 MHz: `clk_sys` must be a multiple.
- The frame goes in the low half of the FIFO word, as both programs start
  with `out y, 16`; in the high half a valid throttle-zero frame goes out.
- `send_pulses_DMAR()` returns the group to IDLE itself, as no DMA
  completion will. Checksum direction comes from `is_bidir_dshot_enabled()`,
  not `bdshot.enabled`, which tracks input capture DMA this chip lacks.
- BLHeli_S bit timings from `SERVO_DSHOT_ESC` do not apply. eRPM does not
  need EDT; with `SERVO_DSHOT_ESC` 3 the RPI_UAVFC ESCs never armed.

The rate thread calls `set_dshot_rate(SERVO_DSHOT_RATE, attitude_rate)`,
where `attitude_rate` is gyro rate over `FSTRATE_DIV`, not `SCHED_LOOP_RATE`.
The result is raised to at least 800 Hz, and a multiple above 1 is cut back
while it exceeds 4096 Hz. `SRV_Channels::init()` calls it first with the
200 Hz scheduler rate; the rate thread's call replaces that. 4 kHz gyro:

| `SERVO_DSHOT_RATE` | `FSTRATE_DIV` 2 | `FSTRATE_DIV` 4 |
|---|---|---|
| 0 | 1 kHz timer | 1 kHz timer |
| 1 | 2 kHz | 1 kHz |
| 2 | 4 kHz | 2 kHz |

At 1 no timer is armed and each rate-thread `push()` sends. A boot-flash
write parks core1 and so stops DShot to every motor; see Dual-core
architecture.

`AP_RCOUT_USE_32BIT_TIME`, set only on these three boards, makes
`rcout_timer_t` 32-bit and `rcout_micros()` `micros()`, taking the
`micros64()` veneer off core1's DShot path. `micros()` wraps at 71.6
minutes: subtract `rcout_timer_t` values from `rcout_micros()`, never from
`micros64()`. The wrap is unsoaked; reboot a board up for over an hour.

### Bidirectional DShot and eRPM

`dshot_600_bidir` sends, releases the line, waits for the start edge and
samples 128 times, 5.56 samples a telemetry bit, autopushing four words.
`read_telemetry()`, called at each send for the reply to the last frame,
turns runs into GCR bits (after Betaflight `decodeOversampledTelemetry()`)
and shares `bdshot_decode_gcr_erpm()` with the timer path. `write_frame()`
hands over a frame only at `pull` or past `complete`, after flushing stale
RX words, or autopush blocks mid-sample. An SM parked on a wait for two
updates running (a silent ESC) is restarted, counted in `_stall_restarts[]`.

The eRPM scale matches the motor peak in pre-filter accel spectra on two
airframes (slope 1.01-1.03, `SERVO_BLH_POLES` 14), good enough for a
per-motor notch. `read_telemetry()` reports each read as GOOD, BAD (a
reply that did not decode), MISSING (the SM still parked on a wait a whole
update after its frame went out) or NONE, and `bdshot_count_reply()` counts
the first three into `erpm_clean_frames[]` and `erpm_errors[]` with the
timer path's 5 s reset, so `ESC.Err` shows frame loss. The counter is not
yet checked against a flight log. It lives in RAMFUNC2 because Scratch Y
has 8 bytes left. Betaflight reports 5-8% decode failures for this program
under load.

### NeoPixel

WS2812 output (`HAL_SERIALLED_ENABLED`, RPI_UAVFC only) uses one PIO1 SM per
LED output when the broker gives PIO1 to NeoPixel; otherwise the group
reports "NeoPixel off: PIO1 is the OSD's". 10 PIO cycles a bit at 800 kHz
(divider 28.125 at 225 MHz), autopull at 24 bits, colour in bits 31..8, GRB
or RGB (`MODE_NEOPIXELRGB`); ProfiLED is refused. With no DMA,
`serial_led_send()` streams words into an eight-word FIFO from the LED
thread, waiting about 30 us per LED beyond eight; a timeout still ends the
frame, so a partial frame goes out. The 300 us latch gap is timed from when
the queued words should leave the wire, a future stamp, hence the signed
compare. The pin is `PWM(n)` only for a channel and group, and the mode
switch happens at the first colour write, so the boot banner can say PWM.

### SPI

The PL022 runs from `clk_peri`, which follows `clk_sys`. SSPCPSR is 2 and
SCR rounds so the clock never exceeds the request: at 225 MHz, 16 MHz gives
14.06 and 25 MHz gives 22.5. SCR tops out at 255, a 439 kHz floor, so a
slower request takes the smallest even SSPCPSR that reaches it: the microSD
400 kHz init clock runs at 399 kHz with SSPCPSR 4.

- `spiStart()` frees and reallocates both DMA channels under the kernel
  lock. `SPIBus::apply_config()` cycles the peripheral only when SSPCR0,
  SSPCPSR or the CS pad change (they are compared there, or a device switch
  on a shared bus would look unchanged); cycling per transfer cost about 4%
  of core1. CS is set to an SIO output, high, before select.
- `start_peripheral()` checks both DMA channels after `spiStart()`; on
  failure it restores SPI_STOP, counts `spi_start_fail_count[]` and
  `acquire_bus()` fails rather than arming transfers against nothing.
- ChibiOS gates `spiAbort()` on `SPI_SUPPORTS_CIRCULAR`, FALSE on RP. A
  timeout (20 ms plus 32 us a byte) calls `abandon_transfer()`: still active
  means the fork's `spi_lld_abort()` (stop DMA, drain RX, cycle SSE) and
  `spi_fail`; already completed by the ISR means `spi_late_count[]` and no
  error, as `spi_fail` is sticky and blocks arming. `clock_pulse()` too.
- SPI DMA is `SHARED_DMA_NONE` in `HAL_SPIn_CONFIG`; a real channel number
  would let Shared_DMA stop a bus under the SD card.
- Never `spiStop()` behind `SPIBus`. `sdcard_init()` once did, leaving
  `spi_started` true with the peripheral in reset: every transfer timed out,
  60 `spi_fail`s per reinit. It now stops the bus through `SPIBus` under the
  bus semaphore, and the next `acquire_bus()` restarts it on its own core.

Over SWD: `SPID0/1.rxoverruns` and `.aborts`, `spi_late_count[]`,
`spi_start_fail_count[]`, and with `AP_RP2350_SPI_CYCLE_STATS_ENABLED`,
`spi_stopstart_count[]`/`_us[]`.

### microSD

RP2350 has no SDIO host, so the card runs in SPI mode through MMC-SPI
(`HAL_USE_MMC_SPI`). 4-bit SDIO in PIO needs DAT1-3 on contiguous GPIOs,
which no board wires; on Laurel SPI1 is the SD card or the AT7456E, not both.

- `hal_mmc_spi.c` maps its SPI calls to the hooks in `sdcard.cpp`, so the
  clock comes from `SPIDEV sdcard`; the SSPCR0 values in `lowspeed` and
  `highspeed` only mark which speed is wanted.
- A 512-byte block is prologue, data, CRC skip and response, then a
  byte-at-a-time busy poll, each a `do_transfer()` round trip in the
  lowest-priority IO thread: round trips and core0 load set the throughput,
  not the 22.5 MHz wire.
- Only the SDC path calls `set_io_size()`, so here `io_size` stays 4096 and
  the logger syncs every 4 KB: the directory entry plus, when a cluster was
  allocated, FSINFO and each FAT copy, all single-sector writes.
  RPI_UAVFC's `AP_FATFS_MAX_IO_SIZE` 32768 needs #34476 to act; #34477 makes
  each block one exchange. With both, a bench sweep wrote 119 KB/s of card
  blocks at a 4 KB sync and 265 KB/s at 32 KB.
- Upstream, on every FATFS board: FATFS clips a multi-sector write at the
  cluster edge, so 4 KB clusters cap `disk_write()` at 8 sectors; and
  `AP_Logger_File::_writebuf_chunk` is fixed before the card mounts, at 4096.
- Busy is in-band on MISO and ends only while clocked, so it cannot be read
  off the pad and blocks cannot be chained blind; an ISR-chained write was
  tried and backed out after wedging the bus.
- A reset mid multi-block write leaves the card deaf to CMD0 until power is
  removed: power cycle between flashes while logging. Prefer SDHC, as SPI
  mode is optional for SDXC.

### I2C, ADC and GPIO levels

The RP I2C LLD uses no DMA and gets `SHARED_DMA_NONE`, but the hwdef
generator still allocates channels to I2C unless told `NODMA I2C*`. On
RPI_UAVFC that line is what leaves the ADC a channel: without it configure
fails with "Missing required DMA for ADC1". Laurel and RPI_UAVFC run
100 kHz. Each transaction ends in `i2cStop()`, which resets the block (the
soft stop is empty); a bad LLD state fails the transfer rather than
asserting, and there is no SMBus mode. `I2C_ORDER` accepts `I2C0`.

One 12-bit ADC round-robins the hwdef channels at 5 kHz total. Pin numbers
are ADC channels: GPIO26-29 are 0-3 on QFN-60, GPIO40-47 are 0-7 on QFN-80
(`define RP2350B_QFN80 1`). With `HAL_WITH_MCU_MONITORING` the temperature
sensor (channel 4 or 8) is the group's last channel, pin 253; there is no
VREFINT, so MCU voltage reads 3.3 V. The LLD stops on FIFO overflow, which a
long critical section can cause, and `adcerrorcallback()` restarts it.
`HAL_RP_ADC_GPIOS` gives exactly the declared pins their analog pad setup.

There is no port ODR to preload, so the generator emits
`HAL_GPIO_INIT_LEVELS` and `board_rp2350.c` writes the levels before any pad
mode, so no pin briefly drives the wrong way. It is a separate macro because
`HAL_GPIO_PINS` initialises a struct whose next member is the IRQ handler. A
`GPIO(n)` OUTPUT with no HIGH/LOW gets HIGH, as on STM32; an OUTPUT without
`GPIO(n)` is set only if it states a level. Board init also makes every
non-PWM `GPIO(n)` pin a push-pull output, so do not number an input pin.

### Upstream bugs still relevant

- `ESC_CALIBRATION` 2 and 3 block in `while(1)` before their
  `set_and_save(ESCCAL_NONE)` reaches flash, as the IO and storage threads
  wait for `_hal_initialized`. The board boots straight back into
  calibration with no MAVLink; erase the parameter area over SWD (see
  Recovering a board). Use mode 1.
- Carried in the ChibiOS fork, so check on any rebase: RP `spi_lld_abort()`
  was empty upstream, and `spi_lld_start()` reported DMA allocation failure
  only through `osalDbgAssert()`, which flight builds compile out.

## The PIO analog OSD

RPI_UAVFC has no MAX7456. Its analog OSD is a PIO state machine that
follows the camera's sync and switches a video mux pixel by pixel, drawing
AP_OSD's character grid with the MCM fonts the MAX7456 backend uses. The
program and video timing are Betaflight's (`src/platform/PICO/osd`, GPLv3);
the setup around them is written against the RP2350 registers and ChibiOS,
as `RCOutput_pico.cpp` does for WS2812 and DShot. The backend is
`libraries/AP_OSD/AP_OSD_pico.{h,cpp}`; the program is
`libraries/AP_HAL_ChibiOS/rp2350/pio/osd_tx.pio`, unmodified, with
`osd_tx.pio.h` generated by `Tools/scripts/rp2350_pioasm.py` (only
`osd_tx_pal` and `osd_tx_ntsc` are used). The thread core table, PIO block
allocation, RAMFUNC2/XIP and SWD are covered in their own sections.

### Selecting it

- Build: `define AP_OSD_PICO_ENABLED 1` in the hwdef (default 0 in
  `AP_OSD_config.h`). The `OSD_PICO` enum value and its cases exist only
  then. `PIO1_OSD_TYPE` in `RP2350_pio1.h` repeats the value for the HAL,
  which cannot include AP_OSD; a `static_assert` keeps the two equal.
- Runtime: `OSD_TYPE` or `OSD_TYPE2` = 6 (`RP2350_PIO`), then reboot. The
  board default is 5, MSP DisplayPort on SERIAL1. It can run beside an MSP
  or MSP DisplayPort instance, not beside MAX7456 or SITL.
- PIO1 holds the OSD or NeoPixel, never both. `pio1_intended_owner()` in
  `RP2350_pio1.cpp` reads the two parameters once `AP_Param::initialised()`
  (not `is_system_initialized()`, which is too late for RCOutput) and says
  "PIO1: OSD" or "PIO1: NeoPixel". With the OSD selected the J2 LED output
  (`SERVO5_FUNCTION` 120) is dead. More than two PIO UARTs on a board give
  PIO1 to them at build time.
- `OSD_FONT` is read once in `init()`; unlike MAX7456 a change needs a reboot.

### Hardware front end

From page 3 of the RPI_UAVFC schematic (RP-010038-SD-1, R2 Rev C), read off
the sheet rather than measured; the working NTSC overlay confirms the
polarities. The pins are not declared in hwdef.dat: the defaults are the
`HAL_OSD_PICO_{W,EN,SYNC}_PIN` defines in `AP_OSD_pico.h`.

| Net | GPIO | PIO use |
|---|---|---|
| `OSD_W` | 21 | OUT and SET base: bit 0 of each pixel |
| `OSD_EN` | 22 | OUT and SET base + 1: bit 1 |
| `OSD_SYNC` | 23 | IN base for `wait`, and the `jmp pin` |

Sync: camera -> R66 75R -> R63/C58 filter (about 500 kHz) -> C59 coupling
-> D7 clamp -> U9 comparator -> `OSD_SYNC`, low on sync tips as `wait 0 pin`
expects. Overlay: `OSD_W` sets 1.0 V or 0.3 V through a 3.9k/12k/1.4k
divider, U8 (74LVC1G3157) on `OSD_EN` picks camera (B0) or overlay (B1), and
U10 (TLV3541, gain 2) drives VID through R67 75R. EN 0 is transparent, EN 1
with W 0 black, with W 1 white; both idle low, so the camera passes through.

### Resource budget

| Resource | As built |
|---|---|
| PIO | all of PIO1: 31 of 32 instruction slots at offset 0, SM0, GPIOBASE 0, FUNCSEL 7 |
| PIO clock | 75 MHz, integer divider 3 from the 225 MHz clk_sys |
| DMA | one channel, any free, claimed on core1; DREQ PIO1 TX0; about 1.3 MB/s |
| Interrupts | PIO1_IRQ_0 (field flag) at 7; core1's DMA vector, which runs at 2 |
| Thread | `OSD_c1`, 1 KB stack, `PRIORITY_IO` (58), core1 |
| Heap | about 19 KB: font 13,824 B, backend object about 3.9 KB, stack 1 KB |
| Code | the per-block path is listed in the RAMFUNC2 registry, so runs from SRAM |

All of it is claimed in `init()`, only when selected; the object is mostly
three 828-byte block buffers and two 480-byte character frames. `init()`
holds the main thread up to 600 ms at boot. Each core has one DMA vector,
run at the most urgent priority its channels asked for, so the OSD
completion (asking for 6) runs at 2, level with SPI0, the IMU bus.

### How it works

The program does its own sync detection. Each field it raises IRQ flag 0,
waits for a sync low longer than about 10 us (a broad vsync pulse), then
counts 17 hsyncs, telling them from equalising and broad pulses by timing.
It loads the line count from ISR and for each line waits for hsync, delays
to the left edge and shifts 23 words out at 2 bits and 10 PIO clocks a
pixel: 368 pixels in 49 us, then `set pins, 0`. Sync still low at the left
edge is taken as vsync and restarts the field. The line loop is `pull
noblock`: an empty FIFO loads X instead of stalling and every line still
takes 23 words, so each missed word shifts the rest of the field 16 pixels.

PAL and NTSC differ in one instruction, word 19, the delay from hsync to the
first pixel (`set y, 26 [12]` against `set y, 25 [8]`); `skipHsyncs` is 16 in
both. The driver loads the PAL program, patches word 19 and seeds ISR with
287 or 233. `start_scanout()` counts field flags for 400 ms with scan-out
held off: 20 is PAL and 24 NTSC, split at 22, and under 8 means no camera,
so PAL stands. While disarmed `flush()` re-measures in one-second windows
(split at 55 Hz); two disagreeing windows make the core1 thread switch
standard, which blanks the overlay for a field or two.

A 12-pixel cell is 3 bytes by 18 lines, so a blit is a byte-aligned copy:
30 columns fill 90 of the 92 bytes in a line, with 16 rows on PAL (288
lines) and 13 on NTSC (234). Character codes are glyph indices.

The DMA sees a word stream with no line boundaries, so one transfer carries
a 9-line block (207 words, about 572 us) and the completion runs at about
1.6 kHz. Nine is half a cell, so a block never straddles two character rows
and `render_block()` hoists the row lookup; a `static_assert` keeps
`OSD_PICO_BLOCK_LINES` dividing 18. Three buffers form a queue, one with
the DMA and up to two rendered, giving the renderer about 1.1 ms for about
16 us of work: what it waits for is to be scheduled behind core1's
priority-181 threads, and with one block of slack about 1% of blocks were
late. `produced` and `consumed` have one writer each; each slot carries its
block number and a blank flag.

- `OSD_c1` claims the DMA channel and enables the PIO1 vector itself, as
  both land on the calling core's NVIC, then stays as the renderer, where
  the rate and SPI0 threads preempt it; the interrupts only arm and signal.
  Measured non-preemptible OSD time on core1 is about 0.8%.
- Only the renderer reads the character frame. A block whose cells are all
  transparent over that half-cell is flagged rather than rendered.
- `field_start()` clears TXSTALL, empties the FIFO by toggling `FJOIN_TX` so a
  short field cannot displace the next, and arms block 0; `block_complete()`
  arms the next, dropping queue heads whose tag is not the block due.
- Blank and late blocks go out as filler, one zero word read without
  increment for exactly `block_words()`, so the stream stays in step. An
  empty queue is a late block: `late_blocks` counts it and `resync_block`
  points the renderer past it.
- TXSTALL at a completion means the FIFO ran dry and the phase is gone: the
  renderer blanks the rest of the field and `desyncs` counts it once.
- `serve_interrupt()` clears `CTRL_TRIG` to its error bits before the
  callback, so the DMA mode is rewritten on every arm. `dmaChannelAllocI()`
  does not set the channel's INTE bit; `dmaChannelEnableInterruptX()` does.

The character frame is double-buffered. AP_OSD's thread, on core0, runs
`clear()`, the element draws and `flush()` at 10 Hz into `back`; `flush()`
swaps the pointers so the renderer sees only complete frames, then copies
`front` into `back`. The copy is for scripts, which call `draw_screen()`
(clear, draw, flush) and then `write()` and `flush()` with no clear; without
it their text lands on the frame before last.

`load_font_data()` takes `fontN.bin` from the microSD, then ROMFS, then font
0, and anything but 256 x 54 bytes fails init. Glyphs are read from that
copy through a 256-byte table: each MCM pair is mapped (00 black -> 01,
01 -> 00, 10 white -> 11, 11 -> 00) and the byte bit-reversed, since MCM puts
the leftmost pixel in the high pair and the PIO shifts right. The reversal
also swaps each pair's bits, which is why the map looks transposed. It is
Betaflight's `mcm2h.py` encoding, checked against `font0.bin`.

The font is not read from XIP flash. ROMFS is compressed here, so the copy
is on the heap; `HAL_ROMFS_UNCOMPRESSED` would save 13.8 KB but push up to
1.3 MB/s of core1 glyph reads through the 16 KB XIP cache core0 relies on.
If the RAM is needed, cache only the glyphs in use (typically 60-90, about
8 KB saved), filled from `flush()` on core0.

### What must not break

- The program is 31 of 32 slots: there is no room for a change.
- W and EN must be adjacent with W first (`out pins, 2` from one base);
  nothing checks `HAL_OSD_PICO_EN_PIN` is W + 1. All pins below GPIO32.
- `OSD_c1` must stay on core1 in `AP_THREAD_CORE_TABLE`. A board table
  without it silently moves both interrupts and the renderer to core0.
- The per-block path stays in SRAM: the 8-word FIFO is 17 us of slack and
  one XIP miss can exceed it. The filler word is in BSS, not const, so the
  DMA does not read it from XIP either.
- Drawing belongs in the thread: the completion shares priority 2 with the
  IMU's SPI DMA.
- Keep the clock a multiple of 75 MHz so the PIO divider stays an integer.
- A late block costs one block, a desync the rest of the field. Treating a
  late block as a desync blanks about 200 lines instead of 9.

A failed `init()` frees the DMA but PIO1 stays the OSD's, so NeoPixel stays
off until reboot. Font failures are sent as STATUSTEXT; PIO1, DMA and core1
failures only to `hal.console`. A core1 XIP park (2-4 ms in a flash write,
so disarmed only) costs the rest of a field as a desync. Longer blocks ride
out parks at no CPU cost; 18 lines is the next size that divides the cell
(2.3 ms deadline, 4968 bytes of buffers).

### Untested

PAL has never been run: the 288-line field, the PAL centring word and 50 Hz
detection are untested; NTSC is verified on a camera at 234 lines and
59.94 Hz. microSD throughput in flight with the OSD on is unmeasured.

### How to check it works

1. Set `OSD_TYPE` 6 and reboot. Expect "PIO1: OSD", and on the console
   `OSD: pico NTSC on PIO1 sm0, 234 lines, 24 fields in 400ms` (PAL: 288
   lines, about 20 fields; 0 fields means no camera yet, PAL assumed).
   `OSD_TYPE` 5 should bring back "PIO1: NeoPixel" and the J2 LED.
2. With camera and VTX the overlay should be steady and centred, and
   should follow a PAL/NTSC camera swap within seconds while disarmed.
3. Build with `AP_RP2350_DEBUG_REPORT_ENABLED 1` for an `OSD: fields= late=
   blank= desync=` line every 10 s: fields about 599 on NTSC or 500 on PAL,
   late at most a few, desync only alongside XIP parks, blank high. The
   counters are `AP_OSD_pico::vsync_count`, `late_blocks`, `desyncs` and
   `blank_blocks`, readable over SWD.
4. With the OSD running, the SPI `rxoverruns`, `aborts`, `spi_late_count`
   and `internal_errors` should stay 0, and rate-loop `RTDT` dtMax no worse than
   with `OSD_TYPE` 5 (1.164 ms on the bench; no OSD-off baseline yet).

## Board notes

What a developer needs per board beyond its `README.md`. The clock, flash
timing and core split are common to all three and described above.

### RPI_UAVFC

The board that flies, and the source of every flight measurement here. Do
not cross-reference Laurel for pins: only clock, QMI and SMP are shared.

Pin data comes from the R2 Rev C schematic in the vendor support pack,
which is not in the tree, and it wins over the vendor GPIO assignment
sheet. The sheet lists the ESC channels ascending, but the connector is
wired descending, DSHOT1 on GPIO9 to DSHOT4 on GPIO6, so PWM1-2 sit on slice
4 and PWM3-4 on slice 3. It also names an ICM42688P and DPS310 where the
schematic has an ICM-56686 and DPS368. Its regulator enables are right for
the final revision, GPIO18 for 5 V and GPIO19 for 9 V; an earlier revision
had them swapped and is not supported. SWD is on dedicated pins. Left
unbound: GPIO27 (IMU CLKIN), GPIO12-14 (spare pads), and GPIO5 (ESC
telemetry) and GPIO41 (SBUS), which can only reach UART1 RX, owned by the
GPS; a third PIO UART would land on PIO1, which the OSD and NeoPixel use.

The IMU is either part. The schematic shows a TDK ICM-56686, early samples
carry an ICM42688P, and both have flown. The device is named `imu1` and
Invensensev3 probes both WHOAMI registers, so no hwdef change is needed. The
ICM-56686 is not a rebadged ICM-45686: its user bank from `PWR_MGMT0` up is
the 45686 map shifted by 4, applied through `reg456()`; its device ID is
0x08; and it powers up big-endian, so the driver clears that bit in
`SREG_CTRL` before the first FIFO read. The FIFO packet is the same 20-bit
format, so the parser is shared, and the part runs at 32 g and 4000 dps.
Its ODR is 3.2 kHz where the ICM42688P gives 4 kHz, so `FSTRATE_DIV` 2
gives a 1.6 kHz rate loop rather than 2 kHz. `INS_ACC_ID` / 65536 tells
them apart: 0x34 is the ICM42688P, 0x44 the ICM-56686 (datasheet
DS-000563). Leave `HAL_INS_ALLOW_NO_SENSORS` unset: a failed probe then
substitutes flat data with no gravity term, which once cost a day on
Laurel. The DPS368 runs on the DPS310 backend, with the same product ID.

Orientation is two rotations and both are real. The hwdef's
`ROTATION_PITCH_180` is chip to board; `AHRS_ORIENTATION` is the user's,
and the airframes flown mount the board inverted with 8, a net
`ROTATION_YAW_180`. Do not fold them into one. Check against the gyro:
nose up, right side down and clockwise yaw from above all read positive.

`defaults.parm`, and why:

- `SCHED_LOOP_RATE` 200, `INS_GYRO_RATE` 2, `INS_FAST_SAMPLE` 1,
  `FSTRATE_ENABLE` 2, `FSTRATE_DIV` 2: 200 Hz main loop on core0, rate loop
  at half the gyro rate on core1, divisor fixed once armed.
- `SERVO_DSHOT_RATE` 1 multiplies the rate-loop rate, not
  `SCHED_LOOP_RATE`, making the DShot cycle timeout one rate-loop period.
- `EK3_IMU_MASK` 1: one IMU, one EKF core, which keeps the DCM roll/pitch
  consistency pre-arm check live.
- `LOG_FILE_RATEMAX` 100. The limiter caches its decision per message per
  scheduler tick, so at 200 Hz it gates whole ticks: 100 passes every
  second tick, about 507 Hz for the rate-loop streams, 67 every third, and
  200 or more does nothing. Drop to 67 if `DSF.Dp` climbs in flight.
- `SERVO5_FUNCTION` 120, as GPIO2 only reaches the J2 LED connector;
  `RELAY2_FUNCTION`/`RELAY3_FUNCTION` 1, as there is no hwdef define.
- `BRD_OPTIONS` 5 (watchdog; internal parameters writable for the GCS baro
  calibration), `BRD_IDLE_STATS` 1 (per-core load in `threads.txt`),
  `SCHED_OPTIONS` 1 (`tasks.txt` filled on the first FTP fetch).
- Unset on purpose: orientation, frame, compass, notch, battery scales and
  `FS_THR_VALUE`, which belong to the airframe. There is no DCM decimation
  setting any more.

In the hwdef, `AP_STORAGE_NO_WRITE_WHILE_ARMED` matters most: a boot-flash
write parks core1 and every motor output with it, and gaps of up to
17.6 ms were measured during boot-time writes. `HAL_HNF_MAX_FILTERS` 64 is
for the per-motor triple notch.

Flight state: flown on two 5 inch quads, one per IMU part, the second an
iFlight Nazgul Evoque F5 on 6S, in Stabilize, Loiter and Acro at 800 deg/s
including inverted, for up to about 270 s, on bidirectional DShot600 with an
ESC-RPM notch. The notch has to be per-motor, as the four motors spread over
164-212 Hz, and takes about 34 dB out of the motor band. Armed,
`RTDT.dtMax` never exceeded 1.7 ms against the 500 us period. Gains are
hand-tuned on the first airframe, 8:1 thrust to weight, where stock gains
limit-cycle; no AUTOTUNE has completed. Its 17% diagonal RPM split is yaw
trim, not the output chain: equal commands give RPM within 2%.

Logging for tuning: the stock `LOG_BITMASK` logs `RATE` at 10 Hz, which
aliases. Use 442367 (stock plus `ATTITUDE_FAST`, bit 0, and `IMU_FAST`, bit
18) with `INS_LOG_BAT_MASK` 3 and `INS_LOG_BAT_OPT` 4. The card cannot keep
up with that offer, least of all in position modes, so take spectra from
`ISBH`/`ISBD`: with one IMU, `ISBH` instance 0 is pre-filter and 1
post-filter, and pooling them gives a plausible wrong answer. Check a block
has all 32 `ISBD` records. Batch gyro is scaled for 2000 dps even on the
4000 dps ICM-56686, so a hard manoeuvre clips the log, not the sensor.
`DSF.Dp` is cumulative per file, a few hundred of it from the boot format
dump.

The current input does not give usable amps. The ESC's CUR line reaches
GPIO47 through 120 ohm, with 100 nF and an 82.5k pull-down at the pin, so a
driven input is 1:1; `HAL_BATT_CURR_SCALE` 50 assumes 20 mV/A. The first
airframe's ESC showed no response to load. On the Evoque F5 the pin tracks
changes (r 0.95-0.97 on 0.5 s differences, about 14.6 mV/A), but its gain
moved 26% between two flights a day apart, and its zero follows the flight
controller's die temperature, not pack voltage: +28.4 mV/degC against
`MCU.MTemp`, R2 0.996 over 32-45 degC with motors stopped. A 0.5-0.6 V
standing offset with motors stopped shows with two different ESCs. The
leading explanation, unproven, is a few microamps of temperature-dependent
leakage into a node whose only DC return is the 82.5k; a 10k pull-down
would cut it eightfold. Treat the pin as a relative indicator and keep
`BATT_FS_VOLTSRC` 0: at 1 the resting-voltage estimate inherits the offset,
reads 0.7-1.5 V high, and the critical failsafe fires late. `BATT_MONITOR`
25 does not work on a multirotor, as nothing sets the `k_throttle` output it
reads. The fallback is a Lua `BATT_MONITOR` 29 model from eRPM,
`I = 0.5 + 9.0e-4 * SUM (RPM/1000)^3` amps, fitted on the Evoque F5.
`HAL_BATT_VOLT_SCALE` 11.1 is measured against a bench supply.

VTX power: GPIO18 (5 V, `RELAY2`, GPIO 81) and GPIO19 (9 V to the VID
connector, `RELAY3`, GPIO 82) each drive an MP4334 EN pin with a pull-down,
so both are active high. Both are `OUTPUT HIGH` with `RELAYn_DEFAULT` 1, on
from boot; the bootloader holds them low. `RELAY3` has not been seen to
switch the 9 V rail. Meter GPIO19 and the EN pin while toggling it before
concluding anything about polarity; `MAV_CMD_DO_SET_RELAY` counts from 0,
so `RELAY3` is instance 2.

### Laurel

The original RP2350B carrier: a different pinout, an ICM42688P on SPI0 at
`ROTATION_NONE`, a DPS310 on I2C0, and SPI1 shared between the microSD and
an AT7456E footprint, of which the hwdef takes the microSD. Pins are marked
verified against the Laurel schematic. Not autobuilt. It has been airborne
only in Stabilize early in bring-up, on firmware that ran the EKF on core1
and before parameters persisted; none of that applies now, and no flight on
the current architecture is recorded. `RESET`/SW2 is the RUN pin and
`BOOT`/SW1 pulls the `QSPI_SS` strap, so holding BOOT through a reset enters
the ROM USB bootloader; neither is a firmware GPIO.

It lags RPI_UAVFC in ways that matter before it flies again:

- `defaults.parm` still bakes in airframe choices: frame, a fixed notch and
  `GPS_DRV_OPTIONS` 4.
- Thread statistics are on. On RPI_UAVFC they cost 10.2% of core0 and
  13.6% of core1 non-idle time.
- `HAL_OSD_TYPE_DEFAULT` 1 with no SPI OSD declared; DisplayPort on SERIAL1
  needs `OSD_TYPE` 5.
- The 9 V rail (GPIO15) is held low with no `GPIO(n)` or relay, so a VTX on
  J5 cannot be powered without a hwdef edit.
- Current scale is a 1.0 placeholder; voltage is from one 5.9 V reading.
- Serial LED is off: the LED pin is above GPIO31, and the NeoPixel PIO
  block runs at GPIOBASE 0.
- RX-only pads GPIO36 (SBUS) and GPIO37 (ESC telemetry) are undeclared, as
  the RP2350 serial path needs a TX and RX pair.
- Parameters share 64 KB erase block 0 with the bootloader; RPI_UAVFC moved
  them to block 1.
- The watchdog lockup test passed on RPI_UAVFC has not been run here.

### Pico2

A bare Pico 2 or Pico 2 W (RP2350A, GPIO0-29) running Simulation on Hardware
through `include/SimOnHW.inc`, with `AHRS_EKF_TYPE` 10, `GPS1_TYPE` 100, a
quad frame and accel offsets that pass the calibration check, so nothing
need be attached but USB. It runs the flight boards' clock, flash timing and
SMP setup, which makes it the place for scheduler, core1 and timing work.
Keep `SIM_RATE_HZ` equal to `SCHED_LOOP_RATE`, as the physics steps once per
main loop. `FSTRATE_ENABLE` defaults to 0 here, so core1's rate thread is
off until set. No DShot (eight outputs against the PIO driver's four), no
microSD or dataflash, so MAVLink logging is built in but off by default
(`LOG_BACKEND_TYPE` 0); PIO UART pins must be GPIO16 or above. A second Pico
2 running debugprobe is the SWD probe for all three boards. The same SoH
build on a flight board comes from
`Tools/scripts/sitl-on-hardware/sitl-on-hw.py --board RPI_UAVFC -v copter`.

## Known issues and open work

### Estimation and control

- Earlier builds ran the DCM backup at 1/2, then 1/16 rate
  (`AP_AHRS_DCM_BACKUP_DECIMATION` in `PICO2.py`). That was a correctness
  bug, not a trade-off: `AP_AHRS_DCM::matrix_update()` rotates by the INS's
  latest delta angle only, so skipped updates lose rotation, and
  `drift_correction()` adds one loop's `delta_t` per call, so the GPS
  acceleration term was scaled 16x. DCM walked past 100 deg of roll in acro,
  and with one EKF core its roll/pitch check locked out re-arming. The
  define is gone: DCM runs every AHRS update, costs about 1.5% of core0 and
  held 0.02 deg from EKF3 on the bench, disarmed. Open: no flight log since
  has been checked for DCM drift; compare `DCM` with `ATT`, and fly
  `AHRS_GPS_GAIN` 0 if it drifts. Decimating again would need delta angle
  and time accumulated across skipped calls. DCM cannot be built out:
  `COMPASS_CAL_ENABLED` depends on it.
- Not flown on any RP2350 board: RTL, Auto and the GPS failsafe paths.

### Logging and storage

- Log bandwidth falls with core0 load. With the ring saturated, drain
  capacity was 141-147 KB/s at 77% `PM.Load` against 195-198 KB/s at 65%,
  which dropped about 30,000 messages in 30 s of Loiter with
  `ATTITUDE_FAST` on. The IO thread has the lowest priority and each block
  is CPU work. The logger also syncs every 4 KB on this branch, as
  `AP_FATFS_MAX_IO_SIZE` needs #34476 (see microSD). Open: land #34476,
  measure what position control offers at the source, cut core0 load.
- Log files that came back named `0000012B.IN` for `00000012.BIN` (the
  directory entry read one byte late, so the attribute byte, 0x20, became
  the last extension character) are no longer seen. Which change fixed it
  was not isolated. The RP SPI abort and receive-overrun detection in
  ChibiOS (2026-09-07) fit the symptom, although measurements the next day
  found neither path taken in normal running; the single-exchange block
  write, ArduPilot/ChibiOS#115 with #34477, changed how directory sectors
  are written. If it recurs, check the build has both.
- The RP EFL driver reports success for a refused page program, so
  `stm32_flash_write()` reads back and retries. Whether programs ever need
  the retry is unknown; one counter on the mismatch answers it.
- `flash_func_erase_apparea_fast()` in the bootloader skips any 64 KB block
  whose first and last 4 KB pages read erased, so data in the middle
  survives. A full scan was reverted because it stalls USB. The last page
  is checked for a UF2's RP2350-E10 workaround page, 256 bytes of 0xEF
  addressed to 0x10ffff00, which wraps to the top of a 4 MB part: on a
  Pico 2 it survived the first-page check and failed
  `uploader.py --verify-firmware-is`. `uploader.py --force-erase` erases
  every block, which clears anything else. After an upload `GET_CRC` returns
  the CRC of the received bytes, not a readback; a session that uploaded
  nothing gets the readback.
- The XIP park runs one way, core0 parking core1, so every flash write must
  start on core0, as the storage thread does today.

### Peripherals and power

- Still to do on the RPI_UAVFC current input: parallel a known resistor at
  two values and two temperatures, ESC lead on and off, to measure the
  source impedance; check whether RP2350 erratum E9 applies to an analog pad
  with IE cleared (a lead, not a finding); ask the ESC vendor whether the
  shunt is fitted; try a `MCU.MTemp` zero correction in Lua.
- SmartAudio over a half-duplex PIO UART works on the bench and has not
  flown. The first VTX tried never replied, under Betaflight too.
- Bench-untested: UART inversion including PIO TX, GPIO (PPM) RC input, the
  SBUS-out refusal, FETtec over PIO, the 399 kHz microSD init clock.
  `rp2350_pioasm.py --check` is not in CI.

### Bring-up and release

- RPI_UAVFC's README references no image, which
  `Tools/scripts/test_new_boards.py` requires of a new board.
- 225 MHz at 1.15 V is 1.5 times the datasheet clock, validated only on
  bring-up samples. QMI timing is characterised for Laurel's flash part only.
- Until a board has soaked past the 71.6 minute `micros()` wrap while
  arming and disarming (`AP_RCOUT_USE_32BIT_TIME`), reboot before flying.
- After an SWD flash and OpenOCD detach, core1 sometimes never launches
  (PC 0x19e in the bootrom) and core0 stalls in the XIP lockout wait. A
  MAVLink reboot does not clear it, a power cycle does; the guess is debug
  state left set. Power-cycle after SWD.
- Disarmed, some boots report the rate thread at about 2 kHz and 40% load,
  others at 4 kHz and 0%, whatever the build. Unexplained.
- Bootloader binaries do not build reproducibly.
- Next board spin: microSD DAT1/DAT2 contiguous with DAT0 and pull-ups on
  DAT0-3 and CMD, the only route to 4-bit SD; ESC telemetry and SBUS on
  pins a free UART can reach; a stronger current-sense pull-down and a
  clamp, as a 5 V ESC output would push about 12 mA through the 120 ohm
  into the pad's ESD diode.

### Found in the code while writing these notes

None of these has been fixed or tested on hardware yet.

- When `c1_xip_lock_ready` is 0, `rpEflBeforeXipOff()` returns without
  setting the park start time, so `rpEflAfterXipOn()` records a stale
  park duration.
