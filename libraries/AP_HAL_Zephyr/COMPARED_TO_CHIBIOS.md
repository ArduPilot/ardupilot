# AP_HAL_Zephyr compared to AP_HAL_ChibiOS

If you know AP_HAL_ChibiOS, this is the document that tells you what's
different. Nobody arrives at the Zephyr backend as their first ArduPilot HAL,
so we're not going to explain what a HAL is.

Words used below mean exactly what they say:

* **verified** - run on real hardware, and we say which board and when
* **untested** - written and it builds, nobody has put it on a bench
* **partial** - works, with a named limitation
* **absent** - not implemented, falls back to the base class stub
* **n/a** - the hardware has no such thing

"Builds" is not "works". Most of what follows is only verified on
mr_vmu_rt1176, because that's where the hardware time went.

**Nothing here has flown.** Not one of these boards has been in the air. Every
"verified" below means bench work: a board on a desk, sensors streaming, a
receiver in someone's hand, a scope or an SWD probe on a pin. That is a real
limit on what any of it is worth, and it is the first thing to say rather than
something to find out later.

## Which boards actually do what

|                 | mr_vmu_rt1176 | CubeOrangeZephyr | ESP32S3Zephyr | native_sim |
| --------------- | ------------- | ---------------- | ------------- | ---------- |
| Boots and runs  | verified      | verified         | verified      | verified   |
| SPI + IMUs      | verified      | untested         | untested      | n/a        |
| Bus DMA         | verified      | absent           | absent        | n/a        |
| UART / GPS      | verified      | untested         | untested      | n/a        |
| USB CDC MAVLink | verified      | untested         | verified      | n/a        |
| RC in           | verified      | untested         | untested      | n/a        |
| PWM out         | verified      | untested         | absent        | absent     |
| Storage         | verified      | untested         | untested      | verified   |
| SD logging      | verified      | untested         | absent        | RAM disk   |
| CAN / DroneCAN  | verified      | absent           | absent        | loopback   |
| WiFi            | n/a           | n/a              | absent        | n/a        |
| Crash dump      | verified      | absent           | absent        | absent     |
| Bootloader      | verified      | absent           | absent        | n/a        |

**CubeOrangeZephyr boots and runs**, and `Tools/CPUInfo/output-CubeOrange.zephyr.txt`
is the artifact: that file is runtime output captured off the board, so the
scheduler runs, code executes and console output works. What has not been
exercised on it is the peripheral set - sensors, storage, CAN, PWM, RC - so
those rows are read off hwdef, Kconfig and devicetree rather than measured.

That board's `micros()` comes from TIM5, and it used to run short - with the
counter gated, `time_boot_ms` was measured advancing at 0.106x wall. TIM5 was
not mis-scaled: its Sleep-mode clock gate had been left clear, so the counter
stopped during WFI and `micros()` lost exactly the idle time, which is why the
error was never the same number twice. `hrt_init()` in `system.cpp` now sets
that bit explicitly rather than trusting its reset value. The full write-up is
the `AP_NO_WFI_IDLE` help text in `zephyr/Kconfig`; the H7 gate pair and why
the WFI veto stays on regardless are in
[ARCHITECTURAL.md](ARCHITECTURAL.md).

What has not been redone is a `Scheduler::delay()` measurement on that board
since the fix, so what it delivers now is not recorded anywhere. On rt1176,
where `micros()` comes from the kernel cycle counter, `delay()` delivers
102-106% of the requested time.

**DMA is an RT1176 story only.** No `dmas` property exists anywhere in the
CubeOrangeZephyr, ESP32-S3 or ESP32-C6 devicetrees, and
`CONFIG_UART_ASYNC_API=y` appears only in `prj.mr_vmu_rt1176.conf`. On the
other boards every bus is interrupt-driven.

We are not going to quote a single loop rate. It has ranged from 190 Hz to
about 605 Hz across builds on the same board depending on what was resident in
fast memory, and any number without a firmware hash beside it is folklore.

## Buses

| Feature           | ChibiOS                                            | Zephyr                            | Notes                                                                                                                                                                                                                       |
| ----------------- | -------------------------------------------------- | --------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| SPI master        | own STM32 driver                                   | verified, rt1176                  | On Zephyr's portable `spi` API. Both forms of `transfer_fullduplex()` must be overridden. Overriding only the 3-arg form leaves 2-arg calls failing silently, which accumulates transfer failures and makes boot very slow. |
| SPI DMA           | `Shared_DMA` + `spiStartExchangeI`                 | verified rt1176, absent elsewhere | `CONFIG_SPI_NXP_LPSPI_DMA=y` plus `dmas` on all three lpspi nodes. 150 s soak with 3 SPI + 3 I2C + 7 UART DMA running together.                                                                                             |
| I2C               | own driver                                         | verified, rt1176                  | BMP388 streaming; INA228 read 12.90 V against a 13.1 V bench supply.                                                                                                                                                        |
| I2C DMA           | yes                                                | verified rt1176, absent elsewhere | All three LPI2Cs, about 2 IRQ per transfer, per-bus fallback to the IRQ path on error.                                                                                                                                      |
| UART              | own driver                                         | verified, rt1176                  | u-blox GPS on LPUART3, register state read back over SWD, survives a real power cycle.                                                                                                                                      |
| UART DMA          | `dma_tx_allocate`                                  | verified rt1176, absent elsewhere | All 7 LPUARTs on Zephyr's async API, one IRQ per chunk instead of per byte. USB CDC deliberately stays interrupt-driven.                                                                                                    |
| UART flow control | software RTS bit-bang                              | verified, one port                | Real hardware flow control here, no register-poke workaround needed. Only reachable on the one port with RTS and CTS in pinctrl.                                                                                            |
| UART port options | full set                                           | partial                           | `OPTION_RXINV` and `OPTION_TXINV` only. Everything else reports unsupported rather than lying.                                                                                                                              |
| Shared DMA broker | `Shared_DMA`, runtime borrow/evict, `@SYS/dma.txt` | absent                            | Every channel is a fixed devicetree binding owned by one peripheral at compile time. No contention stats exist. See "enough channels is not fair arbitration" below.                                                        |
| Wide/Octal SPI    | QUADSPI/OCTOSPI                                    | n/a on all five                   | Aliases the `AP_HAL_Empty` stub.                                                                                                                                                                                            |

## GPIO and RC

| Feature                       | ChibiOS                         | Zephyr                                     | Notes                                                                                                                                                                                                                                  |
| ----------------------------- | ------------------------------- | ------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| GPIO core                     | full                            | verified                                   | Portable Zephyr GPIO API, board-agnostic.                                                                                                                                                                                              |
| ISR-flood protection          | 10k IRQ/100 ms then disable pin | untested                                   | Ported with ChibiOS's exact figure, plus a real `arming_checks()`. `AP_Arming` had been calling the always-pass base stub. Never tested against a real flood.                                                                          |
| `wait_pin()`                  | yes                             | absent                                     | Only caller repo-wide is the ADIS1647x family.                                                                                                                                                                                         |
| `get_mode()` / `set_mode()`   | yes                             | absent                                     | Silently defeats Spektrum DSM bind: `start_bind()` bails with no log.                                                                                                                                                                  |
| RC in, serial protocols       | `AP_RCProtocol` UART scan       | untested                                   | The shared scan is wired and runs every boot on rt1176. No serial receiver has been on the bench. Needed real `configure_parity()` and `set_stop_bits()` first; they had been no-op stubs, so `serial_configs[]` could only vary baud. |
| RC in, PPM/CPPM               | timer capture + DMA             | verified rt1176, untested CubeOrangeZephyr | Hardware timer capture through the XBARA1 crossbar. Per-channel stdev 0.1-2.2 us against 120-375 us for the software-timestamp path. Stick tracking and failsafe confirmed by a human at the transmitter.                              |
| Same-pin UART/PPM arbitration | separate pins                   | verified, rt1176                           | That board's RC connector feeds one net, so a runtime pad-MUX arbiter switches the pad between roles and latches to whichever wins.                                                                                                    |

## RC output

| Feature                     | ChibiOS                            | Zephyr                                                       | Notes                                                                                                                                                                                                                                                                                                           |
| --------------------------- | ---------------------------------- | ------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| PWM                         | full                               | verified rt1176, untested CubeOrangeZephyr and C6, absent S3 | Servo sweeps on CH1/4/8/11, one per FlexPWM instance. The rt1176 servo rail is not board-powered; you need an external BEC.                                                                                                                                                                                     |
| Channel grouping            | `pwm_group`, `set_freq_group()`    | absent                                                       | `_freq_hz[]` is per channel with no grouping. Harmless today because every rt1176 channel owns a FlexPWM submodule, but a remap putting two channels on one submodule would silently give a wrong rate.                                                                                                         |
| `cork()` / `push()`         | atomic shadow-array commit         | partial                                                      | `write()` updates the live array regardless of corking. Multi-motor mixing can show per-channel skew.                                                                                                                                                                                                           |
| DShot                       | timer + DMA, all rates             | partial, rt1176, not electrically verified                   | FlexIO driver carried in-tree, CH1-8 switch between FlexPWM and FlexIO at runtime. **Bitrate is fixed at DShot600** by the devicetree `speed` property whatever `MOT_PWM_TYPE` says, switching is bank-granular, `SERVO_DSHOT_RATE` is not honoured. OneShot and Brushed are accepted and behave as plain PWM. |
| Bidirectional DShot         | 875 lines, feeds the dynamic notch | absent                                                       | That in-tree driver already does the physical layer. What's missing is AP-side: no channel enables it, `Zephyr::RCOutput` doesn't inherit `AP_ESC_Telem_Backend`.                                                                                                                                               |
| BLHeli passthrough          | `RCOutput_serial.cpp`              | absent                                                       | All five methods are base stubs.                                                                                                                                                                                                                                                                                |
| Serial LEDs via `hal.rcout` | yes                                | absent                                                       | A separate `AP_Notify` LED-strip backend is hardware-verified on C6, but that is not this API.                                                                                                                                                                                                                  |
| Safety switch               | real                               | absent                                                       | `force_safety_on()` returns false unconditionally. Moot on rt1176, which has no such pin. A genuine gap on CubeOrangeZephyr, whose hwdef claims one.                                                                                                                                                            |
| Optional virtuals           | ~30 overridden                     | absent                                                       | We override the 9 pure virtuals plus `set_output_mode`, `get_output_mode`, `cork`, `push`. Everything else resolves to the base class default.                                                                                                                                                                  |

## Sensors, storage, logging

| Feature                | ChibiOS                                         | Zephyr                                                | Notes                                                                                                                                                                                                                                                                                                              |
| ---------------------- | ----------------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| AnalogIn               | circular-DMA capture, 16 pins, per-pin dividers | partial rt1176, absent CubeOrangeZephyr, n/a on ESP32 | Each read is a fresh synchronous single-shot on the calling thread, no filtering, capped at 4 pins with one global scale. `board_voltage()` returns a hardcoded 5.0 V and `mcu_temperature()` 25.0 C, both self-documented. On CubeOrangeZephyr it cannot work at all: `init()` only looks for NXP `lpadc` labels. |
| Storage / parameters   | FRAM or internal flash                          | verified rt1176, untested elsewhere                   | Different model from ChibiOS: backends are probed in order - board flash on RT1176 ROM-API builds, then ZMS, then FRAM over raw SPI, then a file on FAT - and the first that initialises wins.                                                                                                                                              |
| Deferred write-behind  | dirty bitmask, drained by a tick                | verified in service, rt1176                           | Before this, every `PARAM_SET` including in flight ran a synchronous flash commit, sector erase included, on the calling thread.                                                                                                                                                                                   |
| `Storage::healthy()`   | false with no backend                           | verified fixed                                        | It used to return true unconditionally, which made the "Param storage failed" prearm structurally dead. A vehicle could arm with zero persistent storage. The failure is silent: the code runs and appears to succeed, and the only signal is a `printk` on the console.                                           |
| SD + dataflash logging | full                                            | verified rt1176, untested CubeOrangeZephyr, absent S3  | Uses **Zephyr's** FatFs, not ArduPilot's, because only one `ff.c` may link. `f_mount()` takes a volume name here, not a number. Costs roughly 10 Hz of loop rate while logging. CubeOrangeZephyr has the same stack switched on since 2026-09-11 - `HAL_OS_FATFS_IO` and `HAL_LOGGING_FILESYSTEM_ENABLED` in its hwdef, `CONFIG_DISK_ACCESS` / `CONFIG_DISK_DRIVER_SDMMC` / `CONFIG_SDMMC_STM32` / `CONFIG_FAT_FILESYSTEM_ELM` in `prj.CubeOrangeZephyr.conf`, and SDMMC1 on a 48 MHz PLL2_R kernel clock in its `.dts` - and it writes a dataflash log under Renode. Nothing on a bench. |
| DSP / gyro FFT         | hand-tuned CMSIS radix-8                        | partial rt1176, not validated with real vibration     | Goes through the public `arm_rfft_fast_f32()`; ChibiOS's internal radix-8 shortcuts are `static` in this CMSIS-DSP tree and won't link. ARM-only by construction. The rest of the harmonic notch is unaffected: fixed, throttle and RPM modes don't need `HAL_WITH_DSP`.                                           |
| ExternalAHRS           | full backend set                                | off by default                                        | `AP_EXTERNAL_AHRS_ENABLED` defaults to 0 in the Zephyr board header and no Zephyr board turns it back on. A hwdef can, but none of the backends have been exercised on this HAL.|

## System services

| Feature                                          | ChibiOS                               | Zephyr                                             | Notes                                                                                                                                                                                                                                                                                                                                                                |
| ------------------------------------------------ | ------------------------------------- | -------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CAN + DroneCAN                                   | `CANIface.cpp`                        | verified rt1176; enabled CubeOrange, no wire test                  | Both buses, two live peripherals by name and unique ID, 423k and 73k frames, zero bus errors, autopilot on the wire as node 10. Three never-exercised bugs had to go first, including an ISR-illegal `k_mutex` dropping every RX frame at the first hop. `native_sim` runs CAN loopback, so it's the one place the DroneCAN stack regression-tests without hardware. |
| CAN-FD                                           | `CANFDIface.cpp`                      | absent                                             | `send()` hard-rejects over 8 bytes; no `init(bitrate, fdbitrate)` overload.                                                                                                                                                                                                                                                                                          |
| CAN RX filtering                                 | message-RAM filters                   | absent                                             | One wildcard pass-all filter. A regression against either ChibiOS backend.                                                                                                                                                                                                                                                                                           |
| USB CDC                                          | `SerialUSB`                           | verified rt1176, S3, C6; untested CubeOrangeZephyr | S3's test included a 75 s-late host attach.                                                                                                                                                                                                                                                                                                                          |
| Second CDC (SMP/mcumgr)                          | n/a                                   | verified, rt1176                                   | 1.4 MB image to slot 1 in 88 s with MAVLink streaming concurrently. Standard mcumgr tooling works.                                                                                                                                                                                                                                                                   |
| Hardware watchdog                                | IWDG + escalating monitor diagnostics | partial                                            | Verified armed and not false-triggering on rt1176, 2 s, fed every 100 ms. Missing: the escalating 200/500/1800 ms diagnostics, and honouring `AP_BoardConfig::watchdog_enabled()` at runtime rather than arming from Kconfig.                                                                                                                                        |
| Software watchdog                                | pat right after `loop()` returns      | verified after a fix                               | The pat used to be issued unconditionally from the timer thread, so the staleness check tracked "is the timer thread alive", not "is the main loop alive".                                                                                                                                                                                                           |
| Crash dump                                       | CrashCatcher to a flash region        | verified end to end, rt1176                        | Built on **Zephyr's** coredump subsystem. CrashCatcher's capture engine is hand-written ARMv7-M assembly and cannot go to Xtensa or RISC-V at all. Injected fault, 1371 B written from fault context, watchdog reset, reboot, prearm raised, dump fetched over MAVFTP. No dump-clear mechanism yet, so the prearm persists until the partition is erased.            |
| Reset forensics                                  | RTC backup registers                  | partial                                            | A `.noinit` buffer survives a real watchdog reset and the `WDG:` statustext works. `fault_addr`, `icsr`, `lr`, `line`, `type` are never populated because there's no equivalent of ChibiOS's `save_fault_watchdog()`. Everything shared AP code writes into the struct is covered.                                                                                   |
| RTC                                              | backup-domain, survives power-off     | untested rt1176, absent elsewhere                  | Worth knowing on any HAL: `BRD_RTC_TYPES` defaults to GPS-only, so the boot-restore path is masked with default parameters on ChibiOS too.                                                                                                                                                                                                                           |
| Board unique ID                                  | STM32 UDID                            | verified                                           | One `hwinfo_get_device_id()` covers NXP OCOTP, the STM32 UDID and the ESP32 eFuse.                                                                                                                                                                                                                                                                                   |
| `thread_info()` / `@SYS/threads.txt`             | yes                                   | verified                                           | Emits the same `ThreadsV2` format, so ChibiOS parsers and the same MAVProxy workflow work unchanged. Opt in with `--enable-stats`.                                                                                                                                                                                                                                   |
| `mem_info()`                                     | yes                                   | verified CubeOrange                                | ChibiOS's `MemInfoV1` format, all six H743 RAM banks at ChibiOS's addresses and `TYPE` codes. Verified against the same physical CubeOrange under both HALs. |
| `uart_info()`, `timer_info()`, `dma_info()`      | yes                                   | verified CubeOrange                                | `@SYS/uarts.txt`, `timers.txt`, `dma.txt` in ChibiOS's `UARTV1`/`TIMERV1`/`DMAV1` formats, column for column. `dma.txt`'s lock-contention columns are structurally zero on Zephyr: streams are bound in the devicetree, so nothing arbitrates and there is nothing to count. |
| `log_stack_info()`                               | yes                                   | absent                                             | No `STAK` message. `thread_info()` gives a snapshot; the missing part is the trend that catches a slow stack leak before it overflows. |
| `malloc_type()` DMA-safe alloc                   | region table                          | verified                                           | Matters more than it looks. The `AP_HAL` base default is plain `calloc()`, so without this override every `MEM_DMA_SAFE` allocation, Invensense v3 FIFO buffer included, is ordinary heap, and Zephyr's LPSPI DMA driver does no cache maintenance.                                                                                                                  |
| Tone alarm                                       | yes                                   | absent                                             | Moot on rt1176, no buzzer pin. A live gap on CubeOrangeZephyr, which has the same buzzer a ChibiOS CubeOrange has.                                                                                                                                                                                                                                                   |
| Lua scripting                                    | full                                  | untested                                           | Builds on rt1176, 36 Lua symbols in the ELF. No script has been run. Off by default.                                                                                                                                                                                                                                                                                 |
| IOMCU                                            | `AP_IOMCU` over UART                  | absent                                             | `HAL_WITH_IO_MCU 0` is an unconditional `#define`, not an `#ifndef` default. n/a on rt1176, no co-processor fitted. A real gap on CubeOrangeZephyr, which has the same STM32F103 every CubeOrange has. Its hwdef carries `IOMCU_UART` and `ROMFS` lines that are inert: `zephyr_hwdef.py` parses neither.                                                            |
| WiFi                                             | n/a                                   | verified C6, absent S3                             | softAP, TCP 5760 and broadcast UDP 14550, auto-placed into empty SERIAL slots. One open defect: the SSID vanishes after minutes while the vehicle and console keep running, with the interface still reporting up. On S3 the stack overflows DRAM by 90,588 B so the board opts out.                                                                                 |
| Bootloader                                       | `AP_Bootloader`                       | verified, rt1176                                   | A Zephyr port of AP_Bootloader is resident and exercised by every upload. Also verified: SNVS fast reboot matching `board_get/set_rtc_signature()`, and reboot-to-bootloader by MAVLink.                                                                                                                                                                             |
| MCUboot A/B                                      | n/a                                   | verified, rt1176                                   | Overwrite-only A/B with SHA-256 slot validation.                                                                                                                                                                                                                                                                                                                     |
| In-app bootloader update                         | `flash_bootloader()`                  | verified both directions, rt1176                   | Pacing decides whether this works: unpaced back-to-back erase and program starved the watchdog feeder and reset the SoC mid-flash. Fixed by writing 4 KB slices with real sleeps.                                                                                                                                                                                               |
| Fast rate thread                                 | supported                             | verified rt1176 and C6                             | Runs above the main loop and genuinely competes for CPU, same as ChibiOS.                                                                                                                                                                                                                                                                                            |
| Semaphores                                       | mutex + binary sem                    | verified                                           | `k_mutex` / `k_sem`, including `signal_ISR()`.                                                                                                                                                                                                                                                                                                                       |

## What will surprise you

These came out of things that went wrong here. Three are read off the code
rather than measured on a board: the compass default, what `HAL_OS_FATFS_IO`
gates, and the DMA cache section.

### Priority numbers run backwards, and the ladder is by ChibiOS rank

ChibiOS treats a **larger** priority number as more urgent. Zephyr's
preemptible band treats a **smaller** number as more urgent.
`Scheduler::_zephyr_priority()` therefore computes `prio = base - offset` where
ChibiOS's `calculate_thread_priority()` adds, and that sign flip happens in
exactly one place. A caller's `+1` still means "one step more urgent" on both
HALs; a raw ChibiOS *number* copied across inverts the thread it lands on.

The ladder itself is a rank-for-rank translation of ChibiOS's, not a different
design. Most urgent first, as `Scheduler.h` defines it:

| Zephyr | who runs there | ChibiOS |
| ------ | ------------------------------------------------------------- | ------- |
| 0  | monitor                                                            | 183 |
| 1  | main while boosted in the INS wait; the fast rate thread           | 182 |
| 2  | timer, rcout, SPI bus threads, unbuffered UART                     | 181 |
| 3  | **main - the flight loop**                                         | 180 |
| 4  | free - the one unused level between main and CAN (main-1, CAN+1)   | 179 |
| 5  | CAN                                                                | 178 |
| 6  | rcin                                                               | 177 |
| 7  | I2C bus threads                                                    | 176 |
| 8  | net+1 - lwip's tcpip thread, on a board that builds lwip           | 61  |
| 9  | LED, UART and net; Zephyr's log-process and mcumgr threads         | 60  |
| 10 | storage, and `log_io`                                              | 59  |
| 11 | the HAL's io thread, and `thread_create(PRIORITY_IO)` callers      | 58  |
| 12 | io-1 user threads                                                  | 57  |
| 13 | main while `setup()` runs                                          | 10  |
| 14 | scripting, and the floor for every user thread                     | 2   |
| 15 | Zephyr's idle thread                                               | 1   |

So SPI sits above the loop and CAN, rcin, I2C, storage and io sit below it,
which is ChibiOS's order.

It did not always read that way. io sat above main here on purpose, because the
loop never yielded and nothing drained the MAVLink send queue:
`check_called_boost()` had been ported to this HAL and then never called, so
main gave up time only when something else happened to block it. ChibiOS gives
up 50 us after every `loop()` on exactly that condition. That yield is back -
`AP_SCHEDULER_LOOP_YIELD_US` in `HAL_Zephyr_Class.cpp`, 50 us by default and
overridable by a board.

The yield size and the io level are coupled, and it cost six emulated flights
to find that out. Raising the yield to 160 us hangs the board outright while io
sits above main, and buys nothing once it is below. Change one and re-check the
other. The comment at `AP_SCHEDULER_LOOP_YIELD_US` is still written as though
io were above main, and is due a refresh.

A board whose loop leaves no headroom gets an escape hatch rather than a fork,
and `Scheduler.h` is precise about how many: two. `APM_STORAGE_PRIORITY` and
`APM_IO_PRIORITY` each set a marker inside their own `#ifndef`, and each
guards one order assert on that marker, so a `define` in `hwdef.dat` moves
storage or io and still compiles. The boost and the SPI, CAN, rcin and I2C
levels are `#ifndef` too, but their asserts are unconditional - a hwdef moving
SPI off level 2 does not build at all. And the hatch is not free: there is no
level between main (3) and timer (2), so a lifted thread lands on the timer and
SPI level, where its work then delays IMU sampling. ChibiOS never does that.

Get a priority wrong and nothing faults - it hangs, with no output at all.
[DEBUGGING.md](DEBUGGING.md) describes that failure mode and the one it is
easily confused with. What catches it here is the `static_assert` block in
`Scheduler.h`: it asserts order and range, never numbers, and one of them ties
`APM_MAIN_PRIORITY` to `CONFIG_MAIN_THREAD_PRIORITY` because Zephyr creates the
main thread from Kconfig and would otherwise win silently.

One kernel behaviour has no ChibiOS equivalent, and it presents as a priority
that will not stay set. `k_mutex_lock()` snapshots the owner's priority and
`k_mutex_unlock()` writes that snapshot back, so a priority change made while a
`HAL_Semaphore` is held is undone at the `give()`. `Semaphore::give()` re-asserts
main's intended level for that reason, and holds main up while a more urgent
thread is pending on a semaphore main owns - which is what `chMtxUnlock()` does
for free. See [ARCHITECTURAL.md](ARCHITECTURAL.md).

Read the comment block in `Scheduler.h` before changing any number in it.

### On a work-limited board, a thread below the main loop gets almost nothing

Not "gets less CPU". A thread below main is fed by the INS wait and by that
50 us yield, and by nothing else - on this HAL and on ChibiOS alike. When the
loop saturates, that is the whole budget: 50 us out of the 8 ms period of a
125 Hz board is 0.6%, against the 2% the same 50 us buys ChibiOS at 400 Hz. At
that setting `AP_Logger`'s io callback once did not run at all - it reported
`stuck thread ()`, and the empty parentheses are `last_io_operation`, never
set. That particular starvation was later traced to AP_Logger's own `log_io`
thread rather than to the io level, and the emulated flight does write its
dataflash log today with io below main. The budget it lives on is still 0.6%.

It is not only your own threads. The RTOS has service threads too,
and they starve the same way: Zephyr's sockets-service dispatcher carries the
DHCP server's receive path, so a DHCP server below the main loop accepts client
DISCOVERs and answers none of them. Storage threads, WiFi service pumps and the
console behave the same.

Audit every service thread's priority against the main loop, yours and the
RTOS's.

### Devicetree owns what hwdef.dat owns

Peripheral enables and clocks live in devicetree, not `hwdef.dat`. Our
`hwdef.dat` is a much smaller file than the ChibiOS one: sensor probing, bus
order, the serial map, and pin mux.

Pin mux is the exception, and it used to be devicetree-only. `PIN` lines in
`hwdef.dat` are collected by `Tools/ardupilotwaf/zephyr_hwdef.py` and turned
into pinctrl groups in a generated overlay by
`Tools/ardupilotwaf/zephyr_class_generator.py`, which has per-family backends
for i.MX RT, STM32 and ESP32-S3. Three boards use it today - `mr_vmu_rt1176`
has 75 `PIN` lines, `CubeOrangeZephyr` 42, `ESP32S3Zephyr` 7 - so a pin change
on those boards is a `hwdef.dat` edit, as it would be on ChibiOS. The
hand-written board DTS is not edited by the generator.

The corollary: **a devicetree edit is never the expensive part of anything.** If a capability is gated behind a devicetree property, add the
property. SPI DMA on this board needed about four lines of `dmas` in a file we
own: the Zephyr driver defaults to DMA and only falls back to the CPU path when
the devicetree declares no channels.

### `status = "okay"` is not enough to get a driver

A node can be enabled in devicetree and still have no driver bound, because the
driver also needs its Kconfig symbol. The symptom is `device_is_ready()`
returning false on a node you can see in the generated devicetree. Check both.

### The compass backends are off unless a hwdef asks for one

`AP_COMPASS_BACKEND_DEFAULT_ENABLED` is 0 for every Zephyr board
(`libraries/AP_HAL/board/zephyr.h`). It is an `#ifndef` a hwdef can raise, but
by default the only backends compiled in are the ones a `COMPASS` line in
`hwdef.dat` names - that line is what generates the per-driver
`AP_COMPASS_<part>_ENABLED`. So a compass that is on the bus and never detected
is not a probe fault, a bus fault or a clock fault: the driver is not in the
firmware at all. A symbol count settles it in seconds.

```text
nm -C build/<board>/zephyr_build/zephyr/zephyr.elf | grep -c IST8310
```

A CubeOrangeZephyr build has dozens of AK09916 symbols - the backend its hwdef
names - and zero for IST8310, LIS3MDL, QMC5883L and RM3100.

### `HAL_OS_FATFS_IO` and the SD card are two switches, not one

On ChibiOS that define is the SD switch. Here it is one of two, and they do not
line up.

`HAL_OS_FATFS_IO` gates more than flight logs. `AP_Filesystem_config.h` defines
`AP_FILESYSTEM_FATFS_ENABLED` straight from it, so at 0 the whole AP_Filesystem
FATFS backend goes - MAVFTP file access with it - and all of
`AP_HAL_Zephyr/sdcard.cpp` compiles to nothing, which is where
`disk_access_init()` and `f_mount()` are called from.

What it does not gate is `Storage::_try_file_mount()`. That is compiled in on
`CONFIG_FAT_FILESYSTEM_ELM` and mounts the card for parameter storage through
Zephyr's own `fs_mount()`, needing nothing from `sdcard.cpp` - but only when no
earlier backend answered. ZMS and FRAM are tested first, and board flash joins
that test only on the RT1176's ROM-API build. CubeOrangeZephyr declares a
`fram0` node, so on a board whose FRAM comes up the parameter path never
reaches the card.

Check both before concluding a board is off the card.

### Config that applies cleanly can be completely inert

`prj*.conf` fragments are merged into
`build/<board>/zephyr_build/ardupilot_prj_autogen.conf`. If your symbol isn't in
that file, Kconfig never saw it, and nothing will tell you, because
incremental builds never re-parse Kconfig.

### Some build dependency edges don't exist

Three of them, all silent:

* `hwdef.h` is generated by `configure`, and no AP source declares a dependency
  on it. Edit `hwdef.dat`, rebuild without reconfiguring, and your objects keep
  the old contents. Added `IMU` probe lines can be absent from the firmware
  entirely, and the link still succeeds because nothing references them.
* AP sources get Kconfig through `-imacros autoconf.h`, which waf's header
  scanner does not follow. Flip a `CONFIG_AP_*` symbol that C++ reads and the
  already-compiled objects keep the old value.
* The mavgen task declares no outputs, so deleting generated headers can never
  trigger regeneration.

### Console output is synchronous per character

A `printk` on a stalled console blocks the calling thread for roughly 87 us per
byte. Put one in a flight-critical path and you have moved the problem, not
found it.

### Enough DMA channels is not fair DMA arbitration

Having a channel per peripheral does not mean they share bandwidth sensibly.
There is no `Shared_DMA` equivalent here and no contention statistics, so a
starved bus shows up as a symptom somewhere else entirely.

### Bus DMA on CubeOrangeZephyr needs an MPU region before it needs `dmas`

The `absent` in that board's Bus DMA row is not only a missing devicetree
property. `CONFIG_ARM_MPU` is unset there, and on Cortex-M
`ARCH_HAS_NOCACHE_MEMORY_SUPPORT` is selected only when the MPU is on
(`modules/zephyr/arch/arm/core/Kconfig`), so with `CONFIG_DCACHE=y` nothing on
that board can be marked non-cacheable. The `MEM_DMA_SAFE` pool in `Util.cpp`
falls to its non-`__nocache` branch and is ordinary cached RAM; the comment on
that branch says as much. Bounce buffers still get an explicit flush or
invalidate (`bouncebuffer.cpp`), but the combined full-duplex scratch path in
`SPIDevice.cpp` and the UART DMA buffers get neither - both assume the pool
really is non-cacheable, which is true on the RT1176 and not here.

Adding `dmas` and `CONFIG_DMA=y` without an MPU region would not corrupt
quietly, though. The STM32 drivers test the buffer and refuse a cached one:
`spi_stm32.c` returns `-ENOTSUP` and `uart_stm32.c` `-EFAULT`, each with a log
line naming the reason. Every DMA transfer would fail and nothing would move.

Nor is the gap dormant. That board already drives SDMMC's internal DMA into
cached RAM, and `sdmmc_stm32.c` does its own cache maintenance around each
transfer to make it safe - which is also why `CONFIG_FS_FATFS_WINDOW_ALIGNMENT`
is set to the cache line size in `prj.CubeOrangeZephyr.conf`.

### Execute-in-place is a tax you have to measure

On a board that runs code from external flash, where a function sits matters
more than what it does. The single biggest loop-rate win on rt1176 came from
moving the kernel context-switch path into instruction tightly-coupled memory,
not from making anything faster. Non-halting PC sampling is how you find it;
see [DEBUGGING.md](DEBUGGING.md).

## What we haven't tested

Being blunt about this is more useful than a table full of green ticks.

* **CubeOrangeZephyr's peripherals.** The board boots and runs, but nothing
  beyond that has been exercised on hardware.
* **`Scheduler::delay()` on CubeOrangeZephyr since the TIM5 sleep-gate fix.**
  The cause is known and fixed in `hrt_init()`; nobody has re-measured what the
  board delivers now.
* FRAM storage on any board.
* DShot electrically. The mode round-trip works; the pad switch is inconclusive
  by servo alone and needs a scope or a mux-register read.
* SBUS or CRSF lock-on. The autodetect scan runs; no receiver has been on it.
* Watchdog on anything except rt1176.
* SNVS clock persistence across a real power cycle.
* Storage retry and re-init escalation, which is build-verified only.
* Lua at runtime.
* eDMA fair arbitration under sustained multi-bus load.
* **The UART RX re-arm defect, and the fix for it.** `_rx_need_restart`
  (`UARTDriver.h`, `UARTDriver.cpp`) is set from the async callback whenever RX
  reports itself disabled - including when `_rx_timer_tick()` disabled it
  deliberately one line earlier - so on a board using the async API every wired
  UART tears RX down and rebuilds it at the 1 kHz timer rate. The fix designed
  for it, a flag around the deliberate teardown, has never been written or run,
  and landing it changes UART behaviour on silicon and not only under the
  emulator. Carried as an open item in [PARITY_DETAIL.md](PARITY_DETAIL.md).
