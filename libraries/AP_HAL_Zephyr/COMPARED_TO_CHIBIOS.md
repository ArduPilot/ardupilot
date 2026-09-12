# AP_HAL_Zephyr compared to AP_HAL_ChibiOS

If you know AP_HAL_ChibiOS, this is the document that tells you what's
different. Nobody arrives at the Zephyr backend as their first ArduPilot HAL,
so we're not going to explain what a HAL is.

Words used below mean exactly what they say:

- **verified** - run on real hardware, and we say which board and when
- **untested** - written and it builds, nobody has put it on a bench
- **partial** - works, with a named limitation
- **absent** - not implemented, falls back to the base class stub
- **n/a** - the hardware has no such thing

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
| SD logging      | verified      | absent           | absent        | RAM disk   |
| CAN / DroneCAN  | verified      | absent           | absent        | loopback   |
| WiFi            | n/a           | n/a              | absent        | n/a        |
| Crash dump      | verified      | absent           | absent        | absent     |
| Bootloader      | verified      | absent           | absent        | n/a        |

**CubeOrangeZephyr boots and runs**, and `Tools/CPUInfo/output-CubeOrange.zephyr.txt`
is the artifact: that file is runtime output captured off the board, so the
scheduler runs, code executes and console output works. What has not been
exercised on it is the peripheral set - sensors, storage, CAN, PWM, RC - so
those rows are read off hwdef, Kconfig and devicetree rather than measured.

That board also has one open bug you need to know about before trusting any
timing from it. `micros()` there comes from TIM5, and `Scheduler::delay()`
measures about **68x short**, while the identical code on rt1176, where
`micros()` comes from the kernel cycle counter, delivers 102-106% of the
requested time. TIM5 being mis-scaled is a hypothesis with supporting evidence,
not a demonstrated fact: the cross-check probe was built and never run. Until
it is, treat every `micros()`-derived number from CubeOrangeZephyr as
unverified, including anything that looks like a win over ChibiOS.

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
| DShot                       | timer + DMA, all rates             | partial, rt1176, not electrically verified                   | FlexIO driver vendored in-tree, CH1-8 switch between FlexPWM and FlexIO at runtime. **Bitrate is fixed at DShot600** by the devicetree `speed` property whatever `MOT_PWM_TYPE` says, switching is bank-granular, `SERVO_DSHOT_RATE` is not honoured. OneShot and Brushed are accepted and behave as plain PWM. |
| Bidirectional DShot         | 875 lines, feeds the dynamic notch | absent                                                       | The vendored driver already does the physical layer. What's missing is AP-side: no channel enables it, `Zephyr::RCOutput` doesn't inherit `AP_ESC_Telem_Backend`.                                                                                                                                               |
| BLHeli passthrough          | `RCOutput_serial.cpp`              | absent                                                       | All five methods are base stubs.                                                                                                                                                                                                                                                                                |
| Serial LEDs via `hal.rcout` | yes                                | absent                                                       | A separate `AP_Notify` LED-strip backend is hardware-verified on C6, but that is not this API.                                                                                                                                                                                                                  |
| Safety switch               | real                               | absent                                                       | `force_safety_on()` returns false unconditionally. Moot on rt1176, which has no such pin. A genuine gap on CubeOrangeZephyr, whose hwdef claims one.                                                                                                                                                            |
| Optional virtuals           | ~30 overridden                     | absent                                                       | We override the 9 pure virtuals plus `set_output_mode`, `get_output_mode`, `cork`, `push`. Everything else resolves to the base class default.                                                                                                                                                                  |

## Sensors, storage, logging

| Feature                | ChibiOS                                         | Zephyr                                                | Notes                                                                                                                                                                                                                                                                                                              |
| ---------------------- | ----------------------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| AnalogIn               | circular-DMA capture, 16 pins, per-pin dividers | partial rt1176, absent CubeOrangeZephyr, n/a on ESP32 | Each read is a fresh synchronous single-shot on the calling thread, no filtering, capped at 4 pins with one global scale. `board_voltage()` returns a hardcoded 5.0 V and `mcu_temperature()` 25.0 C, both self-documented. On CubeOrangeZephyr it cannot work at all: `init()` only looks for NXP `lpadc` labels. |
| Storage / parameters   | FRAM or internal flash                          | verified rt1176, untested elsewhere                   | Different model from ChibiOS: four backends are probed in order, board flash then ZMS then FRAM over raw SPI then a file on FAT, and the first that initialises wins.                                                                                                                                              |
| Deferred write-behind  | dirty bitmask, drained by a tick                | verified in service, rt1176                           | Before this, every `PARAM_SET` including in flight ran a synchronous flash commit, sector erase included, on the calling thread.                                                                                                                                                                                   |
| `Storage::healthy()`   | false with no backend                           | verified fixed                                        | It used to return true unconditionally, which made the "Param storage failed" prearm structurally dead. A vehicle could arm with zero persistent storage. The failure is silent: the code runs and appears to succeed, and the only signal is a `printk` on the console.                                           |
| SD + dataflash logging | full                                            | verified rt1176, absent elsewhere                     | Uses **Zephyr's** FatFs, not ArduPilot's, because only one `ff.c` may link. `f_mount()` takes a volume name here, not a number. Costs roughly 10 Hz of loop rate while logging. CubeOrangeZephyr's SDMMC driver hangs during Zephyr device init, so SD and FatFS are both off there.                               |
| DSP / gyro FFT         | hand-tuned CMSIS radix-8                        | partial rt1176, not validated with real vibration     | Goes through the public `arm_rfft_fast_f32()`; ChibiOS's internal radix-8 shortcuts are `static` in this CMSIS-DSP tree and won't link. ARM-only by construction. The rest of the harmonic notch is unaffected: fixed, throttle and RPM modes don't need `HAL_WITH_DSP`.                                           |
| ExternalAHRS           | full backend set                                | off by default                                        | `AP_EXTERNAL_AHRS_ENABLED` defaults to 0 in the Zephyr board header and no Zephyr board turns it back on. A hwdef can, but none of the backends have been exercised on this HAL.|

## System services

| Feature                                          | ChibiOS                               | Zephyr                                             | Notes                                                                                                                                                                                                                                                                                                                                                                |
| ------------------------------------------------ | ------------------------------------- | -------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CAN + DroneCAN                                   | `CANIface.cpp`                        | verified rt1176, absent elsewhere                  | Both buses, two live peripherals by name and unique ID, 423k and 73k frames, zero bus errors, autopilot on the wire as node 10. Three never-exercised bugs had to go first, including an ISR-illegal `k_mutex` dropping every RX frame at the first hop. `native_sim` runs CAN loopback, so it's the one place the DroneCAN stack regression-tests without hardware. |
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
| `mem_info()`                                     | yes                                   | untested                                           | Implemented in ChibiOS's `MemInfoV1` format.                                                                                                                                                                                                                                                                                                                         |
| `dma_info()`, `timer_info()`, `log_stack_info()` | yes                                   | absent                                             | No `@SYS/dma.txt`, no `@SYS/timers.txt`, no `STAK` message. `thread_info()` gives a snapshot; the missing part is the trend that catches a slow stack leak before it overflows.                                                                                                                                                                                      |
| `malloc_type()` DMA-safe alloc                   | region table                          | verified                                           | Matters more than it looks. The `AP_HAL` base default is plain `calloc()`, so without this override every `MEM_DMA_SAFE` allocation, Invensense v3 FIFO buffer included, is ordinary heap, and Zephyr's LPSPI DMA driver does no cache maintenance.                                                                                                                  |
| Tone alarm                                       | yes                                   | absent                                             | Moot on rt1176, no buzzer pin. A live gap on CubeOrangeZephyr, which has the same buzzer a ChibiOS CubeOrange has.                                                                                                                                                                                                                                                   |
| Lua scripting                                    | full                                  | untested                                           | Builds on rt1176, 36 Lua symbols in the ELF. No script has been run. Off by default.                                                                                                                                                                                                                                                                                 |
| IOMCU                                            | `AP_IOMCU` over UART                  | absent                                             | `HAL_WITH_IO_MCU 0` is an unconditional `#define`, not an `#ifndef` default. n/a on rt1176, no co-processor fitted. A real gap on CubeOrangeZephyr, which has the same STM32F103 every CubeOrange has. Its hwdef carries `IOMCU_UART` and `ROMFS` lines that are inert: `zephyr_hwdef.py` parses neither.                                                            |
| WiFi                                             | n/a                                   | verified C6, absent S3                             | softAP, TCP 5760 and broadcast UDP 14550, auto-placed into empty SERIAL slots. One open defect: the SSID vanishes after minutes while the vehicle and console keep running, with the interface still reporting up. On S3 the stack overflows DRAM by 90,588 B so the board opts out.                                                                                 |
| Bootloader                                       | `AP_Bootloader`                       | verified, rt1176                                   | A Zephyr port of AP_Bootloader is resident and exercised by every upload. Also verified: SNVS fast reboot matching `board_get/set_rtc_signature()`, and reboot-to-bootloader by MAVLink.                                                                                                                                                                             |
| MCUboot A/B                                      | n/a                                   | verified, rt1176                                   | Overwrite-only A/B with SHA-256 slot validation.                                                                                                                                                                                                                                                                                                                     |
| In-app bootloader update                         | `flash_bootloader()`                  | verified both directions, rt1176                   | Pacing is load-bearing: unpaced back-to-back erase and program starved the watchdog feeder and reset the SoC mid-flash. Fixed by writing 4 KB slices with real sleeps.                                                                                                                                                                                               |
| Fast rate thread                                 | supported                             | verified rt1176 and C6                             | Runs above the main loop and genuinely competes for CPU, same as ChibiOS.                                                                                                                                                                                                                                                                                            |
| Semaphores                                       | mutex + binary sem                    | verified                                           | `k_mutex` / `k_sem`, including `signal_ISR()`.                                                                                                                                                                                                                                                                                                                       |

## What will surprise you

Every one of these is a measured incident, not a caution.

### Priority numbers run backwards, and our ordering isn't ChibiOS's

Two traps stacked.

ChibiOS treats a **larger** priority number as more urgent. Zephyr's
preemptible range treats a **smaller** number as more urgent. Copy a ChibiOS
value across rather than a ChibiOS ordering and you invert the priority: it
takes seven levels of inversion to hold the flight loop at 20 Hz. `Scheduler.h`
carries `static_assert`s on the relationships so an edit that breaks the
ordering fails to compile.

The second trap is that our ordering is not ChibiOS's, on purpose. Our main
loop sits **below** the rcin, rcout and io threads; ChibiOS puts main above
them. ChibiOS's ordering does not work here: ArduPilot's vehicle loop runs at
effectively 100% CPU, so a main thread above the io thread never yields to it
and nothing drains the MAVLink send queue. The symptom is boot stalling after
`Init Gyro` with no heartbeat. SPI bus threads do stay above main, as in
ChibiOS, so IMU samples still preempt the loop.

Read the comment block in `Scheduler.h` before changing any number in it.

### On a work-limited board, every thread below the main loop is dead

Not "gets less CPU". Zero, forever, if the loop never sleeps.

It is not only your own threads. The RTOS has service threads too,
and they starve the same way: Zephyr's sockets-service dispatcher carries the
DHCP server's receive path, so a DHCP server below the main loop accepts client
DISCOVERs and answers none of them. Storage threads, WiFi service pumps and the
console behave the same.

Audit every service thread's priority against the main loop, yours and the
RTOS's.

### Devicetree owns what hwdef.dat owns

Pin mux, peripheral enables and clocks live in devicetree, not `hwdef.dat`. Our
`hwdef.dat` is a much smaller file than the ChibiOS one and only covers sensor
probing, bus order and the serial map.

The corollary: **a devicetree edit is never the expensive part of anything.** If a capability is gated behind a devicetree property, add the
property. SPI DMA on this board needed about four lines of `dmas` in a file we
own: the Zephyr driver defaults to DMA and only falls back to the CPU path when
the devicetree declares no channels.

### `status = "okay"` is not enough to get a driver

A node can be enabled in devicetree and still have no driver bound, because the
driver also needs its Kconfig symbol. The symptom is `device_is_ready()`
returning false on a node you can see in the generated devicetree. Check both.

### Config that applies cleanly can be completely inert

`prj*.conf` fragments are merged into
`build/<board>/zephyr_build/ardupilot_prj_autogen.conf`. If your symbol isn't in
that file, Kconfig never saw it, and nothing will tell you, because
incremental builds never re-parse Kconfig.

### Some build dependency edges don't exist

Three of them, all silent:

- `hwdef.h` is generated by `configure`, and no AP source declares a dependency
  on it. Edit `hwdef.dat`, rebuild without reconfiguring, and your objects keep
  the old contents. Added `IMU` probe lines can be absent from the firmware
  entirely, and the link still succeeds because nothing references them.
- AP sources get Kconfig through `-imacros autoconf.h`, which waf's header
  scanner does not follow. Flip a `CONFIG_AP_*` symbol that C++ reads and the
  already-compiled objects keep the old value.
- The mavgen task declares no outputs, so deleting generated headers can never
  trigger regeneration.

### Console output is synchronous per character

A `printk` on a stalled console blocks the calling thread for roughly 87 us per
byte. Put one in a flight-critical path and you have moved the problem, not
found it. This is why `ship.conf` exists.

### Enough DMA channels is not fair DMA arbitration

Having a channel per peripheral does not mean they share bandwidth sensibly.
There is no `Shared_DMA` equivalent here and no contention statistics, so a
starved bus shows up as a symptom somewhere else entirely.

### Execute-in-place is a tax you have to measure

On a board that runs code from external flash, where a function sits matters
more than what it does. The single biggest loop-rate win on rt1176 came from
moving the kernel context-switch path into instruction tightly-coupled memory,
not from making anything faster. Non-halting PC sampling is how you find it;
see [DEBUGGING.md](DEBUGGING.md).

## What we haven't tested

Being blunt about this is more useful than a table full of green ticks.

- **CubeOrangeZephyr's peripherals.** The board boots and runs, but nothing
  beyond that has been exercised on hardware.
- **Whether TIM5 is really the cause of the 68x `delay()` error** on
  CubeOrangeZephyr. The cross-check probe exists and has never been run.
- FRAM storage on any board.
- DShot electrically. The mode round-trip works; the pad switch is inconclusive
  by servo alone and needs a scope or a mux-register read.
- SBUS or CRSF lock-on. The autodetect scan runs; no receiver has been on it.
- Watchdog on anything except rt1176.
- SNVS clock persistence across a real power cycle.
- Storage retry and re-init escalation, which is build-verified only.
- Lua at runtime.
- eDMA fair arbitration under sustained multi-bus load.
