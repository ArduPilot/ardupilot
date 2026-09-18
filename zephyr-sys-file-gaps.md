# `@SYS` files: AP_HAL_Zephyr against AP_HAL_ChibiOS

Every `@SYS` file comes from one table, `sysfs_file_list[]` in
`libraries/AP_Filesystem/AP_Filesystem_Sys.cpp`, which is HAL-agnostic. A file
is missing on a HAL for one of three reasons: the `hal.util->*_info()` method
that fills it is not overridden (the base class is a no-op, the result is
empty, and `open()` turns an empty result into `ENOENT`); a compile-time macro
keeps the name out of the table; or the board has no such hardware.

Determined from source and probed over MAVFTP against CubeOrange hardware
running CubeOrangeZephyr. Where the two disagree that is called out rather
than resolved in favour of either.

Status as of 2026-09-18, after `c6d91933ec`, `4da7a36ce2` and `5e1ee3001b`.

## Now working, and comparable with ChibiOS

| File | Notes |
|---|---|
| `memory.txt` | Seven regions, all six H743 RAM banks at ChibiOS's addresses and `TYPE` codes. See `zephyr-cubeorange-memory.txt` and `chibios-cubeorange-memory.txt` for the two captures from the same board. |
| `uarts.txt` | `UARTV1`, ChibiOS's exact field set and `OTG1`/`UART4` port tokens. Fixing this exposed two real bugs behind it: TX came from `stats.tx.update(0)`, always zero, so no port had ever reported a transmitted byte; RX fed buffer occupancy into a tracker that subtracts the previous value, which underflows as the reader drains. |
| `timers.txt` | `TIMERV1`. `TIM` is the real timer number, from the channel map - it used to read the first digit of a devicetree name that is `pwm` for every STM32 timer, so every row said `TIM0`. `CLK` and `FREQ` are the input clock and the post-prescaler counter rate, the two different clocks ChibiOS prints. |
| `dma.txt` | `DMAV1`, ChibiOS's columns, with zeros - **read the caveat below**. |
| `tasks.txt` | HAL-agnostic, serves on both. Only the 8-byte header without a stats-enabled build, where ChibiOS is populated in a normal build. |
| `flash.bin` | Enabled for Zephyr on STM32 only. `AP_Filesystem_Sys.cpp` hardcodes `0x08000000`, which is right for an STM32 and wrong for the RT1176's external NOR at `0x30000000`, so this follows the SoC rather than the HAL. |

### The caveat on `dma.txt`

`CONT=0.0%` does **not** mean the same thing on the two HALs, and they print
identically. ChibiOS's `ULCK`/`CLCK`/`CONT` count `Shared_DMA` lock
contention, which exists because peripherals compete for one stream. Zephyr
binds a stream to a peripheral in the devicetree at build time, so nothing
arbitrates and nothing can be counted. On Zephyr the zero means "contention is
structurally impossible"; on ChibiOS it means "measured, and none found".

`TX` is a real transaction count, and covers the UART async path only - SPI and
I2C DMA runs inside the Zephyr drivers, which keep no counters this HAL can
read, so those streams do not appear rather than appear as zero.

## Switchable per board, but nothing to serve yet

| File | Notes |
|---|---|
| `can0_stats.txt`, `can1_stats.txt` | CAN is now enabled on CubeOrangeZephyr and `HAL_NUM_CAN_IFACES` is derived from the hwdef CAN pins, so the names are in the table. They stay absent at default parameters, and that is **correct**: `AP_CANManager` only allocates `hal.can[i]` once `CAN_P1_DRIVER` is non-zero, and a ChibiOS board behaves the same. Untested with traffic on a wire. |
| `crash_dump.bin` | The generator now emits the `AP_CRASHDUMP_FLASH_ENABLED 0` scaffolding, as `chibios_hwdef.py` does, so a hwdef can raise it - and ChibiOS CubeOrange has it at 0 too. It will still serve nothing on CubeOrangeZephyr: the Zephyr crash-dump backing is `CONFIG_AP_RT1176_ROMAPI_FLASH`-only, so `last_crash_dump_size()` returns 0. On the RT1176 it would serve. |

## Still missing

| File | Why |
|---|---|
| `persistent.parm` | Two independent blockers. `Util::load_persistent_params()` is not overridden, **and** the name only enters the table under `#if !defined(HAL_BOOTLOADER_BUILD) && (defined(STM32F7) \|\| defined(STM32H7))` - ChibiOS hwdef macros the Zephyr build never defines. Implementing the method alone would not make the file appear. |

## Unresolved - implemented but not serving

Both are implemented in AP_HAL_Zephyr and should serve. They did not on the
bench, and that is not yet explained. Listed here so nobody records them as
working on the strength of the source alone.

| File | What happened |
|---|---|
| `threads.txt` | `Util::thread_info()` is implemented and emits the same `ThreadsV2` header ChibiOS does. Over MAVFTP it returned no content on 5 attempts, at both 10 s and 15 s timeouts - so not a timeout. Worth checking whether the read fails or `thread_info()` produces nothing. |
| `storage.bin` | `Storage::get_storage_ptr()` returns a real pointer and size. The fetch failed on every attempt. |

## Method note

MAVFTP's returned error code is not usable as evidence: `memory.txt` reports
`error_code=4 (InvalidSession)` on the same call that successfully transfers
the file, so the field reflects session state rather than the read. The only
reliable test is whether bytes landed on disk.

## Overlap

`libraries/AP_HAL_Zephyr/COMPARED_TO_CHIBIOS.md`'s "System services" table
carries the same status for `mem_info()`, `uart_info()`, `timer_info()` and
`dma_info()`; keep the two in step.
