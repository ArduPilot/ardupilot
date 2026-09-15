# Debugging ArduPilot on Zephyr

Most of this is about getting information off a board that has stopped telling
you anything useful. The tools live in `Tools/zephyr/zephyr_*` and
`Tools/renode/`, and you will not find them by accident, so the inventory is
first. `ls Tools/zephyr/zephyr_*` lists almost all of them; a count written
here would go stale the next time somebody adds one. One helper named below,
`Tools/zephyr/rt1176_linkserver_flash.py`, is named for the board it came from
and does not match that glob.

Nearly all of them carry a full docstring saying why they exist. Read the file
before running it, particularly anything that writes flash.

## Where the console is

| Board            | Console                                   |
| ---------------- | ----------------------------------------- |
| mr_vmu_rt1176    | USB CDC, plus LPUART1 on the debug header |
| CubeOrangeZephyr | UART                                      |
| ESP32S3Zephyr    | USB CDC                                   |
| native_sim       | stdout                                    |

## The tools

### Getting firmware onto a board

| Tool                                       | What it does                                                                                                                                                                                                                                                                                                                                                                          | Needs          |
| ------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------- |
| `zephyr_upload_app.py`                     | Wraps `uploader.py` with the port pinned to one board's `/dev/serial/by-id/` **glob**. Use this whenever more than one ArduPilot board is attached: a bare `uploader.py` fires MAVLink reboot bytes at every matching port, including someone else's board. The pin has to be a glob so `uploader.py` still catches the `-BL` identity when the board re-enumerates mid-upload.       | serial         |
| `zephyr_flash.sh`                          | Retrying flasher. Repeated upload attempts, a hardware reset fired into the retry window, then a check that the board came back as the *application* and not the bootloader.                                                                                                                                                                                                          | serial + probe |
| `zephyr_flash_and_measure.sh`              | flash, wait-ready, transfer histogram, sysinfo, unattended. Refuses to flash a `.apj` older than the ELF it should have come from.                                                                                                                                                                                                                                                    | serial + probe |
| `zephyr_install_ap_bootloader.py`          | Installs AP_Bootloader over a resident PX4 bootloader via SWD, no BOOT0 strap. Checks the image fits the 128 KB slot, has its `FCFB` tag at +0x400 and a sane vector pair, backs up the existing bootloader first, and programs without a mass erase so the app slot survives. **Cannot** upgrade an existing AP_Bootloader; use the in-app `MAV_CMD_FLASH_BOOTLOADER` path for that. | probe          |
| `zephyr_smp_upload.py`                     | Uploads an MCUboot `.img` over mcumgr to the second CDC interface. Needs `pip install smpclient`.                                                                                                                                                                                                                                                                                     | serial         |
| `zephyr_pin_reset.py`                      | Pulses nRST through the probe with zero DAP transactions. For when the board is wedged *and* took the debug port with it, so `pyocd reset` cannot even connect.                                                                                                                                                                                                                       | probe          |
| `zephyr_pin_reset_bmp.py`                  | The same reset on a Black Magic Probe bench. pyOCD only sees CMSIS-DAP probes, so on a BMP `pyocd list` reports no probes at all and `zephyr_pin_reset.py` cannot run. This speaks the GDB remote-serial protocol straight down the probe's first CDC interface - `swdp_scan` then `reset` - so it needs no gdb binary and no toolchain.                          | probe          |
| `zephyr_make_sample_into_apj_flashable.py` | Builds any upstream Zephyr sample for one of our boards and packages it as a flashable `.apj`. Takes ArduPilot out of the picture entirely to answer "does this board boot at all, does this UART work at all".                                                                                                                                                                       | host + serial  |

### Getting output off a board

| Tool                     | What it does                                                                                                                                                                                                                                                                                                                                                                                     | Needs  |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------ |
| `zephyr_bootlog.py`      | Polls for the CDC device node every 20 ms and opens it the instant it enumerates, so the *first* boot after a flash is captured. Nothing on the board buffers the banner. Start it before the reset, not after.                                                                                                                                                                                  | serial |
| `zephyr_uart_capture.py` | Generic capture to a timestamped file. Deliberately has no `ttyACM` default: a bare device number silently talks to the wrong board after any re-enumeration.                                                                                                                                                                                                                                    | serial |
| `zephyr_cdc_gaps.py`     | Reports gaps in emitted traffic above a threshold. An independent liveness check that shares nothing with the SWD counters, so if SWD says 0 Hz *and* the byte stream has a matching silence, the board really stalled. Watch the observer effect: attaching a reader also drains the CDC buffer, and a board that is healthy with a reader attached and stalls without one is itself a finding. | serial |
| `zephyr_sysfiles_ftp.py` | Fetches `@SYS/threads.txt`, `tasks.txt` and `mem.txt` over MAVFTP, retrying each file up to four times because an `@SYS` transfer often fails the first attempt. Writes a provenance header carrying the firmware's git hash so a `--enable-stats` capture is never compared against a plain one. Takes a serial device or `tcp:127.0.0.1:5762`, so it works against Renode too.                     | serial |
| `zephyr_sysinfo.py`      | Reads the on-target `@SYS/threads.txt` and `@SYS/tasks.txt` out of `g_ap_sysinfo` over SWD. The way to get them when MAVFTP will not serve them.                                                                                                                                                                                                                                                 | probe  |
| `zephyr_sysinfo_gdb.py`  | The same two buffers over SWD through a Black Magic Probe and gdb, for benches pyOCD cannot see. The core is halted from attach to detach - up to about 0.6 s per read - so it refuses a board whose `g_ap_soft_armed` is set unless you pass `--force`. Reads twice and keeps the second, and retries if the firmware's sequence counter moved or was odd during the dump.                        | probe  |
| `zephyr_read_fatal.py`   | Reads the last fatal-error record over SWD, for when the fault handler wedged before it got its message out.                                                                                                                                                                                                                                                                                     | probe  |
| `zephyr_can_nodes.py`    | Enumerates DroneCAN nodes on both buses and fetches the per-bus stats files. Bench settings are hardcoded; read the header.                                                                                                                                                                                                                                                                      | serial |

### Measuring where the time goes

| Tool                        | What it does                                                                                                                                                                                                                                                                                                                                                                                                                             | Needs |
| --------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----- |
| `zephyr_pcsr_sample.py`     | Statistical PC profiler using DWT_PCSR: the debugger reads a recently-executed PC while the core keeps running. Histograms by symbol *and* by memory region, so ITCM code is distinguished from execute-in-place. The best single answer to "where is the CPU actually". Collect at least 2000 samples before believing a line: a handful of samples produces attributions that do not reproduce. `0xFFFFFFFF` means halted or sleeping. | probe |
| `zephyr_isr_composition.py` | Per-vector interrupt rates, read twice across a window and printed as interrupts per second. Vector names are parsed out of the SoC header's `IRQn` enum at runtime rather than hand-copied. For when the totals say "interrupts are the cost" but not which source.                                                                                                                                                                     | probe |
| `zephyr_chain_sample.py`    | Samples the whole `g_ap_prof` block in one transaction, so phase and counters come from the same instant. Attributes loop time to a pipeline stage: bus callback, SPI transfer, FIFO read, wait-for-sample, INS, EKF, AHRS.                                                                                                                                                                                                              | probe |
| `zephyr_xfer_hist.py`       | SPI transfer-duration histogram. A mean cannot separate the two explanations that need opposite fixes: a tight distribution means the cost is genuinely per transfer, a long tail or a bimodal shape means queueing.                                                                                                                                                                                                                     | probe |
| `zephyr_timeseries.py`      | Loop rate as a time series with a spread verdict. Exists because a rate can oscillate on a multi-second period, which makes any single 8-second A/B window a lottery ticket. Falls back to the ungated `g_ap_loop_count` so a shipping build stays measurable.                                                                                                                                                                           | probe |
| `zephyr_wait_ready.py`      | Polls until the board is actually in steady state: enumerated as the app not the bootloader, counters advancing, rate stable across two windows. Use instead of a blind `sleep`, which is wrong in both directions: too short and the sample covers gyro calibration rather than flight, too long and it wastes the run.                                                                                                                 | probe |

The four `g_ap_prof` tools need `CONFIG_AP_CHAIN_PROFILE=y`;
`zephyr_isr_composition.py` needs `CONFIG_AP_ISR_COUNT=y` plus `CONFIG_TRACING=y` and `CONFIG_TRACING_USER=y`.

### Reading a flight log

With default parameters dataflash logging starts at arming, so a `.bin`
describes the flight and says nothing about the boot. `LOG_DISARMED` 1 gets you
the disarmed period as well, which its own parameter text recommends for
startup problems, but not the part before the logger is up. That is not only an
inconvenience: the INS sample-rate estimate gets its fast convergence window
only while
`AP_HAL::millis64() < HAL_INS_CONVERGANCE_MS` and the vehicle is disarmed
(`sensors_converging()` in `AP_InertialSensor_Backend.cpp`; the constant is
30000 in `AP_InertialSensor.h`), so a board whose first IMU sample lands after
that window flies on the driver's compiled-in nominal rate, and the log only
opens afterwards. The boot has to be observed with a different instrument.

| Tool                                        | What it does                                                                                                                                                                                                                                                                                                                                                                                                                             | Needs          |
| ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------- |
| `Tools/zephyr/zephyr_ins_rate_probe.py`    | Compares two dataflash logs for the INS sample-rate fault. Four checks with their own verdicts: `IMU.GHz` slope per instance, the AP clock regressed against GPS time, accelerometer scale binned by tilt, and VIBE with sample-to-sample accel jumps for context. Read its docstring before reading its output - each check says what a pass and a fail look like.                                                                       | host           |
| `Tools/zephyr/zephyr_crash_report_data.py` | One pass over each log, producing every series the `mavlink-and-crash-analysis` skill asks for: events, errors, modes, attitude, VIBE, clipping, RCOU, EKF innovations and variances, magnetometer, `PM`, GPS and position. Every field is checked against the message's own fieldnames before use, so a missing field reports as absent instead of throwing.                                                                              | host           |
| `Tools/renode/zephyr_boot_timeline.py`      | Boots the firmware under Renode, attaches to the emulated UART immediately, requests all streams, and timestamps every STATUSTEXT, the first HEARTBEAT and the first IMU message against the board's own `SYSTEM_TIME.time_boot_ms` - the same `millis()` the convergence window uses. The first IMU message is an upper bound: it cannot arrive before MAVLink is up. The STATUSTEXT timeline is the part that says where the boot went. | host + Renode  |

### Build and repo helpers

| Tool                          | What it does                                                                                                                                                                  | Needs |
| ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----- |
| `zephyr_get_prerequisites.sh` | First-time setup. Run from the repo root.                                                                                                                                     | host  |
| `zephyr_get_sdk.sh`           | Installs the Zephyr SDK toolchain, with the version and a SHA-256 per artifact pinned in the script and a download that refuses to proceed if either fails to match. Deliberately not `west sdk install`: this repository vendors Zephyr as a submodule and does not use west, and `west sdk` is an extension command that only resolves inside a west workspace. | host  |
| `zephyr_dts_ours_only.py`     | Filters a generated `zephyr.dts` down to only what AP_HAL_Zephyr contributed, keeping the enclosing node structure. Saves reading thousands of lines of upstream SoC `.dtsi`. | host  |

The two LinkServer tools, `Tools/zephyr/rt1176_linkserver_flash.py` and
`Tools/zephyr/zephyr_install_ap_bootloader.py`, honour `$LINKSERVER` for the binary and
`$AP_PROBE_SERIAL` for the probe. Set the probe serial if more than one probe
is attached: an unpinned LinkServer will pick one for you and erase the wrong
target. The pyOCD-based tools do their own probe selection.

## Threads and tasks

`@SYS/threads.txt` and `@SYS/tasks.txt` are the first thing to look at for
"where is the time going". We emit the same `ThreadsV2` and `TasksV2` format
AP_HAL_ChibiOS does, on purpose, so the same parsers work and you can compare a
Zephyr capture against a ChibiOS one column for column.

Per-thread CPU and stack accounting is opt-in:

```sh
./waf configure --board=<board> --enable-stats
```

It adds a timestamp read to every context switch, so do not ship it. Fetch the
files over MAVFTP with `zephyr_sysfiles_ftp.py`, or read them straight out of
`g_ap_sysinfo` over SWD where MAVFTP will not serve them - `zephyr_sysinfo.py`
on a pyOCD bench, `zephyr_sysinfo_gdb.py` on a Black Magic Probe.

Never keep the first read after boot. It includes initialisation and tells you
about gyro calibration, not about flight.

## Build-time diagnostics

Six Kconfig symbols turn on instrumentation that costs real time:

| Symbol                       | What it adds                   |
| ---------------------------- | ------------------------------ |
| `CONFIG_AP_SPI_PROBE_DIAG`   | boot-time SPI bus scan         |
| `CONFIG_AP_I2C_PROBE_DIAG`   | boot-time I2C bus scan         |
| `CONFIG_AP_SCHED_TRACE`      | scheduler tracing              |
| `CONFIG_AP_DELAY_CB_PROFILE` | delay-callback profiling       |
| `CONFIG_AP_CHAIN_PROFILE`    | the `g_ap_prof` pipeline block |
| `CONFIG_AP_ISR_COUNT`        | per-vector interrupt counters  |

These are not free. The I2C probe scan has been measured at 80% of the CPU with
boot taking over 170 seconds, which makes any profile captured with it enabled
a measurement of the scan.

`./waf configure --ship` merges `ship.conf` last and forces all six off. That
file is the authoritative list, so add any new diagnostic symbol to it when you
create one. Crash dump capture and the storage backend deliberately stay on;
they are features, not diagnostics.

### Check the config actually took

```sh
grep <SYMBOL> build/<board>/zephyr_build/ardupilot_prj_autogen.conf
```

If your symbol is not in that file, Kconfig never saw it and nothing will tell
you, because incremental builds do not re-parse Kconfig.

### `HAL_SPI_CHECK_CLOCK_FREQ` does not measure SCK here

The macro exists in this HAL
(`SPIDevice::test_clock_freq()` in `libraries/AP_HAL_Zephyr/SPIDevice.cpp`,
called from `HAL_Zephyr_Class.cpp` when a hwdef defines it) and it is the same
bring-up check AP_HAL_ChibiOS carries under the same name, so the name invites
a comparison that does not hold.

It clocks 1024 bytes with no chip select asserted, times them with `micros()`,
and reports bits per second. On ChibiOS that is close to SCK because ChibiOS
transfers over DMA, so transfer time is roughly wire time. On a Zephyr board
with no `CONFIG_DMA` the SPI driver does programmed I/O and the CPU loop sets
the rate, so the number is per-transfer software overhead and not the wire at
all. The tell is free: run it at two requested frequencies. On CubeOrangeZephyr
a tenfold change in the requested clock moved the measured figure by about
0.1 %, which means the wire was never the limit. Treat the result as a floor on
the transfer period, and get SCK from a register decode or a scope.

That is still worth knowing, because the throughput ceiling it does measure is
real and applies to IMU FIFO reads on hardware. On CubeOrangeZephyr it came out
at 1-4 Mbit/s per bus, not the 8-20 MHz requested: 1024 bytes at 1.36 Mbit/s is
about 6041 us where a register-decoded 15 MHz SCK predicts about 546 us, so the
per-transfer overhead is roughly eleven times the wire time. Where a register
decode exists, CPUInfo's Busses section is the better instrument.

### The `spi_cs_is_gpio` trap

Zephyr's `spi.h` says the chip-select device pointer "can be set to NULL to
fully inhibit CS control". It cannot. `spi_cs_is_gpio()` tests a separate
`cs.cs_is_gpio` flag, so clearing only `cs.gpio.port` leaves the driver taking
the CS path and dereferencing the null port, which hard-faults during boot.
Zero the whole `cs` struct instead - `cfg.cs = (struct spi_cs_control){};` -
as `test_clock_freq()` does, with the reason in a comment beside it.

### Kconfig reaches AP through `-imacros`, and waf cannot see it

The opposite of a symbol that never reached Kconfig is a symbol that reached
Kconfig and stopped there. AP sources get Kconfig through `-imacros
autoconf.h`, which waf's header scanner does not follow, so flipping a
`CONFIG_AP_*` symbol that C++ reads leaves already-compiled objects holding the
old value. After changing one, delete the objects of every translation unit
that reads it.

It is not only ArduPilot's own symbols that this bites. Zephyr's Kconfig reaches
AP through the same `-imacros`, and some of it changes struct layout.
`struct uart_driver_api` puts its six async entry points first, under
`CONFIG_UART_ASYNC_API` (`modules/zephyr/include/zephyr/drivers/uart/uart_internal.h`),
so every slot after them shifts by 24 bytes on a 32-bit target when that symbol
is flipped. AP objects compiled against the old `autoconf.h` and linked against
a newly built `libzephyr.a` then call the wrong function pointer through the
`z_impl_*` inline wrappers in `uart_internal.h`, which `uart.h` includes - and
because the flag string on the command line
does not change, waf rebuilds nothing. It presented as a hang inside
`UARTDriver::_begin`, not as a link error. `rm -rf build/<board>` before the
configure is the reliable fix; to confirm a suspicion first, disassemble the
`uart_irq_rx_disable` thunk in the AP object and check which api slot it loads.

## Renode as an instrument

Renode runs this firmware with no board and no probe on the desk. That makes it
the cheapest way to bisect a boot failure - rebuild, rerun, read any register
from the monitor - and the only way when the board you need is not in front of
you. `.github/workflows/test_renode_zephyr.yml` boots and flies two Zephyr
boards this way, CubeOrangeZephyr and mr_vmu_rt1176, with a CubeOrange ChibiOS
flight as the reference.

| Tool                                     | What it does                                                                                                                                                                                                                                                       |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `Tools/renode/zephyr_boot_check.py`      | Boots a firmware and waits for a MAVLink heartbeat. Exits 0 on heartbeat and 1 on timeout, prints the elapsed time, and on a timeout prints whatever STATUSTEXT the board did manage plus the emulator's own log tail.                                              |
| `Tools/renode/tests/test_physics_flight.py` | Flies a full copter mission against the physics model and prints a verdict line.                                                                                                                                                                                 |
| `Tools/renode/zephyr_boot_timeline.py`   | Timestamps the boot itself (see "Reading a flight log").                                                                                                                                                                                                           |

A Zephyr board needs no Renode platform of its own: `run.py` builds the platform
from the MCU in `hwdef.dat`, so CubeOrangeZephyr runs on CubeOrange's H743
platform. A board whose silicon no ChibiOS board uses needs a script instead -
`--resc Tools/renode/scripts/ardupilot_imxrt1176.resc` for mr_vmu_rt1176.

Three traps.

**An orphaned emulator answers the next run's heartbeat check and fakes a
pass.** `zephyr_boot_check.py` used to call `proc.terminate()` on `run.py`,
which stopped `run.py` and left its Renode child alive holding UART port 5762.
The next run connected to the *previous* emulator, read its heartbeat and
exited 0; three orphans were found running at once. This is worse than a stuck
`uploader.py`, because nobody investigates a pass. The tool now starts `run.py`
with `start_new_session=True` and stops it with
`process_utils.terminate_process_group()`, but the reading rule stands: record
the time-to-heartbeat, never just the exit code. A CubeOrangeZephyr boot took
209-269 s of wall time when this was measured, so a pass under about ten seconds
is contamination rather than a fast boot. Calibrate the threshold per board
before trusting it: the rt1176 path heartbeats far sooner than that. Kill strays by PID resolved from
`/proc/PID/exe`; `pgrep -f` and `pkill -f` match the searching shell and have
taken out the wrong process here three times.

**Listening on the wrong UART times out on a board that is perfectly healthy.**
`zephyr_boot_check.py --uart` defaults to `sysbus.lpuart1`. On mr_vmu_rt1176
that is the Zephyr console (`zephyr,console` in the board DTS); MAVLink goes to
SERIAL1, which `SERIAL_ORDER` in `hwdef/mr_vmu_rt1176/hwdef.dat` puts on
USART4, so the CI job passes `--uart sysbus.lpuart4`. Point it at the wrong one
and you get `no heartbeat within <timeout>s` followed by `the board sent no
STATUSTEXT on <uart> at all`, which reads exactly like a board that never
started.

**A heartbeat does not prove the DMA path ran.** `UARTDriver::_begin` falls back
to the interrupt-driven path whenever `uart_callback_set()` fails or a DMA
buffer cannot be allocated. The allocation failure prints "DMA pool exhausted,
falling back to IRQ path"; the `uart_callback_set()` failure prints nothing at
all. Either way the board boots and heartbeats with a completely dead eDMA. Ask
the model directly: `sysbus.edma0 BytesMoved` and `sysbus.edma0
BeatsPerChannel` from the monitor, or pass `--assert-edma`, which reads both
after the heartbeat and fails the run if nothing moved.

Two limits worth knowing before you trust a result.

**A green CI job is not a passed flight.** All three flight steps - both Zephyr
boards and the ChibiOS reference the Zephyr flights are judged against - and the
rt1176 boot check are `continue-on-error: true` and end in `exit 0`; a failure becomes a
`::warning title=... (tolerated)` annotation and a line in the step summary, and
the tick stays green. Read the summary, not the tick.

**Code placement and cache effects cannot be measured here.** The memories in
`Tools/renode/platforms/stm32h743_base.repl` are plain RAM models - almost all
`Memory.MappedMemory` - with no wait states and no cache model, so ITCM-resident
and flash-resident code
cost exactly the same and any placement change reads as zero. Measure that on
silicon with `zephyr_pcsr_sample.py`.

## When the board won't boot

"The console is broken" and "no code is running" look identical from outside: a
board that flashes and verifies byte-exact, no output, no fault dump, no reset
banner. Tuning UART config is wasted work if `main()` is never reached.

### Step 0: invert the question

Stop asking why the UART is silent. Ask whether the CPU is executing your code
at all, and how far it gets. That gives you a bounded question with a yes/no
answer per address: is address X ever executed.

### Step 1: get debugger access, and treat connect mode as a variable

On mr_vmu_rt1176 with an application flashed, the obvious modes all fail with
`SWD/JTAG communication failure (WAIT ACK)` or `FAULT ACK` on every AP:

```sh
pyocd gdbserver --target mimxrt1170_cm7                                # fails
pyocd gdbserver --target mimxrt1170_cm7 --connect attach               # fails
pyocd gdbserver --target mimxrt1170_cm7 --connect under-reset          # fails
pyocd gdbserver --target mimxrt1170_cm7 --connect pre-reset -p 3333    # works
```

Enumerate the connect modes before concluding that SWD is broken. `pyocd reset
-m hw` likewise succeeds where a plain `pyocd reset` fails, because it drives
nRESET directly instead of going through an AP. If the board took the debug
port down with it and pyOCD cannot connect at all, `zephyr_pin_reset.py` pulses
nRST with zero DAP transactions.

How you restart the target decides what you are measuring:

| Command                                       | What it does                                                                                      |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `pyocd commander --connect attach -c "reset"` | Soft reset. On RT1176 the chip lands in the BootROM at `0x223104` and the application never runs. |
| `pyocd commander --connect pre-reset -c "go"` | Genuine boot. The bootloader runs, jumps to the app, the app executes.                            |

Any claim about the application built on the first row is a claim about a boot
that never happened.

A debug port that dies at the moment the firmware misbehaves is a measurement,
not a flaky probe. A hung AHB transaction blocks the debug access ports as well
as the core, so reproducible AP faults after a genuine boot point at a bus
stall.

### Step 2: extract ground-truth addresses from the ELF

Never breakpoint a guessed address.

```sh
E=build/<board>/zephyr_build/zephyr/zephyr.elf
NM=$ZEPHYR_SDK/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm

for s in z_arm_reset z_prep_c z_cstart clock_init mcux_lpuart_init main \
         k_sys_fatal_error_handler z_arm_fault z_arm_hard_fault; do
  a=$($NM $E | grep -iE " [tTwW] $s\$" | awk '{print $1}')
  [ -n "$a" ] && echo "$s = 0x$a"
done
```

The other direction, when a trace shows an unexpected jump:

```sh
arm-zephyr-eabi-addr2line -f -e $E 0x300223a8
```

### Step 3: binary-search the boot chain with hardware breakpoints

Set a breakpoint, `continue`, and read GDB's exit code: `0` means hit, `124`
means `timeout` fired and it was never hit.

```sh
printf 'set confirm off\nset pagination off\n\
target extended-remote localhost:3333\n\
hbreak *0x3002f0e0\ncontinue\n' > /tmp/q.gdb

timeout 30 arm-zephyr-eabi-gdb -batch -x /tmp/q.gdb > /tmp/q.txt 2>&1
echo "EXIT=$?   # 0 = HIT, 124 = NOT hit"
```

Keep it synchronous. GDB `-batch` with an async `continue &` plus `interrupt`
is unreliable; a plain `continue` bounded by `timeout` is not.

Walk the chain in order, `z_arm_reset`, `z_prep_c`, `z_cstart`, `clock_init`,
the driver inits, `main`, and find the last address hit and the first not hit.
That pair brackets the failure. On mr_vmu_rt1176 it came out `z_arm_reset` hit,
`z_prep_c` not hit, which made UART config, console selection and every driver
irrelevant.

### Step 4: classify the failure by which breakpoints hit

| Observation                                          | Diagnosis                                       |
| ---------------------------------------------------- | ----------------------------------------------- |
| Entry breakpoint hit repeatedly every few seconds    | Reset loop, watchdog or fault-reset             |
| Any fault handler breakpoint hit                     | CPU fault, read CFSR/HFSR/BFAR                  |
| Entry hit exactly once, nothing downstream, no fault | Hang or bus stall                               |
| Nothing hit at all                                   | Code never started, bad vector table or no jump |

To rule out a reset loop, breakpoint the reset vector and `continue` twice; if
the second `continue` times out, it is not looping. To rule out a fault,
breakpoint every fault entry point at once.

**A breakpoint answers "did execution reach here", not "is the CPU alive".** A
breakpoint that never fires fits "code is stuck before this point" and equally
fits "code ran and the chip reset out from under the debugger". Free-run then
halt assumes nothing about what the target is doing:

```sh
pyocd commander --connect pre-reset -c "reset" -c "go"   # let it run
sleep 12
pyocd commander --connect attach -c "halt" -c "reg pc"   # then look
```

Do that early, not after twenty breakpoint experiments.

### Step 5: bound the window with static disassembly

Disassemble the bracketed range and read it, looking for externally observable
operations: peripheral accesses, barriers, loops.

```sh
arm-zephyr-eabi-objdump -d --start-address=0x30030a60 \
                           --stop-address=0x30030b40 $E
```

Free, deterministic, and it eliminates whole theories at once. On RT1176 it
showed NXP's `SystemInit()` contains no loops at all, so it cannot hang in the
ordinary sense and had to be stalling on a bus access. It also showed
`soc_reset_hook` and `_soc_reset_hook` are tail branches, `b.w` not `bl`, so
the return path is not what the call graph suggests.

### Step 6: single-step to the exact instruction

```sh
{ echo "set confirm off"; echo "set pagination off"
  echo "target extended-remote localhost:3333"
  echo "hbreak *0x30030a6a"; echo "continue"
  for i in $(seq 1 8); do
    echo "stepi"; echo "printf \"step%d pc=%#x\\n\", $i, \$pc"
  done
} > /tmp/step.gdb
```

A step that produces no output is the answer. Steps 1 to 4 printing normally
and steps 5 to 8 printing nothing means the CPU never completed the fifth
instruction. An instruction that will not retire under single-step is a
hardware stall, not a software bug. On mr_vmu_rt1176 that instruction is a
`dsb sy` at `0x30030a74`, reached only on the bootloader-mediated boot path.

### Step 7: test assumptions directly instead of reasoning about them

Is the stack RAM actually backed? Write a pattern, read it back.

```text
set *(unsigned int*)0x20207b00 = 0xDEADBEEF
x/2xw 0x20207b00      # reads back 0xdeadbeef, so the memory is fine
```

Is the vector table where the bootloader expects it? Read live flash back with
a tool other than the one that wrote it.

```sh
LinkServer flash <device>.json verify --addr 0x30022000 expected_vt.bin
```

The independence is the point: it confirms the image when the primary tool's
own verify is one of the things under suspicion.

A theory that survives a round of argument is a theory you should have tested.
Write the breakpoint or the memory read.

### Step 8: exploit a working and broken pair

One working configuration and one broken one, differing in a small number of
variables, beats any amount of single-path debugging. "Flashed directly to
`0x30000000` gives UART comms, flashed via the bootloader does not" reframes
the problem from "our UART code is wrong", which it is not since it works in
the first path, to "what state does the bootloader leave the hardware in".

Ask early whether there is a configuration where this works.

### Cumulative status registers

`SRC_SRSR` on RT1176 is write-1-to-clear and captures every reset source since
it was last cleared. Read without clearing, it tells you what has ever
happened, not what just happened, and a session full of power cycles and
`pyocd reset` calls leaves plenty of debris to build a hypothesis on.

1. Clear it, on RT1176 by writing `0xFFFFFFFF` to `0x40C04010`.
2. Perform exactly one boot of the kind you care about.
3. Re-read. Whatever is set now is from that boot.

Done that way on the silent-boot case it read `0x00000000`, eliminating the
Code Watchdog, CPU lockup and every other watchdog in one measurement. The
earlier non-zero read was cumulative debris.

The SRC general-purpose registers are not all yours either: on RT1176 the ROM
reserves `SRC_GPR` 0 to 4 and 9. The memory-map table does not say so, the note
underneath it does, and a boot handoff marker written into a reserved GPR will
not survive.

### Anti-patterns

**Trusting a comment over the code.** Two DTS comments on this board named the
wrong UART and the wrong pins while the `pinctrl` was correct. Verify against
the generated `autoconf.h`, the `pinctrl.dtsi`, and `nm` or `objdump` output,
not prose.

**Believing a stuck tool means broken hardware.** `uploader.py` sitting at
`Erase: 0.0%` looks fatal and usually is not; one run needed 26 retries and
about 78 seconds. Give it `timeout 180`, or use `zephyr_flash.sh`, which does
the retrying for you.

**Leaving orphaned processes.** A stale `uploader.py` holding `/dev/ttyACM1`
open makes every subsequent attempt fail, and concurrent pyOCD instances fight
over the probe with `Unable to claim interface`. Check `lsof /dev/ttyACM*` and
`pgrep -f pyocd` before believing a failure.

**Recording only what you found.** Write down the theories you refuted and the
evidence that killed them, or the same dead ends get re-explored.

### Tooling notes

`pyocd gdbserver` generally serves one GDB session. Restart it per test.

Start the gdbserver and run the GDB test in separate shell invocations.
Combining them behind a leading `pkill` tends to kill the whole command.

Watchdogs and some peripherals freeze under a debug halt, so "works when
stepped, hangs when free-running" is a signal, not noise.

### Case study: a hard-coded address from a different SoC

`libraries/AP_HAL_Zephyr/zephyr/src/main.cpp` carried a diagnostic block
immediately after USB init:

```c
/* --- RTC SLOW MEM diagnostics (survives OpenOCD system reset) ---
 * Capture UDC initialized/enabled state so OpenOCD can read it
 * with: mdw 0x50000000 8 */
volatile uint32_t *rtc = (volatile uint32_t *)0x50000000U;
rtc[0] = 0xC0FFEE00U;                            /* magic */
rtc[1] = (uint32_t)udc_is_initialized(udc_dev);
rtc[2] = (uint32_t)udc_is_enabled(udc_dev);
...
```

`0x50000000` is RTC slow memory on an ESP32. On i.MX RT1176 it is not memory at
all: the peripheral map runs at `0x40xx_xxxx`, USB_OTG1 at `0x4043_0000` and
USBPHY1 at `0x4043_4000`, and there is no `0x5000_0000` region. Every one of
those stores went to unmapped address space.

The fault was `K_ERR_ARM_BUS_IMPRECISE_DATA_BUS`, and *imprecise* is the whole
story. An imprecise bus fault is reported asynchronously, after the store has
retired and the pipeline has moved on. `BFAR` is not valid for it
(`BFARVALID = 0`), so there is no faulting address, and the fault appears to
originate at whatever happens to be executing when it lands. That was
`usbd_enable()`, so the fault was recorded as "`usbd_enable()` faults the
moment it is called" and USB was blamed. The dump also showed `r3 = 0x50000000`
and that was read as a peripheral-range address implicating USB; it was the
pointer the diagnostic block had just loaded.

With the block removed, USB init runs to completion with no fault:
`g_usb_phase = 3`, `CFSR = 0`, `HFSR = 0`.

The address was checkable against the reference manual in one command:

```sh
grep -nE "^5[0-9A-F]{3}_[0-9A-F]{4}" IMXRT1170RM_reference_manual_rev3.no-images.txt
# no match, so 0x5000_0000 is not in the RT1176 memory map
```

The reference manual and its text extraction are NXP copyright and are
deliberately not committed. Download the PDF and generate the `.no-images.txt`
dump into `libraries/AP_HAL_Zephyr/zephyr/boards/arm/mr_vmu_rt1176/docs/`
first; that folder's `README.md` has the URL and the `pdftotext` command.

What generalises:

1. **Porting drags addresses with it.** Copied diagnostic scaffolding is more
   dangerous than copied driver code, because nobody reviews debug helpers.
2. **Diagnostics are code.** This block existed only to observe a bug, became
   the bug, then hid itself by blaming the subsystem it was watching. Prefer
   plain globals read over SWD, `nm` plus `pyocd commander read32`, to writes
   at hard-coded scratch addresses. A global cannot be at a wrong address by
   construction.
3. **Know which faults carry an address.** On ARMv7-M a precise bus fault gives
   you `BFAR`; an imprecise one gives you nothing and points at the wrong code.
   Check `BFARVALID` and `IMPRECISERR` in `CFSR` before believing any location
   attributed to a bus fault. If `IMPRECISERR` is set, the reported PC is not
   where the bad access came from.
4. **Register values in a fault dump are not evidence of intent.** `r3` held
   `0x50000000` because the code had just loaded that pointer, not because a
   peripheral there was involved.
5. **Be suspicious of conclusions that disable a subsystem.** "USB is broken,
   disable it" ends the investigation and makes the answer self-confirming. A
   finding that removes your ability to test it deserves more scrutiny than one
   that does not.

## Crashes

Two mechanisms, and they answer different questions.

**The fatal-error record** is a handful of globals written from fault context:
count, reason, PC, LR, CFSR. Read them with `zephyr_read_fatal.py` over SWD.
This is what you get when the fault handler itself wedged before printing.

**The coredump** is Zephyr's own `coredump` subsystem, not CrashCatcher.
CrashCatcher's capture engine is hand-written ARMv7-M assembly and cannot be
ported to Xtensa or RISC-V at all, which is why we did not.

The full cycle - a real fault, the write from fault context, the reset, and
retrieving the dump afterwards - has not been run on hardware. The backend says
so in its own header
(`libraries/AP_HAL_Zephyr/zephyr/src/rt1176_coredump_backend.c`), and so does
`hwdef/mr_vmu_rt1176/README.md`. Treat it as implemented and reasoned through,
not as verified.

The dump cannot yet be fetched over MAVFTP. `@SYS/crash_dump.bin` is gated on
`AP_CRASHDUMP_FLASH_ENABLED` (`AP_Filesystem_Sys.cpp:54` and `:142`), which
defaults to 0 at `:31` and is only ever set by the ChibiOS hwdef generator
(`chibios_hwdef.py`, in the block that writes the crash-dump macros). The one
Zephyr hwdef that turns crash dumps on, mr_vmu_rt1176's, sets
`AP_CRASHDUMP_ENABLED`, which is a different macro and does not reach that
gate, so the `@SYS` directory entry is compiled out and MAVFTP cannot list or
fetch the file.

Do not check this by grepping the image for `crash_dump.bin`: the string is in
there regardless, from `AP_Logger`'s own crash-dump save path, which is gated on
a third macro. Check the directory entry instead - `AP_Filesystem_Sys.cpp:54`.
Read the dump over SWD until this is wired up. Once `last_crash_dump_size()`
and `ptr()` exist and the board defines `AP_CRASHDUMP_FLASH_ENABLED`, the
shared `AP_Filesystem_Sys.cpp` path needs no changes. `PARITY_DETAIL.md` has
the rest of the macro story.

Dump size is a watchdog budget, not a storage one: the write happens with
interrupts locked, so a larger dump is a longer window in which nothing feeds
the watchdog. There is no clear mechanism yet, so once a dump exists the prearm
persists until the partition is erased.

## Techniques

**Sample the PC without halting.** On a board executing from external flash,
where a function sits matters more than what it does. Bucket the histogram by
memory region as well as by symbol: by symbol alone you learn which function is
hot, not that the reason is its address. Moving hot code into tightly-coupled
memory is then the fix, and the kernel context-switch path is worth checking
first because every thread pays for it.

**Two instruments that share nothing.** If SWD counters say the board stalled,
confirm it with the byte stream before believing it. A single instrument
reporting zero is as likely to be a broken instrument.

**A negative watchpoint result is void if the debug session died during the
window.** A conclusion of "nothing wrote this register" is worthless if the
event under investigation also killed the debugger. Never filter a debug
session's output down to only the lines you expect, or the USB errors and
unexpected-reset lines that would have told you get discarded.

**Never trust a bug seen once, and state the count.** Say 3 of 3 boots before
naming a fault. All-zero readings are a tooling symptom first.

**Diagnostics that share the resource under test are suspects.** An AP-mode
WiFi scan takes the radio off-channel and suppresses the board's own beacons,
so a diagnostic scan probe made a softAP invisible on every boot that ran one:
0 of 5 visible with it, 3 of 3 without.

**pyOCD non-halting reads can be session-cached.** A counter that looks frozen
may be a cache artifact. Halt, read, resume, or compare across separate
sessions, or use a behavioural test instead.

**Multiply per-unit costs by the rate, then actually do the multiplication.**
"Saves 0.20 us of a 162 us transfer, 0.1%" is arithmetically true and useless as
a decision input - but the rate has to be carried through, not waved at. At 3100
transfers a second that saving is 620 us/s, which is 0.06% of a core and still
not worth having. What the same rate makes large is the transfer itself:
162 us x 3100/s is about 0.50 s/s, half a core, and that is what to go after.
Attach the rate to every per-unit figure and compute the product before quoting
it.

**Check that your check can fail.** A negative result from an instrument you
have never seen report positive is not evidence. Confirm the probe works by
making it fire deliberately first.

**Diff against ChibiOS.** The file layout tracks AP_HAL_ChibiOS closely so that
you can. When behaviour differs and you cannot see why, put the two
implementations side by side before theorising.

**Take a time series, not a sample.** Loop rate oscillates on a multi-second
period on some builds. One 8-second window can show you whatever you were
hoping for.

**Compare two runs over a physical variable, not over the same clock window.**
Averaging a fixed window out of each of two logs compares whatever flight phase
each run happened to be in at that point. Done here on the first 10 s of a
Zephyr log against the first 10 s of a ChibiOS one, it produced a confident
"Zephyr's accelerometers read 1.15% high" which was withdrawn: binned by tilt
across the whole of both logs the two agreed to 0.03%. Pick a variable that
means the same thing in both runs, bin on it, and use the whole log.
`zephyr_ins_rate_probe.py` does its accelerometer check that way for this
reason.

**A rate that looks perfectly steady can be the failure, not reassurance.**
`IMU.GHz` in a dataflash log is not a measurement, it is an estimate re-fitted
once a second and clamped to +/-0.1%/s once the convergence window has shut. A
lane sitting exactly on that rail for a whole flight is not stable, it is
pinned, and the rate the EKF believes is whatever the driver compiled in - see
"Reading a flight log" for why a late first sample leaves it there. The test is
the one in `zephyr_ins_rate_probe.py` check 1: fit the slope, and if it
is on the rail, extrapolate the line backwards and see whether it lands on a
round nominal. The corroborating signature is in `XKF3` - velocity, position
*and* magnetometer innovations all out at once - on the emulated flight pair
where this was measured, around a hundred times the reference flight's. No
single sensor produces that pattern; a prediction running on a different clock
from the measurements does.

**Two silent hangs share one signature, and only one of them is in the HAL.**
The first is a Zephyr thread-priority mistake, which does not fault.
`K_PRIO_PREEMPT` counts down, so a "+1" carried over from ChibiOS raises a
thread instead of lowering it. Several threads belong above main by design -
the timer, rcout, SPI and unbuffered-UART threads sit one level above it in
`Scheduler.h`, and there are hwdef hatches that deliberately lift storage and
io there on a board with no headroom - but raising the AP_io thread above main
by accident stopped the firmware booting: no crash, no fault handler, no
message, and under Renode an hour of run time with the image loaded and nothing
on the wire. The second is not a HAL fault at all but an emulated peripheral: a
driver waiting on "transfer done OR error" where the error flag can never be
set. Renode's stock `STM32HSDMMC` models DTIMEOUT and RXOVERR as tagged flags
that read permanently false and pins DCRCFAIL false, so a transfer the model
does not finish has nothing to break the wait and the guest parks in the idle
thread. This tree's `AP_STM32H7_SDMMC.cs` synthesizes a DTIMEOUT to catch
exactly that; the base class it subclasses still cannot. From outside, both
failures look the same - process alive, no telemetry, empty log - so neither is
diagnosable by staring at the symptom.

**When two faults share one signature, tabulate instead of reasoning.** Those
two silent hangs were present at the same time and each was mis-attributed to
the other in turn. What settled it was a table with one row per run and one
column per suspect - here io thread priority, main-loop yield period, whether
the SD mount succeeded, and whether the board booted - filled in by varying one
suspect at a time. Two rows that differ in one column and disagree on the
outcome name the cause; two rows that differ in one column and agree eliminate
it. Build the table before the next run, so you know which cell that run is
filling.
