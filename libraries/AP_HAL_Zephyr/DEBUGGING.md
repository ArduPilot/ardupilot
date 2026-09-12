# Debugging ArduPilot on Zephyr

Most of this is about getting information off a board that has stopped telling
you anything useful. There are 23 tools in `Tools/scripts/zephyr_*` and you
will not find them by accident, so the inventory is first.

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
| `zephyr_make_sample_into_apj_flashable.py` | Builds any upstream Zephyr sample for one of our boards and packages it as a flashable `.apj`. Takes ArduPilot out of the picture entirely to answer "does this board boot at all, does this UART work at all".                                                                                                                                                                       | host + serial  |

### Getting output off a board

| Tool                     | What it does                                                                                                                                                                                                                                                                                                                                                                                     | Needs  |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------ |
| `zephyr_bootlog.py`      | Polls for the CDC device node every 20 ms and opens it the instant it enumerates, so the *first* boot after a flash is captured. Nothing on the board buffers the banner. Start it before the reset, not after.                                                                                                                                                                                  | serial |
| `zephyr_uart_capture.py` | Generic capture to a timestamped file. Deliberately has no `ttyACM` default: a bare device number silently talks to the wrong board after any re-enumeration.                                                                                                                                                                                                                                    | serial |
| `zephyr_cdc_gaps.py`     | Reports gaps in emitted traffic above a threshold. An independent liveness check that shares nothing with the SWD counters, so if SWD says 0 Hz *and* the byte stream has a matching silence, the board really stalled. Watch the observer effect: attaching a reader also drains the CDC buffer, and a board that is healthy with a reader attached and stalls without one is itself a finding. | serial |
| `zephyr_sysinfo.py`      | Reads the on-target `@SYS/threads.txt` and `@SYS/tasks.txt` out of `g_ap_sysinfo` over SWD. The way to get them when MAVFTP will not serve them.                                                                                                                                                                                                                                                 | probe  |
| `zephyr_read_fatal.py`   | Reads the last fatal-error record over SWD, for when the fault handler wedged before it got its message out.                                                                                                                                                                                                                                                                                     | probe  |
| `zephyr_can_nodes.py`    | Enumerates DroneCAN nodes on both buses and fetches the per-bus stats files. Bench settings are hardcoded; read the header.                                                                                                                                                                                                                                                                      | serial |

### Measuring where the time goes

| Tool                        | What it does                                                                                                                                                                                                                                                                                                                                                                                                                             | Needs |
| --------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----- |
| `zephyr_pcsr_sample.py`     | Statistical PC profiler using DWT_PCSR: the debugger reads a recently-executed PC while the core keeps running. Histograms by symbol *and* by memory region, so ITCM code is distinguished from execute-in-place. The best single answer to "where is the CPU actually". Collect at least 2000 samples before believing a line: a handful of samples produces attributions that do not reproduce. `0xFFFFFFFF` means halted or sleeping. | probe |
| `zephyr_c6_pc_sample.py`    | The RISC-V equivalent for ESP32-C6 over the built-in USB Serial/JTAG. RISC-V has no PCSR, so this halts, reads `pc`, resumes, stealing about 1 ms per sample at 20 Hz. Run it in its own window, never during a live telemetry capture.                                                                                                                                                                                                  | probe |
| `zephyr_isr_composition.py` | Per-vector interrupt rates, read twice across a window and printed as interrupts per second. Vector names are parsed out of the SoC header's `IRQn` enum at runtime rather than hand-copied. For when the totals say "interrupts are the cost" but not which source.                                                                                                                                                                     | probe |
| `zephyr_chain_sample.py`    | Samples the whole `g_ap_prof` block in one transaction, so phase and counters come from the same instant. Attributes loop time to a pipeline stage: bus callback, SPI transfer, FIFO read, wait-for-sample, INS, EKF, AHRS.                                                                                                                                                                                                              | probe |
| `zephyr_xfer_hist.py`       | SPI transfer-duration histogram. A mean cannot separate the two explanations that need opposite fixes: a tight distribution means the cost is genuinely per transfer, a long tail or a bimodal shape means queueing.                                                                                                                                                                                                                     | probe |
| `zephyr_timeseries.py`      | Loop rate as a time series with a spread verdict. Exists because a rate can oscillate on a multi-second period, which makes any single 8-second A/B window a lottery ticket. Falls back to the ungated `g_ap_loop_count` so a shipping build stays measurable.                                                                                                                                                                           | probe |
| `zephyr_wait_ready.py`      | Polls until the board is actually in steady state: enumerated as the app not the bootloader, counters advancing, rate stable across two windows. Use instead of a blind `sleep`, which is wrong in both directions: too short and the sample covers gyro calibration rather than flight, too long and it wastes the run.                                                                                                                 | probe |

The four `g_ap_prof` tools need `CONFIG_AP_CHAIN_PROFILE=y`;
`zephyr_isr_composition.py` needs `CONFIG_AP_ISR_COUNT=y` plus `CONFIG_TRACING=y` and `CONFIG_TRACING_USER=y`.

### Build and repo helpers

| Tool                          | What it does                                                                                                                                                                  | Needs |
| ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----- |
| `zephyr_get_prerequisites.sh` | First-time setup. Run from the repo root.                                                                                                                                     | host  |
| `zephyr_dts_ours_only.py`     | Filters a generated `zephyr.dts` down to only what AP_HAL_Zephyr contributed, keeping the enclosing node structure. Saves reading thousands of lines of upstream SoC `.dtsi`. | host  |

The two LinkServer tools, `rt1176_linkserver_flash.py` and
`zephyr_install_ap_bootloader.py`, honour `$LINKSERVER` for the binary and
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
files over MAVFTP, or use `zephyr_sysinfo.py` over SWD where MAVFTP will not
serve them.

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

The second trap is the other direction. AP sources get Kconfig through
`-imacros autoconf.h`, which waf's header scanner does not follow, so flipping
a `CONFIG_AP_*` symbol that C++ reads leaves already-compiled objects holding
the old value. After changing one, delete the objects of every translation unit
that reads it.

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

The chain has been verified end to end on mr_vmu_rt1176: injected fault, 1371
bytes written from fault context, watchdog reset, autonomous reboot,
`PreArm: CrashDump data detected`, dump fetched over MAVFTP as
`@SYS/crash_dump.bin`. Once `last_crash_dump_size()` and `ptr()` exist the
shared `AP_Filesystem_Sys.cpp` path needs no changes.

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

**Multiply per-unit costs by the rate before dismissing them.** "Saves 0.20 us
of a 162 us transfer, 0.1%" is arithmetically true and useless as a decision
input. At 3100 transfers a second the same saving is most of a core. Any
per-unit figure needs its rate attached before it means anything.

**Check that your check can fail.** A negative result from an instrument you
have never seen report positive is not evidence. Confirm the probe works by
making it fire deliberately first.

**Diff against ChibiOS.** The file layout tracks AP_HAL_ChibiOS closely so that
you can. When behaviour differs and you cannot see why, put the two
implementations side by side before theorising.

**Take a time series, not a sample.** Loop rate oscillates on a multi-second
period on some builds. One 8-second window can show you whatever you were
hoping for.
