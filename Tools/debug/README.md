# Hardware Debugging with ArduPilot

This directory contains files that are useful for setting up to debug
ArduPilot using either a black magic probe or a stlink-v2 with
openocd.

This assumes you are debugging a ChibiOS based firmware on a STM32 board.

## Debugging over USB on STM32H7

The experimental USB debugger is disabled by default and requires an STM32H7
board with two CDC interfaces on USB OTG1. Enabling it
reserves the second CDC interface for GDB. External-flash boards are unsupported:
the monitor only reads internal flash and the configured RAM regions, and uses
internal-flash hardware breakpoints and RAM software breakpoints. Boards with an external watchdog are also rejected,
as the monitor services only the STM32 internal watchdog while stopped.

Build and upload the vehicle with debug symbols, for example:

```sh
./waf configure --board CubeOrangePlus --enable-USB-debug --debug-symbols
./waf plane --upload
```

Install `pyserial` and `arm-none-eabi-gdb`, then use the matching ELF and the
second USB interface. On Linux, prefer its persistent `by-id` path:

```sh
python3 Tools/debug/gdb_usb.py build/CubeOrangePlus/bin/arduplane \
    --port /dev/serial/by-id/usb-CubePilot_CubeOrange+_<SERIAL>-if02
```

The launcher requests attachment from the vehicle's main loop, waits
for USB to re-enumerate, and starts interactive GDB. Once attached, the USB IRQ runs at priority zero and services the raw controller
so Ctrl-C can interrupt execution independently of the main loop. USB start-of-frame
events also service queued GCS output. SysTick remains available to the application.
The first CDC interface (`-if00`) remains available for a GCS while the target
is running. Its USB connection stays open at a debugger stop, but telemetry and
command processing pause until `continue`. Attachment and detach re-enumerate
both interfaces, so the GCS must reconnect after those transitions. An attached
hardware debugger prevents USB debugger attachment.

Useful GDB commands:

```gdb
bt
info registers
info float
set $r4 = 7
p AP_HAL::millis()
set var plane.auto_state.height_below_takeoff_to_level_off_cm = 7
p plane.auto_state.height_below_takeoff_to_level_off_cm
info threads
thread apply all bt 3
break Plane::one_second_loop
watch plane.auto_state.height_below_takeoff_to_level_off_cm
continue
# Press Ctrl-C to stop again.
stepi
delete breakpoints
detach
```

`break` and `hbreak` use the CPU's limited hardware breakpoint comparators in
internal flash, excluding the debugger itself. Flash is never patched. `break`
in RAM inserts a Thumb BKPT, preserves the original instruction for reads and
step-over, and restores it on removal or detach. Code writes clean the data
cache and invalidate the instruction cache.

`watch`, `rwatch` and `awatch` use DWT comparators for write, read and access
watches. Each comparator supports an aligned power-of-two range up to 32 bytes;
availability depends on the MCU. The monitor disables watches during debugger
memory access. `stepi` uses DebugMonitor. Stepping interrupt handlers, fault
stops, SVC, CPS, MSR, WFI and WFE is rejected. Flash programming is unsupported.

RAM-backed variables and saved registers can be assigned with `set var`,
`p variable = value`, or `set $r4 = value`. Floating-point registers are exposed
as `s0`–`s31`, `d0`–`d15` and `fpscr`. Optimised builds may not retain every local
variable. GDB can call simple functions such as `p AP_HAL::millis()` on the
stopped thread. Calls execute target code and can block if they need a lock
held by the interrupted context; they are unsuitable at a fault or IRQ stop.

Do not place breakpoints in NMI/HardFault handlers or code running with
PRIMASK set: the debug event can escalate to HardFault. To continue from a
breakpoint in an interrupt handler, remove that breakpoint first, since its
automatic step-over is not supported. A literal BKPT instruction is skipped
when resumed. Debugging is available while armed, including attachment and
breakpoints in code that only runs in the armed state.

`info threads` lists the ChibiOS registry when stopped in thread mode outside
a kernel critical section. Otherwise, only the interrupted CPU context is
exposed. A blocked thread provides its saved callee-saved registers, stack
pointer and return PC for stack unwinding. Its other registers are unavailable.
Saved registers of a blocked thread can also be edited; unavailable registers
and its descriptive stack pointer cannot. Resuming an individual blocked thread
is not supported. The current CPU stack pointer can be changed within RAM while
preserving its exception-frame alignment. Memory reads are restricted to internal flash and the
board's configured RAM regions. Memory writes are restricted to RAM and cannot overwrite
the monitor's own state or stack. The entire request is validated before writing;
a hardware fault during a write reports an error but may leave a prefix changed.

`continue` keeps the USB debug connection open. `detach` resumes execution and
restores normal USB, which re-enumerates. `monitor reset` reboots the board;
the resulting USB disconnect is expected. Use `--no-break` to reconnect to an
existing debug session; a running target is interrupted first. A stop with no
valid GDB packet reboots after 30 seconds; once connected, the monitor waits for GDB and services the hardware
watchdog, even if the host disconnects.

This is for bench debugging. Interrupts and normal telemetry stop
while the target is halted; IOMCU and peripheral timeouts may occur. Reboot
after a session before using the vehicle. Initial attachment requires a
working main loop by default. Add `--enable-USB-debug-startup-wait` alongside
`--enable-USB-debug` to wait for attachment after USB/scheduler initialization
and before vehicle setup. Use the same launcher, set breakpoints, then
`continue` to start setup. The wait has no timeout and is disabled by default.
Failures before USB initialization still require a debug adapter or crash dump. After attachment, Ctrl-C requires
the USB IRQ to remain runnable: PRIMASK, FAULTMASK or a priority-zero handler can
prevent interruption.

After attachment, HardFault, MemManage, BusFault and UsageFault can stop in GDB
with the interrupted registers and stack. `monitor fault` reports the exception
number, CFSR, HFSR, MMFAR and BFAR (address registers are meaningful only when
their validity bits are set). Repair the cause or PC before `continue`, which
retries the instruction; `monitor reset` reboots instead. Stacking faults and faults before attachment retain crash-dump handling.
A second fault while handling HardFault cannot be recovered, so memory that is
inaccessible even to the fault handler cannot be inspected safely.

Run the launcher protocol tests with:

```sh
python3 -m unittest discover -s Tools/debug/tests
```

### VS Code (Linux, Windows and macOS)

After building/uploading USB-debug firmware, use **File > Open Workspace from
File** and open [`.vscode/usb-debug.code-workspace`](../../.vscode/usb-debug.code-workspace).
Install the recommended **C/C++** extension (`ms-vscode.cpptools`), Python with
`pyserial`, and an ARM-capable GDB.

Select **USB: Attach** in Run and Debug and press **F5**. The prompts ask for
the matching ELF path relative to the repository, second CDC port, Python
executable and GDB executable. Use `python3` on Linux/macOS or `py` on Windows,
or enter the full interpreter path. Both executable prompts accept paths with
spaces. **Terminal > Run Task > USB: List serial ports** helps identify the
port: prefer `/dev/serial/by-id/...-if02` on Linux, `/dev/cu.usbmodem...` on macOS,
or the second COM port on Windows. Use the **exact ELF used for the upload**.
No workspace generation command is needed.

**USB: Reconnect to existing monitor** reconnects after an interrupted IDE
session, stopping the CPU first if it was running. For a startup-wait build,
use the normal **USB: Attach** configuration, set breakpoints, then Continue.
These workspaces leave your existing `launch.json` and `settings.json` intact.

To save fixed paths instead of using prompts, you can still generate a personal
workspace with the launcher. Use `--gdb /path/to/gdb` if needed:

```sh
python3 Tools/debug/gdb_usb.py build/CubeOrangePlus/bin/arduplane \
    --port /dev/serial/by-id/YOUR-BOARD-if02 \
    --vscode .vscode/usb-debug-local.code-workspace
code .vscode/usb-debug-local.code-workspace
```

On native Windows, run this with `py -3` and a port such as `--port COM40`.
On macOS use the second CDC's `/dev/cu.usbmodem...` device. List ports with
`python3 -m serial.tools.list_ports -v` (`py -3 -m serial.tools.list_ports -v`
on Windows). Generate the workspace on the OS where the debugger runs: its
Python, GDB, source and ELF paths are local to that OS. Existing workspace files
are not overwritten; edit the generated configuration or choose a new filename.

The generated configuration is named **ArduPilot USB debug**. The launcher
starts a loopback-only TCP bridge, requests USB attachment, and leaves the CPU
stopped. The same configuration handles startup-wait firmware. Set breakpoints
in the editor, use Continue/Pause and stepping controls, and inspect threads,
stack frames, variables and Watch expressions. In the Debug Console, prefix
GDB commands with `-exec`, for example `-exec info registers` or `-exec detach`.
Detach restores normal USB and resumes execution. VS Code's normal **Stop**
button terminates the target instead, which resets the board. Reflashing is not
part of this configuration. After an interrupted IDE session, generate a workspace with
`--no-break` to reconnect to the existing monitor.

The bridge exits when GDB disconnects. `--tcp-port` selects a different local
port if 3333 is occupied. The workspace uses the C/C++ extension's
[debug-server configuration](https://code.visualstudio.com/docs/cpp/launch-json-reference).
You can copy its `launch.configurations` entry into your existing `launch.json`.
If the ELF was built elsewhere, set `sourceFileMap` in that entry, for example
`{"/original/ardupilot": "C:/src/ardupilot"}`. Debug symbols alone do not provide
the corresponding source files.

### Windows frontend with a WSL workspace

Open the repository using VS Code's
[WSL extension](https://code.visualstudio.com/docs/remote/wsl), and install the
C/C++ extension **in WSL**. Build and run GDB inside WSL;
the Windows VS Code frontend displays the debugging UI.

If USB is attached directly to WSL, open `usb-debug.code-workspace` in the WSL
window and use **USB: Attach** with its Linux port. Microsoft's
[USB forwarding guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
describes `usbipd bind` and `usbipd attach --wsl`. Both CDC interfaces move into
WSL, so Windows applications cannot use them while attached. USB debugging
re-enumerates the device; USB forwarding must reconnect after this transition.

Alternatively, keep USB on Windows so a Windows GCS can use the first COM port.
With SSH access to the Windows host configured inside WSL, this can be done
entirely from VS Code:

1. In a **local Windows** VS Code window, open `usb-debug.code-workspace` from
   a Windows-accessible checkout. Choose **Terminal > Run Task > USB: Start
   bridge for WSL**. Enter the Windows Python executable, second COM port and
   matching ELF. The ELF may be a Windows copy or a `\\wsl.localhost\...` path.
   Wait for the task to print `USB GDB server listening`.
2. In the **WSL** window, open
   [`.vscode/usb-debug-wsl.code-workspace`](../../.vscode/usb-debug-wsl.code-workspace)
   and press **F5**. Enter the WSL ELF and GDB paths and the Windows SSH host.
   The workspace starts the loopback SSH tunnel before attaching. Complete any
   SSH authentication or host-key prompt in the task terminal.
3. Restart the Windows bridge task for each new debug session. It waits up to
   five minutes for GDB. The WSL tunnel remains available between sessions;
   close it with **Terminal > Terminate Task** when finished.

Both workspaces use local TCP port 3333. If changing it, update the Windows
bridge task, WSL forwarding task and debugger address together. The tunnel
uses SSH credentials/configuration from **WSL**, not Windows.

The equivalent command-line setup is:

```powershell
py -3 Tools/debug/gdb_usb.py path/to/arduplane --port COM40 --serve --timeout 300
```

Forward its loopback port into WSL using your configured Windows SSH host:

```sh
ssh -N -L 3333:127.0.0.1:3333 WINDOWS_SSH_HOST
```

In another WSL terminal, generate and open the workspace:

```sh
python3 Tools/debug/gdb_usb.py build/CubeOrangePlus/bin/arduplane \
    --gdb-server 127.0.0.1:3333 --vscode .vscode/usb-debug-local.code-workspace
code .vscode/usb-debug-local.code-workspace
```

The bridge waits for GDB before stopping firmware. Restart the Windows bridge
for each new session; after an unclean disconnect use `--no-break` there.
The forwarding keeps the unauthenticated GDB protocol off the local network.

The optional adapter integration test exercises the actual C/C++ debug adapter
against an already loaded Plane image. Set `USB_DEBUG_WORKSPACE` to the generated
workspace and `USB_DEBUG_ADAPTER` to the extension's `debugAdapters/bin/OpenDebugAD7`
(`OpenDebugAD7.exe` on Windows), then run `tests/test_usb_debug_vscode.py`. It
attaches, inspects threads/frames/variables, breaks in `Plane::one_second_loop`,
steps, continues, pauses and detaches.

## Debugging with a Black Magic Probe

If you have a [black magic probe](https://1bitsquared.com/products/black-magic-probe)) then first make
sure it has the latest firmware. See the [blacksphere wiki](https://github.com/blacksphere/blackmagic/wiki) for details.

Next, copy the file gdb-black-magic.init to the ArduPilot source
directory, in the same directory where you will be starting the
debugger. Rename the file to ".gdbinit"

Now either edit the .gdbinit to give the path to the serial port for
your black magic probe, or install the provided udev rules file so
that the probe will be loaded as /dev/ttyBmpGdb

Now make sure you have the right version of arm-none-eabi-gdb
installed. We recommend version 10-2020-q4-major, which is available
on the [ArduPilot firmware server](https://firmware.ardupilot.org/Tools/STM32-tools/) .

Now build ArduPilot with the --debug configure option. You may also
like to include the --enable-asserts. Enabling asserts will slow down
the firmware quite a lot, but will help catch ChibiOS API usage bugs.

For example:

  ./waf configure --board Pixhawk1 --debug --enable-asserts

Now build and install your firmware:

  ./waf copter --upload

After it is loaded you can attach with gdb like this:

 arm-none-eabi-gdb build/Pixhawk4/bin/arducopter

then you can use normal gdb commands. If you are not familiar with gdb
then do a google search.

Note that for a source view the command "layout src" or "layout spit"
is useful.

## Debugging with a STLink-v2

If you have a STLink-V2 adapter (or one of the very cheap clones) then
you can debug with openocd. Using openocd has the advantage that you
can debug threads properly, unlike the black magic probe which can't
see ChibiOS threads.

Start by installing the latest version of openocd, then copy the
openocd.cfg file from this directory to the directory where you will
be debugging.

You may need to edit the openocd.cfg file to set the MCU type. The one
in this directory is setup for a STM32F4 board. If you have a STM32F7
or STM32H7 then edit the file in the obvious way.

Now start openocd in a terminal. You should get output like this:

```text
Open On-Chip Debugger 0.10.0+dev-00272-gedb6796 (2018-01-19-17:26)
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
adapter speed: 1800 kHz
adapter_nsrst_delay: 100
srst_only separate srst_nogate srst_open_drain connect_deassert_srst
Info : clock speed 1800 kHz
Info : STLINK v2 JTAG v29 API v2 SWIM v18 VID 0x0483 PID 0x374B
Info : using stlink api v2
Info : Target voltage: 3.253404
Info : stm32h7x.cpu: hardware has 8 breakpoints, 4 watchpoints
Info : Listening on port 3333 for gdb connections
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
```

the above output is for a STM32H743 Nucleo board, but others are
similar

In another terminal, copy the gdb-openocd.init file to the directory
where you will be debugging, calling it .gdbinit.

Now build and load the debug enabled firmware for ArduPilot in the
same manner as given above for the Black Magic probe, and start
arm-none-eabi-gdb in the same manner.

To see ChibiOS threads use the "info threads" command. See the gdb
documentation for more information.

`Tools/debug/debug_interface.py` is the portable GDB remote server for post-mortem
debugging. It needs only Python 3 and replaces the platform-specific
CrashDebug executables. It accepts binary or hexadecimal CrashCatcher dumps,
GDB memory logs created by `crash_dump.scr`, ELF or raw firmware images, and
optional memory aliases. For full-memory dumps it exposes the ChibiOS registry
to GDB, including thread names, states and saved register contexts, so normal
commands such as `info threads`, `thread N` and `thread apply all bt` work.

## Debugging Hardfaults

## Getting fault dump via Flash

If a fault happens the information gets recorded in flash sector defined in hwdef define HAL_CRASH_DUMP_FLASHPAGE xx .

Only one crash will be recorded per flash cycle. At every new firmware update the flash will be ready again to record the crash log. Maybe we can erase the crash flash page via a parameter or maybe right after we fetch the crash_dump.bin.
To fetch the crash dump @SYS/crash_dump.bin can be fetched via MAVFTP.

Once fetched one can either use the following command to immediately dump backtrace with locals:

`./Tools/debug/crash_debugger.py  /path/to/elf --dump-debug --dump-filein crash_dump.bin`

## Getting fault dump via microSD

Fetch `APM/CrashDump.DAT` from the microSD card directly or via MAVFTP.

For an SD crashdump containing all RAM, use a debug-symbol build and add
`--threads` to show the saved ChibiOS registry and a backtrace for every
thread:

`./Tools/debug/crash_debugger.py /path/to/elf --dump-debug --dump-filein CrashDump.DAT --threads`

The SD crashdump is preallocated, so the debugger automatically uses the dump
length stored in its final sector and ignores the remaining `0xFF` padding. New
SD crashdumps also include the firmware Git hash, image size and CRC. The debug
tools verify the CRC against the supplied ELF and stop before starting GDB if
the firmware does not match.

or to open in gdb for further postmortem do the following:

`arm-none-eabi-gdb -nx path/to/elf/file -ex "set target-charset ASCII" -ex "target remote | python3 Tools/debug/debug_interface.py --elf path/to/elf/file --dump crash_dump.bin"`

## Debugging faults using GDB

* Connect hardware over SWD
* Place breakpoint at hardfault using `b *&HardFault_Handler`
* If one is lucky process stack remained untouched they can do `set $sp = $psp`
* Now you can simply run `backtrace` and potentially reach the fault
* If fault happens at startup one can run and then wait for breakpoint hit at HardFault_Handler

and then `set $sp = $psp` and do `backtrace`

* One can also log the RAM, refer crash_debugger app and Tools/debug/crash_dump.scr for the same.

### References

[Memfault Interrupt](https://interrupt.memfault.com/blog/cortex-m-fault-debug)

[CrashCatcher](https://github.com/adamgreen/CrashCatcher/tree/c8e801225bfa12da70c01ea25b58090b2b7a2e0a)

[Blog](http://www.cyrilfougeray.com/2020/07/27/firmware-logs-with-stack-trace.html)
