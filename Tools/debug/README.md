# Hardware Debugging with ArduPilot

This directory contains files that are useful for setting up to debug
ArduPilot using either a black magic probe or a stlink-v2 with
openocd.

This assumes you are debugging a ChibiOS based firmware on a STM32 board.

## Debugging with a Black Magic Probe

If you have a black magic probe (see
https://1bitsquared.com/products/black-magic-probe) then first make
sure it has the latest firmware. See the wiki here for details:
https://github.com/blacksphere/blackmagic/wiki

Next, copy the file gdb-black-magic.init to the ArduPilot source
directory, in the same directory where you will be starting the
debugger. Rename the file to ".gdbinit"

Now either edit the .gdbinit to give the path to the serial port for
your black magic probe, or install the provided udev rules file so
that the probe will be loaded as /dev/ttyBmpGdb

Now make sure you have the right version of arm-none-eabi-gdb
installed. We recommend version 10-2020-q4-major, which is available
here: https://firmware.ardupilot.org/Tools/STM32-tools/

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

```
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

# Debugging Hardfaults

## Getting fault dump via Serial
All one needs to do is connect the First USART(not OTG) in the SERIAL_ORDER of the board via FTDI. In the case of CubeOrange that is Telem1 and for most boards that should be the case as well. Once connected run following command:

`./Tools/debug/crash_debugger.py /path/to/elf --ser-debug --ser-port /dev/ttyxxx path/to/elf/file --dump-filename logfile.txt`

Additionally the logfile.txt contains a memory dump, which can be shared along with elf file. And devs can then just start up gdb using following command, and do all the analysis that needs done.

`arm-none-eabi-gdb -nx path/to/elf/file -ex "set target-charset ASCII" -ex "target remote | modules/CrashDebug/bins/lin64/CrashDebug --elf path/to/elf/file --dump logfile.txt"`

## Getting fault dump via Flash
If a fault happens the information gets recorded in flash sector defined in hwdef define HAL_CRASH_DUMP_FLASHPAGE xx .

Only one crash will be recorded per flash cycle. At every new firmware update the flash will be ready again to record the crash log. Maybe we can erase the crash flash page via a parameter or maybe right after we fetch the crash_dump.bin.
To fetch the crash dump @SYS/crash_dump.bin can be fetched via MAVFTP.

Once fetched one can either use the following command to immediately dump backtrace with locals:

`./Tools/debug/crash_debugger.py  /path/to/elf --dump-debug --dump-filein crash_dump.bin`

or to open in gdb for further postmortem do the following:

`arm-none-eabi-gdb -nx path/to/elf/file -ex "set target-charset ASCII" -ex "target remote | modules/CrashDebug/bins/lin64/CrashDebug --elf path/to/elf/file --dump crash_dump.bin"`

## Debugging faults using GDB:
* Connect hardware over SWD
* Place breakpoint at hardfault using `b *&HardFault_Handler`
* If one is lucky process stack remained untouched they can do `set $sp = $psp`
* Now you can simply run `backtrace` and potentially reach the fault
* If fault happens at startup one can run and then wait for breakpoint hit at HardFault_Handler
and then `set $sp = $psp` and do `backtrace`
* One can also log the RAM, refer crash_debugger app and Tools/debug/crash_dump.scr for the same.

### References:
https://interrupt.memfault.com/blog/cortex-m-fault-debug

https://github.com/adamgreen/CrashCatcher/tree/c8e801225bfa12da70c01ea25b58090b2b7a2e0a

http://www.cyrilfougeray.com/2020/07/27/firmware-logs-with-stack-trace.html
