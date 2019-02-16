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

Now make sure you have the right version of arm-none-eabi-gdb
installed. We recommend version 6-2017-q2-update, which is available
here: http://firmware.ardupilot.org/Tools/STM32-tools/

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
