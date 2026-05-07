See Also Pico2/README.md

./waf copter        # (or plane, rover, sub, heli, etc.)
```
Flash `build/Pico2/bin/arducopter.elf` via SWD — the target's SWD
port is on the 3-pin debug header (SWCLK/GND/SWDIO), which connects
to GPIO0 (board pin 1) and GPIO1 (board pin 2).
```

Flash `build/Pico2/bin/arducopter.elf` via SWD — the target's SWD
port is on the 3-pin debug header (SWCLK/GND/SWDIO), which connects
to GPIO0 (board pin 1) and GPIO1 (board pin 2)., using *two* Pico2Ws, one running debugprobe_on_pico2.uf2
    https://github.com/raspberrypi/debugprobe/releases/download/debugprobe-v2.3.0/debugprobe_on_pico2.uf2
    as the debugger, and the other running ardupilot as the "target"

    BOOTSEL flash the above file to a Pico2w, label it "debugger", and ..

    Holding the board with component side facing you and USB at the top:
    board pins 1-20 run down the LEFT column, board pins 21-40 run UP the RIGHT column.
    On the debugger board, skip the first two pins, then use the next 5:

    board-pin-1 / GPIO0  skip
    board-pin-2 / GPIO1  skip
    board-pin-3 / GND    use / GND
    board-pin-4 / GPIO2  use / SWCLK
    board-pin-5 / GPIO3  use / SWDIO
    board-pin-6 / GPIO4  use / UART0RX  → target GPIO1 (board pin 2)
    board-pin-7 / GPIO5  use / UART0TX  → target GPIO0 (board pin 1)

    Debugger board pin | Debugger GPIO | Target signal        | Notes
    board pin 3        | GND           | GND                  | Common ground — mandatory
    board pin 4        | GPIO2         | SWCLK                | SWD clock
    board pin 5        | GPIO3         | SWDIO                | SWD data
    board pin 6        | GPIO4/UART0RX | target GPIO1 (board pin 2) | console RX←TX
    board pin 7        | GPIO5/UART0TX | target GPIO0 (board pin 1) | console TX→RX

    SWCLK/GND/SWDIO are also available on the 3-pin debug header in the centre of the
    target board: left=SWCLK, middle=GND, right=SWDIO (same orientation as above).

# Get a compatible openocd... eg get premade binaries here:
    https://github.com/raspberrypi/pico-sdk-tools/releases/tag/v2.1.0-0

    # preereq:
    sudo apt-get update && sudo apt-get install libhidapi-hidraw0

    cd ~/Downloads
    wget https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.1.0-0/openocd-0.12.0+dev-x86_64-lin.tar.gz
    mkdir ~/openocd-pico
    mv openocd-0.12.0+dev-x86_64-lin.tar.gz ~/openocd-pico
    cd ~/openocd-pico
    tar -zxvf openocd-0.12.0+dev-x86_64-lin.tar.gz
    ~/openocd-pico/openocd
        Open On-Chip Debugger 0.12.0+dev-gebec950-dirty (2024-11-25-10:19)

# start openocd and if it finds your usb connected hardware/debugger/picoprobe, leave it running.
    cp ./Tools/debug/gdb-openocd-rp2350.init .gdbinit
    ~/openocd-pico/openocd -c "gdb_port 50000" -c "tcl_port 50001" -c "telnet_port 50002" -s ~/openocd-pico/scripts  -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000"
    Error: Error connecting DP: cannot read IDR
    # if u get this, you didnt plug the second usb cable in, this setup needs both, the target isnt booted/powered.
    # if it stays running without errors, mentions 'Cortex-M33 r1p0 processor detected' and is last line says ..
    'Info : Listening on port 50000 for gdb connections', then its working perfectly.

    good example output: 
        Open On-Chip Debugger 0.12.0+dev-gebec950-dirty (2024-11-25-10:19)
        Licensed under GNU GPL v2
        For bug reports, read
            http://openocd.org/doc/doxygen/bugs.html
        Info : Hardware thread awareness created
        Info : Hardware thread awareness created
        Info : Hardware thread awareness created
        Info : Hardware thread awareness created
        cortex_m reset_config sysresetreq
        adapter speed: 5000 kHz
        Info : Listening on port 50001 for tcl connections
        Info : Listening on port 50002 for telnet connections
        Info : Using CMSIS-DAPv2 interface with VID:PID=0x2e8a:0x000c, serial=E5CC2F739DCD6475
        Info : CMSIS-DAP: SWD supported
        Info : CMSIS-DAP: Atomic commands supported
        Info : CMSIS-DAP: Test domain timer supported
        Info : CMSIS-DAP: FW Version = 2.0.0
        Info : CMSIS-DAP: Interface Initialised (SWD)
        Info : SWCLK/TCK = 0 SWDIO/TMS = 0 TDI = 0 TDO = 0 nTRST = 0 nRESET = 0
        Info : CMSIS-DAP: Interface ready
        Info : clock speed 5000 kHz
        Info : SWD DPIDR 0x4c013477
        Info : [rp2350.dap.core0] Cortex-M33 r1p0 processor detected
        Info : [rp2350.dap.core0] target has 8 breakpoints, 4 watchpoints
        Info : [rp2350.dap.core0] Examination succeed
        Info : [rp2350.dap.core1] Cortex-M33 r1p0 processor detected
        Info : [rp2350.dap.core1] target has 8 breakpoints, 4 watchpoints
        Info : [rp2350.dap.core1] Examination succeed
        Info : starting gdb server for rp2350.dap.core0 on 50000
        Info : Listening on port 50000 for gdb connections



# make a .gdbinit file.
    cd ~/ardupilot/
    echo "# this sets up gdb to use openocd. You must start openocd first" > .gdbinit
    echo "target extended-remote :50000">> .gdbinit
    echo "mon reset halt">> .gdbinit
    echo "set confirm off">> .gdbinit

# locate files and run your debugger, pointing it at the *elf*, not the uf2 or aything.
    which arm-none-eabi-gdb
        /opt/gcc-arm-none-eabi-10-2020-q4-major/bin//arm-none-eabi-gdb 
    file build/Pico2/bin/arducopter
        build/Pico2/bin/arducopter: ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), statically linked, with debug_info, not stripped
    # run it...
    arm-none-eabi-gdb --quiet ./build/Pico2/bin/arducopter

       #example output of what it should look like:
            arm-none-eabi-gdb --quiet ./build/Pico2/bin/arducopter 
            Reading symbols from ./build/Pico2/bin/arducopter...
            warning: multi-threaded target stopped without sending a thread-id, using first non-exited thread
            0x00000088 in ?? ()
            [rp2350.dap.core1] VECTRESET is not supported on this Cortex-M core, using SYSRESETREQ instead.
            [rp2350.dap.core1] Set 'cortex_m reset_config sysresetreq'.
            [rp2350.dap.core0] halted due to debug-request, current mode: Thread 
            xPSR: 0xf9000000 pc: 0x00000088 msp: 0xf0000000
            [rp2350.dap.core1] halted due to debug-request, current mode: Thread 
            xPSR: 0xf9000000 pc: 0x00000088 msp: 0xf0000000
            (gdb) 

        # yay! 
    
    # recompile it all with --debug for a better gdb experience. ( add it to both waf commands)

# eg, possible three terminal setup...
    terminal 1 for build and upload:
    make clean ; 
    make -d  > make3.debug.log 2>&1 ; 
    make upload
    [then hold button and re-plug usb within 10 seconds to upload]

    terminal 2 just runs openocd, most of the time:
    started openocd:
    ~/openocd-pico/openocd -c "gdb_port 50000" -c "tcl_port 50001" -c "telnet_port 50002" -s ~/openocd-pico/scripts  -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000"

    terminal 3 for debugger/gdb etc:
    cd /home/buzz2/Chibi/ChibiOS-RT/demos/RP/RT-RP2040-PICO
    gdb connected, and poked around - seems to not crash immediatly any more:
    /opt/gcc-arm-none-eabi-10-2020-q4-major/bin//arm-none-eabi-gdb ./build/ch.elf


