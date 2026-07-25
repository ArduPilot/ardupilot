*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 MAX32625.                             **
*****************************************************************************

** TARGET **

The demo runs on an ADI MAX32625MBED# board.


** The Demo **

The demo flashes the board LEDs using a thread, by pressing the button P2.3 the 
test procedure is activated with output on the serial port SD1 
(USAR1, mapped on the CMSIS DAP Virtual COM Port).

** Build Procedure **

The demo has been tested by using the free Codesourcery GCC-based toolchain
and YAGARTO. just modify the TRGT line in the makefile in order to use
different GCC toolchains.