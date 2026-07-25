*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 STM32F469.                            **
*****************************************************************************

** TARGET **

The demo runs on an ADI EVAL-SDP-CK1Z board.


** The Demo **

The demo flashes the board LEDs using a thread, by short circuiting ARD-A0 
to GND the test procedure is activated with output on the serial port SD5 
(USART5, mapped on the CMSIS DAP Virtual COM Port).

** Build Procedure **

The demo has been tested by using the free Codesourcery GCC-based toolchain
and YAGARTO. just modify the TRGT line in the makefile in order to use
different GCC toolchains.