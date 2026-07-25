*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 STM32L476.                            **
*****************************************************************************

** TARGET **

The demo runs on an STM32 Nucleo64-L476RG board.

** The Demo **

The demo flashes the board LED using a thread, by pressing the button located
on the board the test procedure is activated with output on the serial port
SD2 (USART2, mapped on USB virtual COM port).

** Build Procedure **

The demo has been tested by using the free Codesourcery GCC-based toolchain
and YAGARTO.
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com
