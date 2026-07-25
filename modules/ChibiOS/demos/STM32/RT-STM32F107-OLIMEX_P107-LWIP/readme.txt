*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M3 STM32F107.                            **
*****************************************************************************

** TARGET **

The demo runs on an Olimex STM32-P107 board.

** The Demo **

The demo currently just flashes a LED using a thread and serves HTTP requests
at address 192.168.1.20 on port 80.
The button activates che ChibiOS/RT test suite, output on SD3.

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
