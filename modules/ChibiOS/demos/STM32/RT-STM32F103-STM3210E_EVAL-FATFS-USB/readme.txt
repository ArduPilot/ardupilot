*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 STM32F103.                            **
*****************************************************************************

** TARGET **

The demo runs on an Olimex STM32-E407 board.

** The Demo **

FatFs integrated using SDIO.
The USB-FS port is used as USB-CDC and a command shell is ready to accepts
commands there.

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
