*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 STM32F469.                            **
*****************************************************************************

** TARGET **

The demo runs on an ST STM32F469I-Discovery board.

** The Demo **

A simple command shell is activated on STLink v2-1 virtual serial port SD3
driver.

** Build Procedure **

The demo has been tested by using the free Codesourcery GCC-based toolchain
and YAGARTO. just modify the TRGT line in the makefile in order to use
different GCC toolchains.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com
