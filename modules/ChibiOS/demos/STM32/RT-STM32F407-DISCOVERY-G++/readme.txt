*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 STM32F407.                            **
*****************************************************************************

** TARGET **

The demo runs on an ST STM32F4-Discovery board.

** The Demo **

The demo shows how to use PWM and SPI drivers using synchronous APIs. The PWM
driver the four board LEDs with the data read from the LIS320DL accelerometer.
The data is also transmitted on the SPI2 port.
A simple command shell is activated on virtual serial port SD2 via USB-CDC
driver (use micro-USB plug on STM32F4-Discovery board).

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
