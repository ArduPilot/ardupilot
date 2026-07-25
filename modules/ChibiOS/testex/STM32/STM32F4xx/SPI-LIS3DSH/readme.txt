*****************************************************************************
** ChibiOS/HAL + ChibiOS/EX - SPI + LIS3DSH demo for STM32F4xx.            **
*****************************************************************************

** TARGET **

The demo runs on an STM32F407 Discovery board REV C or later.

** The Demo **

The application demonstrates the use of the STM32F4xx SPI driver in order
to acquire data from LIS3DSH using ChibiOS/EX.

** Board Setup **

None required.

** Build Procedure **

The demo has been tested using the free Codesourcery GCC-based toolchain
and YAGARTO.
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com