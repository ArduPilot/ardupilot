*****************************************************************************
** ChibiOS/HAL - ADC driver demo for STM32L1xx.                            **
*****************************************************************************

** TARGET **

The demo will on an STMicroelectronics STM32L-Discovery board.

** The Demo **

The application demonstrates the use of the STM32L1xx ADC driver.

** Board Setup **

- Remove the LCD module.
- Connect PC0 to 3.3V and PC1 to GND for analog measurements.

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
