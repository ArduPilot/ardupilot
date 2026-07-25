*****************************************************************************
** ChibiOS/HAL - PWM/ICU driver demo for STM32.                            **
*****************************************************************************

** TARGET **

The demo runs on an Olimex STM32-P103 board.

** The Demo **

The application demonstrates the use of the STM32 PWM and ICU drivers. Pins
PA8 and PB6 must be connected in order to trigger the ICU input with the
PWM output. The ICU unit will measure the generated PWM.

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
