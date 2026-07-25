*****************************************************************************
** ChibiOS/HAL - PWM-ICU drivers demo for STM32F3xx.                       **
*****************************************************************************

** TARGET **

The demo runs on an STMicroelectronics STM32F3-Discovery board.

** The Demo **

The application demonstrates the use of the STM32F3xx PWM-ICU drivers.

** Board Setup **

- Connect PD12 (PWM output) and PC6 (ICU input) together.

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
