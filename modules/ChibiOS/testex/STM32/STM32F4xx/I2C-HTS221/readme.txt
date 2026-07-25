*****************************************************************************
** ChibiOS/HAL + ChibiOS/EX - I2C + HTS221 demo for STM32F4xx.             **
*****************************************************************************

** TARGET **

The demo runs on an STM32 Nucleo64-F401RE board. It has been tested with both 
X-NUCLEO-IKS01A1 and X-NUCLEO-IKS01A2 shield.

** The Demo **

The application demonstrates the use of the STM32F4xx I2C driver in order
to acquire data from HTS221 using ChibiOS/EX.

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