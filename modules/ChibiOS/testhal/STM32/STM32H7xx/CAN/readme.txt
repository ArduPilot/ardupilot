*****************************************************************************
** ChibiOS/HAL - CAN driver demo for STM32H7xx.                            **
*****************************************************************************

** TARGET **

The demo runs on an ST STM32H750-Discovery board.

** The Demo **

The application demonstrates the use of the STM32H7xx CAN driver.


** Board Setup **

Close jumper JP7 and JP8 to use resistor 120ohm.

Connect CN11-1 (CAN1-H) to CN10-1 (CAN2-H).
Connect CN11-2 (CAN1-L) to CN10-2 (CAN2-L).


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
